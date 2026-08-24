#include "ir_remote.h"

#include <stddef.h>

#include "main.h"

/*
 * main.c가 정의한 TIM1 핸들을 직접 extern 참조.
 * main.h/main.c는  같이 쓰는 공용 파일이라 거기에 새 extern 선언을
 * 추가하지 않고, 이 파일 안에서만 필요한 참조를 끝내기 위함.
 */
extern TIM_HandleTypeDef htim1;

/* 직접 측정한 NEC 리모컨의 주소와 버튼 command 값 */
#define NEC_REMOTE_ADDRESS     0x00U
#define NEC_COMMAND_FORWARD    0x18U
#define NEC_COMMAND_BACKWARD   0x52U
#define NEC_COMMAND_LEFT       0x08U
#define NEC_COMMAND_RIGHT      0x5AU
#define NEC_COMMAND_STOP       0x43U
#define NEC_COMMAND_SPEED_UP   0x15U
#define NEC_COMMAND_SPEED_DOWN 0x07U

/*
 * TIM1 카운터는 1MHz(1tick = 1us)로 동작.
 * (TIM1CLK 84MHz, PSC=83 -> 84MHz / 84 = 1MHz)
 * NEC 표준 타이밍(리더 9000/4500us, bit0 562.5+562.5us, bit1 562.5+1687.5us, repeat 9000/2250us)에 여유 허용오차를 둔 임계값이다.
 */
#define NEC_LEADER_MARK_MIN_US   7500U
#define NEC_LEADER_MARK_MAX_US  10500U
#define NEC_LEADER_SPACE_MIN_US  3500U
#define NEC_LEADER_SPACE_MAX_US  5500U
#define NEC_REPEAT_SPACE_MIN_US  1800U
#define NEC_REPEAT_SPACE_MAX_US  2800U
#define NEC_BIT0_SPACE_MIN_US     300U
#define NEC_BIT0_SPACE_MAX_US     850U
#define NEC_BIT1_SPACE_MIN_US    1300U
#define NEC_BIT1_SPACE_MAX_US    2100U
#define NEC_FRAME_BITS             32U
#define NEC_IDLE_TIMEOUT_US     12000U

/* NEC repeat 프레임은 버튼을 누르고 있는 동안 약 108ms 간격으로 재전송된다. 그보다 긴 공백이면 남의 신호로 간주한다. */
#define NEC_REPEAT_WINDOW_MS       140U

/* 하나의 NEC 프레임을 하강 에지 단위로 조립하기 위한 진행 상태. */
typedef enum
{
    NEC_STATE_IDLE = 0,
    NEC_STATE_LEADER_MARK,
    NEC_STATE_DATA,
} nec_decode_state_t;

/*
 * 인터럽트 컨텍스트와 메인 스레드에서 공유되는 변수는 volatile로 선언한다.
 */
static volatile RemoteCommand pending_command = DRIVE_CMD_NONE;
static volatile drive_command_t last_valid_command = DRIVE_CMD_NONE;
/* last_valid_command가 확인/갱신된 시각(ms). repeat 프레임의 유효 시간창 판정에 쓰인다. */
static volatile uint32_t last_frame_tick = 0U;

static volatile nec_decode_state_t decode_state = NEC_STATE_IDLE;
static volatile uint16_t last_edge_capture = 0U;
static volatile uint32_t frame_bits = 0U;
static volatile uint32_t bit_count = 0U;

/* 검증을 통과한 NEC command를 차량 제어 명령으로 변환한다. */
static drive_command_t map_nec_command(uint8_t command)
{
    switch (command)
    {
        case NEC_COMMAND_FORWARD:
            return DRIVE_CMD_FORWARD;
        case NEC_COMMAND_BACKWARD:
            return DRIVE_CMD_BACKWARD;
        case NEC_COMMAND_LEFT:
            return DRIVE_CMD_LEFT;
        case NEC_COMMAND_RIGHT:
            return DRIVE_CMD_RIGHT;
        case NEC_COMMAND_STOP:
            return DRIVE_CMD_STOP;
        case NEC_COMMAND_SPEED_UP:
            return DRIVE_CMD_SPEED_UP;
        case NEC_COMMAND_SPEED_DOWN:
            return DRIVE_CMD_SPEED_DOWN;
        default:
            return DRIVE_CMD_NONE;
    }
}

/* 리더/스페이스 길이가 NEC 규격을 벗어나면 프레임을 버리고 처음부터 다시 찾도록 초기화한다. */
static void nec_decode_reset(void)
{
    decode_state = NEC_STATE_IDLE;
    frame_bits = 0U;
    bit_count = 0U;
}

void ir_remote_init(void)
{
    /* 아직 전달할 리모컨 명령이 없는 초기 상태로 설정한다. */
    pending_command = DRIVE_CMD_NONE;
    last_valid_command = DRIVE_CMD_NONE;
    last_frame_tick = 0U;
    last_edge_capture = 0U;
    nec_decode_reset();

    /* TIM1_CH3(PA10) Input Capture 인터럽트를 시작해야 에지가 실제로 감지된다. */
    if (HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }
}

/*
 * docs/interface.md 호환: 주기적으로 신호(명령)를 탐색하여
 * 새로 확정된 명령이 있으면 반환하고 소비한다. 없으면 DRIVE_CMD_NONE 반환.
 */
RemoteCommand ir_remote_update(void)
{
    return ir_remote_take_command();
}

/*
 * docs/interface.md의 반환형(RemoteCommand)에 맞추어 보관된 명령을 반환하고 소비한다.
 * 동시성 문제(Race Condition)를 방지하기 위해 임계 구역으로 보호한다.
 */
RemoteCommand ir_remote_take_command(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    RemoteCommand command = pending_command;
    pending_command = DRIVE_CMD_NONE;
    __set_PRIMASK(primask);
    return command;
}

void ir_remote_on_raw_code(uint32_t raw_code)
{
    /*
     * NEC 32비트 프레임을 바이트 단위로 분리한다.
     * [7:0] address, [15:8] address inverse,
     * [23:16] command, [31:24] command inverse
     */
    const uint8_t address = (uint8_t)(raw_code & 0xFFU);
    const uint8_t address_inverse = (uint8_t)((raw_code >> 8U) & 0xFFU);
    const uint8_t command = (uint8_t)((raw_code >> 16U) & 0xFFU);
    const uint8_t command_inverse = (uint8_t)((raw_code >> 24U) & 0xFFU);
    drive_command_t mapped_command;

    /* address와 command가 각각 inverse 바이트와 보수 관계인지 검증한다. */
    if (((uint8_t)(address ^ address_inverse) != 0xFFU) ||
        ((uint8_t)(command ^ command_inverse) != 0xFFU))
    {
        /* 손상된 새 프레임 뒤의 repeat이 직전 명령을 재생하지 않도록 무효화한다. */
        last_valid_command = DRIVE_CMD_NONE;
        return;
    }

    /*
     * 주변의 다른 NEC 리모컨 신호는 차량 명령으로 받아들이지 않는다.
     * 이 프레임은 체크섬을 통과한 "진짜 남의 리모컨" 신호이므로, 혹시 남아있던
     * last_valid_command도 즉시 지워서 뒤이어 오는 repeat이 재생되지 않게 한다.
     */
    if (address != NEC_REMOTE_ADDRESS)
    {
        last_valid_command = DRIVE_CMD_NONE;
        return;
    }

    /*
     * 측정되지 않은 버튼 command는 무시한다.
     * 이때도 last_valid_command를 지워서, 뒤이어 이 버튼의 repeat이 오더라도
     * 이전에 확정됐던(다른 버튼의) 명령이 잘못 재생되지 않게 한다.
     */
    mapped_command = map_nec_command(command);
    if (mapped_command == DRIVE_CMD_NONE)
    {
        last_valid_command = DRIVE_CMD_NONE;
        return;
    }

    /* 상위 제어 코드가 가져갈 수 있도록 유효한 명령을 보관하고, repeat 유효 시간창의 기준 시각을 갱신한다. */
    pending_command = mapped_command;
    last_valid_command = mapped_command;
    last_frame_tick = HAL_GetTick();
}

/*
 * TIM1_CH3(PA10) Both Edge Input Capture 콜백.
 * 에지 간격(delta)과 에지 직후의 핀 레벨로 NEC 리더/스페이스/리피트를 구분해
 * 32비트 프레임을 조립하고, 완성되면 ir_remote_on_raw_code()로 전달한다.
 */
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    uint16_t capture;
    uint16_t delta;
    GPIO_PinState level;

    /* 이 콜백은 HAL 전체에서 공유되는 weak 함수라, TIM1의 IR용 채널이 아니면 무시한다. */
    if ((htim->Instance != TIM1) || (htim->Channel != HAL_TIM_ACTIVE_CHANNEL_3))
    {
        return;
    }

    capture = (uint16_t)HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_3);
    /*
     * TIM1은 16비트 카운터라 uint16_t 뺄셈이 오버플로우를 자동으로 모듈러 처리해준다.
     */
    delta = (uint16_t)(capture - last_edge_capture);
    last_edge_capture = capture;

    /*
     * IR 수신 모듈은 active-low라 하강 에지가 새 마크(버스트) 시작, 상승 에지가
     * 마크 종료(스페이스 시작)다. BOTHEDGE로 잡히는 두 에지를 구분하려면
     * 캡처 직후의 현재 핀 레벨을 읽어야 한다.
     */
    level = HAL_GPIO_ReadPin(IR_REMOTE_IN_GPIO_Port, IR_REMOTE_IN_Pin);

    if (level == GPIO_PIN_RESET)
    {
        /* 하강 에지: 직전 스페이스 구간이 끝나고 새 마크가 시작된다. */

        /*
         * 직전 에지 이후 긴 유휴 시간(> 12ms)이 지났다면,
         * 이전 프레임의 미완료/잡음 상태를 강제 리셋하여 새 프레임 수신을 보장한다.
         */
        if (delta > NEC_IDLE_TIMEOUT_US)
        {
            nec_decode_reset();
        }

        switch (decode_state)
        {
            case NEC_STATE_IDLE:
                /* 프레임의 첫 하강 에지: 리더 마크 시작. */
                decode_state = NEC_STATE_LEADER_MARK;
                frame_bits = 0U;
                bit_count = 0U;
                break;

            case NEC_STATE_LEADER_MARK:
                /* 리더 스페이스(약 4500us) 또는 리피트 스페이스(약 2250us) 길이를 검증한다. */
                if ((delta >= NEC_LEADER_SPACE_MIN_US) && (delta <= NEC_LEADER_SPACE_MAX_US))
                {
                    decode_state = NEC_STATE_DATA;
                }
                else if ((delta >= NEC_REPEAT_SPACE_MIN_US) && (delta <= NEC_REPEAT_SPACE_MAX_US))
                {
                    /*
                     * 버튼을 꾹 누르고 있을 때 발생하는 repeat 프레임 처리.
                     * repeat 프레임은 NEC 규격상 주소/명령 정보를 담지 않아 어느 리모컨에서
                     * 왔는지 구분할 수 없으므로, 직전 유효 프레임(또는 repeat) 이후
                     * NEC_REPEAT_WINDOW_MS 이내에 도착한 것만 우리 리모컨의 연속 입력으로 인정한다.
                     * 그보다 오래됐으면 주변의 다른 리모컨 신호일 가능성이 높으므로 폐기한다.
                     */
                    uint32_t now = HAL_GetTick();

                    if ((last_valid_command != DRIVE_CMD_NONE) &&
                        ((now - last_frame_tick) <= NEC_REPEAT_WINDOW_MS))
                    {
                        pending_command = last_valid_command;
                        last_frame_tick = now; /* 다음 repeat을 위해 시간창을 갱신한다. */
                    }
                    else
                    {
                        last_valid_command = DRIVE_CMD_NONE;
                    }
                    nec_decode_reset();
                }
                else
                {
                    nec_decode_reset();
                }
                break;

            case NEC_STATE_DATA:
                /*
                 * NEC는 마크 길이(약 562.5us)가 항상 일정하고, 뒤따르는 스페이스 길이만
                 * bit 0/1을 구분한다. 그래서 마크가 아니라 하강 에지 간격(=직전 스페이스)만
                 * 검사하면 비트값을 판별할 수 있다.
                 */
                if ((delta >= NEC_BIT0_SPACE_MIN_US) && (delta <= NEC_BIT0_SPACE_MAX_US))
                {
                    /* bit 0: LSB부터 순서대로 채워 넣는다. */
                    bit_count++;
                }
                else if ((delta >= NEC_BIT1_SPACE_MIN_US) && (delta <= NEC_BIT1_SPACE_MAX_US))
                {
                    /* bit 1: 수신 순서(=LSB 우선)대로 해당 비트 위치에 1을 세팅한다. */
                    frame_bits |= (1UL << bit_count);
                    bit_count++;
                }
                else
                {
                    /* 두 구간 어디에도 안 들어가면 잡음/오수신이므로 프레임을 버린다. */
                    nec_decode_reset();
                    break;
                }

                if (bit_count >= NEC_FRAME_BITS)
                {
                    /* 32비트가 다 모이면 [7:0]=address ... [31:24]=command inverse 배치가 완성된다. */
                    ir_remote_on_raw_code(frame_bits);
                    nec_decode_reset();
                }
                break;

            default:
                nec_decode_reset();
                break;
        }
    }
    else
    {
        /*
         * 상승 에지: 마크 구간이 끝난다. 리더 마크(약 9000us)만 여기서 검증하고,
         * 데이터 비트의 마크 길이는 검사하지 않는다.
         */
        if ((decode_state == NEC_STATE_LEADER_MARK) &&
            ((delta < NEC_LEADER_MARK_MIN_US) || (delta > NEC_LEADER_MARK_MAX_US)))
        {
            nec_decode_reset();
        }
    }
}
