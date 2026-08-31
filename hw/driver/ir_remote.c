#include "main.h"
#include "ir_remote.h"

/* ============================================================================
 * ir_remote.c - PB6/TIM4_CH1 Input Capture 기반 NEC IR 리모컨 드라이버
 *
 * TIM4는 FALLING 엣지만 캡처하도록 CubeMX에서 이미 설정되어 있다
 * (docs/pinmap.md 참고). NEC 신호는 각 구간(Leader/비트)이 "마크(LOW) 시작
 * 엣지"로 시작하므로, FALLING-투-FALLING 간격 하나가 그 구간의
 * (마크 길이 + 스페이스 길이)를 그대로 나타낸다:
 *
 *   Leader : 9000us 마크 + 4500us 스페이스 = 13500us 간격
 *   Repeat : 9000us 마크 + 2250us 스페이스 = 11250us 간격 (뒤에 트레일링 마크 1개)
 *   비트 0 :  562.5us 마크 +  562.5us 스페이스 = 1125us 간격
 *   비트 1 :  562.5us 마크 + 1687.5us 스페이스 = 2250us 간격
 *
 * 그래서 이 드라이버는 FALLING 엣지 하나만으로 32비트 프레임 전체를 해석할
 * 수 있고, RISING 엣지나 폴라리티 토글이 필요 없다.
 *
 * HAL_TIM_IC_CaptureCallback()은 HAL의 weak 콜백이라 프로젝트 전체에 한 곳
 * (hw/driver/encoder.c)에만 정의될 수 있고, TIM4(IR)와 TIM5(엔코더)가 그
 * 콜백 하나를 공유한다. 그래서 encoder.c는 어떤 타이머든 매 호출마다
 * ir_remote_capture_callback()을 무조건 한 번 불러주기만 하고, "이게 TIM4가
 * 맞는지", "캡처 레지스터를 어떻게 읽는지" 같은 IR 관련 지식은 전부 이
 * 파일(ir_remote_capture_callback, ir_remote_on_capture) 안에만 있다.
 *
 * ir_remote_on_capture()(static)는 캡처값을 간격으로 바꿔 링버퍼에 저장하는
 * 것만 한다. Leader 판별, 비트 조립, inverse 검증, Repeat/140ms 처리는
 * 전부 ir_remote_update()(메인 루프에서 호출)에서 한다 - 엔코더 TIM5
 * 인터럽트를 방해하지 않기 위한 docs/pinmap.md의 설계 원칙 그대로.
 * ============================================================================ */

/* CubeMX가 생성한 타이머 핸들 (Core/Src/main.c) */
extern TIM_HandleTypeDef htim4;

/* ----------------------------------------------------------------------------
 * NEC 타이밍 판정 기준 [us] (TIM4 1카운트 = 1us, Prescaler=83 @ 84MHz)
 * 실측 여유를 두고 서로 겹치지 않게 잡았다.
 * -------------------------------------------------------------------------- */
#define IR_LEADER_MIN_US   13000U
#define IR_LEADER_MAX_US   14000U

#define IR_REPEAT_MIN_US   10750U
#define IR_REPEAT_MAX_US   11750U

#define IR_BIT0_MIN_US       800U
#define IR_BIT0_MAX_US      1400U

#define IR_BIT1_MIN_US      1700U
#define IR_BIT1_MAX_US      2600U

/* 마지막 유효 프레임(정규 또는 Repeat) 이후 이 시간 안에 다음 게 없으면
 * Repeat 프레임을 신뢰하지 않는다("버튼 뗌" 이후의 우연한 잡음으로 취급). */
#define IR_VALID_WINDOW_MS   140U

/* ----------------------------------------------------------------------------
 * ISR <-> 메인 루프 간 링버퍼 (캡처 간격[us]을 순서대로 저장)
 * -------------------------------------------------------------------------- */
#define IR_RING_SIZE   64U   /* 2의 거듭제곱 - 마스킹으로 나눗셈 없이 순환 */
#define IR_RING_MASK   (IR_RING_SIZE - 1U)

static volatile uint16_t ir_ring_buf[IR_RING_SIZE];
static volatile uint8_t  ir_ring_head = 0;   /* ISR이 쓰는 위치 */
static volatile uint8_t  ir_ring_tail = 0;   /* update()가 읽는 위치 */
static volatile bool     ir_ring_overflow = false;

static volatile uint16_t ir_last_capture = 0;
static volatile bool     ir_last_capture_valid = false;

/* ----------------------------------------------------------------------------
 * NEC 프레임 해석 상태머신 (메인 루프 컨텍스트에서만 접근, ISR에서는 안 건드림)
 * -------------------------------------------------------------------------- */
typedef enum
{
    IR_DECODE_IDLE = 0,   /* Leader 대기 */
    IR_DECODE_RECEIVING   /* 비트 32개 수신 중 */
} ir_decode_state_t;

static ir_decode_state_t ir_decode_state    = IR_DECODE_IDLE;
static uint32_t          ir_raw_data        = 0;
static uint8_t           ir_bit_count       = 0;
static uint32_t          ir_last_frame_tick = 0;

/* 가장 최근 확정된 유효 명령과 "아직 안 읽힘" 플래그 */
static uint8_t ir_pending_addr = 0;
static uint8_t ir_pending_cmd  = 0;
static bool    ir_pending_new  = false;


/* ---- ISR에서 호출(내부용): 캡처값을 간격으로 바꿔 링버퍼에 저장만 한다 ---- */
static void ir_remote_on_capture(uint16_t captured)
{
    uint16_t interval;
    uint8_t  next_head;

    if (!ir_last_capture_valid)
    {
        /* 첫 캡처는 이전 값이 없어 간격을 계산할 수 없다 - 기준점만 잡는다 */
        ir_last_capture = captured;
        ir_last_capture_valid = true;
        return;
    }

    interval = (uint16_t)(captured - ir_last_capture);   /* 16비트 wrap 자동 처리 */
    ir_last_capture = captured;

    next_head = (uint8_t)((ir_ring_head + 1U) & IR_RING_MASK);
    if (next_head == ir_ring_tail)
    {
        /* 버퍼가 가득 참 - 새 값을 버리고 오버플로만 표시한다(덮어쓰지 않음) */
        ir_ring_overflow = true;
        return;
    }

    ir_ring_buf[ir_ring_head] = interval;
    ir_ring_head = next_head;
}

/* ---- encoder.c의 HAL_TIM_IC_CaptureCallback()이 무조건 호출하는 진입점 ----
 * TIM4가 아니면 즉시 반환한다. TIM4가 맞으면 캡처 레지스터를 읽어
 * ir_remote_on_capture()로 넘긴다. encoder.c는 이 함수 하나만 알면 되고,
 * TIM4 판별이나 캡처 레지스터를 읽는 방법은 몰라도 된다. */
void ir_remote_capture_callback(TIM_HandleTypeDef *htim)
{
    uint32_t captured;

    if (htim->Instance != TIM4)
    {
        return;
    }

    captured = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
    ir_remote_on_capture((uint16_t)captured);
}


/* ---- 내부 함수: 링버퍼에서 하나 꺼내기, 없으면 false (메인 루프 컨텍스트) ---- */
static bool ir_ring_pop(uint16_t *out_interval)
{
    if (ir_ring_tail == ir_ring_head)
    {
        return false;
    }

    *out_interval = ir_ring_buf[ir_ring_tail];
    ir_ring_tail  = (uint8_t)((ir_ring_tail + 1U) & IR_RING_MASK);

    return true;
}

/* ---- 내부 함수: 수신 상태를 전부 IDLE로 되돌린다 ---- */
static void ir_decode_reset(void)
{
    ir_decode_state = IR_DECODE_IDLE;
    ir_raw_data      = 0;
    ir_bit_count     = 0;
}

/* ---- 내부 함수: 32비트가 다 모이면 inverse 검증 후 명령으로 확정한다 ---- */
static void ir_decode_finish_frame(void)
{
    uint8_t addr     = (uint8_t)(ir_raw_data >> 24);
    uint8_t addr_inv = (uint8_t)(ir_raw_data >> 16);
    uint8_t cmd      = (uint8_t)(ir_raw_data >> 8);
    uint8_t cmd_inv  = (uint8_t)(ir_raw_data);

    if ((addr == (uint8_t)~addr_inv) && (cmd == (uint8_t)~cmd_inv))
    {
        ir_pending_addr    = addr;
        ir_pending_cmd     = cmd;
        ir_pending_new     = true;
        ir_last_frame_tick = HAL_GetTick();
    }
    /* inverse 검증 실패 시 그냥 버린다(새 명령으로 취급하지 않음) */

    ir_decode_reset();
}

/* ---- 내부 함수: 간격 하나를 상태머신에 반영한다 ---- */
static void ir_decode_feed(uint16_t interval)
{
    if ((interval >= IR_LEADER_MIN_US) && (interval <= IR_LEADER_MAX_US))
    {
        /* 새 프레임 시작 - 이전까지 모으던 건 버린다 */
        ir_decode_reset();
        ir_decode_state = IR_DECODE_RECEIVING;
        return;
    }

    if ((interval >= IR_REPEAT_MIN_US) && (interval <= IR_REPEAT_MAX_US))
    {
        /* 140ms 창 안에서 온 Repeat만 "같은 버튼 계속 누름"으로 인정한다.
         * Repeat는 새 명령이 아니므로 ir_pending_new는 세우지 않는다. */
        if ((HAL_GetTick() - ir_last_frame_tick) <= IR_VALID_WINDOW_MS)
        {
            ir_last_frame_tick = HAL_GetTick();
        }
        ir_decode_reset();
        return;
    }

    if (ir_decode_state != IR_DECODE_RECEIVING)
    {
        /* Leader/Repeat도 아니고 비트 수신 중도 아니면 잡음 - 무시한다 */
        return;
    }

    if ((interval >= IR_BIT0_MIN_US) && (interval <= IR_BIT0_MAX_US))
    {
        ir_raw_data = (ir_raw_data << 1) | 0U;
        ir_bit_count++;
    }
    else if ((interval >= IR_BIT1_MIN_US) && (interval <= IR_BIT1_MAX_US))
    {
        ir_raw_data = (ir_raw_data << 1) | 1U;
        ir_bit_count++;
    }
    else
    {
        /* 알 수 없는 간격 - 프레임이 깨졌다고 보고 리셋한다 */
        ir_decode_reset();
        return;
    }

    if (ir_bit_count >= 32U)
    {
        ir_decode_finish_frame();
    }
}


/* ============================================================================
 * Public API (hw/driver/ir_remote.h)
 * ============================================================================ */

bool ir_remote_init(void)
{
    ir_ring_head = 0;
    ir_ring_tail = 0;
    ir_ring_overflow = false;
    ir_last_capture_valid = false;

    ir_decode_reset();
    ir_pending_new     = false;
    ir_last_frame_tick = 0;

    return (HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1) == HAL_OK);
}

void ir_remote_update(void)
{
    uint16_t interval;

    if (ir_ring_overflow)
    {
        /* 오버플로 발생 - 지금까지 모으던 프레임은 신뢰할 수 없으므로 버리고
         * 링버퍼와 수신 상태를 안전하게 초기화한다. */
        ir_ring_tail = ir_ring_head;
        ir_ring_overflow = false;
        ir_last_capture_valid = false;
        ir_decode_reset();
        return;
    }

    while (ir_ring_pop(&interval))
    {
        ir_decode_feed(interval);
    }
}

bool ir_remote_get_command(uint8_t *addr, uint8_t *cmd)
{
    if (!ir_pending_new)
    {
        return false;
    }

    if (addr != NULL) { *addr = ir_pending_addr; }
    if (cmd  != NULL) { *cmd  = ir_pending_cmd; }

    ir_pending_new = false;   /* 한 번 읽으면 소비된다 - Repeat로는 다시 안 세워짐 */

    return true;
}
