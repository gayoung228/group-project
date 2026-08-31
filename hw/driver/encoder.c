#include "main.h"
#include "encoder.h"
#include "motor.h"
#include "ir_remote.h"
#include "rover_config.h"

/* ------------------------------------------------------------------
 * 하드웨어 배선 (Pin Map 문서 기준)
 *
 *   좌측 엔코더 : PA0  TIM5_CH1  Input Capture  <- 좌측 HC-020K OUT
 *   우측 엔코더 : PA1  TIM5_CH2  Input Capture  <- 우측 HC-020K OUT
 *
 * TIM5 는 32비트 타이머이고 Prescaler = 83 이므로
 * APB1 타이머 클럭 84MHz 기준으로 1카운트 = 1us 이다.
 *
 * HC-020K 는 채널이 하나뿐이라 회전 방향을 스스로 알 수 없다.
 * 따라서 부호는 motor_get_direction() 으로 판단한다.
 * ------------------------------------------------------------------ */

/* 이 시간 동안 펄스가 없으면 정지로 간주한다 [ms] */
#define ENCODER_TIMEOUT_MS      200

/* 새 측정값을 30%, 이전 필터값을 70% 반영한다. */
#define ENCODER_RPM_EMA_ALPHA   0.30f

/* 1분을 마이크로초로 표현한 값 */
#define ENCODER_US_PER_MINUTE   60000000.0f

/* 엔코더 개수 */
#define ENCODER_COUNT           2


extern TIM_HandleTypeDef htim5;


/* 엔코더 한 개의 하드웨어 정보 */
typedef struct
{
    uint32_t channel;       /* TIM5 의 캡처 채널 */
    uint32_t active_flag;   /* 캡처 인터럽트 활성화용 플래그 */
    motor_t  motor;         /* 회전 방향을 물어볼 모터 */
} encoder_hw_t;

/* 엔코더 한 개의 측정 상태 */
typedef struct
{
    int32_t  count;             /* 부호 있는 누적 펄스 수 */
    int32_t  delta_count;       /* 직전 구간의 펄스 변화량 */
    int32_t  last_count;        /* 직전 구간 종료 시점의 누적값 */
    float    rpm;               /* EMA 필터를 적용한 회전 속도 크기 */
    uint32_t last_pulse_tick;   /* 마지막 펄스가 들어온 시각 [ms] */
    uint32_t last_capture_tick; /* 직전 TIM5 캡처값 [us] */
    uint32_t pulse_interval_us; /* 최근 두 펄스 사이 시간 [us] */
    uint32_t pulse_sequence;    /* 새 간격이 생길 때마다 증가하는 번호 */
    bool     capture_started;   /* 첫 번째 캡처를 받았는지 여부 */
    bool     rpm_initialized;   /* EMA 첫 값을 받았는지 여부 */
} encoder_state_t;


/* 좌우 엔코더의 배선 정보 테이블 */
static const encoder_hw_t encoder_hw[ENCODER_COUNT] =
{
    /* ENCODER_LEFT  */ { TIM_CHANNEL_1, TIM_IT_CC1, MOTOR_LEFT  },
    /* ENCODER_RIGHT */ { TIM_CHANNEL_2, TIM_IT_CC2, MOTOR_RIGHT }
};

/* 인터럽트에서 갱신되므로 volatile 로 선언한다 */
static volatile encoder_state_t encoder_state[ENCODER_COUNT];
static bool encoder_ready[ENCODER_COUNT];
static uint32_t encoder_processed_sequence[ENCODER_COUNT];


/* 좌우 엔코더 타이머 또는 인터럽트를 초기화 */
bool encoder_init(void)
{
    encoder_id_t id;
    bool all_ready = true;

    encoder_reset();

    for (id = ENCODER_LEFT; id < ENCODER_COUNT; id++)
    {
        encoder_ready[id] =
            (HAL_TIM_IC_Start_IT(&htim5, encoder_hw[id].channel) == HAL_OK);
        if (encoder_ready[id] == false)
        {
            all_ready = false;
        }
    }

    return all_ready;
}

/* 오류 복구용 엔코더 재시동.
 * 이미 실행 중인 TIM5 CH1/CH2 인터럽트를 멈춘 뒤 새 측정처럼 다시 시작한다. */
bool encoder_restart(void)
{
    encoder_id_t id;

    for (id = ENCODER_LEFT; id < ENCODER_COUNT; id++)
    {
        (void)HAL_TIM_IC_Stop_IT(&htim5, encoder_hw[id].channel);
        encoder_ready[id] = false;
    }

    return encoder_init();
}

/* 선택한 엔코더 입력 캡처가 정상적으로 시작됐는지 반환 */
bool encoder_is_ready(encoder_id_t encoder)
{
    if (encoder >= ENCODER_COUNT)
    {
        return false;
    }

    return encoder_ready[encoder];
}

/* 좌우 엔코더 카운트와 RPM 계산값을 초기화 */
void encoder_reset(void)
{
    encoder_id_t id;
    uint32_t     now = HAL_GetTick();

    for (id = ENCODER_LEFT; id < ENCODER_COUNT; id++)
    {
        encoder_state[id].count           = 0;
        encoder_state[id].delta_count     = 0;
        encoder_state[id].last_count      = 0;
        encoder_state[id].rpm             = 0.0f;
        encoder_state[id].last_pulse_tick = now;
        encoder_state[id].last_capture_tick = 0;
        encoder_state[id].pulse_interval_us = 0;
        encoder_state[id].pulse_sequence = 0;
        encoder_state[id].capture_started = false;
        encoder_state[id].rpm_initialized = false;
        encoder_processed_sequence[id] = 0;
    }
}

/* 외부 인터럽트 방식에서 엔코더 펄스 발생을 전달
 * 인터럽트 안에서 호출되므로 최소한의 작업만 한다. */
void encoder_on_pulse(encoder_id_t encoder, uint32_t capture_tick)
{
    motor_direction_t direction;

    if (encoder >= ENCODER_COUNT)
    {
        return;
    }

    /* HC-020K 는 방향을 모르므로 모터에 준 명령으로 부호를 정한다 */
    direction = motor_get_direction(encoder_hw[encoder].motor);

    if (direction == MOTOR_REVERSE)
    {
        encoder_state[encoder].count--;
    }
    else
    {
        encoder_state[encoder].count++;
    }

    /* uint32_t 뺄셈은 TIM5가 0xFFFFFFFF에서 0으로 돌아가도 올바르게 동작한다. */
    if (encoder_state[encoder].capture_started == true)
    {
        encoder_state[encoder].pulse_interval_us =
            capture_tick - encoder_state[encoder].last_capture_tick;
        encoder_state[encoder].pulse_sequence++;
    }
    else
    {
        /* 첫 펄스만으로는 간격을 알 수 없으므로 기준 시각만 저장한다. */
        encoder_state[encoder].capture_started = true;
    }

    encoder_state[encoder].last_capture_tick = capture_tick;

    encoder_state[encoder].last_pulse_tick = HAL_GetTick();
}

/* 최근 두 펄스의 시간 간격으로 RPM을 계산하고 EMA 필터를 적용한다.
 * 펄스 개수 방식처럼 30RPM 단위로 끊기지 않고 실제 속도를 세밀하게 얻는다. */
void encoder_update(uint32_t elapsed_time_ms)
{
    encoder_id_t id;
    int32_t      now_count;
    int32_t      delta;
    uint32_t     interval_us;
    uint32_t     sequence;
    uint32_t     last_pulse_tick;
    float        measured_rpm;

    if (elapsed_time_ms == 0)
    {
        return;
    }

    for (id = ENCODER_LEFT; id < ENCODER_COUNT; id++)
    {
        /* 인터럽트가 중간에 값을 바꾸지 못하도록 한 번만 읽어서 쓴다 */
        now_count = encoder_state[id].count;
        delta     = now_count - encoder_state[id].last_count;

        encoder_state[id].delta_count = delta;
        encoder_state[id].last_count  = now_count;

        interval_us = encoder_state[id].pulse_interval_us;
        sequence = encoder_state[id].pulse_sequence;
        last_pulse_tick = encoder_state[id].last_pulse_tick;

        /* 새 펄스가 200ms 동안 없으면 정지로 보고 다음 첫 펄스부터 다시 잰다. */
        if ((HAL_GetTick() - last_pulse_tick) >= ENCODER_TIMEOUT_MS)
        {
            encoder_state[id].rpm = 0.0f;
            encoder_state[id].rpm_initialized = false;
            encoder_state[id].capture_started = false;
            encoder_processed_sequence[id] = sequence;
            continue;
        }

        /* 같은 펄스 간격을 매 제어 주기마다 중복 필터링하지 않는다. */
        if ((sequence == encoder_processed_sequence[id]) || (interval_us == 0U))
        {
            continue;
        }
        encoder_processed_sequence[id] = sequence;

        measured_rpm = ENCODER_US_PER_MINUTE
                     / ((float)interval_us
                        * (float)ROVER_ENCODER_SLOTS_PER_REV);

        /* 너무 짧은 노이즈 펄스는 모터가 낼 수 없는 RPM이므로 버린다. */
        if (measured_rpm > ROVER_ENCODER_MAX_PLAUSIBLE_RPM)
        {
            continue;
        }

        if (encoder_state[id].rpm_initialized == false)
        {
            encoder_state[id].rpm = measured_rpm;
            encoder_state[id].rpm_initialized = true;
        }
        else
        {
            encoder_state[id].rpm =
                (encoder_state[id].rpm * (1.0f - ENCODER_RPM_EMA_ALPHA))
              + (measured_rpm * ENCODER_RPM_EMA_ALPHA);
        }
    }
}

/* 선택한 엔코더의 부호 있는 누적 펄스 수를 반환 */
int32_t encoder_get_count(encoder_id_t encoder)
{
    if (encoder >= ENCODER_COUNT)
    {
        return 0;
    }

    return encoder_state[encoder].count;
}

/* 마지막 측정 구간에서 발생한 펄스 변화량을 반환 */
int32_t encoder_get_delta_count(encoder_id_t encoder)
{
    if (encoder >= ENCODER_COUNT)
    {
        return 0;
    }

    return encoder_state[encoder].delta_count;
}

/* 선택한 바퀴의 현재 회전 속도를 RPM으로 반환 */
float encoder_get_rpm(encoder_id_t encoder)
{
    if (encoder >= ENCODER_COUNT)
    {
        return 0.0f;
    }

    return encoder_state[encoder].rpm;
}

/* 선택한 바퀴에서 최근 펄스가 발생했는지 반환 */
bool encoder_is_running(encoder_id_t encoder)
{
    uint32_t elapsed;

    if (encoder >= ENCODER_COUNT)
    {
        return false;
    }

    elapsed = HAL_GetTick() - encoder_state[encoder].last_pulse_tick;

    return (elapsed < ENCODER_TIMEOUT_MS);
}


void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    /* HAL의 weak 콜백이라 프로젝트에 이 함수가 한 곳에만 있을 수 있어,
     * 다른 타이머(IR/TIM4)를 쓰는 hw/driver/ir_remote.c와 이 콜백 하나를
     * 공유한다. 그쪽 판단(TIM4가 맞는지, 캡처 레지스터를 어떻게 읽는지)은
     * 전부 ir_remote_capture_callback() 안에 있어서 여기서는 그냥 매번
     * 넘겨주기만 하면 된다 - 이 파일은 TIM4에 대해 아무것도 몰라도 된다. */
    ir_remote_capture_callback(htim);

    if (htim->Instance != TIM5)
    {
        return;
    }

    if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_1)
    {
        uint32_t captured = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_1);
        encoder_on_pulse(ENCODER_LEFT, captured);
    }
    else if (htim->Channel == HAL_TIM_ACTIVE_CHANNEL_2)
    {
        uint32_t captured = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        encoder_on_pulse(ENCODER_RIGHT, captured);
    }
}
