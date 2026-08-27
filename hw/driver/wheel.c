#include "main.h"
#include "wheel.h"
#include "motor.h"
#include "encoder.h"

/* ------------------------------------------------------------------
 * wheel.c - 바퀴 단위 PID 폐루프 속도 제어
 *
 * motor(출력) 와 encoder(입력) 를 묶어 목표 RPM 을 추종한다.
 *
 * encoder_get_rpm() 은 크기만 돌려주므로
 * 회전 방향은 motor_get_direction() 으로 판단해 부호를 붙인다.
 * ------------------------------------------------------------------ */

/* 모터 출력의 최대값 [%] */
#define WHEEL_OUTPUT_MAX        100

/* 직진 주행의 기준 출력 [%] */
#define WHEEL_DEFAULT_OUTPUT    90

/* 기어모터가 실제로 돌기 시작하는 최소 출력 [%]
 * 이 값보다 작으면 소리만 나고 바퀴가 움직이지 않으므로
 * 목표가 0 이 아닐 때는 최소한 이만큼은 넣어준다.
 * 실측한 기동 듀티에 맞춰 조정할 것. */
#define WHEEL_MIN_OUTPUT        80

/* 적분항이 쌓일 수 있는 한계 (적분 포화 방지) */
#define WHEEL_INTEGRAL_LIMIT    300.0f

/* 목표 도달로 판단할 오차 범위 [RPM]
 * 엔코더 분해능상 RPM 이 30 단위로 끊기므로 그보다 작게 잡으면 안 된다. */
#define WHEEL_RPM_TOLERANCE     35.0f

/* PID 게인 초기값 (실측 후 튜닝할 것) */
#define WHEEL_DEFAULT_KP        0.08f
#define WHEEL_DEFAULT_KI        0.02f
#define WHEEL_DEFAULT_KD        0.0f

/* 좌우 누적 펄스 차이를 PWM 보정량으로 바꾸는 값.
 * 예: 한쪽이 펄스 10개 앞서면 좌우 출력을 각각 5% 보정한다. */
#define WHEEL_SYNC_KP           0.5f
#define WHEEL_SYNC_LIMIT        10.0f

/* 출발 후 이 시간 안에 양쪽 엔코더 펄스가 모두 들어와야 한다. */
#define WHEEL_STARTUP_TIMEOUT_MS 500U

/* 바퀴 개수 */
#define WHEEL_COUNT             2


/* 바퀴 한 개가 사용하는 모터와 엔코더 */
typedef struct
{
    motor_t      motor;
    encoder_id_t encoder;
} wheel_hw_t;

/* 바퀴 한 개의 제어 상태 */
typedef struct
{
    float   target_rpm;     /* 목표 회전 속도 (전진 +, 후진 -) */
    float   measured_rpm;   /* 부호를 붙인 실측 회전 속도 */
    float   error;          /* 목표와 실측의 차이 */
    float   integral;       /* 오차의 누적값 */
    float   prev_error;     /* 직전 구간의 오차 */
    int16_t output;         /* 모터에 적용한 출력 (-100~100) */
} wheel_state_t;


/* 좌우 바퀴가 어떤 모터와 엔코더를 쓰는지 정리한 표 */
static const wheel_hw_t wheel_hw[WHEEL_COUNT] =
{
    /* WHEEL_LEFT  */ { MOTOR_LEFT,  ENCODER_LEFT  },
    /* WHEEL_RIGHT */ { MOTOR_RIGHT, ENCODER_RIGHT }
};

static wheel_state_t wheel_state[WHEEL_COUNT];

/* 좌우 공통 PID 게인 */
static float wheel_kp = WHEEL_DEFAULT_KP;
static float wheel_ki = WHEEL_DEFAULT_KI;
static float wheel_kd = WHEEL_DEFAULT_KD;

/* 폐루프 제어 사용 여부 */
static bool wheel_enabled = true;

/* 출발 시 양쪽 바퀴가 모두 움직이기 시작했는지 확인하는 상태 */
static bool     wheel_starting = false;
static bool     wheel_startup_fault = false;
static uint32_t wheel_start_tick = 0;


/* 실수값을 지정한 범위 안으로 잘라주는 내부 함수 */
static float wheel_clamp(float value, float min, float max)
{
    if (value > max)
    {
        return max;
    }
    if (value < min)
    {
        return min;
    }
    return value;
}

/* 엔코더가 준 RPM 크기에 모터 방향으로 부호를 붙여 돌려주는 내부 함수 */
static float wheel_read_rpm(wheel_t wheel)
{
    float             rpm       = encoder_get_rpm(wheel_hw[wheel].encoder);
    motor_direction_t direction = motor_get_direction(wheel_hw[wheel].motor);

    /* 엔코더가 이미 부호를 붙여 주는 경우를 대비해 크기만 취한다 */
    if (rpm < 0.0f)
    {
        rpm = -rpm;
    }

    if (direction == MOTOR_REVERSE)
    {
        return -rpm;
    }
    if (direction == MOTOR_FORWARD)
    {
        return rpm;
    }

    /* 정지 명령 상태라면 관성으로 굴러도 0 으로 본다 */
    return 0.0f;
}

/* 바퀴 한 개의 PID 를 계산해 모터에 적용하는 내부 함수 */
static void wheel_control(wheel_t wheel, float dt_s)
{
    wheel_state_t *state = &wheel_state[wheel];
    float          derivative;
    float          direction;
    float          magnitude;
    float          output;

    state->measured_rpm = wheel_read_rpm(wheel);
    state->error        = state->target_rpm - state->measured_rpm;

    /* 목표가 0 이면 제어하지 않고 세운다.
     * 이렇게 해야 정지 중에 적분항이 계속 쌓이지 않는다 */
    if (state->target_rpm == 0.0f)
    {
        state->integral   = 0.0f;
        state->prev_error = 0.0f;
        state->output     = 0;

        motor_stop(wheel_hw[wheel].motor);
        return;
    }

    /* 적분항 : 정상상태 오차를 없애지만 과도하게 쌓이면 응답이 나빠진다 */
    state->integral += state->error * dt_s;
    state->integral  = wheel_clamp(state->integral,
                                   -WHEEL_INTEGRAL_LIMIT,
                                    WHEEL_INTEGRAL_LIMIT);

    /* 미분항 : 오차의 변화율로 오버슈트를 억제한다 */
    derivative        = (state->error - state->prev_error) / dt_s;
    state->prev_error = state->error;

    /* PID 결과를 0에서 시작하면 오차가 음수일 때 모터가 역회전할 수 있다.
     * 기준 90%에 PID 보정량만 더하고, 방향은 목표 RPM의 부호로 고정한다. */
    direction = (state->target_rpm > 0.0f) ? 1.0f : -1.0f;

    magnitude = (float)WHEEL_DEFAULT_OUTPUT
              + direction * ((wheel_kp * state->error)
                           + (wheel_ki * state->integral)
                           + (wheel_kd * derivative));

    magnitude = wheel_clamp(magnitude,
                            (float)WHEEL_MIN_OUTPUT,
                            (float)WHEEL_OUTPUT_MAX);

    output = direction * magnitude;

    state->output = (int16_t)output;

}

/* 좌우가 같은 방향으로 돌 때 누적 펄스 차이를 줄인다.
 * 오른쪽이 앞서면 왼쪽 출력을 높이고 오른쪽 출력을 낮춘다. */
static void wheel_apply_sync(void)
{
    float left_direction;
    float right_direction;
    float left_progress;
    float right_progress;
    float sync_error;
    float correction;
    float left_magnitude;
    float right_magnitude;
    float target_difference;

    if ((wheel_state[WHEEL_LEFT].target_rpm == 0.0f) ||
        (wheel_state[WHEEL_RIGHT].target_rpm == 0.0f))
    {
        return;
    }

    left_direction  = (wheel_state[WHEEL_LEFT].target_rpm > 0.0f) ? 1.0f : -1.0f;
    right_direction = (wheel_state[WHEEL_RIGHT].target_rpm > 0.0f) ? 1.0f : -1.0f;

    /* 좌우가 서로 반대로 돌아야 하는 회전 명령에는 직진 동기화를 적용하지 않는다. */
    if (left_direction != right_direction)
    {
        return;
    }

    /* 자이로 제어가 의도적으로 좌우 RPM을 다르게 주는 동안에는
     * 엔코더 누적 펄스 동기화를 끄어서 두 제어기가 서로 방해하지 않게 한다. */
    target_difference = wheel_state[WHEEL_LEFT].target_rpm
                      - wheel_state[WHEEL_RIGHT].target_rpm;
    if ((target_difference > 0.1f) || (target_difference < -0.1f))
    {
        return;
    }

    left_progress  = (float)encoder_get_count(ENCODER_LEFT)  * left_direction;
    right_progress = (float)encoder_get_count(ENCODER_RIGHT) * right_direction;
    sync_error     = left_progress - right_progress;

    correction = wheel_clamp(sync_error * WHEEL_SYNC_KP,
                             -WHEEL_SYNC_LIMIT,
                              WHEEL_SYNC_LIMIT);

    left_magnitude  = (float)wheel_state[WHEEL_LEFT].output  * left_direction;
    right_magnitude = (float)wheel_state[WHEEL_RIGHT].output * right_direction;

    left_magnitude  = wheel_clamp(left_magnitude - correction,
                                  (float)WHEEL_MIN_OUTPUT,
                                  (float)WHEEL_OUTPUT_MAX);
    right_magnitude = wheel_clamp(right_magnitude + correction,
                                  (float)WHEEL_MIN_OUTPUT,
                                  (float)WHEEL_OUTPUT_MAX);

    wheel_state[WHEEL_LEFT].output  = (int16_t)(left_direction  * left_magnitude);
    wheel_state[WHEEL_RIGHT].output = (int16_t)(right_direction * right_magnitude);
}


/* 모터와 엔코더를 초기화하고 좌우 바퀴의 PID 상태를 준비한다. */
void wheel_init(void)
{
    motor_init();
    encoder_init();

    wheel_kp      = WHEEL_DEFAULT_KP;
    wheel_ki      = WHEEL_DEFAULT_KI;
    wheel_kd      = WHEEL_DEFAULT_KD;
    wheel_enabled = true;

    wheel_reset();
}

/* 좌우 바퀴의 목표 RPM과 PID 누적 상태를 모두 초기화한다. */
void wheel_reset(void)
{
    wheel_t wheel;

    for (wheel = WHEEL_LEFT; wheel < WHEEL_COUNT; wheel++)
    {
        wheel_state[wheel].target_rpm   = 0.0f;
        wheel_state[wheel].measured_rpm = 0.0f;
        wheel_state[wheel].error        = 0.0f;
        wheel_state[wheel].integral     = 0.0f;
        wheel_state[wheel].prev_error   = 0.0f;
        wheel_state[wheel].output       = 0;
    }

    wheel_starting = false;
    wheel_startup_fault = false;
    wheel_start_tick = 0;

    encoder_reset();
}

/* 제어 주기마다 호출한다. 엔코더를 갱신하고 PID로 모터 출력을 계산한다. */
void wheel_update(uint32_t elapsed_time_ms)
{
    wheel_t wheel;
    float   dt_s;

    if (elapsed_time_ms == 0)
    {
        return;
    }

    /* 먼저 엔코더를 갱신해야 최신 RPM 으로 제어할 수 있다 */
    encoder_update(elapsed_time_ms);

    /* 제어를 꺼 둔 상태에서는 측정만 하고 모터는 건드리지 않는다 */
    if (wheel_enabled == false)
    {
        for (wheel = WHEEL_LEFT; wheel < WHEEL_COUNT; wheel++)
        {
            wheel_state[wheel].measured_rpm = wheel_read_rpm(wheel);
        }
        return;
    }

    /* 출발 직후에는 PID가 큰 정지 RPM 오차로 100%를 내지 않게
     * 양쪽을 기준 출력 90%로 동시에 출발시킨다. */
    if (wheel_starting == true)
    {
        bool left_started  = (encoder_get_count(ENCODER_LEFT) != 0);
        bool right_started = (encoder_get_count(ENCODER_RIGHT) != 0);

        if (left_started && right_started)
        {
            /* 두 바퀴가 모두 움직이면 다음 계산부터 PID로 전환한다. */
            wheel_starting = false;

            for (wheel = WHEEL_LEFT; wheel < WHEEL_COUNT; wheel++)
            {
                wheel_state[wheel].integral = 0.0f;
                wheel_state[wheel].prev_error = 0.0f;
            }
        }
        else if ((HAL_GetTick() - wheel_start_tick) >= WHEEL_STARTUP_TIMEOUT_MS)
        {
            /* 한쪽이 시작하지 않으면 재시동하지 않고 오류를 고정한다. */
            wheel_starting = false;
            wheel_startup_fault = true;
            wheel_stop();
            return;
        }
        else
        {
            int16_t left_output = (wheel_state[WHEEL_LEFT].target_rpm >= 0.0f)
                                ? WHEEL_DEFAULT_OUTPUT : -WHEEL_DEFAULT_OUTPUT;
            int16_t right_output = (wheel_state[WHEEL_RIGHT].target_rpm >= 0.0f)
                                 ? WHEEL_DEFAULT_OUTPUT : -WHEEL_DEFAULT_OUTPUT;

            wheel_state[WHEEL_LEFT].measured_rpm  = wheel_read_rpm(WHEEL_LEFT);
            wheel_state[WHEEL_RIGHT].measured_rpm = wheel_read_rpm(WHEEL_RIGHT);
            wheel_state[WHEEL_LEFT].output  = left_output;
            wheel_state[WHEEL_RIGHT].output = right_output;

            motor_set_output(MOTOR_LEFT, left_output);
            motor_set_output(MOTOR_RIGHT, right_output);
            return;
        }
    }

    if (wheel_startup_fault == true)
    {
        wheel_stop();
        return;
    }

    dt_s = (float)elapsed_time_ms / 1000.0f;

    for (wheel = WHEEL_LEFT; wheel < WHEEL_COUNT; wheel++)
    {
        wheel_control(wheel, dt_s);
    }

    wheel_apply_sync();

    for (wheel = WHEEL_LEFT; wheel < WHEEL_COUNT; wheel++)
    {
        motor_set_output(wheel_hw[wheel].motor, wheel_state[wheel].output);
    }
}

/* 선택한 바퀴의 목표 회전 속도를 RPM으로 설정한다. */
void wheel_set_target_rpm(wheel_t wheel, float target_rpm)
{
    if (wheel >= WHEEL_COUNT)
    {
        return;
    }

    /* 목표의 부호가 바뀌면 이전 오차가 방해가 되므로 누적을 지운다 */
    if ((wheel_state[wheel].target_rpm * target_rpm) < 0.0f)
    {
        wheel_state[wheel].integral   = 0.0f;
        wheel_state[wheel].prev_error = 0.0f;
    }

    wheel_state[wheel].target_rpm = target_rpm;
}

/* 좌우 바퀴의 목표 회전 속도를 한 번에 설정한다. */
void wheel_set_target_rpm_both(float left_rpm, float right_rpm)
{
    bool was_stopped;

    if (wheel_startup_fault == true)
    {
        return;
    }

    was_stopped = (wheel_state[WHEEL_LEFT].target_rpm == 0.0f) &&
                  (wheel_state[WHEEL_RIGHT].target_rpm == 0.0f);

    wheel_set_target_rpm(WHEEL_LEFT,  left_rpm);
    wheel_set_target_rpm(WHEEL_RIGHT, right_rpm);

    if (was_stopped && (left_rpm != 0.0f) && (right_rpm != 0.0f))
    {
        wheel_starting = true;
        wheel_start_tick = HAL_GetTick();
    }
}

/* 출발 시간 내에 양쪽 엔코더가 모두 감지되지 않았는지 반환한다. */
bool wheel_has_startup_fault(void)
{
    return wheel_startup_fault;
}

/* 선택한 바퀴에 설정된 목표 RPM을 반환한다. */
float wheel_get_target_rpm(wheel_t wheel)
{
    if (wheel >= WHEEL_COUNT)
    {
        return 0.0f;
    }

    return wheel_state[wheel].target_rpm;
}

/* 선택한 바퀴의 실측 RPM을 반환한다. */
float wheel_get_rpm(wheel_t wheel)
{
    if (wheel >= WHEEL_COUNT)
    {
        return 0.0f;
    }

    return wheel_state[wheel].measured_rpm;
}

/* 선택한 바퀴의 목표 RPM과 실측 RPM의 차이를 반환한다. */
float wheel_get_error(wheel_t wheel)
{
    if (wheel >= WHEEL_COUNT)
    {
        return 0.0f;
    }

    return wheel_state[wheel].error;
}

/* PID가 계산해 모터에 적용한 출력값을 반환한다. */
int16_t wheel_get_output(wheel_t wheel)
{
    if (wheel >= WHEEL_COUNT)
    {
        return 0;
    }

    return wheel_state[wheel].output;
}

/* 좌우 바퀴에 공통으로 적용할 PID 게인을 설정한다. */
void wheel_set_gain(float kp, float ki, float kd)
{
    wheel_t wheel;

    wheel_kp = kp;
    wheel_ki = ki;
    wheel_kd = kd;

    /* 게인이 바뀌면 이전 누적값은 의미가 없으므로 지운다 */
    for (wheel = WHEEL_LEFT; wheel < WHEEL_COUNT; wheel++)
    {
        wheel_state[wheel].integral   = 0.0f;
        wheel_state[wheel].prev_error = 0.0f;
    }
}

/* 폐루프 제어를 켜거나 끈다. */
void wheel_set_enabled(bool enabled)
{
    wheel_t wheel;

    /* 제어를 다시 켤 때 이전 누적값이 튀어나오지 않게 지운다 */
    if ((wheel_enabled == false) && (enabled == true))
    {
        for (wheel = WHEEL_LEFT; wheel < WHEEL_COUNT; wheel++)
        {
            wheel_state[wheel].integral   = 0.0f;
            wheel_state[wheel].prev_error = 0.0f;
        }
    }

    wheel_enabled = enabled;
}

/* 폐루프 제어가 켜져 있는지 반환한다. */
bool wheel_is_enabled(void)
{
    return wheel_enabled;
}

/* 선택한 바퀴가 목표 RPM에 도달했는지 반환한다. */
bool wheel_is_reached(wheel_t wheel)
{
    float error;

    if (wheel >= WHEEL_COUNT)
    {
        return false;
    }

    error = wheel_state[wheel].error;
    if (error < 0.0f)
    {
        error = -error;
    }

    return (error <= WHEEL_RPM_TOLERANCE);
}

/* 좌우 바퀴를 모두 정지시키고 PID 누적 상태를 초기화한다. */
void wheel_stop(void)
{
    wheel_t wheel;

    for (wheel = WHEEL_LEFT; wheel < WHEEL_COUNT; wheel++)
    {
        wheel_state[wheel].target_rpm = 0.0f;
        wheel_state[wheel].error      = 0.0f;
        wheel_state[wheel].integral   = 0.0f;
        wheel_state[wheel].prev_error = 0.0f;
        wheel_state[wheel].output     = 0;
    }

    motor_stop_all();
}
