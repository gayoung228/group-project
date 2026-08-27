#include "main.h"
#include "drive.h"
#include "heading_control.h"
#include "wheel.h"
#include "motor.h"

/* ------------------------------------------------------------------
 * drive.c - 차량 단위 주행 명령
 *
 *   drive -> heading -> wheel -> motor / encoder
 *
 * 이 모듈은 heading 에만 명령을 내린다.
 * 회전도 별도 로직이 아니라 heading 의 기준 방향을 바꾸는 것으로 처리한다.
 * ------------------------------------------------------------------ */

/* 실제로 바퀴가 돌기 시작하는 최소 PWM [%] */
#define DRIVE_DUTY_MIN          80

/* 듀티를 RPM 으로 바꾸기 위한 실측 기준값
 * ★ 개루프 주행으로 측정한 값에 맞춰 반드시 수정할 것 ★ */
#define DRIVE_RPM_AT_MIN_DUTY   150.0f   /* 80% 에서 나온 RPM */
#define DRIVE_RPM_AT_MAX_DUTY   270.0f   /* 100% 에서 나온 RPM */

/* 회전 명령 한 번에 돌릴 각도 [도] */
#define DRIVE_TURN_STEP_DEG     90.0f

/* wheel PID 주기 [ms] : 엔코더 분해능 때문에 heading 보다 길어야 한다 */
#define DRIVE_WHEEL_PERIOD_MS   100

static uint32_t drive_wheel_accum = 0;

/* 현재 설정된 기본 주행 속도 [PWM %] */
static uint8_t drive_speed = DRIVE_SPEED_NORMAL;


/* 0~100 의 듀티 값을 목표 RPM 으로 바꿔주는 내부 함수
 * 최소 구동 듀티와 최대 듀티 사이를 직선으로 잇는다. */
static float drive_duty_to_rpm(uint8_t duty)
{
    float ratio;

    if (duty == 0)
    {
        return 0.0f;
    }

    /* 최소 구동 듀티보다 낮으면 어차피 못 도므로 끌어올린다 */
    if (duty < DRIVE_DUTY_MIN)
    {
        duty = DRIVE_DUTY_MIN;
    }
    if (duty > 100)
    {
        duty = 100;
    }

    ratio = (float)(duty - DRIVE_DUTY_MIN) / (float)(100 - DRIVE_DUTY_MIN);

    return DRIVE_RPM_AT_MIN_DUTY
         + (DRIVE_RPM_AT_MAX_DUTY - DRIVE_RPM_AT_MIN_DUTY) * ratio;
}

/* 모터의 방향과 속도를 부호 있는 값 하나로 합쳐서 돌려주는 내부 함수 */
static int16_t drive_read_motor_speed(motor_t motor)
{
    motor_direction_t direction = motor_get_direction(motor);
    uint8_t           speed     = motor_get_speed(motor);

    if (direction == MOTOR_FORWARD)
    {
        return (int16_t)speed;
    }
    if (direction == MOTOR_REVERSE)
    {
        return -(int16_t)speed;
    }
    return 0;
}


/* 주행 모듈 초기화 */
void drive_init(void)
{
    wheel_init();
    heading_init();

    drive_speed = DRIVE_SPEED_NORMAL;

    drive_stop();
}

/* 제어 주기마다 호출한다.
 * 자이로는 매번, 바퀴 속도 제어는 누적 시간이 찼을 때만 갱신한다. */
void drive_update(uint32_t elapsed_time_ms)
{
    heading_update(elapsed_time_ms);

    drive_wheel_accum += elapsed_time_ms;

    if (drive_wheel_accum >= DRIVE_WHEEL_PERIOD_MS)
    {
        wheel_update(drive_wheel_accum);
        drive_wheel_accum = 0;
    }
}

/* 양쪽 바퀴에 적용할 기본 속도를 0~100 범위로 설정
 * 주행 중이라면 진행 방향은 그대로 두고 속도만 즉시 바꾼다. */
void drive_set_speed(uint8_t speed)
{
    float base_rpm;

    if (speed > 100)
    {
        speed = 100;
    }

    drive_speed = speed;

    base_rpm = heading_get_base_rpm();

    /* 정지 중이거나 제자리 회전 중이면 속도만 기억하고 끝낸다 */
    if (base_rpm == 0.0f)
    {
        return;
    }

    if (base_rpm > 0.0f)
    {
        heading_set_base_rpm(drive_duty_to_rpm(speed));
    }
    else
    {
        heading_set_base_rpm(-drive_duty_to_rpm(speed));
    }
}

/* 설정된 속도로 차량을 전진
 * 기준 방향은 그대로 두므로 주행 중 방향이 틀어져도 스스로 복귀한다. */
void drive_forward(uint8_t speed)
{
    drive_speed = speed;

    heading_set_enabled(true);
    heading_set_base_rpm(drive_duty_to_rpm(speed));
}

/* 설정된 속도로 차량을 후진 */
void drive_backward(uint8_t speed)
{
    drive_speed = speed;

    heading_set_enabled(true);
    heading_set_base_rpm(-drive_duty_to_rpm(speed));
}

/* 제자리에서 좌회전한다.
 * 기본 속도를 0 으로 두면 방향 보정량만 남아 좌우가 반대로 돌게 된다. */
void drive_turn_left(uint8_t speed)
{
    drive_speed = speed;

    heading_set_enabled(true);
    heading_set_base_rpm(0.0f);
    heading_rotate(DRIVE_TURN_STEP_DEG);
}

/* 제자리에서 우회전한다. */
void drive_turn_right(uint8_t speed)
{
    drive_speed = speed;

    heading_set_enabled(true);
    heading_set_base_rpm(0.0f);
    heading_rotate(-DRIVE_TURN_STEP_DEG);
}

/* 좌우 바퀴를 모두 멈춘다. 기준 방향은 유지한다. */
void drive_stop(void)
{
    heading_stop();
}

/* 현재 설정된 기본 주행 속도를 반환 */
uint8_t drive_get_speed(void)
{
    return drive_speed;
}

/* 현재 왼쪽 바퀴에 적용된 속도를 반환 (전진 +, 후진 -, 정지 0) */
int16_t drive_get_left_speed(void)
{
    return drive_read_motor_speed(MOTOR_LEFT);
}

/* 현재 오른쪽 바퀴에 적용된 속도를 반환 (전진 +, 후진 -, 정지 0) */
int16_t drive_get_right_speed(void)
{
    return drive_read_motor_speed(MOTOR_RIGHT);
}