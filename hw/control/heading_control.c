#include "main.h"
#include "heading_control.h"
#include "wheel.h"
#include "mpu6050.h"

/* ------------------------------------------------------------------
 * heading_control.c - 자이로 기반 방향 유지 제어
 *
 * 계층 구조상 이 모듈은 wheel 에만 명령을 내리고,
 * motor 나 encoder 를 직접 건드리지 않는다.
 * ------------------------------------------------------------------ */

/* 좌우에 실을 수 있는 보정량의 한계 [RPM]
 * 너무 크면 직진 중에 차가 좌우로 요동친다. */
#define HEADING_CORRECTION_MAX      80.0f

/* 적분항이 쌓일 수 있는 한계 (적분 포화 방지) */
#define HEADING_INTEGRAL_LIMIT      200.0f

/* 방향이 맞았다고 판단할 오차 범위 [도] */
#define HEADING_ALIGN_TOLERANCE     3.0f

/* 이 각도보다 오차가 작으면 적분을 멈춘다.
 * 미세한 오차까지 적분하면 차가 계속 흔들린다. */
#define HEADING_DEAD_ZONE_DEG       1.0f

/* PID 게인 초기값 (실측 후 튜닝할 것)
 * 오차 1도당 몇 RPM 을 보정할지가 Kp 의 의미이다. */
#define HEADING_DEFAULT_KP          3.0f
#define HEADING_DEFAULT_KI          0.5f
#define HEADING_DEFAULT_KD          0.0f    // 0.1f?


/* 기준 방향 [도] */
static float heading_target_deg = 0.0f;

/* 자이로가 측정한 현재 방향 [도] */
static float heading_current_deg = 0.0f;

/* 기준 방향과 현재 방향의 차이 [도] */
static float heading_error_deg = 0.0f;

/* PID 가 계산한 좌우 보정량 [RPM] */
static float heading_correction = 0.0f;

/* 직진 기본 속도 [RPM] */
static float heading_base_rpm = 0.0f;

/* PID 내부 상태 */
static float heading_integral = 0.0f;
static float heading_prev_error = 0.0f;

/* PID 게인 */
static float heading_kp = HEADING_DEFAULT_KP;
static float heading_ki = HEADING_DEFAULT_KI;
static float heading_kd = HEADING_DEFAULT_KD;

/* 방향 제어 사용 여부 */
static bool heading_enabled = true;


/* 실수값을 지정한 범위 안으로 잘라주는 내부 함수 */
static float heading_clamp(float value, float min, float max)
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

/* PID 누적 상태를 지우는 내부 함수 */
static void heading_clear_pid(void)
{
    heading_integral   = 0.0f;
    heading_prev_error = 0.0f;
    heading_correction = 0.0f;
}

/* 계산된 기본 속도와 보정량을 좌우 목표 RPM 으로 내려보내는 내부 함수 */
static void heading_apply(void)
{
    float left_rpm;
    float right_rpm;

    /* 정지 상태이고 방향도 맞았다면 아무 명령도 내리지 않는다.
     * 미세한 보정량 때문에 모터가 계속 떨리는 것을 막는다 */
    if ((heading_base_rpm == 0.0f) && (heading_is_aligned() == true))
    {
        heading_correction = 0.0f;
        wheel_stop();
        return;
    }

    left_rpm  = heading_base_rpm + heading_correction;
    right_rpm = heading_base_rpm - heading_correction;

    wheel_set_target_rpm_both(left_rpm, right_rpm);
}


/* MPU6050 을 시작하고 방향 제어 상태를 준비한다. */
bool heading_init(void)
{
    heading_kp      = HEADING_DEFAULT_KP;
    heading_ki      = HEADING_DEFAULT_KI;
    heading_kd      = HEADING_DEFAULT_KD;
    heading_enabled = true;

    heading_target_deg  = 0.0f;
    heading_current_deg = 0.0f;
    heading_error_deg   = 0.0f;
    heading_base_rpm    = 0.0f;

    heading_clear_pid();

    /* 초기화 + 워밍업 + 자이로 영점 보정을 한 번에 수행한다.
     * 이 동안 차체를 움직이면 영점이 틀어지므로 반드시 정지해 두어야 한다. */
    return mpu6050_start();
}

/* 현재 향하고 있는 방향을 기준 방향(0도)으로 다시 잡는다. */
void heading_reset(void)
{
    mpu6050_orientation_reset();

    heading_target_deg  = 0.0f;
    heading_current_deg = 0.0f;
    heading_error_deg   = 0.0f;

    heading_clear_pid();
}

/* 제어 주기마다 호출한다. 자이로를 읽고 좌우 목표 RPM 을 갱신한다. */
void heading_update(uint32_t elapsed_time_ms)
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;
    float dt_s;
    float derivative;
    float output;

    if (elapsed_time_ms == 0)
    {
        return;
    }

    /* 센서를 읽고 각속도를 적분해 현재 자세를 갱신한다 */
    if (mpu6050_update() == false)
    {
        return;
    }
    if (mpu6050_orientation_update() == false)
    {
        return;
    }
    if (mpu6050_get_orientation(&roll_deg, &pitch_deg, &yaw_deg) == false)
    {
        return;
    }

    heading_current_deg = yaw_deg;
    heading_error_deg   = heading_target_deg - heading_current_deg;

    /* 제어를 꺼 둔 상태에서는 측정만 하고 보정은 하지 않는다 */
    if (heading_enabled == false)
    {
        heading_correction = 0.0f;
        heading_apply();
        return;
    }

    dt_s = (float)elapsed_time_ms / 1000.0f;

    /* 오차가 아주 작을 때는 적분을 멈춰 미세한 흔들림을 막는다 */
    if ((heading_error_deg > HEADING_DEAD_ZONE_DEG) ||
        (heading_error_deg < -HEADING_DEAD_ZONE_DEG))
    {
        heading_integral += heading_error_deg * dt_s;
        heading_integral  = heading_clamp(heading_integral,
                                          -HEADING_INTEGRAL_LIMIT,
                                           HEADING_INTEGRAL_LIMIT);
    }

    derivative         = (heading_error_deg - heading_prev_error) / dt_s;
    heading_prev_error = heading_error_deg;

    output = (heading_kp * heading_error_deg)
           + (heading_ki * heading_integral)
           + (heading_kd * derivative);

    heading_correction = heading_clamp(output,
                                       -HEADING_CORRECTION_MAX,
                                        HEADING_CORRECTION_MAX);

    heading_apply();
}

/* 직진 기본 속도를 RPM 으로 설정한다. */
void heading_set_base_rpm(float base_rpm)
{
    /* 진행 방향이 뒤집히면 이전 누적값이 방해가 되므로 지운다 */
    if ((heading_base_rpm * base_rpm) < 0.0f)
    {
        heading_clear_pid();
    }

    heading_base_rpm = base_rpm;
}

/* 설정된 직진 기본 속도를 반환한다. */
float heading_get_base_rpm(void)
{
    return heading_base_rpm;
}

/* 기준 방향을 현재 기준에서 상대 각도만큼 돌린다. */
void heading_rotate(float delta_deg)
{
    heading_target_deg += delta_deg;

    /* 목표가 크게 바뀌면 이전 누적값은 의미가 없다 */
    heading_integral   = 0.0f;
    heading_prev_error = 0.0f;
}

/* 기준 방향을 절대 각도로 설정한다. */
void heading_set_target(float target_deg)
{
    heading_target_deg = target_deg;

    heading_integral   = 0.0f;
    heading_prev_error = 0.0f;
}

/* 현재 기준 방향을 반환한다. */
float heading_get_target(void)
{
    return heading_target_deg;
}

/* 자이로가 측정한 현재 방향을 반환한다. */
float heading_get_current(void)
{
    return heading_current_deg;
}

/* 기준 방향과 현재 방향의 차이를 반환한다. */
float heading_get_error(void)
{
    return heading_error_deg;
}

/* PID 가 계산한 좌우 보정량을 RPM 으로 반환한다. */
float heading_get_correction(void)
{
    return heading_correction;
}

/* 방향 제어에 사용할 PID 게인을 설정한다. */
void heading_set_gain(float kp, float ki, float kd)
{
    heading_kp = kp;
    heading_ki = ki;
    heading_kd = kd;

    heading_clear_pid();
}

/* 방향 제어를 켜거나 끈다. */
void heading_set_enabled(bool enabled)
{
    /* 다시 켤 때 이전 누적값이 튀어나오지 않게 지운다 */
    if ((heading_enabled == false) && (enabled == true))
    {
        heading_clear_pid();
    }

    heading_enabled = enabled;
}

/* 방향 제어가 켜져 있는지 반환한다. */
bool heading_is_enabled(void)
{
    return heading_enabled;
}

/* 현재 방향이 기준 방향에 충분히 가까운지 반환한다. */
bool heading_is_aligned(void)
{
    float error = heading_error_deg;

    if (error < 0.0f)
    {
        error = -error;
    }

    return (error <= HEADING_ALIGN_TOLERANCE);
}

/* 주행을 멈추고 PID 누적 상태를 초기화한다. 기준 방향은 유지한다. */
void heading_stop(void)
{
    heading_base_rpm = 0.0f;

    heading_clear_pid();

    wheel_stop();
}