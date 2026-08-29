#include "main.h"
#include "heading_control.h"
#include "wheel.h"
#include "mpu6050.h"
#include "rover_config.h"

/* ------------------------------------------------------------------
 * heading_control.c - 자이로 기반 방향 유지 제어
 *
 * 계층 구조상 이 모듈은 wheel 에만 명령을 내리고,
 * motor 나 encoder 를 직접 건드리지 않는다.
 * ------------------------------------------------------------------ */

/* 좌우에 실을 수 있는 보정량의 한계 [RPM]
 * 너무 크면 직진 중에 차가 좌우로 요동친다. */
#define HEADING_CORRECTION_MAX      25.0f

/* 모터 실측 제어 범위를 넘는 좌우 목표가 만들어지지 않게 한다. */
#define HEADING_WHEEL_RPM_MAX      ROVER_DRIVE_RPM_MAX

/* 보정 힘은 높이되 한 번에 튀지 않도록 20ms마다 최대 8RPM씩 바꾼다. */
#define HEADING_CORRECTION_STEP      8.0f

/* 제자리 회전이 센서/부호 문제로 멈추지 않는 경우의 안전 제한 [ms] */
#define HEADING_ROTATION_TIMEOUT_MS  5000U

/* 적분항이 쌓일 수 있는 한계 (적분 포화 방지) */
#define HEADING_INTEGRAL_LIMIT      200.0f

/* 방향이 맞았다고 판단할 오차 범위 [도] */
#define HEADING_ALIGN_TOLERANCE     3.0f

/* 이 각도보다 오차가 작으면 적분을 멈춘다.
 * 미세한 오차까지 적분하면 차가 계속 흔들린다. */
#define HEADING_DEAD_ZONE_DEG       1.5f

/* PID 게인 초기값 (실측 후 튜닝할 것)
 * 오차 1도당 몇 RPM 을 보정할지가 Kp 의 의미이다. */
#define HEADING_DEFAULT_KP          4.0f
#define HEADING_DEFAULT_KI          0.0f
#define HEADING_DEFAULT_KD          0.0f


/* 기준 방향 [도] */
static float heading_target_deg = 0.0f;

/* 자이로가 측정한 현재 방향 [도] */
static float heading_current_deg = 0.0f;
static float heading_roll_deg = 0.0f;
static float heading_pitch_deg = 0.0f;

/* 기준 방향과 현재 방향의 차이 [도] */
static float heading_error_deg = 0.0f;

/* PID 가 계산한 좌우 보정량 [RPM] */
static float heading_correction = 0.0f;

/* 직진 기본 속도 [RPM] */
static float heading_base_rpm = 0.0f;

/* base RPM이 0일 때 '정지'와 '제자리 회전'을 구분한다. */
static bool heading_rotation_active = false;
static float heading_rotation_direction = 0.0f;
static uint32_t heading_rotation_start_tick = 0;

/* PID 내부 상태 */
static float heading_integral = 0.0f;
static float heading_prev_error = 0.0f;

/* PID 게인 */
static float heading_kp = HEADING_DEFAULT_KP;
static float heading_ki = HEADING_DEFAULT_KI;
static float heading_kd = HEADING_DEFAULT_KD;

/* 방향 제어 사용 여부 */
static bool heading_enabled = false;
static bool heading_sensor_ready = false;


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

/* 각도를 0도 이상 360도 미만으로 바꾼다.
 * 예: 365도 -> 5도, -10도 -> 350도 */
static float heading_normalize_360(float angle_deg)
{
    while (angle_deg >= 360.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

/* 두 각도의 차이를 가장 짧은 회전 방향(-180~180도)으로 바꾼다. */
static float heading_normalize_error(float error_deg)
{
    while (error_deg > 180.0f)
    {
        error_deg -= 360.0f;
    }
    while (error_deg < -180.0f)
    {
        error_deg += 360.0f;
    }
    return error_deg;
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
    float base_magnitude;

    /* 정지와 제자리 회전은 둘 다 base RPM이 0이다.
     * 회전 명령이 없다면 Yaw 오차가 있어도 모터를 절대 구동하지 않는다. */
    if ((heading_base_rpm == 0.0f) && (heading_rotation_active == false))
    {
        heading_correction = 0.0f;
        wheel_stop();
        return;
    }

    /* 제자리 회전이 목표 각도에 도달하면 즉시 정지한다. */
    if ((heading_rotation_active == true) && (heading_is_aligned() == true))
    {
        heading_rotation_active = false;
        heading_correction = 0.0f;
        wheel_stop();
        return;
    }

    if (heading_base_rpm > 0.0f)
    {
        /* 직진 보정은 한쪽을 감속하지 않고 반대쪽을 가속한다.
         * 따라서 마찰 때문에 멈추는 90RPM 아래로 어느 바퀴도 내려가지 않는다. */
        base_magnitude = heading_clamp(heading_base_rpm,
                                       ROVER_DRIVE_RPM_MIN,
                                       HEADING_WHEEL_RPM_MAX);
        left_rpm = base_magnitude;
        right_rpm = base_magnitude;

        if (heading_correction > 0.0f)
        {
            right_rpm += 2.0f * heading_correction;
        }
        else
        {
            left_rpm += -2.0f * heading_correction;
        }
    }
    else if (heading_base_rpm < 0.0f)
    {
        /* 후진도 같은 원칙으로 한쪽의 절댓값만 높인다. */
        base_magnitude = heading_clamp(-heading_base_rpm,
                                       ROVER_DRIVE_RPM_MIN,
                                       HEADING_WHEEL_RPM_MAX);
        left_rpm = -base_magnitude;
        right_rpm = -base_magnitude;

        if (heading_correction > 0.0f)
        {
            left_rpm -= 2.0f * heading_correction;
        }
        else
        {
            right_rpm += 2.0f * heading_correction;
        }
    }
    else
    {
        /* 제자리 회전은 좌우가 반대로 돌아야 하므로 기존 방식을 유지한다. */
        left_rpm  = -heading_correction;
        right_rpm = heading_correction;
    }

    left_rpm = heading_clamp(left_rpm,
                             -HEADING_WHEEL_RPM_MAX,
                              HEADING_WHEEL_RPM_MAX);
    right_rpm = heading_clamp(right_rpm,
                              -HEADING_WHEEL_RPM_MAX,
                               HEADING_WHEEL_RPM_MAX);

    wheel_set_target_rpm_both(left_rpm, right_rpm);
}

/* MPU6050을 읽고 현재 X/Y/Z 및 목표와의 Yaw 오차만 갱신한다. */
bool heading_update_measurement(uint32_t elapsed_time_ms)
{
    float roll_deg;
    float pitch_deg;
    float yaw_deg;

    if (elapsed_time_ms == 0U)
    {
        return heading_sensor_ready;
    }

    if ((mpu6050_update() == false)
        || (mpu6050_orientation_update(elapsed_time_ms) == false)
        || (mpu6050_get_orientation(&roll_deg, &pitch_deg, &yaw_deg) == false))
    {
        heading_sensor_ready = false;
        return false;
    }

    heading_sensor_ready = true;
    heading_roll_deg    = heading_normalize_error(roll_deg);
    heading_pitch_deg   = heading_normalize_error(pitch_deg);
    heading_current_deg = heading_normalize_360(yaw_deg);
    heading_target_deg  = heading_normalize_360(heading_target_deg);
    heading_error_deg   = heading_normalize_error(heading_target_deg
                                                - heading_current_deg);

    return true;
}


/* MPU6050 을 시작하고 방향 제어 상태를 준비한다. */
bool heading_init(void)
{
    heading_kp      = HEADING_DEFAULT_KP;
    heading_ki      = HEADING_DEFAULT_KI;
    heading_kd      = HEADING_DEFAULT_KD;
    heading_enabled = false;
    heading_sensor_ready = false;

    heading_target_deg  = 0.0f;
    heading_current_deg = 0.0f;
    heading_roll_deg    = 0.0f;
    heading_pitch_deg   = 0.0f;
    heading_error_deg   = 0.0f;
    heading_base_rpm    = 0.0f;
    heading_rotation_active = false;
    heading_rotation_direction = 0.0f;
    heading_rotation_start_tick = 0;

    heading_clear_pid();

    /* 초기화 + 워밍업 + 자이로 영점 보정을 한 번에 수행한다.
     * 이 동안 차체를 움직이면 영점이 틀어지므로 반드시 정지해 두어야 한다. */
    heading_sensor_ready = mpu6050_start();
    return heading_sensor_ready;
}

/* 현재 향하고 있는 방향을 기준 방향(0도)으로 다시 잡는다. */
void heading_reset(void)
{
    mpu6050_orientation_reset();

    heading_target_deg  = 0.0f;
    heading_current_deg = 0.0f;
    heading_roll_deg    = 0.0f;
    heading_pitch_deg   = 0.0f;
    heading_error_deg   = 0.0f;
    heading_rotation_active = false;
    heading_rotation_direction = 0.0f;
    heading_rotation_start_tick = 0;
    heading_clear_pid();
}

/* 제어 주기마다 호출한다. 자이로를 읽고 좌우 목표 RPM 을 갱신한다. */
void heading_update(uint32_t elapsed_time_ms)
{
    float dt_s;
    float derivative;
    float desired_correction;
    float correction_delta;

    if (elapsed_time_ms == 0)
    {
        return;
    }

    /* 센서를 읽고 현재 자세 및 Yaw 오차를 먼저 갱신한다. */
    if (heading_update_measurement(elapsed_time_ms) == false)
    {
        wheel_stop();
        return;
    }

    /* 회전이 목표를 지나쳤거나 제한 시간을 넘기면 무조건 정지한다. */
    if (heading_rotation_active == true)
    {
        bool target_passed = ((heading_rotation_direction > 0.0f) &&
                              (heading_error_deg <= 0.0f)) ||
                             ((heading_rotation_direction < 0.0f) &&
                              (heading_error_deg >= 0.0f));
        bool timed_out = ((HAL_GetTick() - heading_rotation_start_tick) >=
                          HEADING_ROTATION_TIMEOUT_MS);

        if (target_passed || timed_out)
        {
            heading_rotation_active = false;
            heading_rotation_direction = 0.0f;
            heading_clear_pid();
            wheel_stop();
            return;
        }
    }

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

    desired_correction = (heading_kp * heading_error_deg)
                       + (heading_ki * heading_integral)
                       + (heading_kd * derivative);

    desired_correction = heading_clamp(desired_correction,
                                       -HEADING_CORRECTION_MAX,
                                        HEADING_CORRECTION_MAX);

    /* 목표 보정량으로 한 번에 튀지 않고 조금씩 가까워진다. */
    correction_delta = desired_correction - heading_correction;
    correction_delta = heading_clamp(correction_delta,
                                     -HEADING_CORRECTION_STEP,
                                      HEADING_CORRECTION_STEP);
    heading_correction += correction_delta;

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

    /* 전진/후진 명령은 제자리 회전 상태가 아니다. */
    if (base_rpm != 0.0f)
    {
        heading_rotation_active = false;
    }
}

/* 설정된 직진 기본 속도를 반환한다. */
float heading_get_base_rpm(void)
{
    return heading_base_rpm;
}

/* 기준 방향을 현재 기준에서 상대 각도만큼 돌린다. */
void heading_rotate(float delta_deg)
{
    heading_target_deg = heading_normalize_360(heading_target_deg + delta_deg);
    heading_rotation_active = true;
    heading_rotation_direction = (delta_deg >= 0.0f) ? 1.0f : -1.0f;
    heading_rotation_start_tick = HAL_GetTick();

    /* 목표가 크게 바뀌면 이전 누적값은 의미가 없다 */
    heading_integral   = 0.0f;
    heading_prev_error = 0.0f;
}

/* 기준 방향을 절대 각도로 설정한다. */
void heading_set_target(float target_deg)
{
    float delta_deg;

    heading_target_deg = heading_normalize_360(target_deg);
    heading_rotation_active = (heading_base_rpm == 0.0f);

    delta_deg = heading_normalize_error(heading_target_deg - heading_current_deg);
    heading_rotation_direction = (delta_deg >= 0.0f) ? 1.0f : -1.0f;
    heading_rotation_start_tick = HAL_GetTick();

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

/* 자세 표시 좌표계를 반환한다.
 * 내부 회전 제어는 좌회전을 +로 쓰지만, 사용자 로그의 Z축은 요청에 맞춰
 * 오른쪽을 +로 표시하므로 Yaw 부호만 반대로 바꾼다. */
bool heading_get_pose(float *x_deg, float *y_deg, float *z_deg)
{
    if ((heading_sensor_ready == false)
        || (x_deg == NULL)
        || (y_deg == NULL)
        || (z_deg == NULL))
    {
        return false;
    }

    *x_deg = heading_roll_deg;
    *y_deg = heading_pitch_deg;
    *z_deg = heading_normalize_error(-heading_current_deg);

    return true;
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

/* MPU6050 초기화와 가장 최근 자세 갱신이 정상인지 반환한다. */
bool heading_is_sensor_ready(void)
{
    return heading_sensor_ready;
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
    heading_rotation_active = false;
    heading_rotation_direction = 0.0f;
    heading_rotation_start_tick = 0;

    heading_clear_pid();

    wheel_stop();
}
