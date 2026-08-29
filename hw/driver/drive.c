#include "main.h"
#include "drive.h"
#include "heading_control.h"
#include "wheel.h"
#include "motor.h"
#include "encoder.h"
#include "rover_config.h"

/* ------------------------------------------------------------------
 * drive.c - 차량 단위 주행 명령
 *
 *   drive -> heading -> wheel -> motor / encoder
 *
 * 이 모듈은 heading 에만 명령을 내린다.
 * 회전도 별도 로직이 아니라 heading 의 기준 방향을 바꾸는 것으로 처리한다.
 * ------------------------------------------------------------------ */

/* 실제로 바퀴가 돌기 시작하는 최소 PWM [%] */
#define DRIVE_DUTY_MIN          ROVER_MOTOR_MIN_OUTPUT

/* 회전 명령 한 번에 돌릴 각도 [도] */
#define DRIVE_TURN_STEP_DEG     90.0f

/* wheel PID 주기 [ms] : 엔코더 분해능 때문에 heading 보다 길어야 한다 */
#define DRIVE_WHEEL_PERIOD_MS   100

static uint32_t drive_wheel_accum = 0;

/* false인 동안에는 제어기 상태와 관계없이 모터 출력을 강제로 0으로 유지한다. */
static bool drive_active = false;
static bool drive_initialized = false;
static bool drive_direct_mode = false;

/* 현재 설정된 기본 주행 속도 [PWM %] */
static uint8_t drive_speed = DRIVE_SPEED_NORMAL;


/* 0~100 의 듀티 값을 목표 RPM 으로 바꿔주는 내부 함수
 * 최소 구동 듀티와 최대 듀티 사이를 직선으로 잇는다. */
float drive_speed_to_rpm(uint8_t duty)
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

    return DRIVE_RPM_MIN
         + (DRIVE_RPM_MAX - DRIVE_RPM_MIN) * ratio;
}

/* 일시적인 전진 목표 RPM을 적용한다.
 * 사용자가 선택한 slow/normal/fast 단계는 바꾸지 않으므로
 * 방지턱 통과 후 drive_set_speed()로 원래 속도를 복구할 수 있다. */
void drive_set_forward_target_rpm(float target_rpm)
{
    if (drive_is_ready() == false)
    {
        drive_stop();
        return;
    }

    if (target_rpm < 0.0f)
    {
        target_rpm = 0.0f;
    }
    if (target_rpm > DRIVE_RPM_MAX)
    {
        target_rpm = DRIVE_RPM_MAX;
    }

    if (target_rpm == 0.0f)
    {
        drive_stop();
        return;
    }

    drive_direct_mode = false;
    drive_active = true;
    heading_set_enabled(true);
    heading_set_base_rpm(target_rpm);
}

/* 좌우 출력을 직접 정하는 회피 제어도 최종 하드웨어 접근은 drive가 담당한다. */
void drive_set_direct_output(int16_t left_output, int16_t right_output)
{
    if (drive_is_ready() == false)
    {
        drive_stop();
        return;
    }

    if (drive_direct_mode == false)
    {
        /* 폐루프 목표와 PID 누적값이 직접 출력에 섞이지 않게 한 번만 정리한다. */
        heading_set_enabled(false);
        heading_stop();
        drive_wheel_accum = 0;
    }

    drive_direct_mode = true;
    drive_active = true;

    motor_set_output(MOTOR_LEFT, left_output);
    motor_set_output(MOTOR_RIGHT, right_output);
}

/* 원래 진행 방향으로 복귀하기 위한 자이로 제자리 회전 명령 */
void drive_recover_heading(float target_heading_deg)
{
    if (drive_is_ready() == false)
    {
        drive_stop();
        return;
    }

    drive_direct_mode = false;
    drive_active = true;
    heading_set_enabled(true);
    heading_set_base_rpm(0.0f);
    heading_set_target(target_heading_deg);
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

/* 엔코더 누적 카운트를 이동 거리 [mm] 로 바꿔주는 내부 함수 */
static float drive_count_to_mm(int32_t count)
{
    if (count < 0)
    {
        count = -count;
    }

    return ((float)count / (float)ROVER_ENCODER_SLOTS_PER_REV)
         * ROVER_PI
         * ROVER_WHEEL_DIAMETER_MM;
}


/* 주행 모듈 초기화 */
bool drive_init(void)
{
    bool wheel_initialized = wheel_init();
    bool heading_initialized = heading_init();

    drive_initialized = wheel_initialized && heading_initialized;

    drive_wheel_accum = 0;
    drive_active = false;
    drive_direct_mode = false;
    drive_speed = DRIVE_SPEED_NORMAL;

    drive_stop();

    return drive_is_ready();
}

/* 차량을 세운 상태에서 MPU6050 초기화와 영점 보정을 다시 수행한다. */
bool drive_retry_heading_init(void)
{
    bool heading_initialized;

    drive_active = false;
    drive_direct_mode = false;
    drive_wheel_accum = 0;
    wheel_stop();

    heading_initialized = heading_init();
    drive_initialized = motor_is_ready()
                     && encoder_is_ready(ENCODER_LEFT)
                     && encoder_is_ready(ENCODER_RIGHT)
                     && heading_initialized;
    drive_stop();

    return drive_is_ready();
}

/* 모터·엔코더 피드백 오류 복구.
 * MPU6050은 건드리지 않고 PWM과 TIM5 입력 캡처만 다시 시작한다. */
bool drive_retry_motion_hardware(void)
{
    bool motor_initialized;
    bool encoder_initialized;

    drive_stop();
    drive_active = false;
    drive_direct_mode = false;
    drive_wheel_accum = 0;

    motor_initialized = motor_restart();
    encoder_initialized = encoder_restart();
    wheel_reset();

    drive_initialized = motor_initialized
                     && encoder_initialized
                     && heading_is_sensor_ready();

    return motor_initialized && encoder_initialized;
}

/* 초기화 성공과 가장 최근 MPU6050 갱신 상태를 함께 확인한다. */
bool drive_is_ready(void)
{
    return drive_initialized && heading_is_sensor_ready();
}

bool drive_is_motor_ready(void)
{
    return motor_is_ready();
}

bool drive_is_left_encoder_ready(void)
{
    return encoder_is_ready(ENCODER_LEFT);
}

bool drive_is_right_encoder_ready(void)
{
    return encoder_is_ready(ENCODER_RIGHT);
}

bool drive_is_heading_ready(void)
{
    return heading_is_sensor_ready();
}

/* 명시적인 새 출발에서만 이전 모터 오류를 해제한다. */
void drive_prepare_start(void)
{
    drive_active = false;
    drive_wheel_accum = 0;
    wheel_reset();
    heading_stop();
}

/* 제어 주기마다 호출한다.
 * 자이로는 매번, 바퀴 속도 제어는 누적 시간이 찼을 때만 갱신한다. */
void drive_update(uint32_t elapsed_time_ms)
{
    if (drive_direct_mode == true)
    {
        /* 회피 중에는 자세만 읽고 heading/wheel이 직접 출력을 덮지 않는다. */
        (void)heading_update_measurement(elapsed_time_ms);
    }
    else
    {
        heading_update(elapsed_time_ms);
    }

    if (drive_is_ready() == false)
    {
        wheel_stop();
        drive_wheel_accum = 0;
        return;
    }

    if (drive_active == false)
    {
        wheel_stop();
        drive_wheel_accum = 0;
        return;
    }

    drive_wheel_accum += elapsed_time_ms;

    if (drive_wheel_accum >= DRIVE_WHEEL_PERIOD_MS)
    {
        if (drive_direct_mode == true)
        {
            /* 직접 출력 중에는 PID를 실행하지 않고 로그·정지 감시용 RPM만 갱신한다. */
            encoder_update(drive_wheel_accum);
        }
        else
        {
            wheel_update(drive_wheel_accum);
        }
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
        heading_set_base_rpm(drive_speed_to_rpm(speed));
    }
    else
    {
        heading_set_base_rpm(-drive_speed_to_rpm(speed));
    }
}

/* 설정된 속도로 차량을 전진
 * 기준 방향은 그대로 두므로 주행 중 방향이 틀어져도 스스로 복귀한다. */
void drive_forward(uint8_t speed)
{
    if (drive_is_ready() == false)
    {
        drive_stop();
        return;
    }

    drive_speed = speed;
    drive_direct_mode = false;
    drive_active = true;

    heading_set_enabled(true);
    heading_set_base_rpm(drive_speed_to_rpm(speed));
}

/* 설정된 속도로 차량을 후진 */
void drive_backward(uint8_t speed)
{
    if (drive_is_ready() == false)
    {
        drive_stop();
        return;
    }

    drive_speed = speed;
    drive_direct_mode = false;
    drive_active = true;

    heading_set_enabled(true);
    heading_set_base_rpm(-drive_speed_to_rpm(speed));
}

/* 제자리에서 지정한 각도만큼 회전한다. (좌회전 +, 우회전 -)
 * 기본 속도를 0 으로 두면 방향 보정량만 남아 좌우가 반대로 돌게 된다. */
void drive_rotate(float delta_deg)
{
    if (drive_is_ready() == false)
    {
        drive_stop();
        return;
    }

    drive_direct_mode = false;
    drive_active = true;

    heading_set_enabled(true);
    heading_set_base_rpm(0.0f);
    heading_rotate(delta_deg);
}

/* 제자리에서 좌회전한다. */
void drive_turn_left(uint8_t speed)
{
    drive_speed = speed;

    drive_rotate(DRIVE_TURN_STEP_DEG);
}

/* 제자리에서 우회전한다. */
void drive_turn_right(uint8_t speed)
{
    drive_speed = speed;

    drive_rotate(-DRIVE_TURN_STEP_DEG);
}

/* 좌우 바퀴를 모두 멈춘다. 기준 방향은 유지한다. */
void drive_stop(void)
{
    drive_active = false;
    drive_direct_mode = false;
    heading_set_enabled(false);
    heading_stop();
}

/* 현재 방향이 목표 방향에 도달했는지 반환 */
bool drive_is_aligned(void)
{
    return heading_is_aligned();
}

/* 현재 방향 기준을 지금 향한 방향으로 다시 잡는다. */
void drive_reset_heading(void)
{
    heading_reset();
}

/* 누적 주행 거리 측정을 0으로 초기화 */
void drive_reset_distance(void)
{
    encoder_reset();
}

/* 좌우 평균 누적 주행 거리를 mm 단위로 반환 */
float drive_get_distance_mm(void)
{
    float left  = drive_count_to_mm(encoder_get_count(ENCODER_LEFT));
    float right = drive_count_to_mm(encoder_get_count(ENCODER_RIGHT));

    return (left + right) / 2.0f;
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

/* 좌우 직접 출력 모드인지 반환한다. */
bool drive_is_direct_mode(void)
{
    return drive_direct_mode;
}
