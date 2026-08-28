#include "main.h"
#include "obstacle_avoidance_test.h"
#include "obstacle_avoidance.h"
#include "drive.h"
#include "encoder.h"
#include "heading_control.h"
#include "ir_remote.h"
#include "motor.h"
#include "speed_bump_control.h"
#include "vl53l0x.h"
#include "wheel.h"
#include <stdio.h>

/* ------------------------------------------------------------------
 * obstacle_avoidance_test.c - 장애물 회피 시험
 *
 * 정상 주행 중에는 drive 가 자이로 방향 제어로 직진을 유지한다.
 * 전방에 장애물이 잡히면 회피 상태 머신에 제어를 넘기고,
 * 그동안에는 상태 머신이 계산한 좌우 속도를 모터에 직접 넣는다.
 * 회피가 끝나면 다시 drive 로 돌아온다.
 *
 * 현재 시험 규칙은 고정되어 있다.
 * 전방 장애물을 확인하면 왼쪽 90도 -> 설정 거리 전진 -> 오른쪽 90도로
 * 원래 방향을 복구한 뒤 자이로 직진 제어로 돌아간다.
 *
 * 터미널 설정 : 115200 8N1
 * ------------------------------------------------------------------ */

#ifdef TEST_OBSTACLE_AVOIDANCE

/* 제어 주기 [ms] */
#define CONTROL_PERIOD_MS       20

/* 센서 측정 주기 [ms] */
#define SENSOR_PERIOD_MS        40

/* 화면 출력 주기 [ms] */
#define PRINT_PERIOD_MS         500

/* 제어 주기가 크게 밀렸을 때 적분에 넣을 최대 시간 [ms] */
#define CONTROL_DT_LIMIT_MS     100

/* 장애물로 판단할 전방 거리 [mm] */
#define TEST_OBSTACLE_MM        250

/* 장애물이 사라졌다고 판단할 거리 [mm]
 * 진입값보다 크게 두어 250mm 근처에서 상태가 흔들리는 것을 막는다. */
#define TEST_OBSTACLE_EXIT_MM   320

/* 장애물/정상 거리로 확정하기 위해 필요한 연속 측정 횟수 */
#define TEST_SENSOR_CONFIRM_COUNT  3U

/* 이 횟수만큼 연속 측정에 실패하면 차량을 안전 정지한다. */
#define TEST_SENSOR_FAIL_COUNT     3U

/* 시작 안내를 띄우기 전에 거리 센서 정상값을 기다리는 최대 시간 [ms] */
#define TEST_SENSOR_STARTUP_TIMEOUT_MS  1500U

/* 거리센서 오류 상태에서 이 시간 안에 S를 세 번 누르면 센서 복구를 시도한다. */
#define TEST_RECOVERY_PRESS_WINDOW_MS   5000U
#define TEST_RECOVERY_PRESS_COUNT       3U
#define TEST_RECOVERY_MAX_ATTEMPTS      3U

/* 회피 방향으로 전진할 거리 [mm] */
#define TEST_BYPASS_MM          500

/* 왼쪽으로 회피한 뒤 오른쪽으로 복귀할 각도 [도] */
#define TEST_TURN_ANGLE_DEG     90.0f

/* 방지턱에서 진행 방향이 크게 틀어졌을 때의 Z축(Yaw) 복구 조건 */
#define BUMP_YAW_TRIGGER_DEG       30.0f
#define BUMP_YAW_DONE_DEG           5.0f
#define BUMP_YAW_PAUSE_MS         300U
#define BUMP_YAW_HOLD_MS          300U
#define BUMP_YAW_TIMEOUT_MS      6000U

/* 실측한 리모컨 NEC 주소와 버튼별 명령값 */
#define TEST_IR_ADDR            0x00U
#define TEST_IR_START_CMD       0xC2U
#define TEST_IR_STOP_CMD        0x90U
#define TEST_IR_RESET_YAW_CMD   0xE2U
#define TEST_IR_DISTANCE_CMD    0xB0U
#define TEST_IR_SLOW_CMD        0x30U
#define TEST_IR_NORMAL_CMD      0x18U
#define TEST_IR_FAST_CMD        0x7AU


extern UART_HandleTypeDef huart2;

/* 주행 중인지 여부 */
static bool test_running = false;

/* 정상 주행 속도 [PWM %] */
static uint8_t test_speed = DRIVE_SPEED_NORMAL;

/* 회피 상태 머신이 제어를 잡고 있는지 여부 */
static bool test_avoiding = false;

/* 거리 센서 필터 상태 */
static uint16_t filtered_front_mm = VL53L0X_MAX_VALID_MM;
static uint8_t  obstacle_count    = 0;
static uint8_t  clear_count       = 0;
static uint8_t  invalid_count     = 0;
static uint8_t  valid_count       = 0;
static bool     obstacle_confirmed = false;
static bool     distance_sensor_fault = true;

/* 사용자가 S 버튼 세 번으로 요청하는 거리센서 수동 복구 상태 */
static uint8_t  blocked_start_count = 0;
static uint32_t blocked_start_first_tick = 0;
static uint8_t  distance_recovery_attempts = 0;
static bool     distance_recovery_in_progress = false;
static uint32_t last_wheel_stall_restart_count = 0;
static uint32_t last_avoid_stall_restart_count = 0;
static uint32_t avoid_encoder_accum_ms = 0;

typedef enum
{
    BUMP_YAW_IDLE = 0,
    BUMP_YAW_PAUSE,
    BUMP_YAW_ROTATE
} bump_yaw_state_t;

/* 방지턱 위에서 틀어진 Z축을 원래 진행 방향으로 복구하는 상태 */
static bump_yaw_state_t bump_yaw_state = BUMP_YAW_IDLE;
static float bump_original_heading_deg = 0.0f;
static uint32_t bump_yaw_state_tick = 0;
static uint32_t bump_yaw_hold_tick = 0;
static bool bump_yaw_holding = false;


/* printf 출력을 UART2(ST-LINK 가상 COM 포트)로 내보낸다. */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
    return ch;
}

/* 최근 정상적으로 측정된 전방 거리값을 반환한다. */
static uint16_t read_front_mm(void)
{
    return filtered_front_mm;
}

/* 장애물 판정과 센서 오류 판정에 쓰는 연속 측정 횟수를 초기화한다. */
static void reset_distance_filter(void)
{
    filtered_front_mm    = VL53L0X_MAX_VALID_MM;
    obstacle_count       = 0;
    clear_count          = 0;
    invalid_count        = 0;
    valid_count          = 0;
    obstacle_confirmed   = false;
    distance_sensor_fault = true;
}

/* 실제 I2C 실패 또는 0mm 결과가 발생했을 때만 오류 횟수를 올린다. */
static void record_distance_sensor_failure(void)
{
    valid_count    = 0;
    obstacle_count = 0;
    clear_count    = 0;

    if (invalid_count < TEST_SENSOR_FAIL_COUNT)
    {
        invalid_count++;
    }
    if (invalid_count >= TEST_SENSOR_FAIL_COUNT)
    {
        distance_sensor_fault = true;
    }
}

/* 전방 센서를 한 번 읽고 연속 감지·히스테리시스·오류 상태를 갱신한다. */
static void update_front_sensor(void)
{
    bool step_ok = vl53l0x_update_sensor(VL53L0X_FRONT);
    bool measurement_error;
    bool measurement_new;
    uint16_t range_mm;

    /* 센서가 초기화되지 않았다면 상태머신 자체를 진행할 수 없다. */
    if ((vl53l0x_is_ready(VL53L0X_FRONT) == false)
        || (step_ok == false))
    {
        record_distance_sensor_failure();
        return;
    }

    /* non-blocking 드라이버는 측정 중간 단계에서도 즉시 반환한다.
     * 새 결과/오류 이벤트가 없는 호출은 연속 측정 횟수에 포함하지 않는다. */
    measurement_error = vl53l0x_take_measurement_error(VL53L0X_FRONT);
    measurement_new   = vl53l0x_take_new_measurement(VL53L0X_FRONT);

    if (measurement_error == true)
    {
        record_distance_sensor_failure();
        return;
    }
    if (measurement_new == false)
    {
        return;
    }

    range_mm = vl53l0x_get_distance_mm(VL53L0X_FRONT);

    /* 0mm는 정상 거리로 사용할 수 없으므로 실제 측정 실패에 포함한다. */
    if (range_mm == 0U)
    {
        record_distance_sensor_failure();
        return;
    }

    /* 새 거리 결과를 정상적으로 읽었다면 8190mm 같은 범위 초과값도 통신 성공이다.
     * 범위 초과는 센서 고장이 아니라 가까운 장애물이 없다는 뜻으로 사용한다. */
    filtered_front_mm = range_mm;
    invalid_count = 0;

    if (valid_count < TEST_SENSOR_CONFIRM_COUNT)
    {
        valid_count++;
    }
    if (valid_count >= TEST_SENSOR_CONFIRM_COUNT)
    {
        distance_sensor_fault = false;
    }

    if (filtered_front_mm <= TEST_OBSTACLE_MM)
    {
        clear_count = 0;

        if (obstacle_count < TEST_SENSOR_CONFIRM_COUNT)
        {
            obstacle_count++;
        }
        if (obstacle_count >= TEST_SENSOR_CONFIRM_COUNT)
        {
            obstacle_confirmed = true;
        }
        return;
    }

    if ((filtered_front_mm > VL53L0X_MAX_VALID_MM)
        || (filtered_front_mm >= TEST_OBSTACLE_EXIT_MM))
    {
        obstacle_count = 0;

        if (clear_count < TEST_SENSOR_CONFIRM_COUNT)
        {
            clear_count++;
        }
        if (clear_count >= TEST_SENSOR_CONFIRM_COUNT)
        {
            obstacle_confirmed = false;
        }
        return;
    }

    /* 250~319mm 구간에서는 직전 장애물 상태를 유지한다. */
    obstacle_count = 0;
    clear_count    = 0;
}

/* non-blocking 측정 상태머신을 반복 실행하며 정상 결과 3회를 기다린다. */
static bool wait_for_distance_sensor(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();

    while ((distance_sensor_fault == true)
           && ((HAL_GetTick() - start_tick) < timeout_ms))
    {
        update_front_sensor();
        HAL_Delay(5);
    }

    return (distance_sensor_fault == false);
}

/* 주행 로그에서 현재 동작을 한눈에 볼 수 있는 이름을 만든다. */
static const char *state_name(void)
{
    static char turn_name[24];
    uint8_t step;

    if (test_avoiding == false)
    {
        if (bump_yaw_state == BUMP_YAW_PAUSE)
        {
            return "BUMP-YAW-PAUSE";
        }
        if (bump_yaw_state == BUMP_YAW_ROTATE)
        {
            return "BUMP-YAW-RECOVER";
        }

        switch (speed_bump_control_get_state())
        {
            case SPEED_BUMP_STATE_CLIMB:      return "BUMP-UPHILL";
            case SPEED_BUMP_STATE_DESCENT:    return "BUMP-DOWNHILL";
            case SPEED_BUMP_STATE_LEVEL_HOLD: return "BUMP-LEVEL-HOLD";
            case SPEED_BUMP_STATE_NORMAL:
            default:                          return "NORMAL";
        }
    }

    switch (obstacle_avoidance_get_state())
    {
        case AVOID_STATE_STOP:
        case AVOID_STATE_CHECK_SPACE:
            return "OBSTACLE-STOP";

        case AVOID_STATE_FIRST_TURN:
            step = obstacle_avoidance_get_turn_step();
            snprintf(turn_name, sizeof(turn_name), "TURN-LEFT %u/3", step);
            return turn_name;

        case AVOID_STATE_FORWARD:
            return "BYPASS-FORWARD";

        case AVOID_STATE_SECOND_TURN:
            step = obstacle_avoidance_get_turn_step();
            snprintf(turn_name, sizeof(turn_name), "TURN-RIGHT %u/3", step);
            return turn_name;

        case AVOID_STATE_COMPLETED: return "AVOID-DONE";
        case AVOID_STATE_FAILED:    return "AVOID-FAILED";
        default:                    return "NORMAL";
    }
}

/* 실수 각도의 절댓값을 구한다. */
static float angle_abs(float angle_deg)
{
    return (angle_deg < 0.0f) ? -angle_deg : angle_deg;
}

/* 방지턱 Z축 복구 상태를 취소하고 일반 주행 안전정지를 다시 켠다. */
static void reset_bump_yaw_recovery(void)
{
    bump_yaw_state = BUMP_YAW_IDLE;
    bump_original_heading_deg = 0.0f;
    bump_yaw_state_tick = 0;
    bump_yaw_hold_tick = 0;
    bump_yaw_holding = false;
    heading_set_runaway_protection(true);
}

/* 방지턱에서 Z축이 30도 이상 틀어지면 먼저 관성을 줄이기 위해 정지한다. */
static void start_bump_yaw_recovery(float pose_z)
{
    bump_original_heading_deg = heading_get_target();
    bump_yaw_state = BUMP_YAW_PAUSE;
    bump_yaw_state_tick = HAL_GetTick();
    bump_yaw_hold_tick = 0;
    bump_yaw_holding = false;

    /* 방지턱 복구 중에만 일반 직진용 45도 안전정지를 해제한다. */
    heading_set_runaway_protection(false);
    drive_stop();

    printf("\r\n[EVENT][BUMP] Z:%d deg -> pause, then recover original heading\r\n",
           (int)pose_z);
}

/* 정지 후 자이로를 계속 읽으며 원래 Z축 방향까지 제자리 회전한다. */
static void update_bump_yaw_recovery(uint32_t dt)
{
    uint32_t now = HAL_GetTick();
    float pose_x = 0.0f;
    float pose_y = 0.0f;
    float pose_z = 0.0f;

    /* 정지 중에도 자이로와 엔코더 상태는 계속 갱신해야 한다. */
    drive_update(dt);
    (void)heading_get_pose(&pose_x, &pose_y, &pose_z);

    if (bump_yaw_state == BUMP_YAW_PAUSE)
    {
        if ((now - bump_yaw_state_tick) >= BUMP_YAW_PAUSE_MS)
        {
            drive_recover_heading(bump_original_heading_deg);
            bump_yaw_state = BUMP_YAW_ROTATE;
            bump_yaw_state_tick = now;

            printf("[RECOVERY][BUMP] Rotate toward Z:0 deg\r\n");
        }
        return;
    }

    if (bump_yaw_state != BUMP_YAW_ROTATE)
    {
        return;
    }

    /* Z가 0도 근처에서 0.3초 유지돼야 관성까지 멈췄다고 판단한다. */
    if (angle_abs(pose_z) <= BUMP_YAW_DONE_DEG)
    {
        if (bump_yaw_holding == false)
        {
            bump_yaw_holding = true;
            bump_yaw_hold_tick = now;
        }
        else if ((now - bump_yaw_hold_tick) >= BUMP_YAW_HOLD_MS)
        {
            drive_stop();
            speed_bump_control_reset();
            reset_bump_yaw_recovery();
            drive_forward(test_speed);

            printf("[RECOVERY][BUMP] Z returned to 0 -> normal drive\r\n");
        }
    }
    else
    {
        bump_yaw_holding = false;
        bump_yaw_hold_tick = 0;
    }

    if ((bump_yaw_state == BUMP_YAW_ROTATE)
        && ((now - bump_yaw_state_tick) >= BUMP_YAW_TIMEOUT_MS))
    {
        printf("\r\n[ERROR][GYRO] Bump heading recovery timeout - stopped\r\n");

        test_running = false;
        speed_bump_control_reset();
        reset_bump_yaw_recovery();
        drive_stop();
    }
}

/* 사용자가 선택한 기본 속도에 현재 방지턱 단계의 배율을 적용한다. */
static float apply_speed_bump_target(void)
{
    float normal_rpm = drive_speed_to_rpm(test_speed);
    float target_rpm = speed_bump_control_get_target_rpm(normal_rpm);

    drive_set_forward_target_rpm(target_rpm);
    return target_rpm;
}

/* 최신 Y축(Pitch)으로 방지턱 상태를 갱신하고 상태 변화만 로그로 알린다. */
static void update_speed_bump_control(uint32_t dt)
{
    speed_bump_state_t previous_state = speed_bump_control_get_state();
    speed_bump_state_t current_state;
    float pose_x;
    float pose_y;
    float pose_z;
    float target_rpm;
    bool timed_out;
    bool forced_recovery;

    if (heading_get_pose(&pose_x, &pose_y, &pose_z) == false)
    {
        return;
    }

    speed_bump_control_update(pose_y, dt);
    current_state = speed_bump_control_get_state();
    timed_out = speed_bump_control_take_timeout();
    forced_recovery = speed_bump_control_take_forced_recovery();

    /* 방지턱 이벤트 중에는 45도 즉시정지 대신 아래의 30도 복구를 쓴다. */
    heading_set_runaway_protection(current_state == SPEED_BUMP_STATE_NORMAL);

    if (current_state == previous_state)
    {
        return;
    }

    target_rpm = apply_speed_bump_target();

    if (forced_recovery == true)
    {
        printf("\r\n[EVENT][BUMP] Downhill 3 s elapsed -> forced normal drive %d RPM\r\n",
               (int)target_rpm);
        return;
    }

    if (timed_out == true)
    {
        printf("\r\n[WARNING][BUMP] State timeout -> normal drive (%d RPM)\r\n",
               (int)target_rpm);
        return;
    }

    switch (current_state)
    {
        case SPEED_BUMP_STATE_CLIMB:
            printf("\r\n[EVENT][BUMP] Uphill Y:%d deg -> +30%% target %d RPM\r\n",
                   (int)pose_y,
                   (int)target_rpm);
            break;

        case SPEED_BUMP_STATE_DESCENT:
            printf("\r\n[EVENT][BUMP] Crest reached Y:%d deg -> -10%% target %d RPM\r\n",
                   (int)pose_y,
                   (int)target_rpm);
            break;

        case SPEED_BUMP_STATE_LEVEL_HOLD:
            printf("\r\n[EVENT][BUMP] Downhill passed -> checking level for 1 s\r\n");
            break;

        case SPEED_BUMP_STATE_NORMAL:
            printf("\r\n[EVENT][BUMP] Level stable for 1 s -> normal drive %d RPM\r\n",
                   (int)target_rpm);
            break;

        default:
            break;
    }
}

/* 회피 모듈의 내부 속도 명령을 로그용 목표 RPM으로 바꾼다. */
static float avoidance_target_rpm(int16_t speed)
{
    float rpm;

    if (speed == 0)
    {
        return 0.0f;
    }

    rpm = drive_speed_to_rpm((uint8_t)((speed < 0) ? -speed : speed));
    return (speed < 0) ? -rpm : rpm;
}

/* 회피 중 직접 구동되는 모터의 실제 RPM에 회전 방향 부호를 붙인다. */
static float avoidance_actual_rpm(motor_t motor, encoder_id_t encoder)
{
    float rpm = encoder_get_rpm(encoder);

    if (rpm < 0.0f)
    {
        rpm = -rpm;
    }

    if (motor_get_direction(motor) == MOTOR_REVERSE)
    {
        return -rpm;
    }
    if (motor_get_direction(motor) == MOTOR_FORWARD)
    {
        return rpm;
    }

    return 0.0f;
}

/* 사용할 수 있는 명령을 안내한다. */
static void print_help(void)
{
    printf("\r\n=========================================\r\n");
    printf(" Obstacle Avoidance Test (front only)\r\n");
    printf("=========================================\r\n");
    printf("  s : start driving\r\n");
    printf("  x : stop\r\n");
    printf("  z : reset heading reference\r\n");
    printf("  1 : slow   (%d RPM)\r\n",
           (int)drive_speed_to_rpm(DRIVE_SPEED_SLOW));
    printf("  2 : normal (%d RPM)\r\n",
           (int)drive_speed_to_rpm(DRIVE_SPEED_NORMAL));
    printf("  3 : fast   (%d RPM)\r\n",
           (int)drive_speed_to_rpm(DRIVE_SPEED_FAST));
    printf("  d : front distance only\r\n");
    printf("  h : help\r\n");
    printf("-----------------------------------------\r\n");
}

/* 전방 센서 상태를 한 줄로 출력한다. */
static void print_distance(void)
{
    if (distance_sensor_fault == true)
    {
        printf("[ERROR][DISTANCE] Front distance is unavailable\r\n");
        return;
    }

    printf("[DISTANCE] Front: %u mm\r\n", read_front_mm());
}

/* 주행 중 상태를 한 줄로 출력한다. */
static void print_status(void)
{
    float pose_x = 0.0f;
    float pose_y = 0.0f;
    float pose_z = 0.0f;
    float target_left;
    float target_right;
    float actual_left;
    float actual_right;

    (void)heading_get_pose(&pose_x, &pose_y, &pose_z);

    if (test_avoiding == true)
    {
        target_left = avoidance_target_rpm(
            obstacle_avoidance_get_left_speed());
        target_right = avoidance_target_rpm(
            obstacle_avoidance_get_right_speed());
        actual_left = avoidance_actual_rpm(MOTOR_LEFT, ENCODER_LEFT);
        actual_right = avoidance_actual_rpm(MOTOR_RIGHT, ENCODER_RIGHT);
    }
    else
    {
        target_left = wheel_get_target_rpm(WHEEL_LEFT);
        target_right = wheel_get_target_rpm(WHEEL_RIGHT);
        actual_left = wheel_get_rpm(WHEEL_LEFT);
        actual_right = wheel_get_rpm(WHEEL_RIGHT);
    }

    printf("[RUN] %-16s | front:%4u mm | pose X:%4d Y:%4d Z:%4d deg | "
           "target L:%4d R:%4d RPM | actual L:%4d R:%4d RPM\r\n",
           state_name(),
           read_front_mm(),
           (int)pose_x,
           (int)pose_y,
           (int)pose_z,
           (int)target_left,
           (int)target_right,
           (int)actual_left,
           (int)actual_right);
}

/* 회피 상태 머신에 제어를 넘기는 내부 함수 */
static void enter_avoidance(void)
{
    uint16_t front = read_front_mm();

    /* 장애물 회피가 최우선이므로 방지턱 속도 상태를 해제한다. */
    speed_bump_control_reset();
    reset_bump_yaw_recovery();

    /* drive 가 모터를 잡고 있으면 안 되므로 먼저 놓게 한다 */
    drive_stop();

    /* 이번 시험 규칙은 항상 왼쪽 90도 회피다. */
    obstacle_avoidance_start(AVOID_DIRECTION_LEFT,
                             heading_get_current(),
                             drive_get_distance_mm());

    test_avoiding = true;
    avoid_encoder_accum_ms = 0;
    last_avoid_stall_restart_count = 0;
    obstacle_confirmed = false;
    obstacle_count = 0;
    clear_count = 0;

    printf("\r\n[EVENT][OBSTACLE] Detected at %u mm -> avoidance start\r\n",
           front);
}

/* 회피가 끝난 뒤 정상 주행으로 돌아가는 내부 함수 */
static void leave_avoidance(void)
{
    test_avoiding = false;

    if (obstacle_avoidance_has_failed() == true)
    {
        switch (obstacle_avoidance_get_failure())
        {
            case AVOID_FAILURE_FRONT_BLOCKED:
                printf("\r\n[ERROR][OBSTACLE] Path still blocked - stopped\r\n");
                break;

            case AVOID_FAILURE_TURN_TIMEOUT:
                printf("\r\n[ERROR][GYRO] Turn target timeout - stopped\r\n");
                break;

            case AVOID_FAILURE_MOTOR_STALL:
                printf("\r\n[ERROR][MOTOR] Restart failed; encoder RPM stayed low - stopped\r\n");
                break;

            default:
                printf("\r\n[ERROR][AVOIDANCE] Avoidance failed - stopped\r\n");
                break;
        }

        test_running = false;
        drive_stop();
        return;
    }

    printf("\r\n[EVENT][OBSTACLE] Avoidance completed -> normal drive\r\n");

    /* 다음 회피를 위해 상태 머신을 대기로 되돌린다 */
    obstacle_avoidance_reset();

    /* 회피 완료 직후 이전 장애물 이벤트가 다시 발생하지 않게 초기화한다. */
    obstacle_confirmed = false;
    obstacle_count = 0;
    clear_count = 0;

    /* 방향 기준은 그대로 두므로 원래 진행 방향으로 복귀한다 */
    speed_bump_control_reset();
    reset_bump_yaw_recovery();
    drive_forward(test_speed);
}

/* 정상 주행과 회피를 오가며 한 주기 제어를 수행하는 내부 함수 */
static void control_step(uint32_t dt)
{
    uint16_t front = read_front_mm();
    bool bump_was_active;

    if (test_running == false)
    {
        drive_update(dt);
        return;
    }

    /* MPU6050이 준비되지 않으면 목표 RPM을 만들 수 없으므로 즉시 안전 정지한다. */
    if (drive_is_ready() == false)
    {
        printf("\r\n[ERROR][GYRO] MPU6050 data unavailable - stopped\r\n");

        test_running  = false;
        test_avoiding = false;
        obstacle_avoidance_reset();
        drive_stop();
        return;
    }

    if (wheel_has_startup_fault() == true)
    {
        printf("\r\n[ERROR][MOTOR] Restart failed; encoder RPM stayed low - stopped\r\n");

        test_running  = false;
        test_avoiding = false;
        obstacle_avoidance_reset();
        drive_stop();
        return;
    }

    /* 거리 센서가 연속으로 실패하면 장애물 없음으로 간주하지 않고 정지한다. */
    if (distance_sensor_fault == true)
    {
        printf("\r\n[ERROR][DISTANCE] VL53L0X measurement failed - stopped\r\n");

        test_running  = false;
        test_avoiding = false;
        obstacle_avoidance_reset();
        drive_stop();
        return;
    }

    if (test_avoiding == true)
    {
        /* 자이로 각도는 계속 적분해야 하므로 heading 은 갱신해 둔다 */
        heading_update(dt);

        if (drive_is_ready() == false)
        {
            printf("\r\n[ERROR][GYRO] MPU6050 failed during avoidance - stopped\r\n");
            test_running  = false;
            test_avoiding = false;
            obstacle_avoidance_reset();
            drive_stop();
            return;
        }

        /* 회피 중에도 100ms마다 엔코더 RPM을 갱신해 정지를 감시한다. */
        avoid_encoder_accum_ms += dt;
        if (avoid_encoder_accum_ms >= 100U)
        {
            encoder_update(avoid_encoder_accum_ms);
            avoid_encoder_accum_ms = 0;
        }

        /* 회피 중에는 상태 머신이 계산한 속도를 모터에 직접 넣는다. */
        obstacle_avoidance_update(heading_get_current(),
                                  drive_get_distance_mm(),
                                  front,
                                  encoder_get_rpm(ENCODER_LEFT),
                                  encoder_get_rpm(ENCODER_RIGHT));

        motor_set_output(MOTOR_LEFT,  obstacle_avoidance_get_left_speed());
        motor_set_output(MOTOR_RIGHT, obstacle_avoidance_get_right_speed());

        {
            uint32_t restart_count = obstacle_avoidance_get_restart_count();

            if (restart_count > last_avoid_stall_restart_count)
            {
                printf("\r\n[RECOVERY][MOTOR] RPM low for 5 s -> startup retry 1/1\r\n");
            }
            last_avoid_stall_restart_count = restart_count;
        }

        if (obstacle_avoidance_is_running() == false)
        {
            leave_avoidance();
        }
        return;
    }

    /* 방지턱에서 Z축 복구를 시작했다면 완료될 때까지 이 상태가 제어권을 갖는다. */
    if (bump_yaw_state != BUMP_YAW_IDLE)
    {
        update_bump_yaw_recovery(dt);
        return;
    }

    /* 정상 주행 중 전방이 막히면 회피로 넘어간다 */
    if (obstacle_confirmed == true)
    {
        enter_avoidance();
        return;
    }

    bump_was_active =
        (speed_bump_control_get_state() != SPEED_BUMP_STATE_NORMAL);

    drive_update(dt);
    update_speed_bump_control(dt);

    /* 같은 주기에서 방지턱 상태가 종료되더라도 이미 발생한 Z 이탈을 놓치지 않는다. */
    if (bump_was_active
        || (speed_bump_control_get_state() != SPEED_BUMP_STATE_NORMAL))
    {
        float pose_x = 0.0f;
        float pose_y = 0.0f;
        float pose_z = 0.0f;

        if ((heading_get_pose(&pose_x, &pose_y, &pose_z) == true)
            && (angle_abs(pose_z) >= BUMP_YAW_TRIGGER_DEG))
        {
            start_bump_yaw_recovery(pose_z);
            return;
        }
    }

    if (heading_has_runaway_fault() == true)
    {
        printf("\r\n[ERROR][GYRO] Heading error exceeded 45 deg - stopped\r\n");
        test_running = false;
        drive_stop();
        return;
    }

    {
        uint32_t restart_count = wheel_get_stall_restart_count();

        if (restart_count > last_wheel_stall_restart_count)
        {
            printf("\r\n[RECOVERY][MOTOR] RPM low for 5 s -> startup retry 1/1\r\n");
        }
        last_wheel_stall_restart_count = restart_count;
    }
}

/* S 버튼 세 번으로 요청된 거리센서 복구를 안전한 정지 상태에서 수행한다. */
static void recover_distance_sensor(void)
{
    bool initialized;
    bool recovered = false;

    if (distance_recovery_in_progress == true)
    {
        printf("\r\n[DIST] recovery already in progress\r\n");
        return;
    }

    if (distance_recovery_attempts >= TEST_RECOVERY_MAX_ATTEMPTS)
    {
        printf("\r\n[DIST] recovery blocked - maximum attempts reached\r\n");
        return;
    }

    distance_recovery_in_progress = true;
    distance_recovery_attempts++;

    /* 센서를 재초기화하는 동안에는 어떤 모터 명령도 남지 않게 한다. */
    test_running  = false;
    test_avoiding = false;
    obstacle_avoidance_reset();
    drive_stop();

    printf("\r\n[DIST] recovery attempt %u/%u\r\n",
           distance_recovery_attempts,
           TEST_RECOVERY_MAX_ATTEMPTS);
    printf("[DIST] XSHUT reset and sensor initialization\r\n");

    reset_distance_filter();
    initialized = vl53l0x_init_all();

    if (initialized == true)
    {
        recovered = wait_for_distance_sensor(TEST_SENSOR_STARTUP_TIMEOUT_MS);
    }

    distance_recovery_in_progress = false;

    if (recovered == true)
    {
        blocked_start_count = 0;
        blocked_start_first_tick = 0;
        distance_recovery_attempts = 0;
        printf("[DIST] recovery success - press S again to start\r\n");
    }
    else
    {
        printf("[DIST] recovery failed - vehicle remains stopped\r\n");
    }
}

/* UART와 IR 리모컨이 공통으로 사용하는 출발 동작.
 * 입력 방법이 달라도 센서 확인·기준 초기화·직진 시작은 항상 같다. */
static void start_driving(const char *source)
{
    uint32_t now;

    if (distance_sensor_fault == true)
    {
        now = HAL_GetTick();

        /* 첫 입력이거나 5초 창이 지났다면 새 3회 입력 묶음을 시작한다. */
        if ((blocked_start_count == 0U)
            || ((now - blocked_start_first_tick)
                > TEST_RECOVERY_PRESS_WINDOW_MS))
        {
            blocked_start_count = 1U;
            blocked_start_first_tick = now;
        }
        else if (blocked_start_count < TEST_RECOVERY_PRESS_COUNT)
        {
            blocked_start_count++;
        }

        printf("\r\n[ERROR][DISTANCE] Start blocked (%u/%u recovery presses)\r\n",
               blocked_start_count,
               TEST_RECOVERY_PRESS_COUNT);

        if (blocked_start_count >= TEST_RECOVERY_PRESS_COUNT)
        {
            blocked_start_count = 0;
            blocked_start_first_tick = 0;
            recover_distance_sensor();
        }
        return;
    }

    if (drive_is_motor_ready() == false)
    {
        printf("\r\n[ERROR][MOTOR] Driver PWM is not initialized - start blocked\r\n");
        return;
    }
    if ((drive_is_left_encoder_ready() == false)
        || (drive_is_right_encoder_ready() == false))
    {
        printf("\r\n[ERROR][ENCODER] Input capture is not initialized - start blocked\r\n");
        return;
    }

    /* 초기 MPU6050 통신이 실패했다면 정지 상태에서 한 번 다시 초기화한다. */
    if (drive_is_heading_ready() == false)
    {
        printf("\r\n[RECOVERY][GYRO] MPU6050 retry; keep vehicle still\r\n");

        if (drive_retry_heading_init() == false)
        {
            printf("[ERROR][GYRO] MPU6050 initialization failed - start blocked\r\n");
            return;
        }

        printf("[RECOVERY][GYRO] MPU6050 ready\r\n");
    }

    /* 센서가 스스로 정상화됐다면 이전의 막힌 출발 횟수는 의미가 없다. */
    blocked_start_count = 0;
    blocked_start_first_tick = 0;
    distance_recovery_attempts = 0;
    last_wheel_stall_restart_count = 0;

    printf("\r\n[COMMAND][%s] START -> normal forward drive\r\n", source);

    drive_prepare_start();
    drive_reset_heading();
    drive_reset_distance();
    obstacle_avoidance_reset();
    speed_bump_control_reset();
    reset_bump_yaw_recovery();

    test_running  = true;
    test_avoiding = false;

    drive_forward(test_speed);
}

/* UART 로 들어온 한 글자를 명령으로 처리하는 내부 함수 */
static void handle_command(uint8_t ch)
{
    switch (ch)
    {
        case 's':
        case 'S':
            start_driving("UART");
            break;

        case 'x':
        case 'X':
            printf("\r\n[COMMAND] STOP\r\n");

            test_running  = false;
            test_avoiding = false;

            obstacle_avoidance_reset();
            speed_bump_control_reset();
            reset_bump_yaw_recovery();
            drive_stop();
            break;

        case 'z':
        case 'Z':
            drive_reset_heading();
            speed_bump_control_reset();
            reset_bump_yaw_recovery();
            if (test_running && (test_avoiding == false))
            {
                (void)apply_speed_bump_target();
            }
            printf("[COMMAND] Pose reference reset to zero\r\n");
            break;

        case '1':
            test_speed = DRIVE_SPEED_SLOW;
            drive_set_speed(test_speed);
            if (test_running && (test_avoiding == false))
            {
                (void)apply_speed_bump_target();
            }
            printf("[COMMAND] Speed: slow (%d RPM)\r\n",
                   (int)drive_speed_to_rpm(test_speed));
            break;

        case '2':
            test_speed = DRIVE_SPEED_NORMAL;
            drive_set_speed(test_speed);
            if (test_running && (test_avoiding == false))
            {
                (void)apply_speed_bump_target();
            }
            printf("[COMMAND] Speed: normal (%d RPM)\r\n",
                   (int)drive_speed_to_rpm(test_speed));
            break;

        case '3':
            test_speed = DRIVE_SPEED_FAST;
            drive_set_speed(test_speed);
            if (test_running && (test_avoiding == false))
            {
                (void)apply_speed_bump_target();
            }
            printf("[COMMAND] Speed: fast (%d RPM)\r\n",
                   (int)drive_speed_to_rpm(test_speed));
            break;

        case 'd':
        case 'D':
            print_distance();
            break;

        case 'h':
        case 'H':
            print_help();
            break;

        default:
            break;
    }
}

/* UART 수신 버퍼를 확인해 명령이 있으면 처리하는 내부 함수 */
static void poll_uart(void)
{
    uint8_t ch;

    /* 타임아웃 0 이므로 들어온 글자가 없으면 즉시 돌아온다 */
    if (HAL_UART_Receive(&huart2, &ch, 1, 0) == HAL_OK)
    {
        handle_command(ch);
    }
}

/* IR 리모컨의 새 명령을 확인하고 S 버튼을 공통 출발 동작으로 연결한다. */
static void poll_ir_remote(void)
{
    uint8_t addr;
    uint8_t cmd;

    ir_remote_update();

    if (ir_remote_get_command(&addr, &cmd) == false)
    {
        return;
    }

    if (addr != TEST_IR_ADDR)
    {
        printf("\r\n[ERROR][REMOTE] Unknown remote address\r\n");
        return;
    }

    /* UART 명령 처리 함수를 그대로 재사용해 두 입력의 동작을 통일한다. */
    switch (cmd)
    {
        case TEST_IR_START_CMD:
            start_driving("IR");
            break;

        case TEST_IR_STOP_CMD:
            handle_command('x');
            break;

        case TEST_IR_RESET_YAW_CMD:
            handle_command('z');
            break;

        case TEST_IR_DISTANCE_CMD:
            handle_command('d');
            break;

        case TEST_IR_SLOW_CMD:
            handle_command('1');
            break;

        case TEST_IR_NORMAL_CMD:
            handle_command('2');
            break;

        case TEST_IR_FAST_CMD:
            handle_command('3');
            break;

        default:
            printf("\r\n[ERROR][REMOTE] Unmapped button\r\n");
            break;
    }
}


/* 장애물 회피 시험에 필요한 모듈들을 초기화한다.
 * 자이로 영점 보정 중에는 차체를 절대 움직이면 안 된다. */
void obstacle_avoidance_test_init(void)
{
    obstacle_avoidance_config_t config;
    bool sensor_initialized;
    bool distance_ready = false;
    bool drive_initialized;
    bool ir_initialized;

    printf("\r\n========== SYSTEM INITIALIZATION ==========\r\n");
    printf("[INIT] Motor, encoders and MPU6050 starting...\r\n");

    drive_initialized = drive_init();

    printf("[INIT] Motor driver ........ %s\r\n",
           drive_is_motor_ready() ? "OK" : "ERROR");
    printf("[INIT] Left encoder ........ %s\r\n",
           drive_is_left_encoder_ready() ? "OK" : "ERROR");
    printf("[INIT] Right encoder ....... %s\r\n",
           drive_is_right_encoder_ready() ? "OK" : "ERROR");
    printf("[INIT] MPU6050 gyro ........ %s\r\n",
           drive_is_heading_ready() ? "OK" : "ERROR");

    reset_distance_filter();
    blocked_start_count = 0;
    blocked_start_first_tick = 0;
    distance_recovery_attempts = 0;
    distance_recovery_in_progress = false;
    last_wheel_stall_restart_count = 0;
    last_avoid_stall_restart_count = 0;
    avoid_encoder_accum_ms = 0;
    sensor_initialized = vl53l0x_init_all();

    obstacle_avoidance_init();
    speed_bump_control_init();
    reset_bump_yaw_recovery();

    config.obstacle_distance_mm = TEST_OBSTACLE_MM;
    config.bypass_distance_mm   = TEST_BYPASS_MM;
    config.forward_speed        = DRIVE_SPEED_NORMAL;
    /* 기존 최대 출력 회전은 관성이 컸으므로 정상 속도로 낮춘다. */
    config.turn_speed           = DRIVE_SPEED_NORMAL;
    config.turn_angle_deg       = TEST_TURN_ANGLE_DEG;

    obstacle_avoidance_set_config(&config);

    /* non-blocking 측정 상태머신을 반복 진행해 출발 전에 정상값을 확보한다. */
    if (sensor_initialized == true)
    {
        distance_ready = wait_for_distance_sensor(TEST_SENSOR_STARTUP_TIMEOUT_MS);
    }

    printf("[INIT] VL53L0X distance .... %s",
           distance_ready ? "OK" : "ERROR");
    if (distance_ready == true)
    {
        printf(" (%u mm)\r\n", read_front_mm());
    }
    else
    {
        printf("\r\n");
    }

    printf("[INIT] Avoidance control ... OK (30 deg x 3 turns)\r\n");
    printf("[INIT] Speed bump control .. OK (Pitch Y axis)\r\n");
    printf("[READY] Speed bump: Y >= +10 deg, level = -5..+5 deg\r\n");
    printf("[READY] Bump yaw recovery: |Z| >= 30 deg -> return to Z=0\r\n");

    /* TIM4 입력 캡처를 시작해 리모컨 명령을 받을 준비를 한다. */
    ir_initialized = ir_remote_init();
    printf("[INIT] Remote control ...... %s\r\n",
           ir_initialized ? "OK" : "ERROR");

    printf("[READY] RPM range: min %d / normal %d / max %d RPM\r\n",
           (int)DRIVE_RPM_MIN,
           (int)drive_speed_to_rpm(DRIVE_SPEED_NORMAL),
           (int)DRIVE_RPM_MAX);

    if (drive_initialized && distance_ready && ir_initialized)
    {
        printf("[READY] All systems ready; waiting for START\r\n");
    }
    else
    {
        printf("[ERROR][INIT] One or more devices failed; check lines above\r\n");
    }
    printf("===========================================\r\n");
}

/* 장애물 회피 시험 루프를 실행한다. */
void obstacle_avoidance_test_run(void)
{
    uint32_t control_tick = HAL_GetTick();
    uint32_t sensor_tick  = HAL_GetTick();
    uint32_t print_tick   = HAL_GetTick();
    uint32_t now;
    uint32_t dt;

    print_help();

    while (1)
    {
        poll_uart();
        poll_ir_remote();

        /* 센서를 먼저 갱신한다.
         * 회피 판단이 최신 거리값을 쓰도록 제어보다 앞에 둔다 */
        now = HAL_GetTick();
        if ((now - sensor_tick) >= SENSOR_PERIOD_MS)
        {
            sensor_tick = now;
            update_front_sensor();
        }

        /* 제어 주기마다 주행과 회피를 갱신한다.
         * 센서 측정에 시간이 걸리므로 실제 경과 시간을 그대로 넘긴다 */
        now = HAL_GetTick();
        if ((now - control_tick) >= CONTROL_PERIOD_MS)
        {
            dt = now - control_tick;
            if (dt > CONTROL_DT_LIMIT_MS)
            {
                dt = CONTROL_DT_LIMIT_MS;
            }
            control_tick = now;

            control_step(dt);
        }

        /* 주행 중에만 상태를 출력한다 */
        now = HAL_GetTick();
        if ((now - print_tick) >= PRINT_PERIOD_MS)
        {
            print_tick = now;

            if (test_running == true)
            {
                print_status();
            }
        }
    }
}

#endif  /* TEST_OBSTACLE_AVOIDANCE */
