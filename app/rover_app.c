#include "main.h"
#include "rover_app.h"
#include "obstacle_avoidance.h"
#include "drive.h"
#include "encoder.h"
#include "heading_control.h"
#include "ir_remote.h"
#include "mission_control.h"
#include "speed_bump_control.h"
#include "vl53l0x.h"
#include "wheel.h"
#include "wheel_calibration.h"
#include <stdio.h>

/* ------------------------------------------------------------------
 * rover_app.c - 자율주행 로버 통합 애플리케이션
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

#ifdef APP_ROVER

/* 제어 주기 [ms] */
#define CONTROL_PERIOD_MS       20

/* 센서 측정 주기 [ms] */
#define SENSOR_PERIOD_MS        40

/* 화면 출력 주기 [ms] */
#define PRINT_PERIOD_MS         500

/* 제어 주기가 크게 밀렸을 때 적분에 넣을 최대 시간 [ms] */
#define CONTROL_DT_LIMIT_MS     100

/* 장애물로 판단할 전방 거리 [mm] */
#define ROVER_OBSTACLE_MM        250

/* 장애물이 사라졌다고 판단할 거리 [mm]
 * 진입값보다 크게 두어 250mm 근처에서 상태가 흔들리는 것을 막는다. */
#define ROVER_OBSTACLE_EXIT_MM   320

/* 장애물/정상 거리로 확정하기 위해 필요한 연속 측정 횟수 */
#define ROVER_SENSOR_CONFIRM_COUNT  3U

/* 이 횟수만큼 연속 측정에 실패하면 차량을 안전 정지한다. */
#define ROVER_SENSOR_FAIL_COUNT     3U

/* 시작 안내를 띄우기 전에 거리 센서 정상값을 기다리는 최대 시간 [ms] */
#define ROVER_SENSOR_STARTUP_TIMEOUT_MS  1500U

/* 거리센서 오류 상태에서 이 시간 안에 S를 세 번 누르면 센서 복구를 시도한다. */
#define ROVER_RECOVERY_PRESS_WINDOW_MS   5000U
#define ROVER_RECOVERY_PRESS_COUNT       3U
#define ROVER_RECOVERY_MAX_ATTEMPTS      3U

/* 회피 방향으로 전진할 거리 [mm] */
#define ROVER_BYPASS_MM          500

/* 왼쪽으로 회피한 뒤 오른쪽으로 복귀할 각도 [도] */
#define ROVER_TURN_ANGLE_DEG     90.0f

/* 방지턱에서 진행 방향이 크게 틀어졌을 때의 Z축(Yaw) 복구 조건 */
#define BUMP_YAW_TRIGGER_DEG       30.0f
#define BUMP_YAW_DONE_DEG           5.0f
#define BUMP_YAW_PAUSE_MS         300U
#define BUMP_YAW_HOLD_MS          300U
#define BUMP_YAW_TIMEOUT_MS      6000U

/* 실측한 리모컨 NEC 주소와 버튼별 명령값 */
#define ROVER_IR_ADDR            0x00U
#define ROVER_IR_START_CMD       0xC2U
#define ROVER_IR_STOP_CMD        0x90U
#define ROVER_IR_RESET_YAW_CMD   0xE2U
#define ROVER_IR_DISTANCE_CMD    0xB0U
#define ROVER_IR_SLOW_CMD        0x30U
#define ROVER_IR_NORMAL_CMD      0x18U
#define ROVER_IR_FAST_CMD        0x7AU


extern UART_HandleTypeDef huart2;

/* 정상 주행 속도 [PWM %] */
static uint8_t rover_speed = DRIVE_SPEED_NORMAL;

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

typedef enum
{
    VEHICLE_FAULT_GYRO = 0,
    VEHICLE_FAULT_MOTOR_ENCODER,
    VEHICLE_FAULT_TURN_CONTROL
} vehicle_fault_t;

/* 하드웨어 복구 뒤에는 사용자가 차체를 놓은 방향을 직접 0도로 확정해야 한다. */
static bool fault_recovery_in_progress = false;
static bool fault_recovery_ready = false;
static bool fault_heading_reference_set = false;
static bool fault_recovery_failed = false;


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

    if (invalid_count < ROVER_SENSOR_FAIL_COUNT)
    {
        invalid_count++;
    }
    if (invalid_count >= ROVER_SENSOR_FAIL_COUNT)
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

    if (valid_count < ROVER_SENSOR_CONFIRM_COUNT)
    {
        valid_count++;
    }
    if (valid_count >= ROVER_SENSOR_CONFIRM_COUNT)
    {
        distance_sensor_fault = false;
    }

    if (filtered_front_mm <= ROVER_OBSTACLE_MM)
    {
        clear_count = 0;

        if (obstacle_count < ROVER_SENSOR_CONFIRM_COUNT)
        {
            obstacle_count++;
        }
        if (obstacle_count >= ROVER_SENSOR_CONFIRM_COUNT)
        {
            obstacle_confirmed = true;
        }
        return;
    }

    if ((filtered_front_mm > VL53L0X_MAX_VALID_MM)
        || (filtered_front_mm >= ROVER_OBSTACLE_EXIT_MM))
    {
        obstacle_count = 0;

        if (clear_count < ROVER_SENSOR_CONFIRM_COUNT)
        {
            clear_count++;
        }
        if (clear_count >= ROVER_SENSOR_CONFIRM_COUNT)
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

    if (mission_control_is_avoiding() == false)
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

/* 방지턱 Z축 복구 상태를 취소한다. */
static void reset_bump_yaw_recovery(void)
{
    bump_yaw_state = BUMP_YAW_IDLE;
    bump_original_heading_deg = 0.0f;
    bump_yaw_state_tick = 0;
    bump_yaw_hold_tick = 0;
    bump_yaw_holding = false;
}

/* 주행 불능 오류가 발생했을 때 사용하는 공통 복구 흐름.
 * 1) 긴급 정지 2) 원인 출력 3) 관련 하드웨어 재시동
 * 4) 사용자가 Z로 방향 기준을 잡은 뒤 START하도록 안내한다. */
static void recover_critical_vehicle_fault(vehicle_fault_t fault,
                                           const char *reason)
{
    bool recovered = false;

    if (fault_recovery_in_progress == true)
    {
        return;
    }

    fault_recovery_in_progress = true;
    fault_recovery_ready = false;
    fault_heading_reference_set = false;
    fault_recovery_failed = false;

    mission_control_emergency_stop();
    obstacle_avoidance_reset();
    speed_bump_control_reset();
    reset_bump_yaw_recovery();
    drive_stop();

    printf("\r\n========== EMERGENCY STOP ==========\r\n");
    printf("[FAULT] Normal driving is not possible.\r\n");
    printf("[CAUSE] %s\r\n", reason);
    printf("[SAFETY] Motors stopped. Keep the vehicle still.\r\n");

    switch (fault)
    {
        case VEHICLE_FAULT_GYRO:
            printf("[RESET][GYRO] Restarting MPU6050 and calibrating; please wait...\r\n");
            recovered = drive_retry_heading_init();
            break;

        case VEHICLE_FAULT_MOTOR_ENCODER:
            printf("[RESET][DRIVE] Restarting motor PWM and encoder TIM5; please wait...\r\n");
            recovered = drive_retry_motion_hardware();

            /* 부팅 단계에서 여러 장치가 함께 실패했으면 자이로도 이어서 복구한다. */
            if (recovered && (drive_is_heading_ready() == false))
            {
                printf("[RESET][GYRO] MPU6050 is also unavailable; calibrating...\r\n");
                recovered = drive_retry_heading_init();
            }
            break;

        case VEHICLE_FAULT_TURN_CONTROL:
            printf("[RESET][DRIVE] Turn failed; restarting motor and encoders...\r\n");
            recovered = drive_retry_motion_hardware();
            if (recovered)
            {
                printf("[RESET][GYRO] Recalibrating MPU6050; please wait...\r\n");
                recovered = drive_retry_heading_init();
            }
            break;

        default:
            recovered = false;
            break;
    }

    fault_recovery_in_progress = false;

    if (recovered == true)
    {
        mission_control_stop();
        fault_recovery_ready = true;
        printf("[RESET] Sensor restart completed.\r\n");
        printf("[READY] Place the vehicle in the desired forward direction.\r\n");
        printf("[ACTION] Press Z to set heading zero, then press START (S).\r\n");
        printf("====================================\r\n");
    }
    else
    {
        fault_recovery_failed = true;
        printf("[RESET][FAILED] Hardware restart failed; vehicle remains stopped.\r\n");
        printf("[ACTION] Check power/wiring and reset the board.\r\n");
        printf("====================================\r\n");
    }
}

/* 방지턱에서 Z축이 30도 이상 틀어지면 먼저 관성을 줄이기 위해 정지한다. */
static void start_bump_yaw_recovery(float pose_z)
{
    bump_original_heading_deg = heading_get_target();
    bump_yaw_state = BUMP_YAW_PAUSE;
    bump_yaw_state_tick = HAL_GetTick();
    bump_yaw_hold_tick = 0;
    bump_yaw_holding = false;

    drive_stop();
    mission_control_begin_bump_recovery();

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
            drive_forward(rover_speed);
            mission_control_resume_driving();

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
        recover_critical_vehicle_fault(
            VEHICLE_FAULT_TURN_CONTROL,
            "Bump heading recovery did not reach the yaw target in time");
        return;
    }
}

/* 사용자가 선택한 기본 속도에 현재 방지턱 단계의 배율을 적용한다. */
static float apply_speed_bump_target(void)
{
    float normal_rpm = drive_speed_to_rpm(rover_speed);
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
static float avoidance_actual_rpm(int16_t commanded_output,
                                  encoder_id_t encoder)
{
    float rpm = encoder_get_rpm(encoder);

    if (rpm < 0.0f)
    {
        rpm = -rpm;
    }

    if (commanded_output < 0)
    {
        return -rpm;
    }
    if (commanded_output > 0)
    {
        return rpm;
    }

    return 0.0f;
}

/* 사용할 수 있는 명령을 안내한다. */
static void print_help(void)
{
    printf("\r\n=========================================\r\n");
    printf(" Autonomous Rover (front sensor)\r\n");
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
    printf("  c : wheel-lift PWM/RPM calibration (press twice)\r\n");
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
    int16_t output_left;
    int16_t output_right;

    (void)heading_get_pose(&pose_x, &pose_y, &pose_z);

    if (mission_control_is_avoiding() == true)
    {
        target_left = avoidance_target_rpm(
            obstacle_avoidance_get_left_speed());
        target_right = avoidance_target_rpm(
            obstacle_avoidance_get_right_speed());
        actual_left = avoidance_actual_rpm(
            obstacle_avoidance_get_left_speed(), ENCODER_LEFT);
        actual_right = avoidance_actual_rpm(
            obstacle_avoidance_get_right_speed(), ENCODER_RIGHT);
        output_left = obstacle_avoidance_get_left_speed();
        output_right = obstacle_avoidance_get_right_speed();
    }
    else
    {
        target_left = wheel_get_target_rpm(WHEEL_LEFT);
        target_right = wheel_get_target_rpm(WHEEL_RIGHT);
        actual_left = wheel_get_rpm(WHEEL_LEFT);
        actual_right = wheel_get_rpm(WHEEL_RIGHT);
        output_left = wheel_get_output(WHEEL_LEFT);
        output_right = wheel_get_output(WHEEL_RIGHT);
    }

    printf("[RUN] %-16s | front:%4u mm | pose X:%4d Y:%4d Z:%4d deg | "
           "target L:%4d R:%4d RPM | actual L:%4d R:%4d RPM | PWM L:%4d R:%4d%%\r\n",
           state_name(),
           read_front_mm(),
           (int)pose_x,
           (int)pose_y,
           (int)pose_z,
           (int)target_left,
           (int)target_right,
           (int)actual_left,
           (int)actual_right,
           (int)output_left,
           (int)output_right);
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

    mission_control_begin_avoidance();
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
    if (obstacle_avoidance_has_failed() == true)
    {
        switch (obstacle_avoidance_get_failure())
        {
            case AVOID_FAILURE_FRONT_BLOCKED:
                printf("\r\n[ERROR][OBSTACLE] Path still blocked - stopped\r\n");
                break;

            case AVOID_FAILURE_TURN_TIMEOUT:
                recover_critical_vehicle_fault(
                    VEHICLE_FAULT_TURN_CONTROL,
                    "Obstacle turn did not reach the gyro target in time");
                return;

            case AVOID_FAILURE_MOTOR_STALL:
                recover_critical_vehicle_fault(
                    VEHICLE_FAULT_MOTOR_ENCODER,
                    "Motor restart failed or encoder RPM remained below 30 RPM");
                return;

            default:
                printf("\r\n[ERROR][AVOIDANCE] Avoidance failed - stopped\r\n");
                break;
        }

        mission_control_emergency_stop();
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
    drive_forward(rover_speed);
    mission_control_resume_driving();
}

/* 정상 주행과 회피를 오가며 한 주기 제어를 수행하는 내부 함수 */
static void control_step(uint32_t dt)
{
    uint16_t front = read_front_mm();
    bool bump_was_active;

    if (wheel_calibration_is_active() == true)
    {
        wheel_calibration_update(dt);

        if (wheel_calibration_take_sensor_fault() == true)
        {
            recover_critical_vehicle_fault(
                VEHICLE_FAULT_GYRO,
                "MPU6050 data update failed during wheel calibration");
            return;
        }

        if (wheel_calibration_take_completed() == true)
        {
            mission_control_stop();
            fault_recovery_ready = true;
            fault_heading_reference_set = false;
            fault_recovery_failed = false;
            printf("[READY] Place the vehicle on the floor in the forward direction.\r\n");
            printf("[ACTION] Press Z to set heading zero, then press START (S).\r\n");
        }
        return;
    }

    if (mission_control_is_running() == false)
    {
        drive_update(dt);
        return;
    }

    /* MPU6050이 준비되지 않으면 목표 RPM을 만들 수 없으므로 즉시 안전 정지한다. */
    if (drive_is_ready() == false)
    {
        recover_critical_vehicle_fault(
            VEHICLE_FAULT_GYRO,
            "MPU6050 data update failed during normal driving");
        return;
    }

    if (wheel_has_startup_fault() == true)
    {
        recover_critical_vehicle_fault(
            VEHICLE_FAULT_MOTOR_ENCODER,
            "Motor restart failed or encoder RPM remained below 30 RPM");
        return;
    }

    /* 거리 센서가 연속으로 실패하면 장애물 없음으로 간주하지 않고 정지한다. */
    if (distance_sensor_fault == true)
    {
        printf("\r\n[ERROR][DISTANCE] VL53L0X measurement failed - stopped\r\n");

        mission_control_emergency_stop();
        obstacle_avoidance_reset();
        drive_stop();
        return;
    }

    if (mission_control_is_avoiding() == true)
    {
        /* 자이로와 엔코더 측정은 drive가 갱신하고 PID는 직접 출력 모드에서 끈다. */
        drive_update(dt);

        if (drive_is_ready() == false)
        {
            recover_critical_vehicle_fault(
                VEHICLE_FAULT_GYRO,
                "MPU6050 data update failed during obstacle avoidance");
            return;
        }

        /* 회피 중에는 상태 머신이 계산한 출력을 drive의 직접 출력 모드로 넣는다. */
        obstacle_avoidance_update(heading_get_current(),
                                  drive_get_distance_mm(),
                                  front,
                                  encoder_get_rpm(ENCODER_LEFT),
                                  encoder_get_rpm(ENCODER_RIGHT));

        drive_set_direct_output(obstacle_avoidance_get_left_speed(),
                                obstacle_avoidance_get_right_speed());

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

    if (distance_recovery_attempts >= ROVER_RECOVERY_MAX_ATTEMPTS)
    {
        printf("\r\n[DIST] recovery blocked - maximum attempts reached\r\n");
        return;
    }

    distance_recovery_in_progress = true;
    distance_recovery_attempts++;

    /* 센서를 재초기화하는 동안에는 어떤 모터 명령도 남지 않게 한다. */
    mission_control_stop();
    obstacle_avoidance_reset();
    drive_stop();

    printf("\r\n[DIST] recovery attempt %u/%u\r\n",
           distance_recovery_attempts,
           ROVER_RECOVERY_MAX_ATTEMPTS);
    printf("[DIST] XSHUT reset and sensor initialization\r\n");

    reset_distance_filter();
    initialized = vl53l0x_init_all();

    if (initialized == true)
    {
        recovered = wait_for_distance_sensor(ROVER_SENSOR_STARTUP_TIMEOUT_MS);
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

    if (wheel_calibration_is_armed() || wheel_calibration_is_active())
    {
        printf("\r\n[START BLOCKED] Wheel calibration is armed/running. Press X to cancel.\r\n");
        return;
    }

    if (fault_recovery_failed == true)
    {
        printf("\r\n[START BLOCKED] Hardware recovery failed. Check wiring and reset the board.\r\n");
        return;
    }

    if (fault_recovery_ready && (fault_heading_reference_set == false))
    {
        printf("\r\n[START BLOCKED] Recovery completed, but heading zero is not set.\r\n");
        printf("[ACTION] Place the vehicle forward, press Z, then press START (S).\r\n");
        return;
    }

    if (distance_sensor_fault == true)
    {
        now = HAL_GetTick();

        /* 첫 입력이거나 5초 창이 지났다면 새 3회 입력 묶음을 시작한다. */
        if ((blocked_start_count == 0U)
            || ((now - blocked_start_first_tick)
                > ROVER_RECOVERY_PRESS_WINDOW_MS))
        {
            blocked_start_count = 1U;
            blocked_start_first_tick = now;
        }
        else if (blocked_start_count < ROVER_RECOVERY_PRESS_COUNT)
        {
            blocked_start_count++;
        }

        printf("\r\n[ERROR][DISTANCE] Start blocked (%u/%u recovery presses)\r\n",
               blocked_start_count,
               ROVER_RECOVERY_PRESS_COUNT);

        if (blocked_start_count >= ROVER_RECOVERY_PRESS_COUNT)
        {
            blocked_start_count = 0;
            blocked_start_first_tick = 0;
            recover_distance_sensor();
        }
        return;
    }

    if (drive_is_motor_ready() == false)
    {
        recover_critical_vehicle_fault(
            VEHICLE_FAULT_MOTOR_ENCODER,
            "Motor PWM was not initialized at START");
        return;
    }
    if ((drive_is_left_encoder_ready() == false)
        || (drive_is_right_encoder_ready() == false))
    {
        recover_critical_vehicle_fault(
            VEHICLE_FAULT_MOTOR_ENCODER,
            "Left or right encoder TIM5 input capture was not initialized");
        return;
    }

    /* 초기 MPU6050 통신이 실패했다면 정지 상태에서 한 번 다시 초기화한다. */
    if (drive_is_heading_ready() == false)
    {
        recover_critical_vehicle_fault(
            VEHICLE_FAULT_GYRO,
            "MPU6050 was not initialized at START");
        return;
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

    mission_control_start();

    drive_forward(rover_speed);

    fault_recovery_ready = false;
    fault_heading_reference_set = false;
    fault_recovery_failed = false;
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

            if (wheel_calibration_is_armed()
                || wheel_calibration_is_active())
            {
                wheel_calibration_cancel();
            }

            mission_control_stop();

            obstacle_avoidance_reset();
            speed_bump_control_reset();
            reset_bump_yaw_recovery();
            drive_stop();
            break;

        case 'z':
        case 'Z':
            drive_reset_heading();
            if (fault_recovery_ready == true)
            {
                fault_heading_reference_set = true;
                printf("[RECOVERY] Heading zero is set. Press START (S).\r\n");
            }
            speed_bump_control_reset();
            reset_bump_yaw_recovery();
            if (mission_control_is_running()
                && (mission_control_is_avoiding() == false))
            {
                (void)apply_speed_bump_target();
                mission_control_resume_driving();
            }
            printf("[COMMAND] Pose reference reset to zero\r\n");
            break;

        case 'c':
        case 'C':
            if (wheel_calibration_is_active() == true)
            {
                printf("\r\n[CAL] Calibration is already running. Press X to stop.\r\n");
                break;
            }

            if (wheel_calibration_is_armed() == true)
            {
                if (wheel_calibration_start() == false)
                {
                    if ((drive_is_motor_ready() == false)
                        || (drive_is_left_encoder_ready() == false)
                        || (drive_is_right_encoder_ready() == false))
                    {
                        recover_critical_vehicle_fault(
                            VEHICLE_FAULT_MOTOR_ENCODER,
                            "Motor or encoder was not ready for wheel calibration");
                    }
                    else if (drive_is_heading_ready() == false)
                    {
                        recover_critical_vehicle_fault(
                            VEHICLE_FAULT_GYRO,
                            "MPU6050 was not ready for wheel calibration");
                    }
                }
                break;
            }

            mission_control_stop();
            obstacle_avoidance_reset();
            speed_bump_control_reset();
            reset_bump_yaw_recovery();
            drive_stop();
            wheel_calibration_arm();
            break;

        case '1':
            rover_speed = DRIVE_SPEED_SLOW;
            drive_set_speed(rover_speed);
            if (mission_control_get_state() == MISSION_STATE_DRIVING)
            {
                (void)apply_speed_bump_target();
            }
            printf("[COMMAND] Speed: slow (%d RPM)\r\n",
                   (int)drive_speed_to_rpm(rover_speed));
            break;

        case '2':
            rover_speed = DRIVE_SPEED_NORMAL;
            drive_set_speed(rover_speed);
            if (mission_control_get_state() == MISSION_STATE_DRIVING)
            {
                (void)apply_speed_bump_target();
            }
            printf("[COMMAND] Speed: normal (%d RPM)\r\n",
                   (int)drive_speed_to_rpm(rover_speed));
            break;

        case '3':
            rover_speed = DRIVE_SPEED_FAST;
            drive_set_speed(rover_speed);
            if (mission_control_get_state() == MISSION_STATE_DRIVING)
            {
                (void)apply_speed_bump_target();
            }
            printf("[COMMAND] Speed: fast (%d RPM)\r\n",
                   (int)drive_speed_to_rpm(rover_speed));
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

    if (addr != ROVER_IR_ADDR)
    {
        printf("\r\n[ERROR][REMOTE] Unknown remote address\r\n");
        return;
    }

    /* UART 명령 처리 함수를 그대로 재사용해 두 입력의 동작을 통일한다. */
    switch (cmd)
    {
        case ROVER_IR_START_CMD:
            start_driving("IR");
            break;

        case ROVER_IR_STOP_CMD:
            handle_command('x');
            break;

        case ROVER_IR_RESET_YAW_CMD:
            handle_command('z');
            break;

        case ROVER_IR_DISTANCE_CMD:
            handle_command('d');
            break;

        case ROVER_IR_SLOW_CMD:
            handle_command('1');
            break;

        case ROVER_IR_NORMAL_CMD:
            handle_command('2');
            break;

        case ROVER_IR_FAST_CMD:
            handle_command('3');
            break;

        default:
            printf("\r\n[ERROR][REMOTE] Unmapped button\r\n");
            break;
    }
}


/* 장애물 회피 시험에 필요한 모듈들을 초기화한다.
 * 자이로 영점 보정 중에는 차체를 절대 움직이면 안 된다. */
void rover_app_init(void)
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
    sensor_initialized = vl53l0x_init_all();

    obstacle_avoidance_init();
    speed_bump_control_init();
    mission_control_init();
    reset_bump_yaw_recovery();
    wheel_calibration_init();
    fault_recovery_in_progress = false;
    fault_recovery_ready = false;
    fault_heading_reference_set = false;
    fault_recovery_failed = false;

    config.obstacle_distance_mm = ROVER_OBSTACLE_MM;
    config.bypass_distance_mm   = ROVER_BYPASS_MM;
    config.forward_speed        = DRIVE_SPEED_NORMAL;
    /* 기존 최대 출력 회전은 관성이 컸으므로 정상 속도로 낮춘다. */
    config.turn_speed           = DRIVE_SPEED_NORMAL;
    config.turn_angle_deg       = ROVER_TURN_ANGLE_DEG;

    obstacle_avoidance_set_config(&config);

    /* non-blocking 측정 상태머신을 반복 진행해 출발 전에 정상값을 확보한다. */
    if (sensor_initialized == true)
    {
        distance_ready = wait_for_distance_sensor(ROVER_SENSOR_STARTUP_TIMEOUT_MS);
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
void rover_app_run(void)
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

            if (mission_control_is_running() == true)
            {
                print_status();
            }
        }
    }
}

#endif  /* APP_ROVER */
