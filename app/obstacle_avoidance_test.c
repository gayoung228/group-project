#include "main.h"
#include "obstacle_avoidance_test.h"
#include "obstacle_avoidance.h"
#include "drive.h"
#include "heading_control.h"
#include "motor.h"
#include "vl53l0x.h"
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
#define PRINT_PERIOD_MS         300

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

/* 회피 방향으로 전진할 거리 [mm] */
#define TEST_BYPASS_MM          300

/* 왼쪽으로 회피한 뒤 오른쪽으로 복귀할 각도 [도] */
#define TEST_TURN_ANGLE_DEG     90.0f


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

/* 전방 센서를 한 번 읽고 연속 감지·히스테리시스·오류 상태를 갱신한다. */
static void update_front_sensor(void)
{
    bool measurement_ok = vl53l0x_update_sensor(VL53L0X_FRONT);
    uint16_t range_mm = vl53l0x_get_distance_mm(VL53L0X_FRONT);

    /* ready == false 또는 측정 트랜잭션 실패는 실제 센서 오류로 처리한다.
     * 0mm도 정상 거리로 사용할 수 없으므로 동일하게 오류 횟수에 포함한다. */
    if ((vl53l0x_is_ready(VL53L0X_FRONT) == false)
        || (measurement_ok == false)
        || (range_mm == 0U))
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
        return;
    }

    /* 측정 트랜잭션이 성공했다면 8190mm 같은 범위 초과값도 통신 성공이다.
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

/* 현재 회피 단계의 이름을 문자열로 돌려주는 내부 함수 */
static const char *state_name(void)
{
    switch (obstacle_avoidance_get_state())
    {
        case AVOID_STATE_IDLE:        return "IDLE";
        case AVOID_STATE_STOP:        return "STOP";
        case AVOID_STATE_CHECK_SPACE: return "CHECK";
        case AVOID_STATE_FIRST_TURN:  return "TURN1";
        case AVOID_STATE_FORWARD:     return "FWD";
        case AVOID_STATE_SECOND_TURN: return "TURN2";
        case AVOID_STATE_COMPLETED:   return "DONE";
        case AVOID_STATE_FAILED:      return "FAIL";
        default:                      return "?";
    }
}

/* 현재 회피 방향의 이름을 문자열로 돌려주는 내부 함수 */
static const char *direction_name(void)
{
    switch (obstacle_avoidance_get_direction())
    {
        case AVOID_DIRECTION_LEFT:  return "L";
        case AVOID_DIRECTION_RIGHT: return "R";
        default:                    return "-";
    }
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
    printf("  1 : slow   (%d %%)\r\n", DRIVE_SPEED_SLOW);
    printf("  2 : normal (%d %%)\r\n", DRIVE_SPEED_NORMAL);
    printf("  3 : fast   (%d %%)\r\n", DRIVE_SPEED_FAST);
    printf("  d : front distance only\r\n");
    printf("  h : help\r\n");
    printf("-----------------------------------------\r\n");
}

/* 전방 센서 상태를 한 줄로 출력한다. */
static void print_distance(void)
{
    printf("[front] %4u mm   ready:%d  valid:%d  obstacle:%d  fault:%d\r\n",
           vl53l0x_get_distance_mm(VL53L0X_FRONT),
           vl53l0x_is_ready(VL53L0X_FRONT),
           vl53l0x_is_valid(VL53L0X_FRONT),
           obstacle_confirmed,
           distance_sensor_fault);
}

/* 주행 중 상태를 한 줄로 출력한다. */
static void print_status(void)
{
    printf("%-5s %s | raw:%4u r:%d v:%d obs:%d fault:%d | front:%4u | yaw:%5d | L:%4d(%4d) R:%4d(%4d) | dist:%5d mm\r\n",
           state_name(),
           direction_name(),
           vl53l0x_get_distance_mm(VL53L0X_FRONT),
           (int)vl53l0x_is_ready(VL53L0X_FRONT),
           (int)vl53l0x_is_valid(VL53L0X_FRONT),
           (int)obstacle_confirmed,
           (int)distance_sensor_fault,
           read_front_mm(),
           (int)heading_get_current(),
           obstacle_avoidance_get_left_speed(),
           motor_get_speed(MOTOR_LEFT),
           obstacle_avoidance_get_right_speed(),
           motor_get_speed(MOTOR_RIGHT),
           (int)drive_get_distance_mm());
}

/* 회피 상태 머신에 제어를 넘기는 내부 함수 */
static void enter_avoidance(void)
{
    uint16_t front = read_front_mm();

    /* drive 가 모터를 잡고 있으면 안 되므로 먼저 놓게 한다 */
    drive_stop();

    /* 이번 시험 규칙은 항상 왼쪽 90도 회피다. */
    obstacle_avoidance_start(AVOID_DIRECTION_LEFT,
                             heading_get_current(),
                             drive_get_distance_mm());

    test_avoiding = true;
    obstacle_confirmed = false;
    obstacle_count = 0;
    clear_count = 0;

    printf("\r\n[avoid] start  front:%u mm  dir:%s\r\n", front, direction_name());
}

/* 회피가 끝난 뒤 정상 주행으로 돌아가는 내부 함수 */
static void leave_avoidance(void)
{
    test_avoiding = false;

    if (obstacle_avoidance_has_failed() == true)
    {
        printf("\r\n[avoid] FAILED - stopped\r\n");

        test_running = false;
        drive_stop();
        return;
    }

    printf("\r\n[avoid] completed\r\n");

    /* 다음 회피를 위해 상태 머신을 대기로 되돌린다 */
    obstacle_avoidance_reset();

    /* 회피 완료 직후 이전 장애물 이벤트가 다시 발생하지 않게 초기화한다. */
    obstacle_confirmed = false;
    obstacle_count = 0;
    clear_count = 0;

    /* 방향 기준은 그대로 두므로 원래 진행 방향으로 복귀한다 */
    drive_forward(test_speed);
}

/* 정상 주행과 회피를 오가며 한 주기 제어를 수행하는 내부 함수 */
static void control_step(uint32_t dt)
{
    uint16_t front = read_front_mm();

    if (test_running == false)
    {
        drive_update(dt);
        return;
    }

    /* 거리 센서가 연속으로 실패하면 장애물 없음으로 간주하지 않고 정지한다. */
    if (distance_sensor_fault == true)
    {
        printf("\r\n>>> DISTANCE SENSOR FAILED - STOP\r\n");

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

        /* 회피 중에는 상태 머신이 계산한 속도를 모터에 직접 넣는다 */
        obstacle_avoidance_update(heading_get_current(),
                                  drive_get_distance_mm(),
                                  front);

        motor_set_output(MOTOR_LEFT,  obstacle_avoidance_get_left_speed());
        motor_set_output(MOTOR_RIGHT, obstacle_avoidance_get_right_speed());

        if (obstacle_avoidance_is_running() == false)
        {
            leave_avoidance();
        }
        return;
    }

    /* 정상 주행 중 전방이 막히면 회피로 넘어간다 */
    if (obstacle_confirmed == true)
    {
        enter_avoidance();
        return;
    }

    drive_update(dt);
}

/* UART 로 들어온 한 글자를 명령으로 처리하는 내부 함수 */
static void handle_command(uint8_t ch)
{
    switch (ch)
    {
        case 's':
        case 'S':
            if (distance_sensor_fault == true)
            {
                printf("\r\n>>> START BLOCKED - DISTANCE SENSOR NOT READY\r\n");
                break;
            }

            printf("\r\n>>> START\r\n");

            drive_reset_heading();
            drive_reset_distance();
            obstacle_avoidance_reset();

            test_running  = true;
            test_avoiding = false;

            drive_forward(test_speed);
            break;

        case 'x':
        case 'X':
            printf("\r\n>>> STOP\r\n");

            test_running  = false;
            test_avoiding = false;

            obstacle_avoidance_reset();
            drive_stop();
            break;

        case 'z':
        case 'Z':
            drive_reset_heading();
            printf("[heading] reference reset\r\n");
            break;

        case '1':
            test_speed = DRIVE_SPEED_SLOW;
            printf("[speed] %d %%\r\n", test_speed);
            break;

        case '2':
            test_speed = DRIVE_SPEED_NORMAL;
            printf("[speed] %d %%\r\n", test_speed);
            break;

        case '3':
            test_speed = DRIVE_SPEED_FAST;
            printf("[speed] %d %%\r\n", test_speed);
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


/* 장애물 회피 시험에 필요한 모듈들을 초기화한다.
 * 자이로 영점 보정 중에는 차체를 절대 움직이면 안 된다. */
void obstacle_avoidance_test_init(void)
{
    obstacle_avoidance_config_t config;
    bool sensor_initialized;
    uint8_t sample;

    drive_init();

    reset_distance_filter();
    sensor_initialized = vl53l0x_init_all();

    obstacle_avoidance_init();

    config.obstacle_distance_mm = TEST_OBSTACLE_MM;
    config.bypass_distance_mm   = TEST_BYPASS_MM;
    config.forward_speed        = DRIVE_SPEED_NORMAL;
    config.turn_speed           = DRIVE_SPEED_FAST;
    config.turn_angle_deg       = TEST_TURN_ANGLE_DEG;

    obstacle_avoidance_set_config(&config);

    /* 출발 전에 연속 정상값을 확보해 오측정과 초기 불안정값을 걸러낸다. */
    if (sensor_initialized == true)
    {
        for (sample = 0; sample < TEST_SENSOR_CONFIRM_COUNT; sample++)
        {
            update_front_sensor();
        }
    }
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
    print_distance();

    while (1)
    {
        poll_uart();

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
