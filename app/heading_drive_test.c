#include "main.h"
#include "heading_drive_test.h"
#include "drive.h"
#include "heading_control.h"
#include "wheel.h"
#include "encoder.h"
#include <stdio.h>

/* ------------------------------------------------------------------
 * heading_drive_test.c - 자이로 직진 유지 시험 애플리케이션
 *
 * 이 파일은 다음 세 가지 역할만 담당한다.
 *
 *   1. UART 한 글자 명령을 차량 동작으로 변환한다.
 *   2. drive 모듈을 일정한 주기로 갱신한다.
 *   3. 주행 상태와 시험 결과를 UART로 출력한다.
 *
 * 실제 모터·엔코더·자이로 제어 계산은 각각의 하위 모듈이 담당한다.
 *
 *   heading_drive_test -> drive -> heading_control -> wheel
 *                                              wheel -> motor / encoder
 *
 * 터미널 설정: 115200 baud, 8 data bits, no parity, 1 stop bit
 * ------------------------------------------------------------------ */

#ifdef TEST_HEADING_DRIVE

/* 자이로 적분과 방향 제어를 갱신하는 주기 [ms] */
#define TEST_CONTROL_PERIOD_MS       20U

/* UART 상태를 출력하는 주기 [ms] */
#define TEST_PRINT_PERIOD_MS         200U

/* 자동으로 정지할 목표 거리 [mm], 0이면 거리 제한을 사용하지 않는다. */
#define TEST_TARGET_DISTANCE_MM      8000

/* encoder.c의 ENCODER_SLOTS_PER_REV와 반드시 같은 값이어야 한다. */
#define TEST_ENCODER_SLOTS_PER_REV   20

/* 실제 차량의 바퀴 지름 [mm] */
#define TEST_WHEEL_DIAMETER_MM       65.0f

#define TEST_PI_VALUE                3.141592f


/* 시험 프로그램이 현재 대기 중인지 주행 중인지 나타낸다. */
typedef enum
{
    TEST_RUN_IDLE = 0,
    TEST_RUN_ACTIVE
} test_run_state_t;

extern UART_HandleTypeDef huart2;

static test_run_state_t test_run_state = TEST_RUN_IDLE;
static uint8_t          test_run_speed = DRIVE_SPEED_NORMAL;
static uint32_t         test_start_tick = 0;


/* ------------------------------------------------------------------
 * UART 출력
 * ------------------------------------------------------------------ */

/* printf가 출력한 문자 한 개를 ST-LINK 가상 COM 포트(UART2)로 보낸다. */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
    return ch;
}


/* ------------------------------------------------------------------
 * 거리 계산
 * ------------------------------------------------------------------ */

/* 엔코더 누적 펄스 수를 한쪽 바퀴의 이동 거리 [mm]로 변환한다. */
static float test_count_to_mm(int32_t count)
{
    if (count < 0)
    {
        count = -count;
    }

    return ((float)count / (float)TEST_ENCODER_SLOTS_PER_REV)
         * TEST_PI_VALUE
         * TEST_WHEEL_DIAMETER_MM;
}

/* 좌우 바퀴 이동 거리의 평균을 차량 이동 거리 [mm]로 사용한다. */
static float test_get_distance_mm(void)
{
    float left_mm  = test_count_to_mm(encoder_get_count(ENCODER_LEFT));
    float right_mm = test_count_to_mm(encoder_get_count(ENCODER_RIGHT));

    return (left_mm + right_mm) / 2.0f;
}


/* ------------------------------------------------------------------
 * UART 화면 출력
 * ------------------------------------------------------------------ */

/* 터미널에서 사용할 수 있는 한 글자 명령을 출력한다. */
static void test_print_help(void)
{
    printf("\r\n=========================================\r\n");
    printf(" Heading Hold Test  (target %d mm)\r\n", TEST_TARGET_DISTANCE_MM);
    printf("=========================================\r\n");
    printf("  s : start forward\r\n");
    printf("  x : stop\r\n");
    printf("  g : heading control ON\r\n");
    printf("  n : heading control OFF\r\n");
    printf("  z : reset heading reference\r\n");
    printf("  l : rotate left  90 deg\r\n");
    printf("  r : rotate right 90 deg\r\n");
    printf("  1 : slow   (%d %%)\r\n", DRIVE_SPEED_SLOW);
    printf("  2 : normal (%d %%)\r\n", DRIVE_SPEED_NORMAL);
    printf("  3 : fast   (%d %%)\r\n", DRIVE_SPEED_FAST);
    printf("  h : help\r\n");
    printf("-----------------------------------------\r\n");
}

/* 현재 방향 제어 사용 여부와 선택된 주행 속도를 출력한다. */
static void test_print_mode(void)
{
    printf("[mode] heading %s / speed %d %%\r\n",
           heading_is_enabled() ? "ON " : "OFF",
           test_run_speed);
}

/* 거리, Yaw, RPM, PWM을 한 줄에 출력해 제어 과정을 관찰할 수 있게 한다. */
static void test_print_status(void)
{
    printf("%5dmm | yaw tgt:%5d cur:%5d err:%5d corr:%5d | "
           "rpm L:%4d R:%4d | out L:%4d R:%4d\r\n",
           (int)test_get_distance_mm(),
           (int)heading_get_target(),
           (int)heading_get_current(),
           (int)heading_get_error(),
           (int)heading_get_correction(),
           (int)wheel_get_rpm(WHEEL_LEFT),
           (int)wheel_get_rpm(WHEEL_RIGHT),
           (int)drive_get_left_speed(),
           (int)drive_get_right_speed());
}

/* 주행이 끝났을 때 최종 거리·시간·Yaw·엔코더 차이를 출력한다. */
static void test_print_result(void)
{
    int32_t left_count  = encoder_get_count(ENCODER_LEFT);
    int32_t right_count = encoder_get_count(ENCODER_RIGHT);
    int32_t count_difference = left_count - right_count;

    if (count_difference < 0)
    {
        count_difference = -count_difference;
    }

    printf("\r\n----------- RESULT -----------\r\n");
    test_print_mode();
    printf(" distance  : %d mm\r\n", (int)test_get_distance_mm());
    printf(" time      : %d ms\r\n", (int)(HAL_GetTick() - test_start_tick));
    printf(" yaw final : %d deg\r\n", (int)heading_get_current());
    printf(" L count   : %ld\r\n", left_count);
    printf(" R count   : %ld\r\n", right_count);
    printf(" diff      : %ld\r\n", count_difference);
    printf("------------------------------\r\n");
    printf("Measure the sideways offset at the finish line.\r\n\r\n");
}


/* ------------------------------------------------------------------
 * 주행 상태 변경
 * ------------------------------------------------------------------ */

/* 새 시험을 시작하기 전에 이전 PID·엔코더·Yaw 상태를 모두 초기화한다. */
static void test_start_run(void)
{
    wheel_reset();
    heading_reset();

    test_start_tick = HAL_GetTick();
    test_run_state  = TEST_RUN_ACTIVE;

    drive_forward(test_run_speed);

    printf("\r\n>>> START\r\n");
    test_print_mode();
}

/* 차량을 정지하고 주행 상태를 대기로 바꾼 뒤 결과를 출력한다. */
static void test_finish_run(void)
{
    drive_stop();
    test_run_state = TEST_RUN_IDLE;
    test_print_result();
}


/* ------------------------------------------------------------------
 * UART 명령 처리
 * ------------------------------------------------------------------ */

/* UART에서 받은 한 글자를 주행 명령으로 변환한다. */
static void test_handle_command(uint8_t command)
{
    switch (command)
    {
        case 's':
        case 'S':
            if (test_run_state == TEST_RUN_IDLE)
            {
                test_start_run();
            }
            break;

        case 'x':
        case 'X':
            if (test_run_state == TEST_RUN_ACTIVE)
            {
                printf("\r\n>>> STOP\r\n");
                test_finish_run();
            }
            else
            {
                drive_stop();
            }
            break;

        case 'g':
        case 'G':
            heading_set_enabled(true);
            test_print_mode();
            break;

        case 'n':
        case 'N':
            heading_set_enabled(false);
            test_print_mode();
            break;

        case 'z':
        case 'Z':
            heading_reset();
            printf("[heading] reference reset\r\n");
            break;

        case 'l':
        case 'L':
            printf("[heading] rotate left 90\r\n");
            drive_turn_left(test_run_speed);
            break;

        case 'r':
        case 'R':
            printf("[heading] rotate right 90\r\n");
            drive_turn_right(test_run_speed);
            break;

        case '1':
            test_run_speed = DRIVE_SPEED_SLOW;
            drive_set_speed(test_run_speed);
            test_print_mode();
            break;

        case '2':
            test_run_speed = DRIVE_SPEED_NORMAL;
            drive_set_speed(test_run_speed);
            test_print_mode();
            break;

        case '3':
            test_run_speed = DRIVE_SPEED_FAST;
            drive_set_speed(test_run_speed);
            test_print_mode();
            break;

        case 'h':
        case 'H':
            test_print_help();
            test_print_mode();
            break;

        default:
            /* 정의되지 않은 문자는 차량 동작에 영향을 주지 않는다. */
            break;
    }
}

/* 수신 문자가 있으면 처리하고, 없으면 기다리지 않고 즉시 반환한다. */
static void test_poll_uart(void)
{
    uint8_t command;

    if (HAL_UART_Receive(&huart2, &command, 1, 0) == HAL_OK)
    {
        test_handle_command(command);
    }
}


/* ------------------------------------------------------------------
 * 주기 작업
 * ------------------------------------------------------------------ */

/* 20ms마다 제어기를 갱신하고 안전 정지 조건을 확인한다. */
static void test_update_control(void)
{
    drive_update(TEST_CONTROL_PERIOD_MS);

    /* 대기 중에는 주행 종료 조건을 검사할 필요가 없다. */
    if (test_run_state != TEST_RUN_ACTIVE)
    {
        return;
    }

    if (wheel_has_startup_fault())
    {
        printf("\r\n>>> WHEEL START FAILED\r\n");
        test_finish_run();
        return;
    }

    if (heading_has_runaway_fault())
    {
        printf("\r\n>>> HEADING ERROR: YAW OVER 45 DEG\r\n");
        test_finish_run();
        return;
    }

    if ((TEST_TARGET_DISTANCE_MM > 0) &&
        (test_get_distance_mm() >= (float)TEST_TARGET_DISTANCE_MM))
    {
        printf("\r\n>>> TARGET REACHED\r\n");
        test_finish_run();
    }
}


/* ------------------------------------------------------------------
 * 공개 함수
 * ------------------------------------------------------------------ */

void heading_drive_test_init(void)
{
    test_run_state  = TEST_RUN_IDLE;
    test_run_speed  = DRIVE_SPEED_NORMAL;
    test_start_tick = 0;

    /* drive_init 안에서 motor, encoder, MPU6050, PID가 차례로 초기화된다. */
    drive_init();
}

void heading_drive_test_run(void)
{
    uint32_t control_tick = HAL_GetTick();
    uint32_t print_tick   = HAL_GetTick();

    test_print_help();
    test_print_mode();

    while (1)
    {
        test_poll_uart();

        if ((HAL_GetTick() - control_tick) >= TEST_CONTROL_PERIOD_MS)
        {
            control_tick += TEST_CONTROL_PERIOD_MS;
            test_update_control();
        }

        if ((HAL_GetTick() - print_tick) >= TEST_PRINT_PERIOD_MS)
        {
            print_tick += TEST_PRINT_PERIOD_MS;
            test_print_status();
        }
    }
}

#endif  /* TEST_HEADING_DRIVE */