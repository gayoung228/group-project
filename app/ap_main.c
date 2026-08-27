#include "main.h"
#include "ap_main.h"
#include "drive.h"
#include "heading_control.h"
#include "wheel.h"
#include "encoder.h"
#include <stdio.h>

/* ------------------------------------------------------------------
 * ap_main.c - 자이로 방향 유지 시험
 *
 * UART 명령으로 방향 제어를 켜고 끄면서
 * 주행 중 방향이 틀어졌을 때 스스로 복귀하는지 확인한다.
 *
 * 터미널 설정 : 115200 8N1
 * ------------------------------------------------------------------ */

/* 제어 주기 [ms]
 * 자이로 적분은 주기가 짧고 일정해야 정확하다. */
#define CONTROL_PERIOD_MS       20

/* 화면 출력 주기 [ms] */
#define PRINT_PERIOD_MS         200

/* 목표 주행 거리 [mm] : 0 이면 거리 제한 없이 계속 달린다 */
#define TARGET_DISTANCE_MM      8000

/* ★ encoder.c 의 ENCODER_SLOTS_PER_REV 과 같은 값이어야 한다 ★ */
#define SLOTS_PER_REV           20

/* 바퀴 지름 [mm] : 실물에 맞게 수정할 것 */
#define WHEEL_DIAMETER_MM       65.0f

#define PI_VALUE                3.141592f


/* 주행 상태 */
typedef enum
{
    RUN_IDLE = 0,   /* 대기 중 */
    RUN_ACTIVE      /* 주행 중 */
} run_state_t;

extern UART_HandleTypeDef huart2;

static run_state_t run_state  = RUN_IDLE;
static uint8_t     run_speed  = DRIVE_SPEED_NORMAL;
static uint32_t    start_tick = 0;


/* printf 출력을 UART2(ST-LINK 가상 COM 포트)로 내보낸다. */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
    return ch;
}

/* 엔코더 누적 카운트를 이동 거리 [mm] 로 바꿔주는 내부 함수 */
static float count_to_mm(int32_t count)
{
    if (count < 0)
    {
        count = -count;
    }

    return ((float)count / (float)SLOTS_PER_REV) * PI_VALUE * WHEEL_DIAMETER_MM;
}

/* 좌우 평균 이동 거리를 [mm] 로 돌려주는 내부 함수 */
static float get_distance_mm(void)
{
    float left  = count_to_mm(encoder_get_count(ENCODER_LEFT));
    float right = count_to_mm(encoder_get_count(ENCODER_RIGHT));

    return (left + right) / 2.0f;
}

/* 사용할 수 있는 명령을 안내한다. */
static void print_help(void)
{
    printf("\r\n=========================================\r\n");
    printf(" Heading Hold Test  (target %d mm)\r\n", TARGET_DISTANCE_MM);
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

/* 현재 선택된 주행 조건을 한 줄로 안내한다. */
static void print_mode(void)
{
    printf("[mode] heading %s / speed %d %%\r\n",
           heading_is_enabled() ? "ON " : "OFF",
           run_speed);
}

/* 주행 중 상태를 한 줄로 출력한다.
 * yaw 오차와 보정량을 함께 봐야 제어가 동작하는지 알 수 있다. */
static void print_status(void)
{
    printf("%5dmm | yaw tgt:%5d cur:%5d err:%5d corr:%5d | rpm L:%4d R:%4d | out L:%4d R:%4d\r\n",
           (int)get_distance_mm(),
           (int)heading_get_target(),
           (int)heading_get_current(),
           (int)heading_get_error(),
           (int)heading_get_correction(),
           (int)wheel_get_rpm(WHEEL_LEFT),
           (int)wheel_get_rpm(WHEEL_RIGHT),
           (int)drive_get_left_speed(),
           (int)drive_get_right_speed());
}

/* 주행이 끝난 뒤 결과를 정리해서 출력한다. */
static void print_result(void)
{
    int32_t left_count  = encoder_get_count(ENCODER_LEFT);
    int32_t right_count = encoder_get_count(ENCODER_RIGHT);
    int32_t diff        = left_count - right_count;

    if (diff < 0)
    {
        diff = -diff;
    }

    printf("\r\n----------- RESULT -----------\r\n");
    print_mode();
    printf(" distance  : %d mm\r\n", (int)get_distance_mm());
    printf(" time      : %d ms\r\n", (int)(HAL_GetTick() - start_tick));
    printf(" yaw final : %d deg\r\n", (int)heading_get_current());
    printf(" L count   : %ld\r\n", left_count);
    printf(" R count   : %ld\r\n", right_count);
    printf(" diff      : %ld\r\n", diff);
    printf("------------------------------\r\n");
    printf("Measure the sideways offset at the finish line.\r\n\r\n");
}

/* 주행을 시작한다. */
static void start_run(void)
{
    /* 이전 주행의 PID 상태와 출발 오류를 모두 초기화한다. */
    wheel_reset();
    heading_reset();

    start_tick = HAL_GetTick();
    run_state  = RUN_ACTIVE;

    drive_forward(run_speed);

    printf("\r\n>>> START\r\n");
    print_mode();
}

/* 주행을 멈추고 결과를 출력한다. */
static void finish_run(void)
{
    drive_stop();

    run_state = RUN_IDLE;

    print_result();
}

/* UART 로 들어온 한 글자를 명령으로 처리하는 내부 함수 */
static void handle_command(uint8_t ch)
{
    switch (ch)
    {
        case 's':
        case 'S':
            if (run_state == RUN_IDLE)
            {
                start_run();
            }
            break;

        case 'x':
        case 'X':
            if (run_state == RUN_ACTIVE)
            {
                printf("\r\n>>> STOP\r\n");
                finish_run();
            }
            else
            {
                drive_stop();
            }
            break;

        case 'g':
        case 'G':
            heading_set_enabled(true);
            print_mode();
            break;

        case 'n':
        case 'N':
            heading_set_enabled(false);
            print_mode();
            break;

        case 'z':
        case 'Z':
            heading_reset();
            printf("[heading] reference reset\r\n");
            break;

        case 'l':
        case 'L':
            printf("[heading] rotate left 90\r\n");
            drive_turn_left(run_speed);
            break;

        case 'r':
        case 'R':
            printf("[heading] rotate right 90\r\n");
            drive_turn_right(run_speed);
            break;

        case '1':
            run_speed = DRIVE_SPEED_SLOW;
            drive_set_speed(run_speed);
            print_mode();
            break;

        case '2':
            run_speed = DRIVE_SPEED_NORMAL;
            drive_set_speed(run_speed);
            print_mode();
            break;

        case '3':
            run_speed = DRIVE_SPEED_FAST;
            drive_set_speed(run_speed);
            print_mode();
            break;

        case 'h':
        case 'H':
            print_help();
            print_mode();
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


/* 애플리케이션에서 사용하는 모듈들을 초기화한다.
 * 자이로 영점 보정 중에는 차체를 절대 움직이면 안 된다. */
void ap_init(void)
{
    drive_init();
}

/* 자이로 방향 유지 시험용 메인 루프. */
void ap_main(void)
{
    extern void vl53l0x_serial_test_temp(void); vl53l0x_serial_test_temp(); /* TODO(vl53l0x 임시 테스트): 테스트 끝나면 이 줄만 삭제 */

    uint32_t control_tick = HAL_GetTick();
    uint32_t print_tick   = HAL_GetTick();

    print_help();
    print_mode();

    while (1)
    {
        poll_uart();

        /* 일정 주기로 자이로와 바퀴 제어를 갱신한다 */
        if ((HAL_GetTick() - control_tick) >= CONTROL_PERIOD_MS)
        {
            control_tick += CONTROL_PERIOD_MS;

            drive_update(CONTROL_PERIOD_MS);

            if ((run_state == RUN_ACTIVE) && wheel_has_startup_fault())
            {
                printf("\r\n>>> WHEEL START FAILED\r\n");
                finish_run();
            }

            if ((run_state == RUN_ACTIVE) && heading_has_runaway_fault())
            {
                printf("\r\n>>> HEADING ERROR: YAW OVER 45 DEG\r\n");
                finish_run();
            }

            /* 목표 거리에 도달하면 스스로 멈춘다 */
            if ((run_state == RUN_ACTIVE) &&
                (TARGET_DISTANCE_MM > 0) &&
                (get_distance_mm() >= (float)TARGET_DISTANCE_MM))
            {
                printf("\r\n>>> TARGET REACHED\r\n");
                finish_run();
            }
        }

        /* 일정 주기로 상태를 출력한다 */
        if ((HAL_GetTick() - print_tick) >= PRINT_PERIOD_MS)
        {
            print_tick += PRINT_PERIOD_MS;
            print_status();
        }
    }
}
