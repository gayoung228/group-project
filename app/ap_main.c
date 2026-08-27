#include "main.h"
#include "ap_main.h"
#include "wheel.h"
#include "motor.h"
#include "encoder.h"
#include <stdio.h>

/* ------------------------------------------------------------------
 * ap_main.c - 직진 주행 시험
 *
 * UART 로 명령을 받아 개루프 주행과 PID 폐루프 주행을 번갈아 시험한다.
 * 평탄한 바닥에서 2m 를 달린 뒤 좌우 엔코더 카운트 차이를 비교하여
 * PID 제어의 효과를 확인하는 것이 목적이다.
 *
 * 터미널 설정 : 115200 8N1
 * ------------------------------------------------------------------ */

/* 제어 주기 [ms] : 엔코더 RPM 이 30 단위로 끊기므로 너무 짧게 하지 않는다 */
#define CONTROL_PERIOD_MS       30

/* 화면 출력 주기 [ms] */
#define PRINT_PERIOD_MS         200

/* 목표 주행 거리 [mm] */
#define TARGET_DISTANCE_MM      2000

/* ★ encoder.c 의 ENCODER_SLOTS_PER_REV 과 반드시 같은 값이어야 한다 ★ */
#define SLOTS_PER_REV           20

/* 바퀴 지름 [mm] : 실물에 맞게 수정할 것 */
#define WHEEL_DIAMETER_MM       65.0f

#define PI_VALUE                3.141592f

/* 저속 / 고속 설정
 * 개루프는 PWM 듀티를, 폐루프는 목표 RPM 을 쓴다.
 * 개루프로 먼저 달려보고 실제로 나온 RPM 을 아래 목표값에 맞추면
 * 두 방식의 속도가 같아져 비교가 정확해진다. */
#define LOW_SPEED_DUTY          70
#define LOW_SPEED_RPM           90.0f
#define HIGH_SPEED_DUTY         100
#define HIGH_SPEED_RPM          180.0f


/* 주행 상태 */
typedef enum
{
    RUN_IDLE = 0,   /* 대기 중 */
    RUN_ACTIVE      /* 주행 중 */
} run_state_t;

extern UART_HandleTypeDef huart2;

static run_state_t run_state  = RUN_IDLE;
static bool        use_pid    = false;   /* false: 개루프, true: 폐루프 */
static bool        use_high   = false;   /* false: 저속,   true: 고속   */
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
    printf(" Straight Drive Test  (target %d mm)\r\n", TARGET_DISTANCE_MM);
    printf("=========================================\r\n");
    printf("  s : start\r\n");
    printf("  x : stop\r\n");
    printf("  o : open loop  (PID off)\r\n");
    printf("  c : closed loop (PID on)\r\n");
    printf("  1 : low  speed\r\n");
    printf("  2 : high speed\r\n");
    printf("  h : help\r\n");
    printf("-----------------------------------------\r\n");
}

/* 현재 선택된 주행 조건을 한 줄로 안내한다. */
static void print_mode(void)
{
    if (use_pid)
    {
        printf("[mode] PID ON  / %s (%d rpm)\r\n",
               use_high ? "HIGH" : "LOW",
               (int)(use_high ? HIGH_SPEED_RPM : LOW_SPEED_RPM));
    }
    else
    {
        printf("[mode] PID OFF / %s (%d %%)\r\n",
               use_high ? "HIGH" : "LOW",
               use_high ? HIGH_SPEED_DUTY : LOW_SPEED_DUTY);
    }
}

/* 주행 중 상태를 한 줄로 출력한다.
 * 좌우 카운트 차이가 직진성을 판단하는 핵심 지표이다. */
static void print_status(void)
{
    int32_t left_count  = encoder_get_count(ENCODER_LEFT);
    int32_t right_count = encoder_get_count(ENCODER_RIGHT);

    printf("%5dmm | L cnt:%5ld rpm:%4d out:%4d | R cnt:%5ld rpm:%4d out:%4d | diff:%4ld\r\n",
           (int)get_distance_mm(),
           left_count,
           (int)wheel_get_rpm(WHEEL_LEFT),
           (int)motor_get_speed(MOTOR_LEFT),
           right_count,
           (int)wheel_get_rpm(WHEEL_RIGHT),
           (int)motor_get_speed(MOTOR_RIGHT),
           left_count - right_count);
}

/* 주행이 끝난 뒤 결과를 정리해서 출력한다. */
static void print_result(void)
{
    int32_t left_count  = encoder_get_count(ENCODER_LEFT);
    int32_t right_count = encoder_get_count(ENCODER_RIGHT);
    int32_t diff        = left_count - right_count;
    int32_t total       = left_count + right_count;
    int     percent     = 0;

    if (diff < 0)
    {
        diff = -diff;
    }

    /* 좌우 차이가 평균 대비 몇 퍼센트인지 계산한다 */
    if (total > 0)
    {
        percent = (int)((diff * 200) / total);
    }

    printf("\r\n----------- RESULT -----------\r\n");
    print_mode();
    printf(" distance : %d mm\r\n", (int)get_distance_mm());
    printf(" time     : %d ms\r\n", (int)(HAL_GetTick() - start_tick));
    printf(" L count  : %ld\r\n", left_count);
    printf(" R count  : %ld\r\n", right_count);
    printf(" diff     : %ld (%d %%)\r\n", diff, percent);
    printf("------------------------------\r\n");
    printf("Measure the sideways offset at the finish line.\r\n\r\n");
}

/* 선택한 방식으로 모터에 출력을 넣는 내부 함수 */
static void apply_drive(void)
{
    if (use_pid)
    {
        /* 폐루프 : 목표 RPM 만 주고 출력은 PID 가 결정한다 */
        float target = use_high ? HIGH_SPEED_RPM : LOW_SPEED_RPM;

        wheel_set_enabled(true);
        wheel_set_target_rpm_both(target, target);
    }
    else
    {
        /* 개루프 : 좌우에 똑같은 듀티를 주고 그대로 둔다.
         * wheel 제어를 꺼야 PID 가 이 출력을 덮어쓰지 않는다 */
        int16_t duty = use_high ? HIGH_SPEED_DUTY : LOW_SPEED_DUTY;

        wheel_set_enabled(false);
        motor_set_output(MOTOR_LEFT,  duty);
        motor_set_output(MOTOR_RIGHT, duty);
    }
}

/* 주행을 시작한다. 카운트를 초기화하고 모터를 돌린다. */
static void start_run(void)
{
    /* 제어를 먼저 끄고 초기화해야 이전 목표값이 남지 않는다 */
    wheel_set_enabled(false);
    wheel_reset();
    encoder_reset();

    start_tick = HAL_GetTick();
    run_state  = RUN_ACTIVE;

    printf("\r\n>>> START\r\n");
    print_mode();

    apply_drive();
}

/* 주행을 멈추고 결과를 출력한다. */
static void finish_run(void)
{
    /* PID 가 다시 출력을 내지 않도록 먼저 끈다 */
    wheel_set_enabled(false);
    motor_stop_all();

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
                wheel_set_enabled(false);
                motor_stop_all();
            }
            break;

        case 'o':
        case 'O':
            use_pid = false;
            print_mode();
            break;

        case 'c':
        case 'C':
            use_pid = true;
            print_mode();
            break;

        case '1':
            use_high = false;
            print_mode();
            break;

        case '2':
            use_high = true;
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


/* 애플리케이션에서 사용하는 모듈들을 초기화한다. */
void ap_init(void)
{
    wheel_init();

    /* 초기 상태는 개루프이므로 PID 가 모터를 건드리지 않게 꺼 둔다 */
    wheel_set_enabled(false);
    motor_stop_all();
}

/* 직진 주행 시험용 메인 루프.
 * 명령을 받아 주행을 제어하고 일정 주기로 상태를 출력한다. */
void ap_main(void)
{
    uint32_t control_tick = HAL_GetTick();
    uint32_t print_tick   = HAL_GetTick();

    print_help();
    print_mode();

    while (1)
    {
        poll_uart();

        /* 일정 주기로 엔코더를 갱신한다.
         * 개루프에서도 측정은 계속해야 비교가 가능하다 */
        if ((HAL_GetTick() - control_tick) >= CONTROL_PERIOD_MS)
        {
            control_tick += CONTROL_PERIOD_MS;

            wheel_update(CONTROL_PERIOD_MS);

            /* 목표 거리에 도달하면 스스로 멈춘다 */
            if ((run_state == RUN_ACTIVE) &&
                (get_distance_mm() >= (float)TARGET_DISTANCE_MM))
            {
                printf("\r\n>>> TARGET REACHED\r\n");
                finish_run();
            }
        }

        /* 주행 중에만 상태를 출력한다 */
        if ((run_state == RUN_ACTIVE) &&
            ((HAL_GetTick() - print_tick) >= PRINT_PERIOD_MS))
        {
            print_tick += PRINT_PERIOD_MS;
            print_status();
        }
    }
}