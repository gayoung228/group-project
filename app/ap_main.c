#include "main.h"
#include "ap_main.h"
#include "drive.h"
#include "encoder.h"
#include <stdio.h>

/* 엔코더 측정 주기 [ms] */
#define ENCODER_PERIOD_MS   100

/* 화면 출력 주기 [ms] */
#define PRINT_PERIOD_MS     500

/* 테스트 주행 속도 [%] */
#define TEST_SPEED          100

extern UART_HandleTypeDef huart2;


/* printf 를 UART2 로 내보내기 위한 함수 */
int __io_putchar(int ch)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 100);
    return ch;
}

/* 애플리케이션에서 사용하는 모듈들을 초기화한다. */
void ap_init(void)
{
    drive_init();
    encoder_init();
}

/* 엔코더 동작 확인용 메인 루프.
 * 처음 10초는 바퀴를 손으로 돌려 카운트를 확인하고,
 * 그 뒤에는 모터를 돌려 좌우 RPM 을 비교한다. */
void ap_main(void)
{
    uint32_t encoder_tick = HAL_GetTick();
    uint32_t print_tick   = HAL_GetTick();
    uint32_t start_tick   = HAL_GetTick();
    uint8_t  motor_started = 0;

    printf("\r\n=== Encoder Test ===\r\n");
    printf("10 sec: turn wheels by hand\r\n\r\n");

    encoder_reset();

    while (1)
    {
        /* 10초가 지나면 모터를 켠다 */
        if ((motor_started == 0) && ((HAL_GetTick() - start_tick) >= 10000))
        {
            motor_started = 1;
            printf("\r\n--- motor start ---\r\n\r\n");

            encoder_reset();
            drive_forward(TEST_SPEED);
        }

        /* 일정 주기로 RPM 을 갱신한다 */
        if ((HAL_GetTick() - encoder_tick) >= ENCODER_PERIOD_MS)
        {
            encoder_tick += ENCODER_PERIOD_MS;
            encoder_update(ENCODER_PERIOD_MS);
        }

        /* 일정 주기로 측정값을 출력한다 */
        if ((HAL_GetTick() - print_tick) >= PRINT_PERIOD_MS)
        {
            print_tick += PRINT_PERIOD_MS;

            printf("L cnt:%6ld  rpm:%6d  run:%d   |   R cnt:%6ld  rpm:%6d  run:%d\r\n",
                   encoder_get_count(ENCODER_LEFT),
                   (int)encoder_get_rpm(ENCODER_LEFT),
                   encoder_is_running(ENCODER_LEFT),
                   encoder_get_count(ENCODER_RIGHT),
                   (int)encoder_get_rpm(ENCODER_RIGHT),
                   encoder_is_running(ENCODER_RIGHT));
        }
    }
}