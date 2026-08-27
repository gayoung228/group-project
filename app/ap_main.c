#include "main.h"
#include "ap_main.h"
#include "drive.h"
#include "mpu6050.h"
#include "stm32f4xx_hal.h"

/* 기본 주행 속도 [%] */
#define DRIVE_SPEED         80

/* 회전 속도 [%] : 제자리 회전은 마찰이 커서 직진보다 높게 준다 */
#define TURN_SPEED          100

/* 동작 사이의 대기 시간 [ms] */
#define ACTION_TIME         1500
#define TURN_TIME           700
#define PAUSE_TIME          500


/* 애플리케이션에서 사용하는 모듈들을 초기화한다. */
void ap_init(void) {
    drive_init();
    mpu6050_start();
}

/* 애플리케이션 메인 루프.
 * 전진 / 후진 / 좌회전 / 우회전 / 정지와 PWM 속도 변화를 순서대로 확인한다. */
void ap_main(void) {
    while (1)
    {
        /* 전진 */
        drive_forward(TURN_SPEED);
        HAL_Delay(ACTION_TIME);

        drive_stop();
        HAL_Delay(PAUSE_TIME);

        /* 후진 */
        drive_backward(TURN_SPEED);
        HAL_Delay(ACTION_TIME);

        drive_stop();
        HAL_Delay(PAUSE_TIME);

        /* 좌회전 */
        drive_turn_left(TURN_SPEED);
        HAL_Delay(TURN_TIME);

        drive_stop();
        HAL_Delay(PAUSE_TIME);

        /* 우회전 */
        drive_turn_right(TURN_SPEED);
        HAL_Delay(TURN_TIME);

        drive_stop();
        HAL_Delay(2000);
        //MPU6050에서 최신 가속도와 자이로 데이터를 읽음 
        mpu6050_update();
    }
}
