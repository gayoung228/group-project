#include "ap_main.h"
#include "motor.h"
#include "stm32f4xx_hal.h"

void ap_init(void)
{
    motor_init();
}

void ap_main(void)
{
    // 왼쪽 모터 30%
    motor_control(MOTOR_LEFT, MOTOR_FORWARD, 30);
    HAL_Delay(3000);
    motor_stop_all();
    HAL_Delay(1000);

    // 왼쪽 모터 60%
    motor_control(MOTOR_LEFT, MOTOR_FORWARD, 60);
    HAL_Delay(3000);
    motor_stop_all();
    HAL_Delay(1000);

    // 오른쪽 모터 30%
    motor_control(MOTOR_RIGHT, MOTOR_FORWARD, 30);
    HAL_Delay(3000);
    motor_stop_all();
    HAL_Delay(1000);

    // 오른쪽 모터 60%
    motor_control(MOTOR_RIGHT, MOTOR_FORWARD, 60);
    HAL_Delay(3000);
    motor_stop_all();
    HAL_Delay(1000);

    // 양쪽 모터 50%
    motor_control(MOTOR_LEFT, MOTOR_FORWARD, 50);
    motor_control(MOTOR_RIGHT, MOTOR_FORWARD, 50);
    HAL_Delay(3000);

    // 테스트 종료
    motor_stop_all();
}
