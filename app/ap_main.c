#include "ap_main.h"
#include "motor.h"
#include "stm32f4xx_hal.h"

void ap_init(void)
{
    motor_init();
}

void ap_main(void)
{
    // 양쪽 모터를 정방향 100%로 출발
    motor_control(MOTOR_LEFT,  MOTOR_FORWARD, 100);
    motor_control(MOTOR_RIGHT, MOTOR_FORWARD, 100);
    HAL_Delay(3000);

    // 양쪽 모터 속도를 90%부터 10%까지 단계적으로 감소
    for (int speed = 90; speed >= 10; speed -= 10)
    {
        motor_set_speed(MOTOR_LEFT,  speed);
        motor_set_speed(MOTOR_RIGHT, speed);

        HAL_Delay(3000);
    }

    // 양쪽 모터 정지
    motor_stop_all();
}