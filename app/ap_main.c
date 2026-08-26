#include "ap_main.h"
#include "motor.h"
#include "stm32f4xx_hal.h"

void ap_init(void) {
    motor_init();
}

void ap_main(void) {
    // 약 70% 정도부터 돌기 시작
    motor_set_speed(MOTOR_LEFT, 70);
    HAL_Delay(10000);
    motor_set_speed(MOTOR_LEFT, 100);
    HAL_Delay(10000);
}