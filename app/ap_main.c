#include "ap_main.h"
#include "motor.h"
#include "stm32f4xx_hal.h"

void apInit(void) {
    Motor_Init();
}

void apMain(void) {
    // 약 70% 정도부터 돌기 시작
    Motor_SetSpeed(MOTOR_LEFT, 70);
    HAL_Delay(10000);
    Motor_SetSpeed(MOTOR_LEFT, 100);
    HAL_Delay(10000);
}