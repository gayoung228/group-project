#include "motor.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;

#define MOTOR_PWM_MAX htim2.Init.Period // ARR?

// 일단 왼쪽만
void Motor_Init(void) {
  HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);

  // Motor stop
  HAL_GPIO_WritePin(MOTOR_A_IN1_GPIO_Port, MOTOR_A_IN1_Pin, GPIO_PIN_RESET);

  HAL_GPIO_WritePin(MOTOR_A_IN2_GPIO_Port, MOTOR_A_IN2_Pin, GPIO_PIN_RESET);

  // start PWM
  HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);

  // set Duty as 0
  __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
}

// 일단 왼쪽만
// speed 0 ~ 100
void Motor_SetSpeed(motor_t motor, uint8_t speed)
{
    if (speed > 100)
        speed = 100;

    HAL_GPIO_WritePin(
        MOTOR_A_IN1_GPIO_Port,
        MOTOR_A_IN1_Pin,
        GPIO_PIN_SET
    );

    HAL_GPIO_WritePin(
        MOTOR_A_IN2_GPIO_Port,
        MOTOR_A_IN2_Pin,
        GPIO_PIN_RESET
    );

    uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim2) + 1;
    uint32_t pwm = (period * speed) / 100;

    __HAL_TIM_SET_COMPARE(
        &htim2,
        TIM_CHANNEL_3,
        pwm
    );
}

void motor_stop(uint8_t motor);

void motor_stop_all(void);