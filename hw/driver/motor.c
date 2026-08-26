#include "motor.h"
#include "main.h"

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

// 선택한 모터의 PWM 타이머를 반환한다.
static TIM_HandleTypeDef *motor_get_timer(motor_t motor)
{
    if (motor == MOTOR_LEFT)
    {
        return &htim2;
    }

    if (motor == MOTOR_RIGHT)
    {
        return &htim3;
    }

    return NULL;
}

// 선택한 모터의 PWM 채널을 반환한다.
static uint32_t motor_get_channel(motor_t motor)
{
    if (motor == MOTOR_LEFT)
    {
        return TIM_CHANNEL_3;
    }

    if (motor == MOTOR_RIGHT)
    {
        return TIM_CHANNEL_2;
    }

    return 0;
}

// 좌우 모터의 방향 핀을 정지 상태로 만들고 PWM을 시작한다.
void motor_init(void)
{
    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_RESET);

    motor_set_direction(MOTOR_LEFT, MOTOR_STOP);
    motor_set_direction(MOTOR_RIGHT, MOTOR_STOP);

    __HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 0);
    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);

    if (HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2) != HAL_OK)
    {
        Error_Handler();
    }

    HAL_GPIO_WritePin(MOTOR_STBY_GPIO_Port, MOTOR_STBY_Pin, GPIO_PIN_SET);
}

// 선택한 모터의 회전 방향을 설정한다.
void motor_set_direction(motor_t motor, motor_direction_t direction)
{
    GPIO_TypeDef *in1_port;
    GPIO_TypeDef *in2_port;
    uint16_t in1_pin;
    uint16_t in2_pin;

    if (motor == MOTOR_LEFT)
    {
        in1_port = MOTOR_L_IN1_GPIO_Port;
        in1_pin = MOTOR_L_IN1_Pin;
        in2_port = MOTOR_L_IN2_GPIO_Port;
        in2_pin = MOTOR_L_IN2_Pin;
    }
    else if (motor == MOTOR_RIGHT)
    {
        in1_port = MOTOR_R_IN1_GPIO_Port;
        in1_pin = MOTOR_R_IN1_Pin;
        in2_port = MOTOR_R_IN2_GPIO_Port;
        in2_pin = MOTOR_R_IN2_Pin;
    }
    else
    {
        return;
    }

    if (direction == MOTOR_FORWARD)
    {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    }
    else if (direction == MOTOR_REVERSE)
    {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_SET);
    }
    else
    {
        HAL_GPIO_WritePin(in1_port, in1_pin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(in2_port, in2_pin, GPIO_PIN_RESET);
    }
}

// 선택한 모터의 PWM 듀티를 0~100% 범위로 설정한다.
void motor_set_speed(motor_t motor, uint8_t speed)
{
    TIM_HandleTypeDef *timer = motor_get_timer(motor);
    uint32_t channel = motor_get_channel(motor);
    uint32_t period;
    uint32_t pulse;

    if ((timer == NULL) || (channel == 0))
    {
        return;
    }

    if (speed > 100)
    {
        speed = 100;
    }

    period = __HAL_TIM_GET_AUTORELOAD(timer) + 1U;
    pulse = (period * speed) / 100U;

    __HAL_TIM_SET_COMPARE(timer, channel, pulse);
}

// 선택한 모터의 방향과 속도를 함께 설정한다.
void motor_control(motor_t motor, motor_direction_t direction, uint8_t speed)
{
    motor_set_speed(motor, 0);

    if ((direction == MOTOR_STOP) || (speed == 0))
    {
        motor_set_direction(motor, MOTOR_STOP);
        return;
    }

    motor_set_direction(motor, direction);
    motor_set_speed(motor, speed);
}

// 좌우 모터의 PWM과 방향을 모두 정지 상태로 만든다.
void motor_stop_all(void)
{
    motor_set_speed(MOTOR_LEFT, 0);
    motor_set_speed(MOTOR_RIGHT, 0);
    motor_set_direction(MOTOR_LEFT, MOTOR_STOP);
    motor_set_direction(MOTOR_RIGHT, MOTOR_STOP);
}
