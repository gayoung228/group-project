#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef enum
{
    MOTOR_LEFT = 0,
    MOTOR_RIGHT
} motor_t;

typedef enum
{
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE
} motor_direction_t;

// 모터 방향 핀을 정지 상태로 만들고 좌우 PWM을 시작한다.
void motor_init(void);

// 선택한 모터의 회전 방향을 설정한다.
void motor_set_direction(motor_t motor, motor_direction_t direction);

// 선택한 모터의 속도를 0~100% 범위로 설정한다.
void motor_set_speed(motor_t motor, uint8_t speed);

// 선택한 모터의 방향과 속도를 함께 설정한다.
void motor_control(motor_t motor, motor_direction_t direction, uint8_t speed);

// 좌우 모터를 모두 정지한다.
void motor_stop_all(void);

#endif
