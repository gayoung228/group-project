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
    MOTOR_STOP = 0, // 모터 정지
    MOTOR_FORWARD, // 모터 정방향 회전
    MOTOR_REVERSE // 모터 역방향 회전
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

// 부호를 포함한 -100~100 값으로 모터 방향과 속도를 함께 설정
void motor_set_output(motor_t motor, int16_t output);

// 선택한 모터 하나를 정지
void motor_stop(motor_t motor);

// 선택한 모터에 설정된 속도를 반환
uint8_t motor_get_speed(motor_t motor);

// 선택한 모터에 설정된 회전 방향을 반환
motor_direction_t motor_get_direction(motor_t motor);

#endif