#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

typedef enum{
    MOTOR_LEFT = 0,
    MOTOR_RIGHT
} motor_t;

typedef enum{
    MOTOR_STOP = 0,
    MOTOR_FORWARD,
    MOTOR_REVERSE
} motor_direction_t;

/* 모터 제어 초기화
   - PWM 시작
   - 필요 시 GPIO 초기 상태 설정 */
void Motor_Init(void);

/* 개별 모터 방향 설정

   motor     : MOTOR_LEFT / MOTOR_RIGHT
   direction : MOTOR_FORWARD / MOTOR_REVERSE / MOTOR_STOP  */
void Motor_SetDirection(motor_t motor, motor_direction_t direction);


/* 개별 모터 속도 설정
  
   speed 범위 : 0 ~ 100 (%)
   0          : 정지
   100        : 최대 속도 */
void Motor_SetSpeed(motor_t motor, uint8_t speed);

// 모터 방향 + 속도를 한 번에 설정
void Motor_Control(motor_t motor, motor_direction_t direction, uint8_t speed);

//좌/우 모터 모두 정지
void Motor_StopAll(void);

#endif