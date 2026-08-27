#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>

/* 주행 속도 단계 [PWM %]
 * 80 아래에서는 차가 아예 움직이지 않으므로 이 범위 안에서만 쓴다. */
#define DRIVE_SPEED_SLOW      80    // 감속 (거친 노면)
#define DRIVE_SPEED_NORMAL    90    // 기본
#define DRIVE_SPEED_FAST     100    // 가속 (평탄한 노면)

// 주행 모듈 초기화
void drive_init(void);

// 제어 주기마다 호출한다. 방향 제어와 바퀴 속도 제어를 갱신한다.
void drive_update(uint32_t elapsed_time_ms);

// 양쪽 바퀴에 적용할 기본 속도를 0~100 범위로 설정
void drive_set_speed(uint8_t speed);

void drive_forward(uint8_t speed);  // 설정된 속도로 차량을 전진

void drive_backward(uint8_t speed);

void drive_turn_left(uint8_t speed);

void drive_turn_right(uint8_t speed);

void drive_stop(void);

uint8_t drive_get_speed(void);  // 현재 설정된 기본 주행 속도를 반환

int16_t drive_get_left_speed(void);  // 현재 왼쪽 바퀴에 적용된 속도를 반환

int16_t drive_get_right_speed(void);  // 현재 오른쪽 바퀴에 적용된 속도를 반환

#endif