#ifndef DRIVE_H
#define DRIVE_H

#include <stdbool.h>
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

// 제자리에서 지정한 각도만큼 회전한다. (좌회전 +, 우회전 -)
void drive_rotate(float delta_deg);

// 현재 방향이 목표 방향에 도달했는지 반환
bool drive_is_aligned(void);

// 현재 방향 기준을 지금 향한 방향으로 다시 잡는다.
void drive_reset_heading(void);

// 누적 주행 거리 측정을 0으로 초기화
void drive_reset_distance(void);

// 좌우 평균 누적 주행 거리를 mm 단위로 반환
float drive_get_distance_mm(void);

uint8_t drive_get_speed(void);  // 현재 설정된 기본 주행 속도를 반환

int16_t drive_get_left_speed(void);  // 현재 왼쪽 바퀴에 적용된 속도를 반환

int16_t drive_get_right_speed(void);  // 현재 오른쪽 바퀴에 적용된 속도를 반환

#endif