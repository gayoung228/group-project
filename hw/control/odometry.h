#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <stdint.h>

typedef struct
{
    float wheel_diameter_mm;  // 바퀴 지름
    uint32_t pulses_per_revolution;  // 바퀴 1회전당 엔코더 펄스 수
} odometry_config_t;

void odometry_init(void);  // 엔코더 기준값과 이동 거리 계산값을 초기화

void odometry_reset(void);  // 누적 이동 거리와 이전 엔코더 값을 0으로 초기화

void odometry_update(void);  // 좌우 엔코더 변화량을 이용해 이동 거리를 갱신

void odometry_set_config(const odometry_config_t *config);  // 바퀴 지름과 회전당 펄스 수를 설정

float odometry_get_left_distance_mm(void);  // 왼쪽 바퀴의 누적 이동 거리를 반환

float odometry_get_right_distance_mm(void);  // 오른쪽 바퀴의 누적 이동 거리를 반환

float odometry_get_displacement_mm(void);  // 시작 위치 기준 부호 있는 진행 거리를 반환

float odometry_get_traveled_distance_mm(void);  // 회전을 제외한 누적 주행 거리를 반환

float odometry_get_left_speed_mm_s(void);  // 왼쪽 바퀴의 현재 속도를 반환

float odometry_get_right_speed_mm_s(void);  // 오른쪽 바퀴의 현재 속도를 반환

#endif