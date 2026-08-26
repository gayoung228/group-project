#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>
#include "drive_command.h"

// 주행 모듈 초기화
void drive_init(void);

// 차량 주행 명령 실행
void drive_control(drive_command_t command);

// 양쪽 바퀴에 적용할 기본 속도를 0~100 범위로 설정
void drive_set_speed(uint8_t speed);

// 좌우 바퀴 속도를 -100~100 범위로 각각 설정
void drive_set_wheel_speed(int16_t left_speed, int16_t right_speed);

void drive_forward(void);  // 설정된 속도로 차량을 전진
void drive_backward(void);
void drive_left(void);
void drive_right(void);
void drive_stop(void);
void drive_speed_up(void);
void drive_speed_down(void);

uint8_t drive_get_speed(void);  // 현재 설정된 기본 주행 속도를 반환

int16_t drive_get_left_speed(void);  // 현재 왼쪽 바퀴에 적용된 속도를 반환

int16_t drive_get_right_speed(void);  // 현재 오른쪽 바퀴에 적용된 속도를 반환

drive_command_t drive_get_command(void);  // 현재 실행 중인 주행 명령을 반환

#endif