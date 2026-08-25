#ifndef DRIVE_H
#define DRIVE_H

#include <stdint.h>
#include "drive_command.h"

// 주행 모듈 초기화
void drive_init(void);

void drive_forward(void);
void drive_backward(void);
void drive_left(void);
void drive_right(void);
void drive_stop(void);
void drive_speed_up(void);
void drive_speed_down(void);

// 차량 주행 명령 실행
void drive_control(drive_command_t command);

// 기본 주행 속도 설정
void drive_set_speed(uint8_t speed);

#endif