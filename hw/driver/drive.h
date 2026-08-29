#ifndef DRIVE_H
#define DRIVE_H

#include <stdbool.h>
#include <stdint.h>
#include "rover_config.h"

/* 주행 속도 단계 [PWM %]
 * 80 아래에서는 차가 아예 움직이지 않으므로 이 범위 안에서만 쓴다. */
#define DRIVE_SPEED_SLOW      ROVER_MOTOR_MIN_OUTPUT
#define DRIVE_SPEED_NORMAL    ROVER_MOTOR_NORMAL_OUTPUT
#define DRIVE_SPEED_FAST      ROVER_MOTOR_MAX_OUTPUT

/* 실측으로 정한 제어 가능 RPM 범위 */
#define DRIVE_RPM_MIN         ROVER_DRIVE_RPM_MIN
#define DRIVE_RPM_MAX         ROVER_DRIVE_RPM_MAX

// 주행 모듈을 초기화하고 MPU6050까지 준비됐는지 반환
bool drive_init(void);

// 정지 상태에서 MPU6050 초기화와 영점 보정을 다시 시도
bool drive_retry_heading_init(void);

// 모터 PWM과 좌우 엔코더 입력 캡처를 정지했다가 다시 초기화한다.
bool drive_retry_motion_hardware(void);

// 모터 주행에 필요한 MPU6050/방향 제어가 준비됐는지 반환
bool drive_is_ready(void);

// 초기화 로그에서 각 장치의 상태를 따로 확인할 때 사용한다.
bool drive_is_motor_ready(void);
bool drive_is_left_encoder_ready(void);
bool drive_is_right_encoder_ready(void);
bool drive_is_heading_ready(void);

// 사용자가 새 출발을 명령했을 때 이전 모터 오류와 PID 상태를 초기화한다.
void drive_prepare_start(void);

// 제어 주기마다 호출한다. 방향 제어와 바퀴 속도 제어를 갱신한다.
void drive_update(uint32_t elapsed_time_ms);

// 양쪽 바퀴에 적용할 기본 속도를 0~100 범위로 설정
void drive_set_speed(uint8_t speed);

// 내부 속도 단계(80~100)를 사람이 비교하기 쉬운 목표 RPM으로 변환한다.
float drive_speed_to_rpm(uint8_t speed);

// 주행 단계값은 유지하면서 방지턱 같은 일시적 목표 RPM을 적용한다.
void drive_set_forward_target_rpm(float target_rpm);

/* 장애물 회피처럼 좌우 출력을 직접 정해야 할 때도 motor를 우회하지 않고
 * drive를 통해 -100~100 출력 명령을 적용한다. */
void drive_set_direct_output(int16_t left_output, int16_t right_output);

// 현재 위치에서 지정한 절대 Yaw 목표로 제자리 회전을 시작한다.
void drive_recover_heading(float target_heading_deg);

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

// 현재 drive가 좌우 직접 출력 모드인지 반환
bool drive_is_direct_mode(void);

#endif
