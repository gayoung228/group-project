#ifndef HEADING_H
#define HEADING_H

#include <stdbool.h>
#include <stdint.h>

/* ------------------------------------------------------------------
 * heading.h - 자이로 기반 방향 유지 제어
 *
 * MPU6050 의 Yaw 각도를 읽어 기준 방향과의 오차를 PID 로 계산하고,
 * 좌우 바퀴의 목표 RPM 에 반대 부호로 나눠 실어 방향을 잡는다.
 *
 *   왼쪽  목표 = 기본 속도 - 보정량
 *   오른쪽 목표 = 기본 속도 + 보정량
 *
 * 기본 속도를 0 으로 두면 보정량만 남아 제자리 회전이 된다.
 * 즉 직진 유지와 회전이 같은 방식으로 처리된다.
 * ------------------------------------------------------------------ */

// MPU6050 을 시작하고 방향 제어 상태를 준비한다.
bool heading_init(void);

// 현재 향하고 있는 방향을 기준 방향(0도)으로 다시 잡는다.
void heading_reset(void);

// 제어 주기마다 호출한다. 자이로를 읽고 좌우 목표 RPM 을 갱신한다.
void heading_update(uint32_t elapsed_time_ms);

// 직진 기본 속도를 RPM 으로 설정한다. (전진 +, 후진 -, 제자리 회전 0)
void heading_set_base_rpm(float base_rpm);

// 설정된 직진 기본 속도를 반환한다.
float heading_get_base_rpm(void);

// 기준 방향을 현재 기준에서 상대 각도만큼 돌린다. (좌회전 +, 우회전 -)
void heading_rotate(float delta_deg);

// 기준 방향을 절대 각도로 설정한다.
void heading_set_target(float target_deg);

// 현재 기준 방향을 반환한다.
float heading_get_target(void);

// 자이로가 측정한 현재 방향을 반환한다.
float heading_get_current(void);

/* 차량 자세를 X=Roll, Y=Pitch, Z=Yaw로 반환한다.
 * 표시용 Z축은 오른쪽 회전이 +, 왼쪽 회전이 -이며 모두 -180~180도 범위다. */
bool heading_get_pose(float *x_deg, float *y_deg, float *z_deg);

// 기준 방향과 현재 방향의 차이를 반환한다.
float heading_get_error(void);

// PID 가 계산한 좌우 보정량을 RPM 으로 반환한다.
float heading_get_correction(void);

// 방향 제어에 사용할 PID 게인을 설정한다.
void heading_set_gain(float kp, float ki, float kd);

// 방향 제어를 켜거나 끈다. 끄면 좌우에 기본 속도만 그대로 내려간다.
void heading_set_enabled(bool enabled);

// 방향 제어가 켜져 있는지 반환한다.
bool heading_is_enabled(void);

// MPU6050 초기화와 가장 최근 자세 갱신이 정상인지 반환한다.
bool heading_is_sensor_ready(void);

// 직진 중 Yaw 오차가 안전 한계를 넘었는지 반환
bool heading_has_runaway_fault(void);

// 직진 중 45도 초과 안전정지를 켜거나 끈다. 방지턱 복구 중에만 잠시 끈다.
void heading_set_runaway_protection(bool enabled);

// 현재 방향이 기준 방향에 충분히 가까운지 반환한다.
bool heading_is_aligned(void);

// 주행을 멈추고 PID 누적 상태를 초기화한다. 기준 방향은 유지한다.
void heading_stop(void);

#endif
