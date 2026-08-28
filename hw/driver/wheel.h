#ifndef WHEEL_H
#define WHEEL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    WHEEL_LEFT = 0,   // 왼쪽 바퀴
    WHEEL_RIGHT       // 오른쪽 바퀴
} wheel_t;

// 모터와 엔코더를 초기화하고 좌우 바퀴의 PID 상태를 준비한다.
bool wheel_init(void);

// 좌우 바퀴의 목표 RPM과 PID 누적 상태를 모두 초기화한다.
void wheel_reset(void);

// 제어 주기마다 호출한다. 엔코더를 갱신하고 PID로 모터 출력을 계산한다.
void wheel_update(uint32_t elapsed_time_ms);

// 선택한 바퀴의 목표 회전 속도를 RPM으로 설정한다. (전진 +, 후진 -)
void wheel_set_target_rpm(wheel_t wheel, float target_rpm);

// 좌우 바퀴의 목표 회전 속도를 한 번에 설정한다.
void wheel_set_target_rpm_both(float left_rpm, float right_rpm);

// 선택한 바퀴에 설정된 목표 RPM을 반환한다.
float wheel_get_target_rpm(wheel_t wheel);

// 선택한 바퀴의 실측 RPM을 반환한다. (전진 +, 후진 -)
float wheel_get_rpm(wheel_t wheel);

// 선택한 바퀴의 목표 RPM과 실측 RPM의 차이를 반환한다.
float wheel_get_error(wheel_t wheel);

// PID가 계산해 모터에 적용한 출력값을 반환한다. (-100~100)
int16_t wheel_get_output(wheel_t wheel);

// 좌우 바퀴에 공통으로 적용할 PID 게인을 설정한다.
void wheel_set_gain(float kp, float ki, float kd);

// 폐루프 제어를 켜거나 끈다. 끄면 모터 출력을 직접 지정할 수 있다.
void wheel_set_enabled(bool enabled);

// 폐루프 제어가 켜져 있는지 반환한다.
bool wheel_is_enabled(void);

// 선택한 바퀴가 목표 RPM에 도달했는지 반환한다.
bool wheel_is_reached(wheel_t wheel);

// 출발 시간 내에 양쪽 엔코더가 모두 감지되지 않았는지 반환
bool wheel_has_startup_fault(void);

// 주행 중 저RPM 5초 감지로 시동 시퀀스를 재실행한 횟수
uint32_t wheel_get_stall_restart_count(void);

// 좌우 바퀴를 모두 정지시키고 PID 누적 상태를 초기화한다.
void wheel_stop(void);

#endif
