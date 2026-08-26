#ifndef HEADING_CONTROL_H
#define HEADING_CONTROL_H

#include <stdbool.h>

typedef struct{
    float kp;                  // 방향 오차에 대한 비례 제어 
    float max_correction;      // 좌우 모터에 적용할 최대 보정값
} heading_control_config_t;

void heading_control_init(void);    // 방향 유지 제어값과 기준 방향을 초기화

void heading_control_reset(float initial_heading_deg);  // 현재 방향을 기준으로 누적 각도와 목표 방향을 초기화

// Z축 각속도를 적분하여 현재 방향과 보정값을 갱신
void heading_control_update(float gyro_z_dps, float delta_time_sec);  

void heading_control_set_target(float target_heading_deg); // 차량이 유지해야 할 목표 진행 방향을 설정

void heading_control_set_config(const heading_control_config_t *config); // 비례 게인과 최대 보정값을 설정

void heading_control_enable(void);  // 방향 유지 보정값 계산을 활성화

void heading_control_disable(void);  // 장애물 회피 등을 위해 방향 유지 보정을 비활성화

bool heading_control_is_enabled(void);  // 방향 유지 기능의 활성화 여부를 반환

float heading_control_get_current(void); // 자이로 적분으로 계산된 현재 방향을 반환

float heading_control_get_target(void); // 설정된 목표 진행 방향을 반환

float heading_control_get_error(void);  // 목표 방향과 현재 방향의 오차를 반환

float heading_control_get_correction(void);  // 좌우 모터에 적용할 방향 보정값을 반환

#endif