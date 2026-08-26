#ifndef SPEED_CONTROL_H
#define SPEED_CONTROL_H

#include <stdint.h>

#include "road_state.h"

typedef struct{
    uint8_t flat_speed;  // 평지에서 사용할 목표 속도
    uint8_t small_vibration_speed;  // 작은 진동 노면에서 사용할 목표 속도
    uint8_t large_vibration_speed;  // 큰 진동 노면에서 사용할 목표 속도
} speed_control_config_t;

void speed_control_init(void);  // 노면 적응 속도 제어 모듈을 초기화

void speed_control_reset(void);  // 목표 속도를 기본 평지 속도로 초기화

void speed_control_update(road_state_t road_state); // 현재 노면 상태에 따라 목표 속도를 갱신

// 노면 단계별 목표 속도를 설정
void speed_control_set_config(const speed_control_config_t *config);    

uint8_t speed_control_get_target_speed(void);  // 현재 노면에 맞는 목표 속도를 반환

#endif