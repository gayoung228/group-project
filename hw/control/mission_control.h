#ifndef MISSION_CONTROL_H
#define MISSION_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    MISSION_STATE_IDLE,                  // 목표 거리가 설정되지 않은 대기 상태
    MISSION_STATE_READY,                 // 목표 거리가 설정되어 출발 가능한 상태
    MISSION_STATE_DRIVING,               // 목표 거리까지 일반 주행 중인 상태
    MISSION_STATE_OBSTACLE_AVOIDANCE,    // 장애물 회피 제어 중인 상태
    MISSION_STATE_COMPLETED,             // 목표 거리 주행 완료 상태
    MISSION_STATE_STOPPED,               // 사용자 또는 외부 명령으로 정지한 상태
    MISSION_STATE_ERROR                  // 센서나 제어 오류가 발생한 상태
} mission_state_t;

void mission_control_init(void);  // 목표 거리와 전체 주행 상태를 초기화

void mission_control_reset(void);  // 현재 임무와 누적 거리를 초기 상태로 복구

bool mission_control_set_target_distance(uint32_t target_distance_mm);  // 차량이 주행할 목표 거리를 mm 단위로 설정

bool mission_control_start(void);  // 설정된 목표 거리까지 자율주행을 시작

void mission_control_update(void);  // 센서와 제어 상태를 확인하고 최종 모터 출력을 결정

void mission_control_stop(void);  // 현재 주행을 중단하고 차량을 정지

void mission_control_emergency_stop(void);  // 위험 상황에서 즉시 모터를 정지하고 오류 상태로 전환

mission_state_t mission_control_get_state(void);  // 현재 전체 주행 상태를 반환

uint32_t mission_control_get_target_distance(void);  // 설정된 목표 이동 거리를 반환

uint32_t mission_control_get_traveled_distance(void);  // 현재까지 누적된 실제 주행 거리를 반환

uint32_t mission_control_get_remaining_distance(void);  // 목표 지점까지 남은 거리를 반환

bool mission_control_is_completed(void);  // 목표 거리 주행 완료 여부를 반환

#endif