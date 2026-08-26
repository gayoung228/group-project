#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <stdbool.h>
#include <stdint.h>

typedef enum
{
    AVOID_DIRECTION_NONE,     // 회피 방향이 결정되지 않은 상태
    AVOID_DIRECTION_LEFT,     // 왼쪽 방향으로 장애물 회피
    AVOID_DIRECTION_RIGHT     // 오른쪽 방향으로 장애물 회피
} avoid_direction_t;

typedef enum
{
    AVOID_STATE_IDLE,              // 장애물 회피 대기
    AVOID_STATE_STOP,              // 장애물 앞에서 정지
    AVOID_STATE_CHECK_SPACE,       // 좌우 공간 확인
    AVOID_STATE_FIRST_TURN,        // 회피 방향으로 첫 번째 90도 회전
    AVOID_STATE_FORWARD,           // 장애물 옆으로 설정 거리만큼 전진
    AVOID_STATE_SECOND_TURN,       // 원래 진행 방향으로 두 번째 90도 회전
    AVOID_STATE_COMPLETED,         // 장애물 회피 완료
    AVOID_STATE_FAILED             // 장애물 회피 실패
} avoid_state_t;

typedef struct
{
    uint16_t obstacle_distance_mm;    // 장애물로 판단할 전방 거리
    uint16_t bypass_distance_mm;      // 회피 방향으로 전진할 거리
    uint8_t forward_speed;            // 회피 중 전진 속도
    uint8_t turn_speed;               // 회피 중 회전 속도
    float turn_angle_deg;             // 한 번 회전할 목표 각도
} obstacle_avoidance_config_t;

void obstacle_avoidance_init(void); // 장애물 회피 상태 머신을 초기화

void obstacle_avoidance_reset(void);  // 진행 중인 장애물 회피를 취소하고 대기 상태로 전환

void obstacle_avoidance_set_config(const obstacle_avoidance_config_t *config);  // 장애물 거리와 회피 동작 설정값을 변경

  // 좌우 거리 중 공간이 더 넓은 회피 방향을 반환
avoid_direction_t obstacle_avoidance_select_direction(uint16_t left_distance_mm,uint16_t right_distance_mm);

// 선택한 방향으로 장애물 회피 동작을 시작
bool obstacle_avoidance_start(avoid_direction_t direction, float current_heading_deg, float current_distance_mm);  


// 현재 방향과 이동 거리를 이용해 회피 상태를 갱신
void obstacle_avoidance_update(float current_heading_deg, float current_distance_mm, uint16_t front_distance_mm); 

avoid_state_t obstacle_avoidance_get_state(void);  // 현재 장애물 회피 단계를 반환

avoid_direction_t obstacle_avoidance_get_direction(void);  // 현재 선택된 회피 방향을 반환

int16_t obstacle_avoidance_get_left_speed(void);  // 회피 동작에 필요한 왼쪽 모터 목표 속도를 반환

int16_t obstacle_avoidance_get_right_speed(void);  // 회피 동작에 필요한 오른쪽 모터 목표 속도를 반환

bool obstacle_avoidance_is_running(void);  // 장애물 회피 동작이 진행 중인지 반환

bool obstacle_avoidance_is_completed(void);  // 장애물 회피가 정상적으로 완료됐는지 반환

bool obstacle_avoidance_has_failed(void);  // 장애물 회피가 실패했는지 반환

#endif