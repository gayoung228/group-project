#ifndef OBSTACLE_AVOIDANCE_H
#define OBSTACLE_AVOIDANCE_H

#include <stdbool.h>
#include <stdint.h>
#include "proximity_monitor.h"

/* 회피 방향은 이벤트를 시작할 때 한 번 선택해 고정(latch)한다. */
typedef enum
{
    AVOID_DIRECTION_NONE = 0,
    AVOID_DIRECTION_LEFT,
    AVOID_DIRECTION_RIGHT
} avoid_direction_t;

/* 부드러운 회피의 각 단계. 상태가 바뀔 때만 판단 기준도 함께 바뀌므로
 * 센서 노이즈가 곧바로 서로 반대되는 모터 명령을 만들지 않는다. */
typedef enum
{
    AVOID_STATE_IDLE = 0,
    AVOID_STATE_TURN_OUT,
    AVOID_STATE_CURVE_OUT,
    AVOID_STATE_FOLLOW_EDGE,
    AVOID_STATE_ALIGN_OPENING,
    AVOID_STATE_CLEAR_BODY,
    AVOID_STATE_RETURN_ORIGINAL,
    AVOID_STATE_SAFETY_STOP,
    AVOID_STATE_SAFETY_TURN,
    AVOID_STATE_SAFETY_ADVANCE,
    AVOID_STATE_COMPLETED,
    AVOID_STATE_FAILED
} avoid_state_t;

/* 실패 원인은 앱이 사용자 로그와 최종 정지 이유를 구분할 때 사용한다. */
typedef enum
{
    AVOID_FAILURE_NONE = 0,
    AVOID_FAILURE_NO_DIRECTION,
    AVOID_FAILURE_SENSOR,
    AVOID_FAILURE_ESCAPE_BLOCKED,
    AVOID_FAILURE_TIMEOUT,
    AVOID_FAILURE_MAX_TRAVEL
} avoid_failure_t;

/* 회피 모듈은 하드웨어를 직접 만지지 않고 이 명령만 계산한다.
 * TRACK_HEADING이면 drive가 자이로 목표를 바퀴 RPM PID로 변환한다. */
typedef enum
{
    AVOID_MOTION_STOP = 0,
    AVOID_MOTION_TRACK_HEADING,
    AVOID_MOTION_ROTATE_HEADING
} avoid_motion_t;

typedef struct
{
    avoid_motion_t motion;
    float base_rpm;
    float target_heading_deg;
} obstacle_avoidance_command_t;

/* 모든 상태·카운터·방향 래치를 초기화한다. 튜닝값은 rover_config.h를 사용한다. */
void obstacle_avoidance_init(void);

/* 진행 중인 회피를 취소하고 다음 장애물 감지 대기 상태로 돌아간다. */
void obstacle_avoidance_reset(void);

/* 최신 3센서 스냅샷, 자이로 Yaw, 누적 이동거리로 상태와 주행 명령을 갱신한다. */
void obstacle_avoidance_update(const proximity_snapshot_t *sensors,
                               float current_heading_deg,
                               float current_distance_mm,
                               uint32_t elapsed_time_ms);

/* 이번 호출 흐름에서 새 회피 이벤트가 시작됐을 때 한 번만 true를 반환한다. */
bool obstacle_avoidance_take_started(void);

/* 상태가 바뀌었을 때 한 번만 true를 반환하고 바뀐 현재 상태를 복사한다. */
bool obstacle_avoidance_take_state_change(avoid_state_t *state);

/* 앱이 drive에 전달할 현재 정지/자이로 추종 명령을 복사한다. */
void obstacle_avoidance_get_command(obstacle_avoidance_command_t *command);

/* 로그와 미션 제어에 필요한 읽기 전용 상태 접근 함수 */
avoid_state_t obstacle_avoidance_get_state(void);
avoid_direction_t obstacle_avoidance_get_direction(void);
avoid_failure_t obstacle_avoidance_get_failure(void);
bool obstacle_avoidance_is_running(void);
bool obstacle_avoidance_is_completed(void);
bool obstacle_avoidance_has_failed(void);
bool obstacle_avoidance_inside_obstacle_seen(void);
const char *obstacle_avoidance_state_name(avoid_state_t state);

#endif
