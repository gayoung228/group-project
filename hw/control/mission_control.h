#ifndef MISSION_CONTROL_H
#define MISSION_CONTROL_H

#include <stdbool.h>

/* 로버 전체에서 현재 제어권을 가진 주행 기능 */
typedef enum
{
    MISSION_STATE_IDLE = 0,
    MISSION_STATE_DRIVING,
    MISSION_STATE_OBSTACLE_AVOIDANCE,
    MISSION_STATE_BUMP_HEADING_RECOVERY,
    MISSION_STATE_STOPPED,
    MISSION_STATE_ERROR
} mission_state_t;

void mission_control_init(void);
void mission_control_start(void);
void mission_control_begin_avoidance(void);
void mission_control_begin_bump_recovery(void);
void mission_control_resume_driving(void);
void mission_control_stop(void);
void mission_control_emergency_stop(void);

mission_state_t mission_control_get_state(void);
bool mission_control_is_running(void);
bool mission_control_is_avoiding(void);

#endif
