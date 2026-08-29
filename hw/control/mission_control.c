#include "mission_control.h"

/* ------------------------------------------------------------------
 * mission_control.c - 로버 전체 주행 상태와 제어권 관리
 *
 * 센서 알고리즘이나 모터 계산은 하지 않는다. 여러 기능이 동시에 모터를
 * 제어하지 않도록 현재 주행 기능을 하나의 상태로만 관리한다.
 * ------------------------------------------------------------------ */

static mission_state_t mission_state = MISSION_STATE_IDLE;

void mission_control_init(void)
{
    mission_state = MISSION_STATE_IDLE;
}

void mission_control_start(void)
{
    mission_state = MISSION_STATE_DRIVING;
}

void mission_control_begin_avoidance(void)
{
    mission_state = MISSION_STATE_OBSTACLE_AVOIDANCE;
}

void mission_control_begin_bump_recovery(void)
{
    mission_state = MISSION_STATE_BUMP_HEADING_RECOVERY;
}

void mission_control_resume_driving(void)
{
    mission_state = MISSION_STATE_DRIVING;
}

void mission_control_stop(void)
{
    mission_state = MISSION_STATE_STOPPED;
}

void mission_control_emergency_stop(void)
{
    mission_state = MISSION_STATE_ERROR;
}

mission_state_t mission_control_get_state(void)
{
    return mission_state;
}

bool mission_control_is_running(void)
{
    return (mission_state == MISSION_STATE_DRIVING)
        || (mission_state == MISSION_STATE_OBSTACLE_AVOIDANCE)
        || (mission_state == MISSION_STATE_BUMP_HEADING_RECOVERY);
}

bool mission_control_is_avoiding(void)
{
    return (mission_state == MISSION_STATE_OBSTACLE_AVOIDANCE);
}
