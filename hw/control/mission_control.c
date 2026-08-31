#include "mission_control.h"

/* ------------------------------------------------------------------
 * mission_control.c - 로버 전체 주행 상태와 제어권 관리
 *
 * 센서 알고리즘이나 모터 계산은 하지 않는다. 여러 기능이 동시에 모터를
 * 제어하지 않도록 현재 주행 기능을 하나의 상태로만 관리한다.
 * ------------------------------------------------------------------ */

static mission_state_t mission_state = MISSION_STATE_IDLE;

/* 전원 초기화 시 모터 제어권을 대기 상태로 만든다. */
void mission_control_init(void)
{
    mission_state = MISSION_STATE_IDLE;
}

/* 사용자가 START를 누르면 독립 이벤트를 감시하는 기본 직진을 시작한다. */
void mission_control_start(void)
{
    mission_state = MISSION_STATE_DRIVING;
}

/* 장애물 회피 모듈이 모터 제어권을 갖는 상태로 전환한다. */
void mission_control_begin_avoidance(void)
{
    mission_state = MISSION_STATE_OBSTACLE_AVOIDANCE;
}

/* 방지턱 Yaw 복구가 모터 제어권을 갖는 상태로 전환한다. */
void mission_control_begin_bump_recovery(void)
{
    mission_state = MISSION_STATE_BUMP_HEADING_RECOVERY;
}

/* 특수 동작이 끝난 뒤 일반 직진 상태로 돌아간다. */
void mission_control_resume_driving(void)
{
    mission_state = MISSION_STATE_DRIVING;
}

/* 사용자가 STOP을 요청했을 때 정지 상태로 전환한다. */
void mission_control_stop(void)
{
    mission_state = MISSION_STATE_STOPPED;
}

/* 복구가 필요한 오류 정지 상태로 전환한다. */
void mission_control_emergency_stop(void)
{
    mission_state = MISSION_STATE_ERROR;
}

/* 현재 모터 제어권 상태를 반환한다. */
mission_state_t mission_control_get_state(void)
{
    return mission_state;
}

/* 정지·대기·오류가 아닌 실제 주행 상태인지 반환한다. */
bool mission_control_is_running(void)
{
    return (mission_state == MISSION_STATE_DRIVING)
        || (mission_state == MISSION_STATE_OBSTACLE_AVOIDANCE)
        || (mission_state == MISSION_STATE_BUMP_HEADING_RECOVERY);
}

/* 현재 장애물 회피 상태 머신이 제어권을 가졌는지 반환한다. */
bool mission_control_is_avoiding(void)
{
    return (mission_state == MISSION_STATE_OBSTACLE_AVOIDANCE);
}
