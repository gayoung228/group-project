#ifndef SPEED_BUMP_CONTROL_H
#define SPEED_BUMP_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

/* 차량의 Y축(Pitch)을 이용한 과속방지턱 통과 단계 */
typedef enum
{
    SPEED_BUMP_STATE_NORMAL,
    SPEED_BUMP_STATE_CLIMB,
    SPEED_BUMP_STATE_DESCENT,
    SPEED_BUMP_STATE_LEVEL_HOLD
} speed_bump_state_t;

void speed_bump_control_init(void);
void speed_bump_control_reset(void);

/* 20ms 같은 일정 주기로 호출해 현재 Pitch와 경과 시간을 전달한다. */
void speed_bump_control_update(float pitch_deg, uint32_t elapsed_time_ms);

speed_bump_state_t speed_bump_control_get_state(void);

/* 현재 단계에 맞춰 기본 RPM을 +30%, -10% 또는 원래 값으로 변환한다. */
float speed_bump_control_get_target_rpm(float normal_rpm);

/* 방지턱 상태가 너무 오래 지속돼 안전하게 해제됐는지 한 번만 반환한다. */
bool speed_bump_control_take_timeout(void);

/* Y값이 복귀하지 않아 다운힐 시작 3초 뒤 강제 정상 복귀했는지 반환한다. */
bool speed_bump_control_take_forced_recovery(void);

/* 방지턱 내리막 뒤 평지 유지까지 정상 완료됐을 때 한 번만 true를 반환한다. */
bool speed_bump_control_take_completed(void);

#endif
