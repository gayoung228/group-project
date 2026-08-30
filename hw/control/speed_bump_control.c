#include "speed_bump_control.h"
#include "rover_config.h"

/* 오르막 이벤트 진입 기준과 평지 근사 범위 [도] */
#define SPEED_BUMP_CLIMB_DEG          10.0f
#define SPEED_BUMP_LEVEL_DEG           5.0f

/* 순간 진동을 방지턱으로 오인하지 않도록 진입 각도를 유지할 시간 [ms] */
#define SPEED_BUMP_TRIGGER_HOLD_MS   100U

/* 방지턱을 완전히 통과했다고 판단할 평지 유지 시간 [ms] */
#define SPEED_BUMP_LEVEL_HOLD_MS    1000U

/* 자이로 적분값이 0으로 돌아오지 않아도 다운힐 시작 후 강제 복귀할 시간 */
#define SPEED_BUMP_DESCENT_MAX_MS   3000U

/* 센서 드리프트로 상태가 영구 고정되지 않도록 두는 안전 제한 [ms] */
#define SPEED_BUMP_EVENT_TIMEOUT_MS 15000U

#define SPEED_BUMP_CLIMB_SCALE        1.30f
#define SPEED_BUMP_DESCENT_SCALE      0.90f

/* 프로젝트에서 실측한 모터 제어 최대 목표 RPM */
#define SPEED_BUMP_MAX_RPM           ROVER_DRIVE_RPM_MAX

static speed_bump_state_t bump_state = SPEED_BUMP_STATE_NORMAL;
static uint32_t bump_trigger_elapsed_ms = 0;
static uint32_t bump_level_elapsed_ms = 0;
static uint32_t bump_event_elapsed_ms = 0;
static uint32_t bump_descent_elapsed_ms = 0;
static bool bump_negative_pitch_seen = false;
static bool bump_timeout_event = false;
static bool bump_forced_recovery_event = false;
static bool bump_completed_event = false;

/* 방지턱 속도 배율 결과를 정지 이상, 실측 최대 RPM 이하로 제한한다. */
static float speed_bump_clamp_rpm(float rpm)
{
    if (rpm > SPEED_BUMP_MAX_RPM)
    {
        return SPEED_BUMP_MAX_RPM;
    }
    if (rpm < 0.0f)
    {
        return 0.0f;
    }
    return rpm;
}

/* 방지턱 상태와 이전에 남은 모든 일회성 이벤트를 초기화한다. */
void speed_bump_control_init(void)
{
    speed_bump_control_reset();
    bump_timeout_event = false;
    bump_forced_recovery_event = false;
    bump_completed_event = false;
}

/* 현재 방지턱 판정만 취소하고 정상 주행 감지 대기로 돌아간다. */
void speed_bump_control_reset(void)
{
    bump_state = SPEED_BUMP_STATE_NORMAL;
    bump_trigger_elapsed_ms = 0;
    bump_level_elapsed_ms = 0;
    bump_event_elapsed_ms = 0;
    bump_descent_elapsed_ms = 0;
    bump_negative_pitch_seen = false;
    bump_completed_event = false;
}

/* 최신 Pitch와 경과 시간으로 오르막·내리막·평지 상태를 갱신한다. */
void speed_bump_control_update(float pitch_deg, uint32_t elapsed_time_ms)
{
    if (elapsed_time_ms == 0U)
    {
        return;
    }

    if (bump_state != SPEED_BUMP_STATE_NORMAL)
    {
        bump_event_elapsed_ms += elapsed_time_ms;

        if (bump_event_elapsed_ms >= SPEED_BUMP_EVENT_TIMEOUT_MS)
        {
            speed_bump_control_reset();
            bump_timeout_event = true;
            return;
        }
    }

    /* 다운힐 진입 이후에는 센서값이 0으로 복귀하지 않더라도
     * 최대 3초 뒤 기본 주행으로 강제 복귀한다. */
    if ((bump_state == SPEED_BUMP_STATE_DESCENT)
        || (bump_state == SPEED_BUMP_STATE_LEVEL_HOLD))
    {
        bump_descent_elapsed_ms += elapsed_time_ms;

        if (bump_descent_elapsed_ms >= SPEED_BUMP_DESCENT_MAX_MS)
        {
            speed_bump_control_reset();
            bump_forced_recovery_event = true;
            return;
        }
    }

    switch (bump_state)
    {
        case SPEED_BUMP_STATE_NORMAL:
            /* Y가 진입 기준 이상인 상태가 100ms 유지돼야 실제 오르막으로 본다. */
            if (pitch_deg >= SPEED_BUMP_CLIMB_DEG)
            {
                bump_trigger_elapsed_ms += elapsed_time_ms;
                if (bump_trigger_elapsed_ms >= SPEED_BUMP_TRIGGER_HOLD_MS)
                {
                    bump_state = SPEED_BUMP_STATE_CLIMB;
                    bump_trigger_elapsed_ms = 0;
                    bump_event_elapsed_ms = 0;
                }
            }
            else
            {
                bump_trigger_elapsed_ms = 0;
            }
            break;

        case SPEED_BUMP_STATE_CLIMB:
            /* 정상 근처를 지나 내리막으로 넘어가면 기본값의 90%로 낮춘다.
             * 샘플 사이에 0도를 빠르게 지나쳐도 놓치지 않게 +5도 이하를 쓴다. */
            if (pitch_deg <= SPEED_BUMP_LEVEL_DEG)
            {
                bump_state = SPEED_BUMP_STATE_DESCENT;
                bump_descent_elapsed_ms = 0;
                bump_negative_pitch_seen =
                    (pitch_deg <= -SPEED_BUMP_LEVEL_DEG);
            }
            break;

        case SPEED_BUMP_STATE_DESCENT:
            /* 실제 내리막을 지나갔다는 증거로 -5도 이하를 한 번 확인한다. */
            if (pitch_deg <= -SPEED_BUMP_LEVEL_DEG)
            {
                bump_negative_pitch_seen = true;
            }

            if (bump_negative_pitch_seen
                && (pitch_deg >= -SPEED_BUMP_LEVEL_DEG)
                && (pitch_deg <= SPEED_BUMP_LEVEL_DEG))
            {
                bump_state = SPEED_BUMP_STATE_LEVEL_HOLD;
                bump_level_elapsed_ms = 0;
            }
            break;

        case SPEED_BUMP_STATE_LEVEL_HOLD:
            if ((pitch_deg >= -SPEED_BUMP_LEVEL_DEG)
                && (pitch_deg <= SPEED_BUMP_LEVEL_DEG))
            {
                bump_level_elapsed_ms += elapsed_time_ms;
                if (bump_level_elapsed_ms >= SPEED_BUMP_LEVEL_HOLD_MS)
                {
                    speed_bump_control_reset();
                    bump_completed_event = true;
                }
            }
            else
            {
                /* 1초가 되기 전에 다시 기울면 내리막 확인부터 이어간다. */
                bump_state = SPEED_BUMP_STATE_DESCENT;
                bump_level_elapsed_ms = 0;
            }
            break;

        default:
            speed_bump_control_reset();
            break;
    }
}

/* 현재 방지턱 통과 단계를 반환한다. */
speed_bump_state_t speed_bump_control_get_state(void)
{
    return bump_state;
}

/* 현재 단계의 속도 배율을 기본 RPM에 적용해 안전 범위로 반환한다. */
float speed_bump_control_get_target_rpm(float normal_rpm)
{
    switch (bump_state)
    {
        case SPEED_BUMP_STATE_CLIMB:
            return speed_bump_clamp_rpm(normal_rpm * SPEED_BUMP_CLIMB_SCALE);

        case SPEED_BUMP_STATE_DESCENT:
        case SPEED_BUMP_STATE_LEVEL_HOLD:
            return speed_bump_clamp_rpm(normal_rpm * SPEED_BUMP_DESCENT_SCALE);

        case SPEED_BUMP_STATE_NORMAL:
        default:
            return speed_bump_clamp_rpm(normal_rpm);
    }
}

/* 전체 상태 제한시간 초과 이벤트를 한 번 소비한다. */
bool speed_bump_control_take_timeout(void)
{
    bool occurred = bump_timeout_event;
    bump_timeout_event = false;
    return occurred;
}

/* 내리막 제한시간 강제 복귀 이벤트를 한 번 소비한다. */
bool speed_bump_control_take_forced_recovery(void)
{
    bool occurred = bump_forced_recovery_event;
    bump_forced_recovery_event = false;
    return occurred;
}

/* 정상적인 오르막→내리막→평지 상태 전이가 끝났다는 이벤트를 한 번 소비한다. */
bool speed_bump_control_take_completed(void)
{
    bool occurred = bump_completed_event;
    bump_completed_event = false;
    return occurred;
}
