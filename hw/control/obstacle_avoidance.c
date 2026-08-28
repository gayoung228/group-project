#include "main.h"
#include "obstacle_avoidance.h"

/* ------------------------------------------------------------------
 * obstacle_avoidance.c - 규칙 기반 장애물 회피 상태 머신
 *
 * 이 모듈은 센서나 모터를 직접 건드리지 않는다.
 * 호출자가 현재 방향과 이동 거리, 전방 거리를 넣어주면
 * 좌우 모터의 목표 속도를 계산해 돌려준다.
 *
 * 상태 흐름
 *
 *   IDLE -> STOP -> CHECK_SPACE -> FIRST_TURN
 *                                      |
 *                                      v
 *               COMPLETED <- SECOND_TURN <- FORWARD
 *
 * 시험 애플리케이션에서는 LEFT 방향과 90도를 넘긴다.
 * 따라서 왼쪽 30도씩 3회 -> 설정 거리 전진 -> 오른쪽 30도씩 3회로
 * 처음 진행 방향을 복구한다.
 * ------------------------------------------------------------------ */

/* 설정값 기본치 */
#define AVOID_DEFAULT_OBSTACLE_MM       250
#define AVOID_DEFAULT_BYPASS_MM         300
#define AVOID_DEFAULT_FORWARD_SPEED     90
#define AVOID_DEFAULT_TURN_SPEED        90
#define AVOID_DEFAULT_TURN_ANGLE_DEG    90.0f

/* 목표 각도에 도달했다고 판단할 오차 범위 [도] */
#define AVOID_ANGLE_TOLERANCE_DEG       5.0f

/* 회전이 끝났다고 인정하기까지 각도가 유지되어야 하는 시간 [ms] */
#define AVOID_ANGLE_HOLD_MS             300

/* 정지 명령 후 차체가 완전히 멎기를 기다리는 시간 [ms] */
#define AVOID_SETTLE_TIME_MS            500

/* 이 시간 안에 회전하지 못하면 안전을 위해 실패 정지한다 [ms] */
#define AVOID_TURN_TIMEOUT_MS          12000U

/* 관성에 의한 과회전을 줄이기 위해 90도를 세 구간으로 나눈다. */
#define AVOID_TURN_STEP_COUNT              3U

/* 회전 직후 정지 마찰을 확실히 넘기기 위한 전진 시동 출력과 유지 시간 */
#define AVOID_FORWARD_KICK_OUTPUT       100
#define AVOID_FORWARD_KICK_MS           300U

/* 회피 전진은 wheel PID를 우회하므로 별도의 정지 감시가 필요하다. */
#define AVOID_STALL_RPM_THRESHOLD       30.0f
#define AVOID_STALL_DETECT_MS         5000U
#define AVOID_STALL_RECOVER_MS        1000U


static obstacle_avoidance_config_t avoid_config;

static avoid_state_t     avoid_state     = AVOID_STATE_IDLE;
static avoid_direction_t avoid_direction = AVOID_DIRECTION_NONE;

/* 회피를 시작한 시점의 방향 */
static float avoid_start_heading_deg = 0.0f;

/* 지금 맞춰야 할 목표 방향 [도] */
static float avoid_target_heading_deg = 0.0f;

/* 전진 구간을 시작한 시점의 이동 거리 [mm] */
static float avoid_forward_start_mm = 0.0f;

/* 현재 상태에 들어온 시각과 각도가 맞기 시작한 시각 [ms] */
static uint32_t avoid_state_tick = 0;
static uint32_t avoid_angle_tick = 0;
static bool     avoid_angle_holding = false;
static uint8_t  avoid_turn_step = 0;

/* 회피 전진 중 한쪽 RPM이 낮을 때 한 번만 100% 재시동한다. */
static bool     avoid_stall_timing = false;
static uint32_t avoid_stall_tick = 0;
static bool     avoid_stall_retry_used = false;
static bool     avoid_stall_kick_active = false;
static uint32_t avoid_stall_kick_tick = 0;
static bool     avoid_stall_healthy_timing = false;
static uint32_t avoid_stall_healthy_tick = 0;
static uint32_t avoid_restart_count = 0;
static avoid_failure_t avoid_failure = AVOID_FAILURE_NONE;

/* 회피 동작이 요구하는 좌우 모터 목표 속도 */
static int16_t avoid_left_speed  = 0;
static int16_t avoid_right_speed = 0;


/* 좌우 모터 목표 속도를 한 번에 설정하는 내부 함수 */
static void avoid_set_output(int16_t left, int16_t right)
{
    avoid_left_speed  = left;
    avoid_right_speed = right;
}

/* 상태를 바꾸고 시간 계측을 초기화하는 내부 함수 */
static void avoid_change_state(avoid_state_t next)
{
    avoid_state         = next;
    avoid_state_tick    = HAL_GetTick();
    avoid_angle_tick    = 0;
    avoid_angle_holding = false;
}

/* 현재 상태에 머문 시간을 돌려주는 내부 함수 */
static uint32_t avoid_state_elapsed(void)
{
    return HAL_GetTick() - avoid_state_tick;
}

/* 각도를 0도 이상 360도 미만으로 정규화한다. */
static float avoid_normalize_360(float angle_deg)
{
    while (angle_deg >= 360.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < 0.0f)
    {
        angle_deg += 360.0f;
    }

    return angle_deg;
}

/* 각도 오차를 -180도 이상 180도 이하의 최단 회전값으로 바꾼다. */
static float avoid_normalize_error(float error_deg)
{
    while (error_deg > 180.0f)
    {
        error_deg -= 360.0f;
    }
    while (error_deg < -180.0f)
    {
        error_deg += 360.0f;
    }

    return error_deg;
}

/* 0/360도 경계를 고려한 두 각도의 최소 차이를 반환한다. */
static float avoid_angle_diff(float a, float b)
{
    float diff = avoid_normalize_error(a - b);

    if (diff < 0.0f)
    {
        diff = -diff;
    }

    return diff;
}

/* 회피 방향에 맞는 회전 부호를 돌려주는 내부 함수
 * 좌회전이면 방향 각도가 커지므로 +1 이다. */
static float avoid_turn_sign(void)
{
    if (avoid_direction == AVOID_DIRECTION_RIGHT)
    {
        return -1.0f;
    }

    return 1.0f;
}

/* 첫 회전의 현재 30도 단위 목표를 설정한다. */
static void avoid_set_first_turn_target(void)
{
    float completed_angle = avoid_config.turn_angle_deg
                          * (float)(avoid_turn_step + 1U)
                          / (float)AVOID_TURN_STEP_COUNT;

    avoid_target_heading_deg = avoid_normalize_360(
        avoid_start_heading_deg + (completed_angle * avoid_turn_sign()));
    avoid_angle_tick = 0;
    avoid_angle_holding = false;
}

/* 복귀 회전의 현재 30도 단위 목표를 설정한다. */
static void avoid_set_second_turn_target(void)
{
    float remaining_angle = avoid_config.turn_angle_deg
                          * (float)(AVOID_TURN_STEP_COUNT
                                    - avoid_turn_step - 1U)
                          / (float)AVOID_TURN_STEP_COUNT;

    avoid_target_heading_deg = avoid_normalize_360(
        avoid_start_heading_deg + (remaining_angle * avoid_turn_sign()));
    avoid_angle_tick = 0;
    avoid_angle_holding = false;
}

/* 목표 방향을 향해 제자리 회전하도록 좌우 속도를 정하는 내부 함수
 * 허용 오차 안에 들어오면 출력을 끊어 목표 근처에서 튕기는 것을 막는다. */
static void avoid_apply_rotation(float current_heading_deg)
{
    int16_t speed = (int16_t)avoid_config.turn_speed;
    float   error = avoid_normalize_error(avoid_target_heading_deg
                                        - current_heading_deg);

    /* 허용 오차 안이면 더 돌 필요가 없다 */
    if ((error <= AVOID_ANGLE_TOLERANCE_DEG) &&
        (error >= -AVOID_ANGLE_TOLERANCE_DEG))
    {
        avoid_set_output(0, 0);
        return;
    }

    if (error > 0.0f)
    {
        /* 왼쪽으로 돌아야 한다 */
        avoid_set_output(-speed, speed);
    }
    else
    {
        /* 오른쪽으로 돌아야 한다 */
        avoid_set_output(speed, -speed);
    }
}

/* 목표 방향에 충분히 도달했는지 판단하는 내부 함수
 * 관성으로 넘어갔다 돌아오는 것을 감안해 일정 시간 유지되어야 인정한다. */
static bool avoid_angle_reached(float current_heading_deg)
{
    float diff = avoid_angle_diff(avoid_target_heading_deg, current_heading_deg);

    if (diff <= AVOID_ANGLE_TOLERANCE_DEG)
    {
        if (avoid_angle_holding == false)
        {
            avoid_angle_holding = true;
            avoid_angle_tick    = HAL_GetTick();
        }
        else if ((HAL_GetTick() - avoid_angle_tick) >= AVOID_ANGLE_HOLD_MS)
        {
            return true;
        }
    }
    else
    {
        avoid_angle_holding = false;
    }

    return false;
}


/* 장애물 회피 상태 머신을 초기화 */
void obstacle_avoidance_init(void)
{
    avoid_config.obstacle_distance_mm = AVOID_DEFAULT_OBSTACLE_MM;
    avoid_config.bypass_distance_mm   = AVOID_DEFAULT_BYPASS_MM;
    avoid_config.forward_speed        = AVOID_DEFAULT_FORWARD_SPEED;
    avoid_config.turn_speed           = AVOID_DEFAULT_TURN_SPEED;
    avoid_config.turn_angle_deg       = AVOID_DEFAULT_TURN_ANGLE_DEG;

    obstacle_avoidance_reset();
}

/* 진행 중인 장애물 회피를 취소하고 대기 상태로 전환 */
void obstacle_avoidance_reset(void)
{
    avoid_direction          = AVOID_DIRECTION_NONE;
    avoid_start_heading_deg  = 0.0f;
    avoid_target_heading_deg = 0.0f;
    avoid_forward_start_mm   = 0.0f;
    avoid_turn_step          = 0;
    avoid_stall_timing       = false;
    avoid_stall_tick         = 0;
    avoid_stall_retry_used   = false;
    avoid_stall_kick_active  = false;
    avoid_stall_kick_tick    = 0;
    avoid_stall_healthy_timing = false;
    avoid_stall_healthy_tick = 0;
    avoid_restart_count      = 0;
    avoid_failure            = AVOID_FAILURE_NONE;

    avoid_set_output(0, 0);
    avoid_change_state(AVOID_STATE_IDLE);
}

/* 장애물 거리와 회피 동작 설정값을 변경 */
void obstacle_avoidance_set_config(const obstacle_avoidance_config_t *config)
{
    if (config == NULL)
    {
        return;
    }

    avoid_config = *config;
}

/* 좌우 거리 중 공간이 더 넓은 회피 방향을 반환
 * 좌우 거리 센서가 없어 두 값이 같으면 왼쪽을 고른다. */
avoid_direction_t obstacle_avoidance_select_direction(uint16_t left_distance_mm,
                                                      uint16_t right_distance_mm)
{
    bool left_ok  = (left_distance_mm  > avoid_config.obstacle_distance_mm);
    bool right_ok = (right_distance_mm > avoid_config.obstacle_distance_mm);

    if ((left_ok == false) && (right_ok == false))
    {
        return AVOID_DIRECTION_NONE;
    }

    if (left_ok && right_ok)
    {
        return (left_distance_mm >= right_distance_mm)
             ? AVOID_DIRECTION_LEFT
             : AVOID_DIRECTION_RIGHT;
    }

    return left_ok ? AVOID_DIRECTION_LEFT : AVOID_DIRECTION_RIGHT;
}

/* 선택한 방향으로 장애물 회피 동작을 시작 */
bool obstacle_avoidance_start(avoid_direction_t direction,
                              float current_heading_deg,
                              float current_distance_mm)
{
    if (direction == AVOID_DIRECTION_NONE)
    {
        avoid_failure = AVOID_FAILURE_NO_DIRECTION;
        avoid_set_output(0, 0);
        avoid_change_state(AVOID_STATE_FAILED);
        return false;
    }

    avoid_direction          = direction;
    avoid_start_heading_deg  = avoid_normalize_360(current_heading_deg);
    avoid_target_heading_deg = avoid_start_heading_deg;
    avoid_forward_start_mm   = current_distance_mm;
    avoid_turn_step          = 0;
    avoid_failure            = AVOID_FAILURE_NONE;

    /* 우선 완전히 멈춘 뒤에 판단한다 */
    avoid_set_output(0, 0);
    avoid_change_state(AVOID_STATE_STOP);

    return true;
}

/* 현재 방향과 이동 거리를 이용해 회피 상태를 갱신 */
void obstacle_avoidance_update(float current_heading_deg,
                               float current_distance_mm,
                               uint16_t front_distance_mm,
                               float left_rpm,
                               float right_rpm)
{
    switch (avoid_state)
    {
        case AVOID_STATE_IDLE:
            avoid_set_output(0, 0);
            break;

        case AVOID_STATE_STOP:
            /* 차체가 완전히 멎어야 자이로와 거리값이 안정적이다 */
            avoid_set_output(0, 0);

            if (avoid_state_elapsed() >= AVOID_SETTLE_TIME_MS)
            {
                avoid_change_state(AVOID_STATE_CHECK_SPACE);
            }
            break;

        case AVOID_STATE_CHECK_SPACE:
            avoid_set_output(0, 0);

            if (avoid_direction == AVOID_DIRECTION_NONE)
            {
                avoid_failure = AVOID_FAILURE_NO_DIRECTION;
                avoid_change_state(AVOID_STATE_FAILED);
                break;
            }

            /* 90도 전체가 아니라 첫 번째 30도 목표부터 시작한다. */
            avoid_turn_step = 0;
            avoid_set_first_turn_target();

            avoid_change_state(AVOID_STATE_FIRST_TURN);
            break;

        case AVOID_STATE_FIRST_TURN:
            avoid_apply_rotation(current_heading_deg);

            if (avoid_state_elapsed() >= AVOID_TURN_TIMEOUT_MS)
            {
                avoid_failure = AVOID_FAILURE_TURN_TIMEOUT;
                avoid_set_output(0, 0);
                avoid_change_state(AVOID_STATE_FAILED);
                break;
            }

            if (avoid_angle_reached(current_heading_deg) == false)
            {
                break;
            }

            if ((avoid_turn_step + 1U) < AVOID_TURN_STEP_COUNT)
            {
                /* 30도에서 잠시 안정된 뒤 다음 30도 목표로 이동한다. */
                avoid_turn_step++;
                avoid_set_first_turn_target();
                break;
            }

            /* 왼쪽을 향한 뒤에도 바로 앞이 막혀 있으면 진행하지 않는다. */
            if (front_distance_mm <= avoid_config.obstacle_distance_mm)
            {
                avoid_failure = AVOID_FAILURE_FRONT_BLOCKED;
                avoid_set_output(0, 0);
                avoid_change_state(AVOID_STATE_FAILED);
                break;
            }

            avoid_forward_start_mm = current_distance_mm;
            avoid_stall_timing = false;
            avoid_stall_retry_used = false;
            avoid_stall_kick_active = false;
            avoid_stall_healthy_timing = false;
            avoid_set_output(AVOID_FORWARD_KICK_OUTPUT,
                             AVOID_FORWARD_KICK_OUTPUT);
            avoid_change_state(AVOID_STATE_FORWARD);
            break;

        case AVOID_STATE_FORWARD:
            /* 회피 전진 중 다시 막히면 추가 동작 없이 안전 정지한다. */
            if (front_distance_mm <= avoid_config.obstacle_distance_mm)
            {
                avoid_failure = AVOID_FAILURE_FRONT_BLOCKED;
                avoid_set_output(0, 0);
                avoid_change_state(AVOID_STATE_FAILED);
                break;
            }

            if (left_rpm < 0.0f)  { left_rpm = -left_rpm; }
            if (right_rpm < 0.0f) { right_rpm = -right_rpm; }

            if ((left_rpm < AVOID_STALL_RPM_THRESHOLD)
                || (right_rpm < AVOID_STALL_RPM_THRESHOLD))
            {
                avoid_stall_healthy_timing = false;

                if (avoid_stall_timing == false)
                {
                    avoid_stall_timing = true;
                    avoid_stall_tick = HAL_GetTick();
                }
                else if ((HAL_GetTick() - avoid_stall_tick) >=
                         AVOID_STALL_DETECT_MS)
                {
                    if (avoid_stall_retry_used == false)
                    {
                        /* 첫 정지는 100% 시동을 한 번만 다시 건다. */
                        avoid_stall_retry_used = true;
                        avoid_restart_count++;
                        avoid_stall_kick_active = true;
                        avoid_stall_kick_tick = HAL_GetTick();
                        avoid_stall_tick = HAL_GetTick();
                    }
                    else
                    {
                        /* 재시동 후에도 다시 5초간 RPM이 없으면 정지한다. */
                        avoid_failure = AVOID_FAILURE_MOTOR_STALL;
                        avoid_set_output(0, 0);
                        avoid_change_state(AVOID_STATE_FAILED);
                        break;
                    }
                }
            }
            else
            {
                avoid_stall_timing = false;

                if (avoid_stall_healthy_timing == false)
                {
                    avoid_stall_healthy_timing = true;
                    avoid_stall_healthy_tick = HAL_GetTick();
                }
                else if ((HAL_GetTick() - avoid_stall_healthy_tick) >=
                         AVOID_STALL_RECOVER_MS)
                {
                    avoid_stall_retry_used = false;
                }
            }

            if (avoid_stall_kick_active
                && ((HAL_GetTick() - avoid_stall_kick_tick) >=
                    AVOID_FORWARD_KICK_MS))
            {
                avoid_stall_kick_active = false;
            }

            /* 회전 직후와 정지 복구 시에는 300ms 동안 100%로 출발한다. */
            if ((avoid_state_elapsed() < AVOID_FORWARD_KICK_MS)
                || avoid_stall_kick_active)
            {
                avoid_set_output(AVOID_FORWARD_KICK_OUTPUT,
                                 AVOID_FORWARD_KICK_OUTPUT);
            }
            else
            {
                avoid_set_output((int16_t)avoid_config.forward_speed,
                                 (int16_t)avoid_config.forward_speed);
            }

            if ((current_distance_mm - avoid_forward_start_mm) >=
                (float)avoid_config.bypass_distance_mm)
            {
                /* 원래 방향까지 30도씩 세 단계로 되돌아간다. */
                avoid_turn_step = 0;
                avoid_set_second_turn_target();

                avoid_change_state(AVOID_STATE_SECOND_TURN);
            }
            break;

        case AVOID_STATE_SECOND_TURN:
            avoid_apply_rotation(current_heading_deg);

            if (avoid_state_elapsed() >= AVOID_TURN_TIMEOUT_MS)
            {
                avoid_failure = AVOID_FAILURE_TURN_TIMEOUT;
                avoid_set_output(0, 0);
                avoid_change_state(AVOID_STATE_FAILED);
                break;
            }

            if (avoid_angle_reached(current_heading_deg) == false)
            {
                break;
            }

            if ((avoid_turn_step + 1U) < AVOID_TURN_STEP_COUNT)
            {
                /* 60도, 30도, 0도 기준으로 30도씩 나누어 복귀한다. */
                avoid_turn_step++;
                avoid_set_second_turn_target();
                break;
            }

            avoid_set_output(0, 0);

            /* 원래 방향으로 돌아왔는데 또 막혀 있으면 직진하지 않는다. */
            if (front_distance_mm <= avoid_config.obstacle_distance_mm)
            {
                avoid_failure = AVOID_FAILURE_FRONT_BLOCKED;
                avoid_change_state(AVOID_STATE_FAILED);
                break;
            }

            avoid_change_state(AVOID_STATE_COMPLETED);
            break;

        case AVOID_STATE_COMPLETED:
        case AVOID_STATE_FAILED:
            avoid_set_output(0, 0);
            break;

        default:
            obstacle_avoidance_reset();
            break;
    }
}

/* 현재 장애물 회피 단계를 반환 */
avoid_state_t obstacle_avoidance_get_state(void)
{
    return avoid_state;
}

/* 현재 선택된 회피 방향을 반환 */
avoid_direction_t obstacle_avoidance_get_direction(void)
{
    return avoid_direction;
}

/* 회피 동작에 필요한 왼쪽 모터 목표 속도를 반환 */
int16_t obstacle_avoidance_get_left_speed(void)
{
    return avoid_left_speed;
}

/* 회피 동작에 필요한 오른쪽 모터 목표 속도를 반환 */
int16_t obstacle_avoidance_get_right_speed(void)
{
    return avoid_right_speed;
}

/* 장애물 회피 동작이 진행 중인지 반환 */
bool obstacle_avoidance_is_running(void)
{
    return ((avoid_state != AVOID_STATE_IDLE) &&
            (avoid_state != AVOID_STATE_COMPLETED) &&
            (avoid_state != AVOID_STATE_FAILED));
}

/* 장애물 회피가 정상적으로 완료됐는지 반환 */
bool obstacle_avoidance_is_completed(void)
{
    return (avoid_state == AVOID_STATE_COMPLETED);
}

/* 장애물 회피가 실패했는지 반환 */
bool obstacle_avoidance_has_failed(void)
{
    return (avoid_state == AVOID_STATE_FAILED);
}

/* 가장 최근 회피 실패 원인을 반환한다. */
avoid_failure_t obstacle_avoidance_get_failure(void)
{
    return avoid_failure;
}

/* 회전 중 현재 진행 중인 30도 단계를 1~3으로 반환한다. */
uint8_t obstacle_avoidance_get_turn_step(void)
{
    if ((avoid_state != AVOID_STATE_FIRST_TURN)
        && (avoid_state != AVOID_STATE_SECOND_TURN))
    {
        return 0;
    }

    return (uint8_t)(avoid_turn_step + 1U);
}

/* 회피 전진 중 5초 저RPM으로 시동을 다시 건 횟수를 반환한다. */
uint32_t obstacle_avoidance_get_restart_count(void)
{
    return avoid_restart_count;
}
