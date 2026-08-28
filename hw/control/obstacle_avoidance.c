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
 * 따라서 왼쪽 90도 회전 -> 설정 거리 전진 -> 오른쪽 90도 회전으로
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
#define AVOID_TURN_TIMEOUT_MS           5000


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
        avoid_set_output(0, 0);
        avoid_change_state(AVOID_STATE_FAILED);
        return false;
    }

    avoid_direction          = direction;
    avoid_start_heading_deg  = avoid_normalize_360(current_heading_deg);
    avoid_target_heading_deg = avoid_start_heading_deg;
    avoid_forward_start_mm   = current_distance_mm;

    /* 우선 완전히 멈춘 뒤에 판단한다 */
    avoid_set_output(0, 0);
    avoid_change_state(AVOID_STATE_STOP);

    return true;
}

/* 현재 방향과 이동 거리를 이용해 회피 상태를 갱신 */
void obstacle_avoidance_update(float current_heading_deg,
                               float current_distance_mm,
                               uint16_t front_distance_mm)
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
                avoid_change_state(AVOID_STATE_FAILED);
                break;
            }

            /* 회피 시작 방향을 기준으로 정확히 왼쪽 90도 목표를 세운다. */
            avoid_target_heading_deg = avoid_normalize_360(
                avoid_start_heading_deg
                + (avoid_config.turn_angle_deg * avoid_turn_sign()));

            avoid_change_state(AVOID_STATE_FIRST_TURN);
            break;

        case AVOID_STATE_FIRST_TURN:
            avoid_apply_rotation(current_heading_deg);

            if (avoid_state_elapsed() >= AVOID_TURN_TIMEOUT_MS)
            {
                avoid_set_output(0, 0);
                avoid_change_state(AVOID_STATE_FAILED);
                break;
            }

            if (avoid_angle_reached(current_heading_deg) == false)
            {
                break;
            }

            /* 왼쪽을 향한 뒤에도 바로 앞이 막혀 있으면 진행하지 않는다. */
            if (front_distance_mm <= avoid_config.obstacle_distance_mm)
            {
                avoid_set_output(0, 0);
                avoid_change_state(AVOID_STATE_FAILED);
                break;
            }

            avoid_forward_start_mm = current_distance_mm;
            avoid_set_output((int16_t)avoid_config.forward_speed,
                             (int16_t)avoid_config.forward_speed);
            avoid_change_state(AVOID_STATE_FORWARD);
            break;

        case AVOID_STATE_FORWARD:
            /* 회피 전진 중 다시 막히면 추가 동작 없이 안전 정지한다. */
            if (front_distance_mm <= avoid_config.obstacle_distance_mm)
            {
                avoid_set_output(0, 0);
                avoid_change_state(AVOID_STATE_FAILED);
                break;
            }

            avoid_set_output((int16_t)avoid_config.forward_speed,
                             (int16_t)avoid_config.forward_speed);

            if ((current_distance_mm - avoid_forward_start_mm) >=
                (float)avoid_config.bypass_distance_mm)
            {
                /* 회피를 시작할 때의 방향으로 되돌린다 */
                avoid_target_heading_deg = avoid_normalize_360(avoid_start_heading_deg);

                avoid_change_state(AVOID_STATE_SECOND_TURN);
            }
            break;

        case AVOID_STATE_SECOND_TURN:
            avoid_apply_rotation(current_heading_deg);

            if (avoid_state_elapsed() >= AVOID_TURN_TIMEOUT_MS)
            {
                avoid_set_output(0, 0);
                avoid_change_state(AVOID_STATE_FAILED);
                break;
            }

            if (avoid_angle_reached(current_heading_deg) == false)
            {
                break;
            }

            avoid_set_output(0, 0);

            /* 원래 방향으로 돌아왔는데 또 막혀 있으면 직진하지 않는다. */
            if (front_distance_mm <= avoid_config.obstacle_distance_mm)
            {
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
