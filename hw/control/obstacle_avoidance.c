#include "main.h"
#include "obstacle_avoidance.h"
#include "rover_config.h"
#include <stddef.h>

/* ------------------------------------------------------------------
 * obstacle_avoidance.c - 3센서 + 자이로 기반 부드러운 회피 상태 머신
 *
 * 거리센서는 위험도·회피 방향·장애물 끝을 결정한다.
 * 이 모듈은 그 판단을 base RPM과 목표 Yaw로 바꾸기만 한다.
 * 실제 Yaw PID, 바퀴 RPM PID, PWM 출력은 drive 이하 계층이 담당한다.
 * ------------------------------------------------------------------ */

static avoid_state_t avoid_state = AVOID_STATE_IDLE;
static avoid_direction_t avoid_direction = AVOID_DIRECTION_NONE;
static avoid_direction_t preferred_direction = AVOID_DIRECTION_LEFT;
static avoid_direction_t escape_direction = AVOID_DIRECTION_NONE;
static avoid_failure_t avoid_failure = AVOID_FAILURE_NONE;
static obstacle_avoidance_command_t avoid_command;

static float original_heading_deg = 0.0f;
static float commanded_heading_deg = 0.0f;
static float open_heading_deg = 0.0f;
static float escape_heading_deg = 0.0f;
static float event_start_distance_mm = 0.0f;
static float state_start_distance_mm = 0.0f;
static float maximum_proximity = 0.0f;

static uint32_t event_start_ms = 0U;
static uint32_t state_start_ms = 0U;
static uint32_t heading_hold_start_ms = 0U;
static bool heading_holding = false;
static bool inside_seen = false;
static bool started_event = false;
static bool state_change_event = false;

static uint32_t last_front_sequence = 0U;
static uint32_t last_inside_sequence = 0U;
static uint8_t front_trigger_count = 0U;
static uint8_t inside_seen_count = 0U;
static uint8_t edge_clear_count = 0U;
static uint8_t open_front_count = 0U;
static uint8_t untracked_clear_count = 0U;

/* 실수값을 지정한 범위에 제한한다. */
static float avoid_clamp(float value, float minimum, float maximum)
{
    if (value < minimum) { return minimum; }
    if (value > maximum) { return maximum; }
    return value;
}

/* 절대 이동거리 차를 반환한다. 엔코더 방향 부호에 영향을 받지 않는다. */
static float avoid_distance_delta(float current, float start)
{
    float delta = current - start;
    return (delta < 0.0f) ? -delta : delta;
}

/* 각도를 0도 이상 360도 미만으로 정규화한다. */
static float avoid_normalize_360(float angle_deg)
{
    while (angle_deg >= 360.0f) { angle_deg -= 360.0f; }
    while (angle_deg < 0.0f) { angle_deg += 360.0f; }
    return angle_deg;
}

/* 두 방향의 최단 오차를 -180도 이상 180도 이하로 바꾼다. */
static float avoid_angle_error(float target_deg, float current_deg)
{
    float error = target_deg - current_deg;

    while (error > 180.0f) { error -= 360.0f; }
    while (error < -180.0f) { error += 360.0f; }
    return error;
}

/* 각도 오차의 절댓값을 반환한다. */
static float avoid_angle_difference(float first_deg, float second_deg)
{
    float difference = avoid_angle_error(first_deg, second_deg);
    return (difference < 0.0f) ? -difference : difference;
}

/* 좌회전은 +1, 우회전은 -1로 바꿔 공통 각도 계산에 사용한다. */
static float avoid_direction_sign(avoid_direction_t direction)
{
    return (direction == AVOID_DIRECTION_RIGHT) ? -1.0f : 1.0f;
}

/* 상태를 바꾸면서 해당 상태의 시간·거리·각도 유지 카운터를 초기화한다. */
static void avoid_change_state(avoid_state_t next,
                               float current_distance_mm)
{
    avoid_state = next;
    state_start_ms = HAL_GetTick();
    state_start_distance_mm = current_distance_mm;
    heading_hold_start_ms = 0U;
    heading_holding = false;
    state_change_event = true;
}

/* 회피를 실패로 끝내고 남아 있는 주행 명령을 0으로 만든다. */
static void avoid_fail(avoid_failure_t failure,
                       float current_distance_mm)
{
    avoid_failure = failure;
    avoid_command.motion = AVOID_MOTION_STOP;
    avoid_command.base_rpm = 0.0f;
    avoid_change_state(AVOID_STATE_FAILED, current_distance_mm);
}

/* 자이로 추종 주행 명령을 안전한 RPM 범위로 제한해 저장한다. */
static void avoid_set_track_command(float base_rpm, float target_heading_deg)
{
    avoid_command.motion = AVOID_MOTION_TRACK_HEADING;
    avoid_command.base_rpm = avoid_clamp(base_rpm,
                                         ROVER_AVOID_MIN_RPM,
                                         ROVER_DRIVE_RPM_MAX);
    avoid_command.target_heading_deg = avoid_normalize_360(target_heading_deg);
}

/* 전진하지 않고 제자리에서 지정한 절대 Yaw를 향하도록 명령한다. */
static void avoid_set_rotate_command(float target_heading_deg)
{
    avoid_command.motion = AVOID_MOTION_ROTATE_HEADING;
    avoid_command.base_rpm = 0.0f;
    avoid_command.target_heading_deg = avoid_normalize_360(target_heading_deg);
}

/* 정지 명령을 저장한다. 상태 머신은 motor/wheel을 직접 호출하지 않는다. */
static void avoid_set_stop_command(void)
{
    avoid_command.motion = AVOID_MOTION_STOP;
    avoid_command.base_rpm = 0.0f;
}

/* 현재 회피 방향에서 장애물과 가까운 안쪽 센서를 선택한다.
 * 왼쪽으로 피하면 원래 장애물은 차량 오른쪽에 남는다. */
static const proximity_sample_t *avoid_inside_sample(
    const proximity_snapshot_t *sensors)
{
    return (avoid_direction == AVOID_DIRECTION_LEFT)
         ? &sensors->right
         : &sensors->left;
}

/* 같은 캐시값을 여러 번 세지 않도록 새 센서 측정인지 sequence로 확인한다. */
static bool avoid_take_new_sequence(uint32_t sequence, uint32_t *last_sequence)
{
    if ((sequence == 0U) || (sequence == *last_sequence))
    {
        return false;
    }

    *last_sequence = sequence;
    return true;
}

/* 좌우 통로 조건과 데드밴드를 적용해 회피 방향을 한 번만 선택한다. */
static avoid_direction_t avoid_select_direction(
    const proximity_snapshot_t *sensors)
{
    uint16_t left_mm = sensors->left.distance_mm;
    uint16_t right_mm = sensors->right.distance_mm;
    bool left_open = (left_mm >= ROVER_AVOID_DIRECTION_OPEN_MM);
    bool right_open = (right_mm >= ROVER_AVOID_DIRECTION_OPEN_MM);

    if (!left_open && !right_open) { return AVOID_DIRECTION_NONE; }
    if (left_open && !right_open) { return AVOID_DIRECTION_LEFT; }
    if (!left_open && right_open) { return AVOID_DIRECTION_RIGHT; }

    if (((uint32_t)left_mm)
        > ((uint32_t)right_mm + ROVER_AVOID_DIRECTION_DEADBAND_MM))
    {
        return AVOID_DIRECTION_LEFT;
    }
    if (((uint32_t)right_mm)
        > ((uint32_t)left_mm + ROVER_AVOID_DIRECTION_DEADBAND_MM))
    {
        return AVOID_DIRECTION_RIGHT;
    }

    /* 두 공간 차이가 작으면 직전 성공 방향을 재사용해 좌우 떨림을 막는다. */
    return preferred_direction;
}

/* 정면 회피 시작~최대 회전 거리를 0~1 위험도로 바꾼다. */
static float avoid_front_proximity(uint16_t front_mm)
{
    float numerator;
    float denominator = (float)(ROVER_AVOID_TRIGGER_MM
                              - ROVER_AVOID_FULL_TURN_MM);

    if (front_mm >= ROVER_AVOID_TRIGGER_MM) { return 0.0f; }
    if (front_mm <= ROVER_AVOID_FULL_TURN_MM) { return 1.0f; }

    numerator = (float)(ROVER_AVOID_TRIGGER_MM - front_mm);
    return avoid_clamp(numerator / denominator, 0.0f, 1.0f);
}

/* 회피 방향의 원래 Yaw 기준 오프셋을 0~최대 회피각 안에 제한한다. */
static void avoid_step_outward_heading(float yaw_rate_deg_s,
                                       uint32_t elapsed_time_ms)
{
    float sign = avoid_direction_sign(avoid_direction);
    float step = yaw_rate_deg_s * ((float)elapsed_time_ms / 1000.0f);
    float signed_offset;

    commanded_heading_deg = avoid_normalize_360(commanded_heading_deg
                                               + (sign * step));
    signed_offset = avoid_angle_error(commanded_heading_deg,
                                      original_heading_deg) * sign;
    signed_offset = avoid_clamp(signed_offset,
                                0.0f,
                                ROVER_AVOID_MAX_OUT_YAW_DEG);
    commanded_heading_deg = avoid_normalize_360(original_heading_deg
                                               + (sign * signed_offset));
}

/* 지정한 목표 Yaw를 향해 명령 Yaw 자체를 서서히 이동시킨다. */
static void avoid_move_heading_toward(float target_deg,
                                      float rate_deg_s,
                                      uint32_t elapsed_time_ms)
{
    float error = avoid_angle_error(target_deg, commanded_heading_deg);
    float maximum_step = rate_deg_s * ((float)elapsed_time_ms / 1000.0f);

    if (error > maximum_step) { error = maximum_step; }
    if (error < -maximum_step) { error = -maximum_step; }

    commanded_heading_deg = avoid_normalize_360(commanded_heading_deg + error);
}

/* 안쪽 45도 센서가 본 절대 방향을 정면 목표로 바꾼다. 센서 광선이 원래
 * 진행 방향의 반대쪽까지 넘어간 경우에는 원래 Yaw에서 제한해 장애물 쪽으로
 * 과도하게 되감기는 것을 막는다. */
static float avoid_open_heading_from_sensor(float current_heading_deg)
{
    float sign = avoid_direction_sign(avoid_direction);
    float candidate = avoid_normalize_360(
        current_heading_deg - (sign * ROVER_SIDE_SENSOR_ANGLE_DEG));
    float outward_offset = avoid_angle_error(candidate,
                                             original_heading_deg) * sign;

    if (outward_offset < 0.0f)
    {
        return original_heading_deg;
    }
    return candidate;
}

/* 실제 Yaw가 목표 범위에 일정 시간 머물렀는지 확인한다. */
static bool avoid_heading_held(float target_deg, float current_deg)
{
    uint32_t now = HAL_GetTick();

    if (avoid_angle_difference(target_deg, current_deg)
        <= ROVER_AVOID_HEADING_TOLERANCE_DEG)
    {
        if (!heading_holding)
        {
            heading_holding = true;
            heading_hold_start_ms = now;
        }
        else if ((now - heading_hold_start_ms) >= ROVER_AVOID_HEADING_HOLD_MS)
        {
            return true;
        }
    }
    else
    {
        heading_holding = false;
        heading_hold_start_ms = 0U;
    }

    return false;
}

/* 회피를 시작하며 원래 방향과 이동거리, 선택 방향을 래치한다. */
static void avoid_start_event(const proximity_snapshot_t *sensors,
                              float current_heading_deg,
                              float current_distance_mm)
{
    avoid_direction = avoid_select_direction(sensors);
    original_heading_deg = avoid_normalize_360(current_heading_deg);
    commanded_heading_deg = original_heading_deg;
    event_start_distance_mm = current_distance_mm;
    event_start_ms = HAL_GetTick();
    maximum_proximity = 0.0f;
    inside_seen = false;
    inside_seen_count = 0U;
    edge_clear_count = 0U;
    open_front_count = 0U;
    untracked_clear_count = 0U;
    last_inside_sequence = 0U;
    avoid_failure = AVOID_FAILURE_NONE;
    started_event = true;

    if (avoid_direction == AVOID_DIRECTION_NONE)
    {
        avoid_fail(AVOID_FAILURE_NO_DIRECTION, current_distance_mm);
        return;
    }

    /* 전진 곡선만으로는 가까운 벽 앞에서 회전각을 확보하기 전에 충돌 거리에
     * 도달한다. 먼저 열린 방향으로 제자리 45도 회전한 뒤 곡선 회피를 시작한다. */
    commanded_heading_deg = avoid_normalize_360(
        original_heading_deg
        + (avoid_direction_sign(avoid_direction)
           * ROVER_AVOID_INITIAL_OUT_YAW_DEG));

    avoid_change_state(AVOID_STATE_TURN_OUT, current_distance_mm);
    avoid_set_rotate_command(commanded_heading_deg);
}

/* 회피 이벤트 직후 전진하지 않고 열린 방향으로 초기 회피각을 먼저 확보한다. */
static void avoid_update_turn_out(float current_heading_deg,
                                  float current_distance_mm)
{
    avoid_set_rotate_command(commanded_heading_deg);

    if (avoid_heading_held(commanded_heading_deg, current_heading_deg))
    {
        /* 제자리 회전의 바퀴 이동량이 이후 장애물 끝 이동거리로 섞이지 않게 한다. */
        event_start_distance_mm = current_distance_mm;
        avoid_change_state(AVOID_STATE_CURVE_OUT, current_distance_mm);
        avoid_set_track_command(ROVER_AVOID_MIN_RPM, commanded_heading_deg);
    }
}

/* 즉시 충돌 위험이 있는 센서값을 확인하고 안전정지 상태로 전환한다. */
static bool avoid_check_emergency(const proximity_snapshot_t *sensors,
                                  float current_distance_mm)
{
    bool front_danger =
        (sensors->front.raw_distance_mm <= ROVER_AVOID_FRONT_EMERGENCY_MM);
    bool left_danger =
        (sensors->left.raw_distance_mm <= ROVER_AVOID_SIDE_EMERGENCY_MM);
    bool right_danger =
        (sensors->right.raw_distance_mm <= ROVER_AVOID_SIDE_EMERGENCY_MM);

    if (!front_danger && !left_danger && !right_danger)
    {
        return false;
    }

    if (left_danger && !right_danger)
    {
        escape_direction = AVOID_DIRECTION_RIGHT;
    }
    else if (right_danger && !left_danger)
    {
        escape_direction = AVOID_DIRECTION_LEFT;
    }
    else
    {
        escape_direction = avoid_select_direction(sensors);
    }

    avoid_set_stop_command();
    avoid_change_state(AVOID_STATE_SAFETY_STOP, current_distance_mm);
    return true;
}

/* IDLE에서 새 정면 측정만 세어 500mm 3회 또는 즉시 위험을 회피 이벤트로 만든다. */
static void avoid_update_idle(const proximity_snapshot_t *sensors,
                              float current_heading_deg,
                              float current_distance_mm)
{
    if (!avoid_take_new_sequence(sensors->front.sequence,
                                 &last_front_sequence))
    {
        return;
    }

    if (sensors->front.distance_mm <= ROVER_AVOID_TRIGGER_MM)
    {
        if (front_trigger_count < ROVER_AVOID_TRIGGER_CONFIRM_COUNT)
        {
            front_trigger_count++;
        }
    }
    else
    {
        front_trigger_count = 0U;
    }

    if (sensors->front.raw_distance_mm <= ROVER_AVOID_FRONT_EMERGENCY_MM)
    {
        front_trigger_count = ROVER_AVOID_TRIGGER_CONFIRM_COUNT;
    }

    if (front_trigger_count >= ROVER_AVOID_TRIGGER_CONFIRM_COUNT)
    {
        front_trigger_count = 0U;
        avoid_start_event(sensors, current_heading_deg, current_distance_mm);
    }
}

/* 장애물에 가까워질수록 속도를 낮추고 목표 Yaw 증가율을 높인다. */
static void avoid_update_curve_out(const proximity_snapshot_t *sensors,
                                   float current_heading_deg,
                                   float current_distance_mm,
                                   uint32_t elapsed_time_ms)
{
    const proximity_sample_t *inside = avoid_inside_sample(sensors);
    float proximity = avoid_front_proximity(sensors->front.distance_mm);
    float yaw_rate;
    float base_rpm;
    float traveled = avoid_distance_delta(current_distance_mm,
                                           event_start_distance_mm);

    /* 정면 센서가 회전 때문에 장애물을 놓쳐도 이미 커진 회전 강도는 줄이지 않는다. */
    if (proximity > maximum_proximity)
    {
        maximum_proximity = proximity;
    }

    yaw_rate = ROVER_AVOID_MIN_YAW_RATE_DEG_S
             + (maximum_proximity
                * (ROVER_AVOID_MAX_YAW_RATE_DEG_S
                 - ROVER_AVOID_MIN_YAW_RATE_DEG_S));
    base_rpm = ROVER_AVOID_BASE_RPM
             - (maximum_proximity
                * (ROVER_AVOID_BASE_RPM - ROVER_AVOID_MIN_RPM));

    avoid_step_outward_heading(yaw_rate, elapsed_time_ms);
    avoid_set_track_command(base_rpm, commanded_heading_deg);

    if (avoid_take_new_sequence(inside->sequence, &last_inside_sequence))
    {
        if (inside->distance_mm <= ROVER_AVOID_SIDE_TRACK_SEEN_MM)
        {
            if (inside_seen_count < ROVER_AVOID_SIDE_TRACK_CONFIRM_COUNT)
            {
                inside_seen_count++;
            }
        }
        else
        {
            inside_seen_count = 0U;
        }

        if (inside_seen_count >= ROVER_AVOID_SIDE_TRACK_CONFIRM_COUNT)
        {
            inside_seen = true;
            edge_clear_count = 0U;
            avoid_change_state(AVOID_STATE_FOLLOW_EDGE,
                               current_distance_mm);
            return;
        }
    }

    /* 폭이 좁아 측면 센서가 장애물을 못 잡은 경우에도 충분히 회전·전진했고
     * 정면과 안쪽이 계속 열려 있으면 열린 방향 정렬로 넘어간다. */
    if (avoid_take_new_sequence(sensors->front.sequence,
                                 &last_front_sequence))
    {
        if ((sensors->front.distance_mm >= ROVER_AVOID_UNTRACKED_CLEAR_MM)
            && (inside->distance_mm >= ROVER_AVOID_EDGE_CLEAR_MM))
        {
            if (untracked_clear_count < ROVER_AVOID_EDGE_CONFIRM_COUNT)
            {
                untracked_clear_count++;
            }
        }
        else
        {
            untracked_clear_count = 0U;
        }
    }

    if ((untracked_clear_count >= ROVER_AVOID_EDGE_CONFIRM_COUNT)
        && (traveled >= ROVER_AVOID_UNTRACKED_TRAVEL_MM)
        && (avoid_angle_difference(commanded_heading_deg,
                                   original_heading_deg)
            >= ROVER_AVOID_UNTRACKED_MIN_YAW_DEG))
    {
        open_heading_deg = avoid_open_heading_from_sensor(current_heading_deg);
        open_front_count = 0U;
        avoid_change_state(AVOID_STATE_ALIGN_OPENING, current_distance_mm);
    }
}

/* 안쪽 센서를 목표 측면 거리 근처에 유지하며 장애물 끝을 찾는다. */
static void avoid_update_follow_edge(const proximity_snapshot_t *sensors,
                                     float current_heading_deg,
                                     float current_distance_mm,
                                     uint32_t elapsed_time_ms)
{
    const proximity_sample_t *inside = avoid_inside_sample(sensors);
    float traveled = avoid_distance_delta(current_distance_mm,
                                           event_start_distance_mm);
    float front_proximity = avoid_front_proximity(sensors->front.distance_mm);
    float side_error = (float)ROVER_AVOID_SIDE_TARGET_MM
                     - (float)inside->distance_mm;
    float normalized_error;
    float yaw_rate;
    float base_rpm = ROVER_AVOID_BASE_RPM
                   - (front_proximity
                      * (ROVER_AVOID_BASE_RPM - ROVER_AVOID_MIN_RPM));

    /* 상태가 FOLLOW_EDGE로 바뀐 직후에도 정면 벽이 남아 있을 수 있다.
     * 이때는 측면 거리보다 정면 충돌 회피를 우선해 바깥쪽 회전을 계속한다. */
    if (front_proximity > 0.0f)
    {
        yaw_rate = ROVER_AVOID_MIN_YAW_RATE_DEG_S
                 + (front_proximity
                    * (ROVER_AVOID_MAX_YAW_RATE_DEG_S
                     - ROVER_AVOID_MIN_YAW_RATE_DEG_S));
        avoid_step_outward_heading(yaw_rate, elapsed_time_ms);
    }
    else if (inside->distance_mm < ROVER_AVOID_EDGE_CLEAR_MM)
    {
        if (side_error > (float)ROVER_AVOID_SIDE_DEADBAND_MM)
        {
            normalized_error = avoid_clamp(
                side_error / (float)ROVER_AVOID_SIDE_TARGET_MM,
                0.0f,
                1.0f);
            yaw_rate = ROVER_AVOID_MIN_YAW_RATE_DEG_S
                     + (normalized_error
                        * (ROVER_AVOID_MAX_YAW_RATE_DEG_S
                         - ROVER_AVOID_MIN_YAW_RATE_DEG_S));
            avoid_step_outward_heading(yaw_rate, elapsed_time_ms);
        }
        else if (side_error < -(float)ROVER_AVOID_SIDE_DEADBAND_MM)
        {
            /* 장애물에서 너무 멀어지면 원래 방향 쪽으로 천천히 되돌린다. */
            avoid_move_heading_toward(original_heading_deg,
                                      ROVER_AVOID_MIN_YAW_RATE_DEG_S,
                                      elapsed_time_ms);
        }
    }

    avoid_set_track_command(base_rpm, commanded_heading_deg);

    if (avoid_take_new_sequence(inside->sequence, &last_inside_sequence))
    {
        if ((inside->distance_mm >= ROVER_AVOID_EDGE_CLEAR_MM)
            && (traveled >= ROVER_AVOID_EDGE_MIN_TRAVEL_MM))
        {
            if (edge_clear_count < ROVER_AVOID_EDGE_CONFIRM_COUNT)
            {
                edge_clear_count++;
            }
        }
        else
        {
            edge_clear_count = 0U;
        }
    }

    if (inside_seen && (edge_clear_count >= ROVER_AVOID_EDGE_CONFIRM_COUNT))
    {
        /* 안쪽 45도 센서가 본 열린 절대 방향을 정면 센서의 목표로 바꾼다. */
        open_heading_deg = avoid_open_heading_from_sensor(current_heading_deg);
        open_front_count = 0U;
        avoid_change_state(AVOID_STATE_ALIGN_OPENING, current_distance_mm);
    }
}

/* 측면 센서가 봤던 열린 방향으로 목표 Yaw를 이동하고 정면 센서로 재확인한다. */
static void avoid_update_align_opening(const proximity_snapshot_t *sensors,
                                       float current_heading_deg,
                                       float current_distance_mm,
                                       uint32_t elapsed_time_ms)
{
    avoid_move_heading_toward(open_heading_deg,
                              ROVER_AVOID_RETURN_YAW_RATE_DEG_S,
                              elapsed_time_ms);
    avoid_set_track_command(ROVER_AVOID_MIN_RPM, commanded_heading_deg);

    if (avoid_take_new_sequence(sensors->front.sequence,
                                 &last_front_sequence))
    {
        if (sensors->front.distance_mm >= ROVER_AVOID_OPEN_FRONT_MM)
        {
            if (open_front_count < ROVER_AVOID_OPEN_CONFIRM_COUNT)
            {
                open_front_count++;
            }
        }
        else
        {
            open_front_count = 0U;
        }
    }

    if ((open_front_count >= ROVER_AVOID_OPEN_CONFIRM_COUNT)
        && (avoid_angle_difference(open_heading_deg, current_heading_deg)
            <= ROVER_AVOID_HEADING_TOLERANCE_DEG))
    {
        avoid_change_state(AVOID_STATE_CLEAR_BODY, current_distance_mm);
    }
}

/* 열린 방향을 유지한다. 실제 복귀 시점은 회전 뒤 총 500mm 이동거리로 정한다. */
static void avoid_update_clear_body(float current_distance_mm)
{
    (void)current_distance_mm;
    avoid_set_track_command(ROVER_AVOID_MIN_RPM, open_heading_deg);
}

/* 초기 제자리 회전 뒤 전진한 총거리가 약 500mm가 되면 센서 상태가 늦게
 * 바뀌더라도 더 멀리 진행하지 않고 원래 Yaw 복귀를 시작한다. */
static void avoid_limit_forward_travel(float current_distance_mm)
{
    bool moving_around_obstacle =
        (avoid_state == AVOID_STATE_CURVE_OUT)
        || (avoid_state == AVOID_STATE_FOLLOW_EDGE)
        || (avoid_state == AVOID_STATE_ALIGN_OPENING)
        || (avoid_state == AVOID_STATE_CLEAR_BODY);

    if (moving_around_obstacle
        && (avoid_distance_delta(current_distance_mm,
                                 event_start_distance_mm)
            >= ROVER_AVOID_FORWARD_TRAVEL_MM))
    {
        avoid_change_state(AVOID_STATE_RETURN_ORIGINAL,
                           current_distance_mm);
    }
}

/* 약 500mm 전진한 뒤 추가 전진 없이 제자리에서 원래 Yaw로 복귀한다. */
static void avoid_update_return_original(const proximity_snapshot_t *sensors,
                                         float current_heading_deg,
                                         float current_distance_mm,
                                         uint32_t elapsed_time_ms)
{
    bool path_clear =
        (sensors->front.distance_mm >= ROVER_AVOID_FINAL_FRONT_CLEAR_MM)
        && (sensors->left.distance_mm > ROVER_AVOID_SIDE_EMERGENCY_MM)
        && (sensors->right.distance_mm > ROVER_AVOID_SIDE_EMERGENCY_MM);

    (void)elapsed_time_ms;
    commanded_heading_deg = original_heading_deg;
    avoid_set_rotate_command(original_heading_deg);

    if (path_clear
        && avoid_heading_held(original_heading_deg, current_heading_deg))
    {
        preferred_direction = avoid_direction;
        avoid_set_stop_command();
        avoid_change_state(AVOID_STATE_COMPLETED, current_distance_mm);
    }
}

/* 비상정지 후 현재 세 방향을 다시 확인하고 장애물 반대쪽 45도 목표를 만든다. */
static void avoid_update_safety_stop(const proximity_snapshot_t *sensors,
                                     float current_heading_deg,
                                     float current_distance_mm)
{
    const proximity_sample_t *escape_side;

    avoid_set_stop_command();
    if ((HAL_GetTick() - state_start_ms) < ROVER_ESCAPE_SETTLE_MS)
    {
        return;
    }

    if (escape_direction == AVOID_DIRECTION_NONE)
    {
        escape_direction = avoid_select_direction(sensors);
    }
    escape_side = (escape_direction == AVOID_DIRECTION_LEFT)
                ? &sensors->left
                : &sensors->right;

    /* 후진 센서가 없으므로 전방과 탈출 방향이 확인되지 않으면 움직이지 않는다. */
    if ((escape_direction == AVOID_DIRECTION_NONE)
        || (sensors->front.distance_mm < ROVER_ESCAPE_REQUIRED_OPEN_MM)
        || (escape_side->distance_mm < ROVER_ESCAPE_REQUIRED_OPEN_MM))
    {
        avoid_fail(AVOID_FAILURE_ESCAPE_BLOCKED, current_distance_mm);
        return;
    }

    commanded_heading_deg = avoid_normalize_360(current_heading_deg);
    escape_heading_deg = avoid_normalize_360(
        current_heading_deg
        + (avoid_direction_sign(escape_direction) * ROVER_ESCAPE_TURN_DEG));
    avoid_change_state(AVOID_STATE_SAFETY_TURN, current_distance_mm);
}

/* 정면 여유를 더 줄이지 않도록 전진 곡선 대신 제자리에서 열린 방향으로 회전한다. */
static void avoid_update_safety_turn(const proximity_snapshot_t *sensors,
                                     float current_heading_deg,
                                     float current_distance_mm,
                                     uint32_t elapsed_time_ms)
{
    const proximity_sample_t *escape_side =
        (escape_direction == AVOID_DIRECTION_LEFT)
      ? &sensors->left
      : &sensors->right;

    if ((sensors->front.raw_distance_mm <= ROVER_AVOID_FRONT_EMERGENCY_MM)
        || (escape_side->raw_distance_mm <= ROVER_AVOID_SIDE_EMERGENCY_MM))
    {
        avoid_fail(AVOID_FAILURE_ESCAPE_BLOCKED, current_distance_mm);
        return;
    }

    (void)elapsed_time_ms;
    commanded_heading_deg = escape_heading_deg;
    avoid_set_rotate_command(escape_heading_deg);

    if (avoid_heading_held(escape_heading_deg, current_heading_deg))
    {
        avoid_change_state(AVOID_STATE_SAFETY_ADVANCE,
                           current_distance_mm);
    }
}

/* 탈출 방향으로 차체 길이 이상 전진한 뒤 원래 Yaw 복귀 단계로 합류한다. */
static void avoid_update_safety_advance(const proximity_snapshot_t *sensors,
                                        float current_distance_mm)
{
    const proximity_sample_t *escape_side =
        (escape_direction == AVOID_DIRECTION_LEFT)
      ? &sensors->left
      : &sensors->right;

    if ((sensors->front.raw_distance_mm <= ROVER_AVOID_FRONT_EMERGENCY_MM)
        || (escape_side->raw_distance_mm <= ROVER_AVOID_SIDE_EMERGENCY_MM))
    {
        avoid_fail(AVOID_FAILURE_ESCAPE_BLOCKED, current_distance_mm);
        return;
    }

    avoid_set_track_command(ROVER_AVOID_MIN_RPM, escape_heading_deg);
    if (avoid_distance_delta(current_distance_mm, state_start_distance_mm)
        >= ROVER_ESCAPE_ADVANCE_MM)
    {
        commanded_heading_deg = escape_heading_deg;
        avoid_change_state(AVOID_STATE_RETURN_ORIGINAL,
                           current_distance_mm);
    }
}

/* 회피 모듈의 초기 선호 방향과 모든 이벤트 상태를 초기화한다. */
void obstacle_avoidance_init(void)
{
    preferred_direction = AVOID_DIRECTION_LEFT;
    obstacle_avoidance_reset();
}

/* 진행 중 상태와 카운터를 지우고 다음 정면 장애물 감지 대기로 돌아간다. */
void obstacle_avoidance_reset(void)
{
    avoid_state = AVOID_STATE_IDLE;
    avoid_direction = AVOID_DIRECTION_NONE;
    escape_direction = AVOID_DIRECTION_NONE;
    avoid_failure = AVOID_FAILURE_NONE;
    avoid_command.motion = AVOID_MOTION_STOP;
    avoid_command.base_rpm = 0.0f;
    avoid_command.target_heading_deg = 0.0f;
    original_heading_deg = 0.0f;
    commanded_heading_deg = 0.0f;
    open_heading_deg = 0.0f;
    escape_heading_deg = 0.0f;
    event_start_distance_mm = 0.0f;
    state_start_distance_mm = 0.0f;
    maximum_proximity = 0.0f;
    event_start_ms = HAL_GetTick();
    state_start_ms = event_start_ms;
    heading_hold_start_ms = 0U;
    heading_holding = false;
    inside_seen = false;
    started_event = false;
    state_change_event = false;
    last_front_sequence = 0U;
    last_inside_sequence = 0U;
    front_trigger_count = 0U;
    inside_seen_count = 0U;
    edge_clear_count = 0U;
    open_front_count = 0U;
    untracked_clear_count = 0U;
}

/* 한 제어 주기의 센서·Yaw·거리를 받아 상태 전이와 다음 주행 명령을 계산한다. */
void obstacle_avoidance_update(const proximity_snapshot_t *sensors,
                               float current_heading_deg,
                               float current_distance_mm,
                               uint32_t elapsed_time_ms)
{
    uint32_t now = HAL_GetTick();

    if ((sensors == NULL) || (elapsed_time_ms == 0U))
    {
        return;
    }

    if (!sensors->all_healthy)
    {
        if (avoid_state != AVOID_STATE_IDLE)
        {
            avoid_fail(AVOID_FAILURE_SENSOR, current_distance_mm);
        }
        return;
    }

    if (avoid_state == AVOID_STATE_IDLE)
    {
        avoid_update_idle(sensors, current_heading_deg, current_distance_mm);
        return;
    }
    if ((avoid_state == AVOID_STATE_COMPLETED)
        || (avoid_state == AVOID_STATE_FAILED))
    {
        return;
    }

    if (((now - event_start_ms) >= ROVER_AVOID_TOTAL_TIMEOUT_MS)
        || ((now - state_start_ms) >= ROVER_AVOID_STATE_TIMEOUT_MS))
    {
        avoid_fail(AVOID_FAILURE_TIMEOUT, current_distance_mm);
        return;
    }
    if (avoid_distance_delta(current_distance_mm, event_start_distance_mm)
        >= ROVER_AVOID_MAX_TRAVEL_MM)
    {
        avoid_fail(AVOID_FAILURE_MAX_TRAVEL, current_distance_mm);
        return;
    }

    if ((avoid_state != AVOID_STATE_SAFETY_STOP)
        && (avoid_state != AVOID_STATE_SAFETY_TURN)
        && (avoid_state != AVOID_STATE_SAFETY_ADVANCE)
        && avoid_check_emergency(sensors, current_distance_mm))
    {
        return;
    }

    /* 충돌 위험 검사가 항상 우선이다. 안전한 상태라면 회전 완료 뒤 약 500mm에서
     * 현재 센서 세부 단계와 관계없이 원래 방향 복귀를 시작한다. */
    avoid_limit_forward_travel(current_distance_mm);

    switch (avoid_state)
    {
        case AVOID_STATE_TURN_OUT:
            avoid_update_turn_out(current_heading_deg,
                                  current_distance_mm);
            break;

        case AVOID_STATE_CURVE_OUT:
            avoid_update_curve_out(sensors,
                                   current_heading_deg,
                                   current_distance_mm,
                                   elapsed_time_ms);
            break;

        case AVOID_STATE_FOLLOW_EDGE:
            avoid_update_follow_edge(sensors,
                                     current_heading_deg,
                                     current_distance_mm,
                                     elapsed_time_ms);
            break;

        case AVOID_STATE_ALIGN_OPENING:
            avoid_update_align_opening(sensors,
                                       current_heading_deg,
                                       current_distance_mm,
                                       elapsed_time_ms);
            break;

        case AVOID_STATE_CLEAR_BODY:
            avoid_update_clear_body(current_distance_mm);
            break;

        case AVOID_STATE_RETURN_ORIGINAL:
            avoid_update_return_original(sensors,
                                         current_heading_deg,
                                         current_distance_mm,
                                         elapsed_time_ms);
            break;

        case AVOID_STATE_SAFETY_STOP:
            avoid_update_safety_stop(sensors,
                                     current_heading_deg,
                                     current_distance_mm);
            break;

        case AVOID_STATE_SAFETY_TURN:
            avoid_update_safety_turn(sensors,
                                     current_heading_deg,
                                     current_distance_mm,
                                     elapsed_time_ms);
            break;

        case AVOID_STATE_SAFETY_ADVANCE:
            avoid_update_safety_advance(sensors, current_distance_mm);
            break;

        default:
            break;
    }
}

/* 새 회피 시작 이벤트를 호출자 한 곳에서 한 번만 소비하게 반환한다. */
bool obstacle_avoidance_take_started(void)
{
    bool occurred = started_event;
    started_event = false;
    return occurred;
}

/* 상태 변경 이벤트를 한 번 소비하고 로그용 현재 상태를 복사한다. */
bool obstacle_avoidance_take_state_change(avoid_state_t *state)
{
    bool occurred = state_change_event;
    state_change_event = false;

    if ((state != NULL) && occurred)
    {
        *state = avoid_state;
    }
    return occurred;
}

/* 하드웨어 독립적인 현재 정지/목표 Yaw 주행 명령을 복사한다. */
void obstacle_avoidance_get_command(obstacle_avoidance_command_t *command)
{
    if (command != NULL)
    {
        *command = avoid_command;
    }
}

/* 현재 회피 상태를 반환한다. */
avoid_state_t obstacle_avoidance_get_state(void)
{
    return avoid_state;
}

/* 이벤트 시작 때 래치한 회피 방향을 반환한다. */
avoid_direction_t obstacle_avoidance_get_direction(void)
{
    return avoid_direction;
}

/* 실패 정지의 원인을 반환한다. */
avoid_failure_t obstacle_avoidance_get_failure(void)
{
    return avoid_failure;
}

/* IDLE·완료·실패가 아닌 실제 회피 진행 상태인지 반환한다. */
bool obstacle_avoidance_is_running(void)
{
    return (avoid_state != AVOID_STATE_IDLE)
        && (avoid_state != AVOID_STATE_COMPLETED)
        && (avoid_state != AVOID_STATE_FAILED);
}

/* 원래 Yaw까지 안전하게 복귀해 회피가 정상 완료됐는지 반환한다. */
bool obstacle_avoidance_is_completed(void)
{
    return (avoid_state == AVOID_STATE_COMPLETED);
}

/* 안전 조건이나 제한을 만족하지 못해 실패 정지했는지 반환한다. */
bool obstacle_avoidance_has_failed(void)
{
    return (avoid_state == AVOID_STATE_FAILED);
}

/* 장애물 끝 판정 전에 안쪽 센서가 가까운 면을 확인했는지 반환한다. */
bool obstacle_avoidance_inside_obstacle_seen(void)
{
    return inside_seen;
}

/* 상태 열거값을 UART 로그용 짧은 문자열로 바꾼다. */
const char *obstacle_avoidance_state_name(avoid_state_t state)
{
    switch (state)
    {
        case AVOID_STATE_IDLE:            return "IDLE";
        case AVOID_STATE_TURN_OUT:        return "TURN-OUT";
        case AVOID_STATE_CURVE_OUT:       return "CURVE-OUT";
        case AVOID_STATE_FOLLOW_EDGE:     return "FOLLOW-EDGE";
        case AVOID_STATE_ALIGN_OPENING:   return "ALIGN-OPEN";
        case AVOID_STATE_CLEAR_BODY:      return "CLEAR-BODY";
        case AVOID_STATE_RETURN_ORIGINAL: return "RETURN-YAW";
        case AVOID_STATE_SAFETY_STOP:     return "SAFETY-STOP";
        case AVOID_STATE_SAFETY_TURN:     return "SAFETY-TURN";
        case AVOID_STATE_SAFETY_ADVANCE:  return "SAFETY-ADVANCE";
        case AVOID_STATE_COMPLETED:       return "AVOID-DONE";
        case AVOID_STATE_FAILED:          return "AVOID-FAILED";
        default:                          return "UNKNOWN";
    }
}
