#include "main.h"
#include "proximity_monitor.h"
#include "rover_config.h"
#include "vl53l0x.h"
#include <string.h>

/* 중앙값 필터에 사용하는 최근 측정 개수. 한 번의 튀는 값을 제거하면서
 * 3회 연속 판정과 별개로 센서 입력 자체를 안정화한다. */
#define PROXIMITY_FILTER_COUNT  3U

typedef struct
{
    proximity_sample_t sample;
    uint16_t history[PROXIMITY_FILTER_COUNT];
    uint8_t history_count;
    uint8_t history_index;
    uint8_t success_count;
    uint8_t failure_count;
} proximity_sensor_state_t;

static proximity_sensor_state_t sensor_state[VL53L0X_COUNT];

/* 3개 이하의 거리값을 정렬해 중앙값을 반환한다. */
static uint16_t proximity_median(const uint16_t *values, uint8_t count)
{
    uint16_t sorted[PROXIMITY_FILTER_COUNT];
    uint8_t i;
    uint8_t j;

    for (i = 0U; i < count; i++)
    {
        sorted[i] = values[i];
    }

    for (i = 1U; i < count; i++)
    {
        uint16_t value = sorted[i];
        j = i;

        while ((j > 0U) && (sorted[j - 1U] > value))
        {
            sorted[j] = sorted[j - 1U];
            j--;
        }
        sorted[j] = value;
    }

    return sorted[count / 2U];
}

/* 센서 한 개의 정상 결과를 필터에 넣고 신선도 판단에 필요한 시각과 순번을 갱신한다. */
static void proximity_record_measurement(vl53l0x_id_t sensor,
                                         uint16_t raw_distance_mm)
{
    proximity_sensor_state_t *state = &sensor_state[sensor];
    uint16_t normalized_distance = raw_distance_mm;

    /* 0은 정상 거리가 아니다. 반면 1200mm를 넘는 정상 완료값은 가까운
     * 반사체가 없다는 뜻이므로 제어용 NO_TARGET 거리로 통일한다. */
    if (raw_distance_mm == 0U)
    {
        if (state->failure_count < UINT8_MAX)
        {
            state->failure_count++;
        }
        return;
    }
    if (raw_distance_mm > VL53L0X_MAX_VALID_MM)
    {
        normalized_distance = PROXIMITY_NO_TARGET_MM;
    }

    state->history[state->history_index] = normalized_distance;
    state->history_index = (uint8_t)((state->history_index + 1U)
                                  % PROXIMITY_FILTER_COUNT);
    if (state->history_count < PROXIMITY_FILTER_COUNT)
    {
        state->history_count++;
    }

    state->sample.raw_distance_mm = normalized_distance;
    state->sample.distance_mm = proximity_median(state->history,
                                                  state->history_count);
    state->sample.updated_ms = HAL_GetTick();
    state->sample.sequence++;
    state->sample.has_target =
        (state->sample.distance_mm <= VL53L0X_MAX_VALID_MM);
    state->failure_count = 0U;

    if (state->success_count < ROVER_DISTANCE_READY_COUNT)
    {
        state->success_count++;
    }
}

/* 센서 한 개의 I2C/측정 실패를 연속 실패 카운터에 반영한다. */
static void proximity_record_failure(vl53l0x_id_t sensor)
{
    proximity_sensor_state_t *state = &sensor_state[sensor];

    if (state->failure_count < UINT8_MAX)
    {
        state->failure_count++;
    }
}

/* ready, 정상 측정 횟수, 연속 오류, 마지막 측정 나이를 종합해 healthy를 갱신한다. */
static void proximity_refresh_health(vl53l0x_id_t sensor, uint32_t now)
{
    proximity_sensor_state_t *state = &sensor_state[sensor];
    bool fresh = (state->sample.sequence > 0U)
              && ((now - state->sample.updated_ms) <= ROVER_DISTANCE_STALE_MS);

    state->sample.ready = vl53l0x_is_ready(sensor);
    state->sample.healthy = state->sample.ready
                         && fresh
                         && (state->success_count >= ROVER_DISTANCE_READY_COUNT)
                         && (state->failure_count < ROVER_DISTANCE_FAIL_COUNT);
}

/* 세 센서의 소프트웨어 상태를 측정 전 초기값으로 되돌린다. */
static void proximity_reset_state(void)
{
    vl53l0x_id_t sensor;

    memset(sensor_state, 0, sizeof(sensor_state));
    for (sensor = VL53L0X_FRONT; sensor < VL53L0X_COUNT; sensor++)
    {
        sensor_state[sensor].sample.distance_mm = PROXIMITY_NO_TARGET_MM;
        sensor_state[sensor].sample.raw_distance_mm = PROXIMITY_NO_TARGET_MM;
    }
}

/* 세 센서 소프트웨어 상태를 지우고 XSHUT 주소 할당을 시작한다. */
bool proximity_monitor_init(void)
{
    proximity_reset_state();
    return vl53l0x_init_all();
}

/* non-blocking 측정을 진행하고 새 값·오류·신선도를 한 주기 갱신한다. */
void proximity_monitor_update(void)
{
    vl53l0x_id_t sensor;
    uint32_t now;

    /* 드라이버는 각 센서의 측정을 한 단계만 진행하므로 메인 루프를 막지 않는다. */
    vl53l0x_update();

    for (sensor = VL53L0X_FRONT; sensor < VL53L0X_COUNT; sensor++)
    {
        if (vl53l0x_take_measurement_error(sensor))
        {
            proximity_record_failure(sensor);
        }

        if (vl53l0x_take_new_measurement(sensor))
        {
            proximity_record_measurement(sensor,
                                          vl53l0x_get_distance_mm(sensor));
        }
    }

    now = HAL_GetTick();
    for (sensor = VL53L0X_FRONT; sensor < VL53L0X_COUNT; sensor++)
    {
        proximity_refresh_health(sensor, now);
    }
}

/* 세 센서가 각각 연속 정상값을 만들 때까지 제한 시간 동안 기다린다. */
bool proximity_monitor_wait_until_ready(uint32_t timeout_ms)
{
    uint32_t start_ms = HAL_GetTick();

    while ((HAL_GetTick() - start_ms) < timeout_ms)
    {
        proximity_monitor_update();
        if (proximity_monitor_has_fault() == false)
        {
            return true;
        }
        HAL_Delay(5U);
    }

    return false;
}

/* 제어 중 값이 섞이지 않도록 현재 세 센서 상태를 구조체 하나로 복사한다. */
void proximity_monitor_get_snapshot(proximity_snapshot_t *snapshot)
{
    if (snapshot == NULL)
    {
        return;
    }

    snapshot->front = sensor_state[VL53L0X_FRONT].sample;
    snapshot->left = sensor_state[VL53L0X_LEFT].sample;
    snapshot->right = sensor_state[VL53L0X_RIGHT].sample;
    snapshot->all_healthy = !proximity_monitor_has_fault();
}

/* 세 센서 중 준비·연속 성공·신선도 조건을 잃은 센서가 있는지 반환한다. */
bool proximity_monitor_has_fault(void)
{
    vl53l0x_id_t sensor;

    for (sensor = VL53L0X_FRONT; sensor < VL53L0X_COUNT; sensor++)
    {
        if (sensor_state[sensor].sample.healthy == false)
        {
            return true;
        }
    }

    return false;
}
