#ifndef PROXIMITY_MONITOR_H
#define PROXIMITY_MONITOR_H

#include <stdbool.h>
#include <stdint.h>

/* 반사 물체가 측정 범위에 없을 때 제어기에 전달하는 충분히 먼 거리값 */
#define PROXIMITY_NO_TARGET_MM  8190U

/* 센서 하나의 최신 상태다. sequence는 새 측정이 확정될 때마다 증가하므로
 * 제어기가 같은 캐시값을 여러 번 연속 측정으로 잘못 세는 일을 막는다. */
typedef struct
{
    uint16_t distance_mm;
    uint16_t raw_distance_mm;
    uint32_t updated_ms;
    uint32_t sequence;
    bool ready;
    bool healthy;
    bool has_target;
} proximity_sample_t;

/* 세 센서를 같은 시점에 복사한 읽기 전용 스냅샷 */
typedef struct
{
    proximity_sample_t front;
    proximity_sample_t left;
    proximity_sample_t right;
    bool all_healthy;
} proximity_snapshot_t;

/* XSHUT으로 LEFT/FRONT/RIGHT에 서로 다른 주소를 부여하고 상태를 초기화한다. */
bool proximity_monitor_init(void);

/* 센서별 non-blocking 측정 상태 머신을 한 단계 진행하고 새 결과를 저장한다. */
void proximity_monitor_update(void);

/* 부팅 또는 복구 시 세 센서가 연속 정상 측정을 만들 때까지 기다린다. */
bool proximity_monitor_wait_until_ready(uint32_t timeout_ms);

/* 최신 거리·유효성·측정 순번을 한 번에 복사한다. */
void proximity_monitor_get_snapshot(proximity_snapshot_t *snapshot);

/* 세 센서 중 하나라도 초기화 실패·연속 오류·오래된 값이면 true다. */
bool proximity_monitor_has_fault(void);

#endif
