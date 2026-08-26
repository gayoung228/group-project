#ifndef ROAD_STATE_H
#define ROAD_STATE_H

typedef enum{
    ROAD_STATE_FLAT,               // 평평하고 안정적인 노면
    ROAD_STATE_SMALL_VIBRATION,    // 작은 진동이 발생하는 노면
    ROAD_STATE_LARGE_VIBRATION     // 큰 진동이 발생하는 위험 노면
} road_state_t;

#endif