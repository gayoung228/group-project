#ifndef ROAD_DETECTOR_H
#define ROAD_DETECTOR_H

#include "mpu6050.h"
#include "road_state.h"

typedef struct{
    float small_vibration_threshold;    // 작은 진동 판단 임계값
    float large_vibration_threshold;    // 큰 진동 판단 임계값
} road_detector_config_t;

void road_detector_init(void);    // 노면 감지 상태와 필터값을 초기화

void road_detector_reset(void);   // 누적된 진동값과 노면 상태를 초기화

// 최신 IMU 데이터를 이용해 진동값과 노면 상태를 갱신
void road_detector_update(const mpu6050_data_t *imu_data);    

// 작은 진동과 큰 진동의 판단 임계값을 설정
void road_detector_set_config(const road_detector_config_t *config);

road_state_t road_detector_get_state(void);    // 현재 판단된 노면 상태를 반환

float road_detector_get_vibration(void);    // 현재 계산된 진동 크기를 반환

#endif