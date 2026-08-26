#ifndef VL53L0X_H
#define VL53L0X_H

#include <stdbool.h>
#include <stdint.h>

#define VL53L0X_DEFAULT_ADDRESS    0x29
#define VL53L0X_FRONT_ADDRESS      0x30
#define VL53L0X_LEFT_ADDRESS       0x31
#define VL53L0X_RIGHT_ADDRESS      0x32

typedef enum{
    VL53L0X_FRONT,    // 차량 전방에 장착된 거리 센서
    VL53L0X_LEFT,     // 차량 좌측에 장착된 거리 센서
    VL53L0X_RIGHT,    // 차량 우측에 장착된 거리 센서
    VL53L0X_COUNT     // 사용하는 VL53L0X 센서 개수
} vl53l0x_id_t;

void vl53l0x_init(void);  // I2C와 XSHUT 제어 상태를 초기화

// XSHUT으로 선택한 센서만 활성화하고 새로운 I2C 주소를 설정
bool vl53l0x_init_sensor(vl53l0x_id_t sensor, uint8_t new_address);  

bool vl53l0x_init_all(void);  // 전방·좌측·우측 센서를 순차적으로 활성화하고 주소를 설정

bool vl53l0x_update_sensor(vl53l0x_id_t sensor);  // 선택한 센서의 거리값을 한 번 측정하고 저장

void vl53l0x_update(void);  // 세 센서를 순차적으로 측정하여 거리값을 갱신

uint16_t vl53l0x_get_distance_mm(vl53l0x_id_t sensor);  // 선택한 센서의 최근 거리값을 mm 단위로 반환

bool vl53l0x_is_ready(vl53l0x_id_t sensor);  // 선택한 센서의 초기화 성공 여부를 반환

bool vl53l0x_is_valid(vl53l0x_id_t sensor);  // 선택한 센서의 최근 측정값이 유효한지 반환

#endif