#ifndef MPU6050_H
#define MPU6050_H

#include <stdbool.h>
#include <stdint.h>

typedef struct{
    float accel_x_g;          // X축 가속도, 단위 g
    float accel_y_g;          // Y축 가속도, 단위 g
    float accel_z_g;          // Z축 가속도, 단위 g

    float gyro_x_dps;         // X축 각속도, 단위 degree/s
    float gyro_y_dps;         // Y축 각속도, 단위 degree/s
    float gyro_z_dps;         // Z축 각속도, 단위 degree/s

    float temperature_c;      // MPU6050 내부 온도, 단위 섭씨
} mpu6050_data_t;

bool mpu6050_init(void);  // MPU6050 연결을 확인하고 측정 레지스터를 초기화

bool mpu6050_update(void);  // MPU6050에서 최신 가속도와 자이로 데이터를 읽음

bool mpu6050_get_data(mpu6050_data_t *data);  // 마지막으로 측정한 IMU 데이터를 출력 매개변수로 전달

// 정지 상태에서 지정된 횟수만큼 측정하여 자이로 영점 오차를 계산
bool mpu6050_calibrate_gyro(uint16_t sample_count);  

bool mpu6050_is_ready(void);  // MPU6050 초기화 및 통신 가능 여부를 반환

// MPU6050 초기화 + 워밍업 대기 + 자이로 영점 보정을 한 번에 수행
bool mpu6050_start(void);

// 현재 자세를 기준(Roll/Pitch/Yaw = 0도)으로 재설정
void mpu6050_orientation_reset(void);

// 최근 mpu6050_update() 결과(보정된 gyro dps)를 실제 dt로 적분하여 Roll/Pitch/Yaw를 누적
bool mpu6050_orientation_update(void);

// 기준 자세 대비 현재 상대 Roll/Pitch/Yaw(degree)를 반환
bool mpu6050_get_orientation(float *roll_deg, float *pitch_deg, float *yaw_deg);

#endif