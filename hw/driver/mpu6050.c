#include "main.h"
#include "mpu6050.h"
#include "stm32f4xx_hal.h"

/* MPU6050 7비트 I2C 주소 (AD0 = GND), HAL은 8비트 주소를 사용하므로 1비트 왼쪽으로 민다 */
#define MPU6050_I2C_ADDR            (0x68 << 1)

/* 레지스터 주소 */
#define MPU6050_REG_SMPLRT_DIV      0x19
#define MPU6050_REG_CONFIG          0x1A
#define MPU6050_REG_GYRO_CONFIG     0x1B
#define MPU6050_REG_ACCEL_CONFIG    0x1C
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_WHO_AM_I        0x75
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_GYRO_XOUT_H     0x43

/* WHO_AM_I 는 개체마다 값이 다를 수 있어 실제로 확인된 값을 모두 허용한다.
 * 0x68 : 데이터시트 기본값 / 0x70 : 실측한 보드에서 확인된 값 */
#define MPU6050_WHO_AM_I_VALUE      0x68
#define MPU6050_WHO_AM_I_VALUE_ALT  0x70

/* 초기화 시 설정하는 감도 (가속도 ±2g, 자이로 ±250 dps) 기준 변환 계수 */
#define MPU6050_ACCEL_SENS_LSB_PER_G    16384.0f
#define MPU6050_GYRO_SENS_LSB_PER_DPS   131.0f

#define MPU6050_I2C_TIMEOUT_MS      100

/* 자이로 영점 보정 시 측정 간격 [ms] */
#define MPU6050_CALIB_SAMPLE_DELAY_MS   10

/* 전원 인가 직후 자이로 바이어스가 안정될 때까지 보정 전에 기다리는 워밍업 시간 [ms] */
#define MPU6050_CALIB_WARMUP_TIME   2000


/* CubeMX 가 생성한 I2C 핸들 */
extern I2C_HandleTypeDef hi2c1;


static bool mpu6050_ready = false;

/* 마지막으로 측정한 가속도·자이로·온도 값 */
static mpu6050_data_t mpu6050_last_data;

/* 자이로 영점 오프셋 (degree/s). mpu6050_calibrate_gyro()로 계산한다.
 * 디버거 Watch에서 축별 계산된 offset 값을 그대로 확인할 수 있다. */
static float mpu6050_gyro_offset_x = 0.0f;
static float mpu6050_gyro_offset_y = 0.0f;
static float mpu6050_gyro_offset_z = 0.0f;

/* 마지막으로 읽은 자이로 원시(raw) 레지스터 값.
 * 디버거 Watch에서 보정 전 raw 값을 확인할 수 있다. */
static int16_t mpu6050_gyro_raw_x = 0;
static int16_t mpu6050_gyro_raw_y = 0;
static int16_t mpu6050_gyro_raw_z = 0;


/* 레지스터를 읽는 내부 함수 */
static bool mpu6050_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    return (HAL_I2C_Mem_Read(&hi2c1, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                              buf, len, MPU6050_I2C_TIMEOUT_MS) == HAL_OK);
}

/* 레지스터 1개에 값을 쓰는 내부 함수 */
static bool mpu6050_write_reg(uint8_t reg, uint8_t value)
{
    return (HAL_I2C_Mem_Write(&hi2c1, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
                               &value, 1, MPU6050_I2C_TIMEOUT_MS) == HAL_OK);
}

/* MPU6050 연결을 확인하고 측정 레지스터를 초기화 */
bool mpu6050_init(void)
{
    uint8_t who_am_i = 0;

    mpu6050_ready = false;

    if (!mpu6050_read_regs(MPU6050_REG_WHO_AM_I, &who_am_i, 1))
    {
        return false;
    }

    if (who_am_i != MPU6050_WHO_AM_I_VALUE && who_am_i != MPU6050_WHO_AM_I_VALUE_ALT)
    {
        return false;
    }

    /* 절전 모드를 해제하고 내부 클럭을 사용 */
    if (!mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00))
    {
        return false;
    }

    if (!mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV, 0x07))
    {
        return false;
    }

    if (!mpu6050_write_reg(MPU6050_REG_CONFIG, 0x00))
    {
        return false;
    }

    /* 자이로 풀스케일 ±250 dps */
    if (!mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, 0x00))
    {
        return false;
    }

    /* 가속도 풀스케일 ±2g */
    if (!mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, 0x00))
    {
        return false;
    }

    mpu6050_ready = true;

    return true;
}

/* MPU6050에서 최신 가속도와 자이로 데이터를 읽음 */
bool mpu6050_update(void)
{
    uint8_t buf[14];
    int16_t raw_accel_x, raw_accel_y, raw_accel_z;
    int16_t raw_temp;

    if (!mpu6050_ready)
    {
        return false;
    }

    if (!mpu6050_read_regs(MPU6050_REG_ACCEL_XOUT_H, buf, 14))
    {
        return false;
    }

    raw_accel_x = (int16_t)((buf[0] << 8) | buf[1]);
    raw_accel_y = (int16_t)((buf[2] << 8) | buf[3]);
    raw_accel_z = (int16_t)((buf[4] << 8) | buf[5]);
    raw_temp    = (int16_t)((buf[6] << 8) | buf[7]);

    /* 보정 전 raw 값 : 디버거 Watch 확인용 */
    mpu6050_gyro_raw_x = (int16_t)((buf[8]  << 8) | buf[9]);
    mpu6050_gyro_raw_y = (int16_t)((buf[10] << 8) | buf[11]);
    mpu6050_gyro_raw_z = (int16_t)((buf[12] << 8) | buf[13]);

    mpu6050_last_data.accel_x_g = raw_accel_x / MPU6050_ACCEL_SENS_LSB_PER_G;
    mpu6050_last_data.accel_y_g = raw_accel_y / MPU6050_ACCEL_SENS_LSB_PER_G;
    mpu6050_last_data.accel_z_g = raw_accel_z / MPU6050_ACCEL_SENS_LSB_PER_G;

    mpu6050_last_data.temperature_c = (raw_temp / 340.0f) + 36.53f;

    /* raw 값을 degree/s로 변환한 뒤 영점 오프셋을 빼서 보정된 값을 사용한다 */
    mpu6050_last_data.gyro_x_dps = (mpu6050_gyro_raw_x / MPU6050_GYRO_SENS_LSB_PER_DPS) - mpu6050_gyro_offset_x;
    mpu6050_last_data.gyro_y_dps = (mpu6050_gyro_raw_y / MPU6050_GYRO_SENS_LSB_PER_DPS) - mpu6050_gyro_offset_y;
    mpu6050_last_data.gyro_z_dps = (mpu6050_gyro_raw_z / MPU6050_GYRO_SENS_LSB_PER_DPS) - mpu6050_gyro_offset_z;

    return true;
}

/* 마지막으로 측정한 IMU 데이터를 출력 매개변수로 전달 */
bool mpu6050_get_data(mpu6050_data_t *data)
{
    if (!mpu6050_ready || data == NULL)
    {
        return false;
    }

    *data = mpu6050_last_data;

    return true;
}

/* 정지 상태에서 지정된 횟수만큼 측정하여 자이로 영점 오차를 계산 */
bool mpu6050_calibrate_gyro(uint16_t sample_count)
{
    int32_t sum_x = 0;
    int32_t sum_y = 0;
    int32_t sum_z = 0;
    uint16_t i;

    if (!mpu6050_ready || sample_count == 0)
    {
        return false;
    }

    for (i = 0; i < sample_count; i++)
    {
        /* mpu6050_update() 와 동일한 14바이트 경로로 읽어서
         * mpu6050_gyro_raw_x/y/z 를 갱신한다 (디버거 Watch 확인용) */
        if (!mpu6050_update())
        {
            return false;
        }

        sum_x += mpu6050_gyro_raw_x;
        sum_y += mpu6050_gyro_raw_y;
        sum_z += mpu6050_gyro_raw_z;

        HAL_Delay(MPU6050_CALIB_SAMPLE_DELAY_MS);
    }

    /* 평균값을 degree/s 단위의 영점 오프셋으로 저장 : 디버거 Watch 확인용 */
    mpu6050_gyro_offset_x = (sum_x / (float)sample_count) / MPU6050_GYRO_SENS_LSB_PER_DPS;
    mpu6050_gyro_offset_y = (sum_y / (float)sample_count) / MPU6050_GYRO_SENS_LSB_PER_DPS;
    mpu6050_gyro_offset_z = (sum_z / (float)sample_count) / MPU6050_GYRO_SENS_LSB_PER_DPS;

    return true;
}

/* MPU6050 초기화 및 통신 가능 여부를 반환 */
bool mpu6050_is_ready(void)
{
    return mpu6050_ready;
}

/* MPU6050 초기화 + 워밍업 대기 + 자이로 영점 보정을 한 번에 수행.
 * 초기화에 실패하면 워밍업/보정 없이 즉시 false를 반환한다. */
bool mpu6050_start(void)
{
    if (!mpu6050_init())
    {
        return false;
    }

    HAL_Delay(MPU6050_CALIB_WARMUP_TIME);

    return mpu6050_calibrate_gyro(300);
}
