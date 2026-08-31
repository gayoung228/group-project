#include "main.h"
#include "mpu6050.h"
#include "stm32f4xx_hal.h"
#include <math.h>

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

/* 전원 직후 첫 WHO_AM_I 접근이 너무 빨라지는 것을 막고, 순간적인 NACK/BUSY를
 * 한 번의 전체 초기화 실패로 확정하지 않기 위한 여유 시간과 재시도 횟수다. */
#define MPU6050_POWER_STABILIZE_MS    200U
#define MPU6050_I2C_RETRY_COUNT         3U
#define MPU6050_I2C_RETRY_DELAY_MS      5U
#define MPU6050_START_RETRY_COUNT       3U
#define MPU6050_START_RETRY_DELAY_MS   50U

/* 자이로 영점 보정 시 측정 간격 [ms] */
#define MPU6050_CALIB_SAMPLE_DELAY_MS   10

/* 전원 인가 직후 자이로 바이어스가 안정될 때까지 보정 전에 기다리는 워밍업 시간 [ms] */
#define MPU6050_CALIB_WARMUP_TIME   2000

/* X/Y 상보 필터 시간상수. 약 1초에 걸쳐 중력 기준으로 천천히 복귀한다. */
#define MPU6050_COMPLEMENTARY_TAU_S  1.0f

/* 큰 선형 가속 중에는 중력 방향이 왜곡되므로 가속도 보정을 잠시 끈다. */
#define MPU6050_ACCEL_MAG_MIN_G      0.75f
#define MPU6050_ACCEL_MAG_MAX_G      1.25f

#define MPU6050_RAD_TO_DEG           57.2957795f


/* CubeMX 가 생성한 I2C 핸들 */
extern I2C_HandleTypeDef hi2c1;


static bool mpu6050_ready = false;

/* 부팅 로그에서 NACK, BUSY, 잘못된 WHO_AM_I를 구별하기 위한 마지막 실패 정보 */
static const char *mpu6050_last_error_stage = "none";
static uint8_t mpu6050_last_who_am_i = 0U;
static HAL_StatusTypeDef mpu6050_last_hal_status = HAL_OK;
static uint32_t mpu6050_last_i2c_error = HAL_I2C_ERROR_NONE;
static HAL_I2C_StateTypeDef mpu6050_last_i2c_state = HAL_I2C_STATE_READY;

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

/* 기준 자세 대비 누적된 상대 회전각 (degree).
 * mpu6050_orientation_reset()으로 0도로 되돌리고, mpu6050_orientation_update()가 갱신한다. */
static float mpu6050_roll_deg  = 0.0f;
static float mpu6050_pitch_deg = 0.0f;
static float mpu6050_yaw_deg   = 0.0f;
static float mpu6050_accel_roll_reference_deg = 0.0f;
static float mpu6050_accel_pitch_reference_deg = 0.0f;


/* 각도 차이를 -180~180도 범위로 정리한다. */
static float mpu6050_normalize_angle(float angle_deg)
{
    while (angle_deg > 180.0f)
    {
        angle_deg -= 360.0f;
    }
    while (angle_deg < -180.0f)
    {
        angle_deg += 360.0f;
    }
    return angle_deg;
}

/* 중력 벡터로 Roll/Pitch 절대 기울기를 계산한다. */
static void mpu6050_get_accel_angles(float *roll_deg, float *pitch_deg)
{
    float ax = mpu6050_last_data.accel_x_g;
    float ay = mpu6050_last_data.accel_y_g;
    float az = mpu6050_last_data.accel_z_g;

    *roll_deg = atan2f(ay, az) * MPU6050_RAD_TO_DEG;
    *pitch_deg = atan2f(-ax, sqrtf((ay * ay) + (az * az)))
               * MPU6050_RAD_TO_DEG;
}


/* 레지스터를 읽는 내부 함수 */
static bool mpu6050_read_regs(uint8_t reg, uint8_t *buf, uint16_t len)
{
    uint8_t attempt;

    for (attempt = 0U; attempt < MPU6050_I2C_RETRY_COUNT; attempt++)
    {
        mpu6050_last_hal_status = HAL_I2C_Mem_Read(
            &hi2c1, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
            buf, len, MPU6050_I2C_TIMEOUT_MS);

        if (mpu6050_last_hal_status == HAL_OK)
        {
            mpu6050_last_i2c_error = HAL_I2C_ERROR_NONE;
            mpu6050_last_i2c_state = HAL_I2C_GetState(&hi2c1);
            return true;
        }

        mpu6050_last_i2c_error = HAL_I2C_GetError(&hi2c1);
        mpu6050_last_i2c_state = HAL_I2C_GetState(&hi2c1);
        HAL_Delay(MPU6050_I2C_RETRY_DELAY_MS);
    }

    return false;
}

/* 레지스터 1개에 값을 쓰는 내부 함수 */
static bool mpu6050_write_reg(uint8_t reg, uint8_t value)
{
    uint8_t attempt;

    for (attempt = 0U; attempt < MPU6050_I2C_RETRY_COUNT; attempt++)
    {
        mpu6050_last_hal_status = HAL_I2C_Mem_Write(
            &hi2c1, MPU6050_I2C_ADDR, reg, I2C_MEMADD_SIZE_8BIT,
            &value, 1, MPU6050_I2C_TIMEOUT_MS);

        if (mpu6050_last_hal_status == HAL_OK)
        {
            mpu6050_last_i2c_error = HAL_I2C_ERROR_NONE;
            mpu6050_last_i2c_state = HAL_I2C_GetState(&hi2c1);
            return true;
        }

        mpu6050_last_i2c_error = HAL_I2C_GetError(&hi2c1);
        mpu6050_last_i2c_state = HAL_I2C_GetState(&hi2c1);
        HAL_Delay(MPU6050_I2C_RETRY_DELAY_MS);
    }

    return false;
}

/* MPU6050 연결을 확인하고 측정 레지스터를 초기화 */
bool mpu6050_init(void)
{
    uint8_t who_am_i = 0;

    mpu6050_ready = false;

    if (!mpu6050_read_regs(MPU6050_REG_WHO_AM_I, &who_am_i, 1))
    {
        mpu6050_last_error_stage = "WHO_AM_I read";
        return false;
    }

    mpu6050_last_who_am_i = who_am_i;

    if (who_am_i != MPU6050_WHO_AM_I_VALUE && who_am_i != MPU6050_WHO_AM_I_VALUE_ALT)
    {
        mpu6050_last_error_stage = "WHO_AM_I value";
        return false;
    }

    /* 절전 모드를 해제하고 내부 클럭을 사용 */
    if (!mpu6050_write_reg(MPU6050_REG_PWR_MGMT_1, 0x00))
    {
        mpu6050_last_error_stage = "PWR_MGMT_1 write";
        return false;
    }

    if (!mpu6050_write_reg(MPU6050_REG_SMPLRT_DIV, 0x07))
    {
        mpu6050_last_error_stage = "SMPLRT_DIV write";
        return false;
    }

    if (!mpu6050_write_reg(MPU6050_REG_CONFIG, 0x03))
    {
        mpu6050_last_error_stage = "CONFIG write";
        return false;
    }

    /* 자이로 풀스케일 ±250 dps */
    if (!mpu6050_write_reg(MPU6050_REG_GYRO_CONFIG, 0x00))
    {
        mpu6050_last_error_stage = "GYRO_CONFIG write";
        return false;
    }

    /* 가속도 풀스케일 ±2g */
    if (!mpu6050_write_reg(MPU6050_REG_ACCEL_CONFIG, 0x00))
    {
        mpu6050_last_error_stage = "ACCEL_CONFIG write";
        return false;
    }

    mpu6050_ready = true;
    mpu6050_last_error_stage = "none";

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
 * 한 번 실패하면 I2C 주변장치를 다시 시작하고 전체 과정을 재시도한다. */
bool mpu6050_start(void)
{
    uint8_t attempt;

    mpu6050_last_error_stage = "none";
    mpu6050_last_who_am_i = 0U;
    mpu6050_last_hal_status = HAL_OK;
    mpu6050_last_i2c_error = HAL_I2C_ERROR_NONE;
    mpu6050_last_i2c_state = HAL_I2C_GetState(&hi2c1);

    /* MCU보다 센서 전원이 늦게 안정되는 보드에서도 첫 접근이 NACK가 되지 않게 한다. */
    HAL_Delay(MPU6050_POWER_STABILIZE_MS);

    for (attempt = 0U; attempt < MPU6050_START_RETRY_COUNT; attempt++)
    {
        if (mpu6050_init())
        {
            HAL_Delay(MPU6050_CALIB_WARMUP_TIME);

            if (mpu6050_calibrate_gyro(300) == true)
            {
                /* 보정이 끝난 정지 자세를 X/Y/Z의 0도 기준으로 잡는다. */
                mpu6050_orientation_reset();
                mpu6050_last_error_stage = "none";
                return true;
            }

            mpu6050_last_error_stage = "gyro calibration";
            mpu6050_ready = false;
        }

        if ((attempt + 1U) < MPU6050_START_RETRY_COUNT)
        {
            /* HAL 내부 BUSY 상태도 함께 지우기 위해 공용 I2C 주변장치를 다시 시작한다.
             * 호출은 단일 스레드의 부팅/정지 복구 구간에서만 이루어진다. */
            (void)HAL_I2C_DeInit(&hi2c1);
            HAL_Delay(MPU6050_I2C_RETRY_DELAY_MS);
            mpu6050_last_hal_status = HAL_I2C_Init(&hi2c1);
            mpu6050_last_i2c_error = HAL_I2C_GetError(&hi2c1);
            mpu6050_last_i2c_state = HAL_I2C_GetState(&hi2c1);
            HAL_Delay(MPU6050_START_RETRY_DELAY_MS);
        }
    }

    return false;
}

const char *mpu6050_get_last_error_stage(void)
{
    return mpu6050_last_error_stage;
}

uint8_t mpu6050_get_last_who_am_i(void)
{
    return mpu6050_last_who_am_i;
}

uint32_t mpu6050_get_last_hal_status(void)
{
    return (uint32_t)mpu6050_last_hal_status;
}

uint32_t mpu6050_get_last_i2c_error(void)
{
    return mpu6050_last_i2c_error;
}

uint32_t mpu6050_get_last_i2c_state(void)
{
    return (uint32_t)mpu6050_last_i2c_state;
}

/* 현재 자세를 기준(Roll/Pitch/Yaw = 0도)으로 재설정 */
void mpu6050_orientation_reset(void)
{
    mpu6050_get_accel_angles(&mpu6050_accel_roll_reference_deg,
                             &mpu6050_accel_pitch_reference_deg);

    mpu6050_roll_deg  = 0.0f;
    mpu6050_pitch_deg = 0.0f;
    mpu6050_yaw_deg   = 0.0f;
}

/* X/Y는 자이로 적분과 가속도 중력각을 상보 필터로 합치고,
 * 절대 기준이 없는 Z(Yaw)는 보정된 자이로를 계속 적분한다.
 * I2C를 다시 읽지 않으므로, 호출자가 mpu6050_update()를 먼저 불러야 한다.
 * 적분 시간을 호출자가 정하므로 제어 주기와 항상 일치한다. */
bool mpu6050_orientation_update(uint32_t elapsed_time_ms)
{
    float dt_s;
    float accel_roll_deg;
    float accel_pitch_deg;
    float accel_magnitude_g;
    float alpha;
    float gyro_roll_deg;
    float gyro_pitch_deg;

    if (!mpu6050_ready || elapsed_time_ms == 0)
    {
        return false;
    }

    dt_s = elapsed_time_ms / 1000.0f;

    gyro_roll_deg = mpu6050_roll_deg
                  + (mpu6050_last_data.gyro_x_dps * dt_s);
    gyro_pitch_deg = mpu6050_pitch_deg
                   + (mpu6050_last_data.gyro_y_dps * dt_s);

    accel_magnitude_g = sqrtf(
        (mpu6050_last_data.accel_x_g * mpu6050_last_data.accel_x_g)
      + (mpu6050_last_data.accel_y_g * mpu6050_last_data.accel_y_g)
      + (mpu6050_last_data.accel_z_g * mpu6050_last_data.accel_z_g));

    if ((accel_magnitude_g >= MPU6050_ACCEL_MAG_MIN_G)
        && (accel_magnitude_g <= MPU6050_ACCEL_MAG_MAX_G))
    {
        mpu6050_get_accel_angles(&accel_roll_deg, &accel_pitch_deg);
        accel_roll_deg = mpu6050_normalize_angle(
            accel_roll_deg - mpu6050_accel_roll_reference_deg);
        accel_pitch_deg = mpu6050_normalize_angle(
            accel_pitch_deg - mpu6050_accel_pitch_reference_deg);

        alpha = MPU6050_COMPLEMENTARY_TAU_S
              / (MPU6050_COMPLEMENTARY_TAU_S + dt_s);

        mpu6050_roll_deg = (alpha * gyro_roll_deg)
                         + ((1.0f - alpha) * accel_roll_deg);
        mpu6050_pitch_deg = (alpha * gyro_pitch_deg)
                          + ((1.0f - alpha) * accel_pitch_deg);
    }
    else
    {
        /* 충격이나 급가속 중에는 가속도각을 믿지 않고 자이로만 사용한다. */
        mpu6050_roll_deg = gyro_roll_deg;
        mpu6050_pitch_deg = gyro_pitch_deg;
    }

    mpu6050_yaw_deg   += mpu6050_last_data.gyro_z_dps * dt_s;

    return true;
}

/* 기준 자세 대비 현재 상대 Roll/Pitch/Yaw(degree)를 반환 */
bool mpu6050_get_orientation(float *roll_deg, float *pitch_deg, float *yaw_deg)
{
    if (!mpu6050_ready || roll_deg == NULL || pitch_deg == NULL || yaw_deg == NULL)
    {
        return false;
    }

    *roll_deg  = mpu6050_roll_deg;
    *pitch_deg = mpu6050_pitch_deg;
    *yaw_deg   = mpu6050_yaw_deg;

    return true;
}
