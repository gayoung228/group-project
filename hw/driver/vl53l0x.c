/* ============================================================================
 * VL53L0X ToF 거리 센서 드라이버 (STM32 HAL I2C1 기반)
 *
 * 초기화/SPAD 캘리브레이션/거리 측정 레지스터 시퀀스는 아래 검증된 오픈소스
 * 구현을 확인한 뒤 STM32 HAL I2C 환경에 맞게 이식했다. Arduino Wire/millis
 * 의존 코드는 모두 제거하고 HAL_I2C_Mem_Read/Write, HAL_GetTick 기반의
 * 정적 함수로 재작성했다.
 *
 *   출처   : Pololu VL53L0X Arduino Library (VL53L0X.h / VL53L0X.cpp)
 *            https://github.com/pololu/vl53l0x-arduino (2026-08-27 확인, master)
 *   라이선스 : MIT License, Copyright (c) 2017-2022 Pololu Corporation.
 *              이 라이브러리 자체가 ST의 VL53L0X API(STSW-IMG005)에서 파생되어
 *              ST의 BSD 스타일 라이선스 고지도 함께 적용된다.
 *              (원본 저장소 LICENSE.txt 참고)
 *
 * 레지스터 이름/주소는 위 소스의 VL53L0X.h `regAddr` enum을 따랐고,
 * 초기화 시퀀스(DataInit → StaticInit → PerformRefCalibration)와 SPAD 설정,
 * DefaultTuningSettings 레지스터 값은 VL53L0X.cpp의 VL53L0X::init()을
 * 그대로 이식했다 (검증되지 않은 값은 사용하지 않음).
 * tBOOT(XSHUT High 이후 통신 가능해질 때까지의 최대 대기시간, 1.2 ms)는
 * ST VL53L0X 데이터시트(DocID029104 Rev 2, 2.9.1절 "Power up and boot
 * sequence")에서 확인했다.
 * ============================================================================ */

#include "main.h"
#include "vl53l0x.h"
#include "stm32f4xx_hal.h"
#include <string.h>

/* ----------------------------------------------------------------------------
 * 3센서(LEFT/FRONT/RIGHT) 순차 초기화 스위치.
 *
 * 0 : FRONT 센서 1개만 초기화한다.
 * 1 : docs/pinmap.md 순서(LEFT → FRONT → RIGHT)대로 3개를 순차 초기화한다.
 * 현재 로버는 3개 장착 구성이므로 1을 사용한다.
 * -------------------------------------------------------------------------- */
#define VL53L0X_MULTI_SENSOR_ENABLE    1

/* I2C 레지스터 1회 접근(HAL_I2C_Mem_*) 타임아웃 [ms] */
#define VL53L0X_I2C_TIMEOUT_MS         50

/* 순간적인 NACK/BUSY는 짧게 다시 시도하고, 주소 할당 전체가 실패하면 모든
 * XSHUT을 내린 뒤 처음부터 다시 시작한다. */
#define VL53L0X_I2C_RETRY_COUNT          3U
#define VL53L0X_I2C_RETRY_DELAY_MS       3U
#define VL53L0X_INIT_RETRY_COUNT         3U
#define VL53L0X_INIT_RETRY_DELAY_MS     20U
#define VL53L0X_RESET_LOW_MS            10U

/* 센서 응답(측정 완료, 캘리브레이션 완료 등) 대기 타임아웃 [ms].
 * Pololu 라이브러리 예제(examples/Single/Single.ino)의 sensor.setTimeout(500)과 동일한 값이다. */
#define VL53L0X_IO_TIMEOUT_MS          500

/* XSHUT High 이후 I2C 통신이 가능해질 때까지의 대기 시간 [ms].
 * 데이터시트 tBOOT(최대 1.2 ms)에 여유를 둔 값이다. */
#define VL53L0X_BOOT_DELAY_MS           10U

/* 레지스터 주소 (Pololu VL53L0X.h의 regAddr enum 중 이 파일에서 이름으로 쓰는 것만 발췌) */
#define VL53L0X_REG_SYSRANGE_START                              0x00
#define VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG                      0x01
#define VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO                0x0A
#define VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR                      0x0B
#define VL53L0X_REG_RESULT_INTERRUPT_STATUS                     0x13
#define VL53L0X_REG_RESULT_RANGE_STATUS                         0x14
#define VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS                    0x8A
#define VL53L0X_REG_MSRC_CONFIG_CONTROL                         0x60
#define VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP                  0x46
#define VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD               0x50
#define VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          0x51
#define VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD             0x70
#define VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        0x71
#define VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT 0x44
#define VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH                     0x84
#define VL53L0X_REG_IDENTIFICATION_MODEL_ID                     0xC0
#define VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV            0x89
#define VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0            0xB0
#define VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT           0xB6
#define VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET            0x4F
#define VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         0x4E

/* 데이터시트에 명시된 모델 ID 레지스터 값 */
#define VL53L0X_MODEL_ID_VALUE          0xEE

/* CubeMX 가 생성한 I2C 핸들 (mpu6050.c와 동일한 I2C1 버스 공유) */
extern I2C_HandleTypeDef hi2c1;


/* 센서별 XSHUT 핀과 최종 I2C 주소 매핑 (main.h GPIO 라벨 + vl53l0x.h 주소 정의를 그대로 사용) */
typedef struct
{
    GPIO_TypeDef *xshut_port;
    uint16_t      xshut_pin;
    uint8_t       target_address;
} vl53l0x_pin_map_t;

static const vl53l0x_pin_map_t vl53l0x_pin_map[VL53L0X_COUNT] =
{
    [VL53L0X_FRONT] = { TOF_FRONT_XSHUT_GPIO_Port, TOF_FRONT_XSHUT_Pin, VL53L0X_FRONT_ADDRESS },
    [VL53L0X_LEFT]  = { TOF_LEFT_XSHUT_GPIO_Port,  TOF_LEFT_XSHUT_Pin,  VL53L0X_LEFT_ADDRESS  },
    [VL53L0X_RIGHT] = { TOF_RIGHT_XSHUT_GPIO_Port, TOF_RIGHT_XSHUT_Pin, VL53L0X_RIGHT_ADDRESS },
};

/* 센서 1개의 non-blocking 단발 측정 상태머신 단계.
 *
 * START(쓰기만 하고 끝)와 READ_RESULT(읽고 캐시 갱신만 하고 끝)는 기다릴 조건이
 * 없는 즉시 실행 액션이라 별도 단계로 유지하지 않고, IDLE 진입 시 / WAIT_RESULT에서
 * 준비 확인된 그 즉시 같은 호출 안에서 처리한다. 그래서 실제로 여러 호출에 걸쳐
 * 유지되는 단계는 이 3개뿐이다. */
typedef enum
{
    VL53L0X_MEAS_IDLE = 0,          /* 새 측정을 시작할 수 있는 상태 */
    VL53L0X_MEAS_WAIT_START_CLEAR,  /* SYSRANGE_START 비트가 0이 되길 기다리는 중 */
    VL53L0X_MEAS_WAIT_RESULT        /* RESULT_INTERRUPT_STATUS 가 준비되길 기다리는 중 */
} vl53l0x_meas_state_t;

/* 센서별 내부 상태 (ready/valid/거리값 + Pololu VL53L0X::init()이 쓰는 stop_variable) */
typedef struct
{
    uint8_t  address;       /* 현재 이 센서와 통신할 7비트 I2C 주소 */
    bool     ready;         /* 초기화 성공 여부 */
    bool     valid;         /* 가장 최근 측정값의 유효 여부 (0 < distance_mm <= VL53L0X_MAX_VALID_MM) */
    uint16_t distance_mm;   /* 가장 최근 측정 거리 [mm] (I2C가 성공한 원시 측정값, 유효성과 무관하게 저장) */
    bool     obstacle;      /* vl53l0x_obstacle_update()가 저장한 가장 최근 장애물 판정 결과 */
    uint8_t  stop_variable; /* VL53L0X::init()에서 읽어 측정 시작마다 재사용하는 내부 상태값 */

    vl53l0x_meas_state_t meas_state;      /* 현재 측정 상태머신 단계 */
    uint32_t              meas_state_tick; /* 현재 단계에 진입한 시각(HAL_GetTick), WAIT_* timeout 판정용 */
    bool                  measurement_new;   /* 새 거리 결과가 준비됐음을 알리는 1회성 이벤트 */
    bool                  measurement_error; /* I2C 실패/timeout이 발생했음을 알리는 1회성 이벤트 */
} vl53l0x_state_t;

static vl53l0x_state_t vl53l0x_state[VL53L0X_COUNT];

/* 한 줄짜리 ERROR 대신 어느 센서의 어느 단계가 실패했는지 남긴다. */
static vl53l0x_id_t vl53l0x_last_init_sensor = VL53L0X_COUNT;
static const char *vl53l0x_last_init_stage = "none";
static uint8_t vl53l0x_last_init_attempt = 0U;
static HAL_StatusTypeDef vl53l0x_last_hal_status = HAL_OK;
static uint32_t vl53l0x_last_i2c_error = HAL_I2C_ERROR_NONE;
static HAL_I2C_StateTypeDef vl53l0x_last_i2c_state = HAL_I2C_STATE_READY;


/* ----------------------------------------------------------------------------
 * 저수준 I2C 레지스터 접근 (HAL_I2C_Mem_*, addr은 7비트 주소)
 * -------------------------------------------------------------------------- */

static bool vl53l0x_i2c_write(uint8_t addr, uint8_t reg,
                              uint8_t *data, uint16_t length)
{
    uint8_t attempt;

    for (attempt = 0U; attempt < VL53L0X_I2C_RETRY_COUNT; attempt++)
    {
        vl53l0x_last_hal_status = HAL_I2C_Mem_Write(
            &hi2c1, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT,
            data, length, VL53L0X_I2C_TIMEOUT_MS);

        if (vl53l0x_last_hal_status == HAL_OK)
        {
            vl53l0x_last_i2c_error = HAL_I2C_ERROR_NONE;
            vl53l0x_last_i2c_state = HAL_I2C_GetState(&hi2c1);
            return true;
        }

        vl53l0x_last_i2c_error = HAL_I2C_GetError(&hi2c1);
        vl53l0x_last_i2c_state = HAL_I2C_GetState(&hi2c1);
        HAL_Delay(VL53L0X_I2C_RETRY_DELAY_MS);
    }

    return false;
}

static bool vl53l0x_i2c_read(uint8_t addr, uint8_t reg,
                             uint8_t *data, uint16_t length)
{
    uint8_t attempt;

    for (attempt = 0U; attempt < VL53L0X_I2C_RETRY_COUNT; attempt++)
    {
        vl53l0x_last_hal_status = HAL_I2C_Mem_Read(
            &hi2c1, (uint16_t)(addr << 1), reg, I2C_MEMADD_SIZE_8BIT,
            data, length, VL53L0X_I2C_TIMEOUT_MS);

        if (vl53l0x_last_hal_status == HAL_OK)
        {
            vl53l0x_last_i2c_error = HAL_I2C_ERROR_NONE;
            vl53l0x_last_i2c_state = HAL_I2C_GetState(&hi2c1);
            return true;
        }

        vl53l0x_last_i2c_error = HAL_I2C_GetError(&hi2c1);
        vl53l0x_last_i2c_state = HAL_I2C_GetState(&hi2c1);
        HAL_Delay(VL53L0X_I2C_RETRY_DELAY_MS);
    }

    return false;
}

static bool vl53l0x_write8(uint8_t addr, uint8_t reg, uint8_t value)
{
    return vl53l0x_i2c_write(addr, reg, &value, 1U);
}

static bool vl53l0x_write16(uint8_t addr, uint8_t reg, uint16_t value)
{
    uint8_t buf[2] = { (uint8_t)(value >> 8), (uint8_t)(value & 0xFF) };

    return vl53l0x_i2c_write(addr, reg, buf, 2U);
}

static bool vl53l0x_write_multi(uint8_t addr, uint8_t reg, const uint8_t *src, uint8_t count)
{
    return vl53l0x_i2c_write(addr, reg, (uint8_t *)src, count);
}

static bool vl53l0x_read8(uint8_t addr, uint8_t reg, uint8_t *value)
{
    return vl53l0x_i2c_read(addr, reg, value, 1U);
}

static bool vl53l0x_read16(uint8_t addr, uint8_t reg, uint16_t *value)
{
    uint8_t buf[2];

    if (!vl53l0x_i2c_read(addr, reg, buf, 2U))
    {
        return false;
    }

    *value = ((uint16_t)buf[0] << 8) | buf[1];
    return true;
}

static bool vl53l0x_read_multi(uint8_t addr, uint8_t reg, uint8_t *dst, uint8_t count)
{
    return vl53l0x_i2c_read(addr, reg, dst, count);
}

/* 실패 시 0을 반환하는 read8/read16.
 * 원본 Pololu 코드의 readReg()도 전송 실패 여부를 호출부에서 따로 검사하지 않고
 * "읽은 값을 그대로 read-modify-write에 사용"하는 방식이라 그 구조를 그대로 따른다. */
static uint8_t vl53l0x_reg8(uint8_t addr, uint8_t reg)
{
    uint8_t value = 0;
    (void)vl53l0x_read8(addr, reg, &value);
    return value;
}

static uint16_t vl53l0x_reg16(uint8_t addr, uint8_t reg)
{
    uint16_t value = 0;
    (void)vl53l0x_read16(addr, reg, &value);
    return value;
}


/* ----------------------------------------------------------------------------
 * 타이밍 계산 유틸리티
 * (VL53L0X_decode_vcsel_period / VL53L0X_calc_macro_period_ps 등을 그대로 이식)
 * -------------------------------------------------------------------------- */

static uint32_t vl53l0x_calc_macro_period_ns(uint8_t vcsel_period_pclks)
{
    /* PLL_period_ps = 1655, macro_period_vclks = 2304 */
    return (((uint32_t)2304 * vcsel_period_pclks * 1655) + 500) / 1000;
}

static uint8_t vl53l0x_decode_vcsel_period(uint8_t reg_val)
{
    return (uint8_t)((reg_val + 1) << 1);
}

static uint16_t vl53l0x_decode_timeout(uint16_t reg_val)
{
    /* 형식 : "(LSByte * 2^MSByte) + 1" */
    return (uint16_t)((reg_val & 0x00FF) << ((reg_val & 0xFF00) >> 8)) + 1;
}

static uint16_t vl53l0x_encode_timeout(uint32_t timeout_mclks)
{
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks == 0)
    {
        return 0;
    }

    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
        ls_byte >>= 1;
        ms_byte++;
    }

    return (uint16_t)((ms_byte << 8) | (ls_byte & 0xFF));
}

static uint32_t vl53l0x_timeout_mclks_to_us(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = vl53l0x_calc_macro_period_ns(vcsel_period_pclks);

    return ((timeout_period_mclks * macro_period_ns) + 500) / 1000;
}

static uint32_t vl53l0x_timeout_us_to_mclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = vl53l0x_calc_macro_period_ns(vcsel_period_pclks);

    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}

static uint8_t vl53l0x_get_vcsel_pulse_period_pre_range(uint8_t addr)
{
    return vl53l0x_decode_vcsel_period(vl53l0x_reg8(addr, VL53L0X_REG_PRE_RANGE_CONFIG_VCSEL_PERIOD));
}

static uint8_t vl53l0x_get_vcsel_pulse_period_final_range(uint8_t addr)
{
    return vl53l0x_decode_vcsel_period(vl53l0x_reg8(addr, VL53L0X_REG_FINAL_RANGE_CONFIG_VCSEL_PERIOD));
}


/* ----------------------------------------------------------------------------
 * 시퀀스 스텝 활성화/타임아웃 조회
 * (VL53L0X_GetSequenceStepEnables / get_sequence_step_timeout 이식)
 * -------------------------------------------------------------------------- */

typedef struct
{
    bool tcc, msrc, dss, pre_range, final_range;
} vl53l0x_seq_enables_t;

typedef struct
{
    uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us,    pre_range_us,    final_range_us;
} vl53l0x_seq_timeouts_t;

static void vl53l0x_get_sequence_step_enables(uint8_t addr, vl53l0x_seq_enables_t *enables)
{
    uint8_t sequence_config = vl53l0x_reg8(addr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG);

    enables->tcc         = (sequence_config >> 4) & 0x1;
    enables->dss         = (sequence_config >> 3) & 0x1;
    enables->msrc        = (sequence_config >> 2) & 0x1;
    enables->pre_range   = (sequence_config >> 6) & 0x1;
    enables->final_range = (sequence_config >> 7) & 0x1;
}

static void vl53l0x_get_sequence_step_timeouts(uint8_t addr, const vl53l0x_seq_enables_t *enables,
                                                vl53l0x_seq_timeouts_t *timeouts)
{
    timeouts->pre_range_vcsel_period_pclks = vl53l0x_get_vcsel_pulse_period_pre_range(addr);

    timeouts->msrc_dss_tcc_mclks = vl53l0x_reg8(addr, VL53L0X_REG_MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    timeouts->msrc_dss_tcc_us = vl53l0x_timeout_mclks_to_us(timeouts->msrc_dss_tcc_mclks,
                                                             timeouts->pre_range_vcsel_period_pclks);

    timeouts->pre_range_mclks = vl53l0x_decode_timeout(
        vl53l0x_reg16(addr, VL53L0X_REG_PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    timeouts->pre_range_us = vl53l0x_timeout_mclks_to_us(timeouts->pre_range_mclks,
                                                          timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = vl53l0x_get_vcsel_pulse_period_final_range(addr);

    timeouts->final_range_mclks = vl53l0x_decode_timeout(
        vl53l0x_reg16(addr, VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

    if (enables->pre_range)
    {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us = vl53l0x_timeout_mclks_to_us(timeouts->final_range_mclks,
                                                            timeouts->final_range_vcsel_period_pclks);
}


/* ----------------------------------------------------------------------------
 * 신호율 제한 / 측정 타이밍 버짓
 * (VL53L0X_SetLimitCheckValue, VL53L0X_[gs]et_measurement_timing_budget_micro_seconds 이식)
 * -------------------------------------------------------------------------- */

static bool vl53l0x_set_signal_rate_limit(uint8_t addr, float limit_mcps)
{
    if (limit_mcps < 0.0f || limit_mcps > 511.99f)
    {
        return false;
    }

    /* Q9.7 고정소수점 형식 (정수부 9비트, 소수부 7비트) */
    return vl53l0x_write16(addr, VL53L0X_REG_FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
                            (uint16_t)(limit_mcps * (float)(1 << 7)));
}

static uint32_t vl53l0x_get_measurement_timing_budget(uint8_t addr)
{
    vl53l0x_seq_enables_t  enables;
    vl53l0x_seq_timeouts_t timeouts;

    const uint16_t start_overhead      = 1910;
    const uint16_t end_overhead        = 960;
    const uint16_t msrc_overhead       = 660;
    const uint16_t tcc_overhead        = 590;
    const uint16_t dss_overhead        = 690;
    const uint16_t pre_range_overhead  = 660;
    const uint16_t final_range_overhead = 550;

    uint32_t budget_us = start_overhead + end_overhead;

    vl53l0x_get_sequence_step_enables(addr, &enables);
    vl53l0x_get_sequence_step_timeouts(addr, &enables, &timeouts);

    if (enables.tcc)
    {
        budget_us += timeouts.msrc_dss_tcc_us + tcc_overhead;
    }

    if (enables.dss)
    {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + dss_overhead);
    }
    else if (enables.msrc)
    {
        budget_us += timeouts.msrc_dss_tcc_us + msrc_overhead;
    }

    if (enables.pre_range)
    {
        budget_us += timeouts.pre_range_us + pre_range_overhead;
    }

    if (enables.final_range)
    {
        budget_us += timeouts.final_range_us + final_range_overhead;
    }

    return budget_us;
}

static bool vl53l0x_set_measurement_timing_budget(uint8_t addr, uint32_t budget_us)
{
    vl53l0x_seq_enables_t  enables;
    vl53l0x_seq_timeouts_t timeouts;

    const uint16_t start_overhead      = 1910;
    const uint16_t end_overhead        = 960;
    const uint16_t msrc_overhead       = 660;
    const uint16_t tcc_overhead        = 590;
    const uint16_t dss_overhead        = 690;
    const uint16_t pre_range_overhead  = 660;
    const uint16_t final_range_overhead = 550;

    uint32_t used_budget_us = start_overhead + end_overhead;

    vl53l0x_get_sequence_step_enables(addr, &enables);
    vl53l0x_get_sequence_step_timeouts(addr, &enables, &timeouts);

    if (enables.tcc)
    {
        used_budget_us += timeouts.msrc_dss_tcc_us + tcc_overhead;
    }

    if (enables.dss)
    {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + dss_overhead);
    }
    else if (enables.msrc)
    {
        used_budget_us += timeouts.msrc_dss_tcc_us + msrc_overhead;
    }

    if (enables.pre_range)
    {
        used_budget_us += timeouts.pre_range_us + pre_range_overhead;
    }

    if (enables.final_range)
    {
        uint32_t final_range_timeout_us;
        uint32_t final_range_timeout_mclks;

        used_budget_us += final_range_overhead;

        if (used_budget_us > budget_us)
        {
            /* 요청한 타이밍 버짓이 너무 작다 */
            return false;
        }

        final_range_timeout_us = budget_us - used_budget_us;

        final_range_timeout_mclks =
            vl53l0x_timeout_us_to_mclks(final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range)
        {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        vl53l0x_write16(addr, VL53L0X_REG_FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
                         vl53l0x_encode_timeout(final_range_timeout_mclks));
    }

    return true;
}


/* ----------------------------------------------------------------------------
 * 레퍼런스 캘리브레이션 / SPAD 정보 조회
 * (VL53L0X_perform_single_ref_calibration, VL53L0X_get_info_from_device 이식)
 * -------------------------------------------------------------------------- */

static bool vl53l0x_perform_single_ref_calibration(uint8_t addr, uint8_t vhv_init_byte)
{
    uint32_t start_tick;
    uint8_t  status;

    if (!vl53l0x_write8(addr, VL53L0X_REG_SYSRANGE_START, 0x01 | vhv_init_byte))
    {
        return false;
    }

    start_tick = HAL_GetTick();

    for (;;)
    {
        if (!vl53l0x_read8(addr, VL53L0X_REG_RESULT_INTERRUPT_STATUS, &status))
        {
            return false;
        }

        if ((status & 0x07) != 0)
        {
            break;
        }

        if ((HAL_GetTick() - start_tick) > VL53L0X_IO_TIMEOUT_MS)
        {
            return false;
        }
    }

    vl53l0x_write8(addr, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);
    vl53l0x_write8(addr, VL53L0X_REG_SYSRANGE_START, 0x00);

    return true;
}

/* 레퍼런스 SPAD(Single Photon Avalanche Diode) 개수와 종류를 읽는다.
 * 아래 0x80~0x94 레지스터는 ST VL53L0X.h regAddr enum에 이름이 없는 내부 레지스터로,
 * Pololu VL53L0X::getSpadInfo()의 원시(raw) 주소를 그대로 사용한다. */
static bool vl53l0x_get_spad_info(uint8_t addr, uint8_t *count, bool *type_is_aperture)
{
    uint32_t start_tick;
    uint8_t  val;
    uint8_t  tmp;

    vl53l0x_write8(addr, 0x80, 0x01);
    vl53l0x_write8(addr, 0xFF, 0x01);
    vl53l0x_write8(addr, 0x00, 0x00);

    vl53l0x_write8(addr, 0xFF, 0x06);
    vl53l0x_write8(addr, 0x83, vl53l0x_reg8(addr, 0x83) | 0x04);
    vl53l0x_write8(addr, 0xFF, 0x07);
    vl53l0x_write8(addr, 0x81, 0x01);

    vl53l0x_write8(addr, 0x80, 0x01);

    vl53l0x_write8(addr, 0x94, 0x6B);
    vl53l0x_write8(addr, 0x83, 0x00);

    start_tick = HAL_GetTick();

    for (;;)
    {
        if (!vl53l0x_read8(addr, 0x83, &val))
        {
            return false;
        }

        if (val != 0x00)
        {
            break;
        }

        if ((HAL_GetTick() - start_tick) > VL53L0X_IO_TIMEOUT_MS)
        {
            return false;
        }
    }

    vl53l0x_write8(addr, 0x83, 0x01);
    tmp = vl53l0x_reg8(addr, 0x92);

    *count = tmp & 0x7F;
    *type_is_aperture = (tmp >> 7) & 0x01;

    vl53l0x_write8(addr, 0x81, 0x00);
    vl53l0x_write8(addr, 0xFF, 0x06);
    vl53l0x_write8(addr, 0x83, vl53l0x_reg8(addr, 0x83) & ~0x04);
    vl53l0x_write8(addr, 0xFF, 0x01);
    vl53l0x_write8(addr, 0x00, 0x01);

    vl53l0x_write8(addr, 0xFF, 0x00);
    vl53l0x_write8(addr, 0x80, 0x00);

    return true;
}


/* ----------------------------------------------------------------------------
 * 센서 초기화 시퀀스
 * (Pololu VL53L0X::init(true) = VL53L0X_DataInit + VL53L0X_StaticInit +
 *  VL53L0X_PerformRefCalibration 를 그대로 이식. 2V8 I/O 모드 고정)
 * -------------------------------------------------------------------------- */

static bool vl53l0x_dev_init(uint8_t addr, uint8_t *stop_variable_out)
{
    uint8_t  spad_count;
    bool     spad_type_is_aperture;
    uint8_t  ref_spad_map[6];
    uint8_t  first_spad_to_enable;
    uint8_t  spads_enabled;
    uint32_t budget_us;
    uint8_t  i;

    /* 모델 ID 확인 (데이터시트 명시 값 0xEE) */
    if (vl53l0x_reg8(addr, VL53L0X_REG_IDENTIFICATION_MODEL_ID) != VL53L0X_MODEL_ID_VALUE)
    {
        return false;
    }

    /* ---- VL53L0X_DataInit() ---- */

    /* 센서 기본 I/O는 1V8 모드이므로 2V8 모드로 전환 (NUCLEO 3.3V 버스에 맞춤) */
    vl53l0x_write8(addr, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV,
                    vl53l0x_reg8(addr, VL53L0X_REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV) | 0x01);

    /* "Set I2C standard mode" */
    vl53l0x_write8(addr, 0x88, 0x00);

    vl53l0x_write8(addr, 0x80, 0x01);
    vl53l0x_write8(addr, 0xFF, 0x01);
    vl53l0x_write8(addr, 0x00, 0x00);
    *stop_variable_out = vl53l0x_reg8(addr, 0x91);
    vl53l0x_write8(addr, 0x00, 0x01);
    vl53l0x_write8(addr, 0xFF, 0x00);
    vl53l0x_write8(addr, 0x80, 0x00);

    /* SIGNAL_RATE_MSRC(bit1), SIGNAL_RATE_PRE_RANGE(bit4) 제한 비활성화 */
    vl53l0x_write8(addr, VL53L0X_REG_MSRC_CONFIG_CONTROL,
                    vl53l0x_reg8(addr, VL53L0X_REG_MSRC_CONFIG_CONTROL) | 0x12);

    /* final range 신호율 제한 0.25 MCPS (Pololu/ST 기본값) */
    vl53l0x_set_signal_rate_limit(addr, 0.25f);

    vl53l0x_write8(addr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xFF);

    /* ---- VL53L0X_StaticInit() ---- */

    if (!vl53l0x_get_spad_info(addr, &spad_count, &spad_type_is_aperture))
    {
        return false;
    }

    /* RefGoodSpadMap을 GLOBAL_CONFIG_SPAD_ENABLES_REF_0..5(6바이트)에서 읽는다 */
    vl53l0x_read_multi(addr, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    /* -- VL53L0X_set_reference_spads() : NVM 값이 유효하다고 가정하고 그대로 사용 -- */

    vl53l0x_write8(addr, 0xFF, 0x01);
    vl53l0x_write8(addr, VL53L0X_REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    vl53l0x_write8(addr, VL53L0X_REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    vl53l0x_write8(addr, 0xFF, 0x00);
    vl53l0x_write8(addr, VL53L0X_REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    first_spad_to_enable = spad_type_is_aperture ? 12 : 0; /* 12는 첫 aperture spad */
    spads_enabled = 0;

    for (i = 0; i < 48; i++)
    {
        if (i < first_spad_to_enable || spads_enabled == spad_count)
        {
            ref_spad_map[i / 8] &= (uint8_t)~(1 << (i % 8));
        }
        else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1)
        {
            spads_enabled++;
        }
    }

    vl53l0x_write_multi(addr, VL53L0X_REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    /* -- VL53L0X_load_tuning_settings() : DefaultTuningSettings (vl53l0x_tuning.h) --
     * 아래 레지스터 주소/값은 모두 Pololu VL53L0X::init()에서 그대로 이식한 것으로,
     * ST VL53L0X API의 비공개(undocumented) 내부 레지스터라 이름이 없다. 순서를
     * 절대 바꾸지 말 것. */

    vl53l0x_write8(addr, 0xFF, 0x01);
    vl53l0x_write8(addr, 0x00, 0x00);

    vl53l0x_write8(addr, 0xFF, 0x00);
    vl53l0x_write8(addr, 0x09, 0x00);
    vl53l0x_write8(addr, 0x10, 0x00);
    vl53l0x_write8(addr, 0x11, 0x00);

    vl53l0x_write8(addr, 0x24, 0x01);
    vl53l0x_write8(addr, 0x25, 0xFF);
    vl53l0x_write8(addr, 0x75, 0x00);

    vl53l0x_write8(addr, 0xFF, 0x01);
    vl53l0x_write8(addr, 0x4E, 0x2C);
    vl53l0x_write8(addr, 0x48, 0x00);
    vl53l0x_write8(addr, 0x30, 0x20);

    vl53l0x_write8(addr, 0xFF, 0x00);
    vl53l0x_write8(addr, 0x30, 0x09);
    vl53l0x_write8(addr, 0x54, 0x00);
    vl53l0x_write8(addr, 0x31, 0x04);
    vl53l0x_write8(addr, 0x32, 0x03);
    vl53l0x_write8(addr, 0x40, 0x83);
    vl53l0x_write8(addr, 0x46, 0x25);
    vl53l0x_write8(addr, 0x60, 0x00);
    vl53l0x_write8(addr, 0x27, 0x00);
    vl53l0x_write8(addr, 0x50, 0x06);
    vl53l0x_write8(addr, 0x51, 0x00);
    vl53l0x_write8(addr, 0x52, 0x96);
    vl53l0x_write8(addr, 0x56, 0x08);
    vl53l0x_write8(addr, 0x57, 0x30);
    vl53l0x_write8(addr, 0x61, 0x00);
    vl53l0x_write8(addr, 0x62, 0x00);
    vl53l0x_write8(addr, 0x64, 0x00);
    vl53l0x_write8(addr, 0x65, 0x00);
    vl53l0x_write8(addr, 0x66, 0xA0);

    vl53l0x_write8(addr, 0xFF, 0x01);
    vl53l0x_write8(addr, 0x22, 0x32);
    vl53l0x_write8(addr, 0x47, 0x14);
    vl53l0x_write8(addr, 0x49, 0xFF);
    vl53l0x_write8(addr, 0x4A, 0x00);

    vl53l0x_write8(addr, 0xFF, 0x00);
    vl53l0x_write8(addr, 0x7A, 0x0A);
    vl53l0x_write8(addr, 0x7B, 0x00);
    vl53l0x_write8(addr, 0x78, 0x21);

    vl53l0x_write8(addr, 0xFF, 0x01);
    vl53l0x_write8(addr, 0x23, 0x34);
    vl53l0x_write8(addr, 0x42, 0x00);
    vl53l0x_write8(addr, 0x44, 0xFF);
    vl53l0x_write8(addr, 0x45, 0x26);
    vl53l0x_write8(addr, 0x46, 0x05);
    vl53l0x_write8(addr, 0x40, 0x40);
    vl53l0x_write8(addr, 0x0E, 0x06);
    vl53l0x_write8(addr, 0x20, 0x1A);
    vl53l0x_write8(addr, 0x43, 0x40);

    vl53l0x_write8(addr, 0xFF, 0x00);
    vl53l0x_write8(addr, 0x34, 0x03);
    vl53l0x_write8(addr, 0x35, 0x44);

    vl53l0x_write8(addr, 0xFF, 0x01);
    vl53l0x_write8(addr, 0x31, 0x04);
    vl53l0x_write8(addr, 0x4B, 0x09);
    vl53l0x_write8(addr, 0x4C, 0x05);
    vl53l0x_write8(addr, 0x4D, 0x04);

    vl53l0x_write8(addr, 0xFF, 0x00);
    vl53l0x_write8(addr, 0x44, 0x00);
    vl53l0x_write8(addr, 0x45, 0x20);
    vl53l0x_write8(addr, 0x47, 0x08);
    vl53l0x_write8(addr, 0x48, 0x28);
    vl53l0x_write8(addr, 0x67, 0x00);
    vl53l0x_write8(addr, 0x70, 0x04);
    vl53l0x_write8(addr, 0x71, 0x01);
    vl53l0x_write8(addr, 0x72, 0xFE);
    vl53l0x_write8(addr, 0x76, 0x00);
    vl53l0x_write8(addr, 0x77, 0x00);

    vl53l0x_write8(addr, 0xFF, 0x01);
    vl53l0x_write8(addr, 0x0D, 0x01);

    vl53l0x_write8(addr, 0xFF, 0x00);
    vl53l0x_write8(addr, 0x80, 0x01);
    vl53l0x_write8(addr, 0x01, 0xF8);

    vl53l0x_write8(addr, 0xFF, 0x01);
    vl53l0x_write8(addr, 0x8E, 0x01);
    vl53l0x_write8(addr, 0x00, 0x01);
    vl53l0x_write8(addr, 0xFF, 0x00);
    vl53l0x_write8(addr, 0x80, 0x00);

    /* -- DefaultTuningSettings 끝 -- */

    /* "Set interrupt config to new sample ready" */
    vl53l0x_write8(addr, VL53L0X_REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    vl53l0x_write8(addr, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH,
                    vl53l0x_reg8(addr, VL53L0X_REG_GPIO_HV_MUX_ACTIVE_HIGH) & (uint8_t)~0x10);
    vl53l0x_write8(addr, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

    budget_us = vl53l0x_get_measurement_timing_budget(addr);

    /* "Disable MSRC and TCC by default" */
    vl53l0x_write8(addr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);

    /* "Recalculate timing budget" */
    vl53l0x_set_measurement_timing_budget(addr, budget_us);

    /* ---- VL53L0X_PerformRefCalibration() ---- */

    vl53l0x_write8(addr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!vl53l0x_perform_single_ref_calibration(addr, 0x40)) /* VHV 캘리브레이션 */
    {
        return false;
    }

    vl53l0x_write8(addr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!vl53l0x_perform_single_ref_calibration(addr, 0x00)) /* Phase 캘리브레이션 */
    {
        return false;
    }

    /* Sequence Config 를 원래 값으로 복원 */
    vl53l0x_write8(addr, VL53L0X_REG_SYSTEM_SEQUENCE_CONFIG, 0xE8);

    return true;
}


/* ----------------------------------------------------------------------------
 * 단발(single-shot) 거리 측정 — non-blocking 상태머신
 * (VL53L0X_PerformSingleRangingMeasurement 을 폴링 루프 없이 1단계씩 이식)
 *
 * 이 함수는 폴링 for(;;)를 절대 쓰지 않는다. 호출 한 번에 현재 단계에 맞는
 * 일만 하고 즉시 리턴하며, 실제 측정이 끝날 때까지는 여러 번(=여러 실행 틱)
 * 호출되어야 한다. 대기 조건이 있는 두 단계(WAIT_START_CLEAR, WAIT_RESULT)만
 * vl53l0x_state[sensor].meas_state 로 다음 호출까지 유지된다.
 * -------------------------------------------------------------------------- */

static void vl53l0x_meas_step(vl53l0x_id_t sensor)
{
    vl53l0x_state_t *state = &vl53l0x_state[sensor];
    uint8_t           addr  = state->address;
    uint8_t           val;

    switch (state->meas_state)
    {
        case VL53L0X_MEAS_IDLE:
            /* ---- START : 측정 트리거만 쓰고 바로 WAIT_START_CLEAR 로 넘어간다 ---- */
            vl53l0x_write8(addr, 0x80, 0x01);
            vl53l0x_write8(addr, 0xFF, 0x01);
            vl53l0x_write8(addr, 0x00, 0x00);
            vl53l0x_write8(addr, 0x91, state->stop_variable);
            vl53l0x_write8(addr, 0x00, 0x01);
            vl53l0x_write8(addr, 0xFF, 0x00);
            vl53l0x_write8(addr, 0x80, 0x00);

            if (!vl53l0x_write8(addr, VL53L0X_REG_SYSRANGE_START, 0x01))
            {
                /* 트리거 쓰기 자체가 실패 : IDLE에 남아 다음 틱에 다시 시도한다 */
                state->valid = false;
                state->measurement_error = true;
                return;
            }

            state->meas_state      = VL53L0X_MEAS_WAIT_START_CLEAR;
            state->meas_state_tick = HAL_GetTick();
            return;

        case VL53L0X_MEAS_WAIT_START_CLEAR:
            /* "Wait until start bit has been cleared" 를 한 번만 확인한다 */
            if (!vl53l0x_read8(addr, VL53L0X_REG_SYSRANGE_START, &val))
            {
                state->valid      = false;
                state->meas_state = VL53L0X_MEAS_IDLE;
                state->measurement_error = true;
                return;
            }

            if ((val & 0x01) == 0)
            {
                state->meas_state      = VL53L0X_MEAS_WAIT_RESULT;
                state->meas_state_tick = HAL_GetTick();
                return;
            }

            if ((HAL_GetTick() - state->meas_state_tick) > VL53L0X_IO_TIMEOUT_MS)
            {
                /* timeout : 이 센서만 invalid 처리하고 IDLE로 복구, 다른 센서엔 영향 없음 */
                state->valid      = false;
                state->meas_state = VL53L0X_MEAS_IDLE;
                state->measurement_error = true;
            }
            return;

        case VL53L0X_MEAS_WAIT_RESULT:
            /* 결과 준비 여부를 한 번만 확인한다 */
            if (!vl53l0x_read8(addr, VL53L0X_REG_RESULT_INTERRUPT_STATUS, &val))
            {
                state->valid      = false;
                state->meas_state = VL53L0X_MEAS_IDLE;
                state->measurement_error = true;
                return;
            }

            if ((val & 0x07) != 0)
            {
                /* ---- READ_RESULT : 준비된 그 즉시 같은 호출 안에서 읽고 캐시 갱신 ---- */
                uint16_t range_mm = 0;

                if (vl53l0x_read16(addr, VL53L0X_REG_RESULT_RANGE_STATUS + 10, &range_mm))
                {
                    vl53l0x_write8(addr, VL53L0X_REG_SYSTEM_INTERRUPT_CLEAR, 0x01);

                    state->distance_mm = range_mm;

                    /* 0mm(측정 실패)와 VL53L0X_MAX_VALID_MM 초과(8190mm 계열 out-of-range 포함)는 무효 처리 */
                    state->valid = (range_mm > 0) && (range_mm <= VL53L0X_MAX_VALID_MM);
                    state->measurement_new = true;
                }
                else
                {
                    state->valid = false;
                    state->measurement_error = true;
                }

                state->meas_state = VL53L0X_MEAS_IDLE;
                return;
            }

            if ((HAL_GetTick() - state->meas_state_tick) > VL53L0X_IO_TIMEOUT_MS)
            {
                /* timeout : 이 센서만 invalid 처리하고 IDLE로 복구, 다른 센서엔 영향 없음 */
                state->valid      = false;
                state->meas_state = VL53L0X_MEAS_IDLE;
                state->measurement_error = true;
            }
            return;

        default:
            state->meas_state = VL53L0X_MEAS_IDLE;
            return;
    }
}


/* ============================================================================
 * Public API (hw/driver/vl53l0x.h)
 * ============================================================================ */

/* I2C와 XSHUT 제어 상태를 초기화 : 모든 센서 상태를 초기화하고 XSHUT을 모두 LOW로 내린다 */
void vl53l0x_init(void)
{
    vl53l0x_id_t sensor;

    memset(vl53l0x_state, 0, sizeof(vl53l0x_state));

    for (sensor = VL53L0X_FRONT; sensor < VL53L0X_COUNT; sensor++)
    {
        HAL_GPIO_WritePin(vl53l0x_pin_map[sensor].xshut_port, vl53l0x_pin_map[sensor].xshut_pin,
                           GPIO_PIN_RESET);
    }
}

/* XSHUT으로 선택한 센서만 활성화하고 새로운 I2C 주소를 설정.
 * 이 함수를 호출하는 시점에 다른 물리 센서가 기본주소(0x29)로 함께 살아있으면 안 되므로,
 * 호출 전에 vl53l0x_init()으로 모든 XSHUT을 LOW로 내려 둔 상태여야 한다. */
bool vl53l0x_init_sensor(vl53l0x_id_t sensor, uint8_t new_address)
{
    uint8_t addr;

    if (sensor >= VL53L0X_COUNT)
    {
        vl53l0x_last_init_sensor = VL53L0X_COUNT;
        vl53l0x_last_init_stage = "invalid sensor id";
        return false;
    }

    vl53l0x_state[sensor].ready = false;
    vl53l0x_state[sensor].valid = false;
    vl53l0x_state[sensor].measurement_new = false;
    vl53l0x_state[sensor].measurement_error = false;

    /* 재초기화 도중일 수 있으므로 진행 중이던 측정 상태머신도 IDLE로 되돌린다 */
    vl53l0x_state[sensor].meas_state = VL53L0X_MEAS_IDLE;

    /* 이 센서만 XSHUT High : 하드웨어 스탠바이 해제 */
    HAL_GPIO_WritePin(vl53l0x_pin_map[sensor].xshut_port, vl53l0x_pin_map[sensor].xshut_pin,
                       GPIO_PIN_SET);

    /* 데이터시트 tBOOT(최대 1.2 ms) 이후에 통신 가능 */
    HAL_Delay(VL53L0X_BOOT_DELAY_MS);

    addr = VL53L0X_DEFAULT_ADDRESS;

    /* 부팅 직후에는 항상 기본 주소(0x29)로 응답하므로, 최종 주소가 다르면 여기서 바꾼다 */
    if (new_address != VL53L0X_DEFAULT_ADDRESS)
    {
        if (!vl53l0x_write8(addr, VL53L0X_REG_I2C_SLAVE_DEVICE_ADDRESS, new_address & 0x7F))
        {
            vl53l0x_last_init_sensor = sensor;
            vl53l0x_last_init_stage = "address assignment";
            HAL_GPIO_WritePin(vl53l0x_pin_map[sensor].xshut_port,
                              vl53l0x_pin_map[sensor].xshut_pin,
                              GPIO_PIN_RESET);
            return false;
        }

        addr = new_address;
    }

    if (!vl53l0x_dev_init(addr, &vl53l0x_state[sensor].stop_variable))
    {
        vl53l0x_last_init_sensor = sensor;
        vl53l0x_last_init_stage = "device initialization";
        HAL_GPIO_WritePin(vl53l0x_pin_map[sensor].xshut_port,
                          vl53l0x_pin_map[sensor].xshut_pin,
                          GPIO_PIN_RESET);
        return false;
    }

    vl53l0x_state[sensor].address = addr;
    vl53l0x_state[sensor].ready   = true;

    return true;
}

/* 전방·좌측·우측 센서를 순차적으로 활성화하고 주소를 설정.
 * VL53L0X_MULTI_SENSOR_ENABLE 매크로로 FRONT 1개 전용 / 3센서 전체 경로를 전환한다. */
bool vl53l0x_init_all(void)
{
    uint8_t attempt;

    vl53l0x_last_init_sensor = VL53L0X_COUNT;
    vl53l0x_last_init_stage = "none";
    vl53l0x_last_init_attempt = 0U;
    vl53l0x_last_hal_status = HAL_OK;
    vl53l0x_last_i2c_error = HAL_I2C_ERROR_NONE;
    vl53l0x_last_i2c_state = HAL_I2C_GetState(&hi2c1);

#if VL53L0X_MULTI_SENSOR_ENABLE
    /* 세 센서가 공유하는 기본 주소 0x29가 충돌하지 않도록 하나씩 깨워
     * LEFT=0x30, FRONT=0x31, RIGHT=0x32 순서로 고유 주소를 부여한다.
     * 하나라도 실패하면 다음 센서를 켜지 않고 모두 LOW로 리셋한다. */
    for (attempt = 1U; attempt <= VL53L0X_INIT_RETRY_COUNT; attempt++)
    {
        bool ok;

        vl53l0x_last_init_attempt = attempt;
        vl53l0x_init();
        HAL_Delay(VL53L0X_RESET_LOW_MS);

        ok = vl53l0x_init_sensor(VL53L0X_LEFT, VL53L0X_LEFT_ADDRESS);
        if (ok)
        {
            ok = vl53l0x_init_sensor(VL53L0X_FRONT, VL53L0X_FRONT_ADDRESS);
        }
        if (ok)
        {
            ok = vl53l0x_init_sensor(VL53L0X_RIGHT, VL53L0X_RIGHT_ADDRESS);
        }

        if (ok)
        {
            vl53l0x_last_init_sensor = VL53L0X_COUNT;
            vl53l0x_last_init_stage = "none";
            return true;
        }

        /* 앞 센서가 기본주소 변경 전 실패했어도 다음 시도에서 충돌하지 않게
         * 성공했던 센서까지 모두 하드웨어 리셋한다. */
        vl53l0x_init();
        HAL_Delay(VL53L0X_RESET_LOW_MS);

        if (attempt < VL53L0X_INIT_RETRY_COUNT)
        {
            HAL_StatusTypeDef restart_status;

            (void)HAL_I2C_DeInit(&hi2c1);
            HAL_Delay(VL53L0X_I2C_RETRY_DELAY_MS);
            restart_status = HAL_I2C_Init(&hi2c1);

            if (restart_status != HAL_OK)
            {
                vl53l0x_last_hal_status = restart_status;
                vl53l0x_last_i2c_error = HAL_I2C_GetError(&hi2c1);
                vl53l0x_last_i2c_state = HAL_I2C_GetState(&hi2c1);
                vl53l0x_last_init_sensor = VL53L0X_COUNT;
                vl53l0x_last_init_stage = "I2C peripheral restart";
                break;
            }

            HAL_Delay(VL53L0X_INIT_RETRY_DELAY_MS);
        }
    }

    return false;
#else
    /* 단일 센서 진단이 필요할 때만 사용하는 FRONT 전용 경로다. */
    vl53l0x_init();
    HAL_Delay(VL53L0X_RESET_LOW_MS);
    vl53l0x_last_init_attempt = 1U;
    return vl53l0x_init_sensor(VL53L0X_FRONT, VL53L0X_FRONT_ADDRESS);
#endif
}

vl53l0x_id_t vl53l0x_get_last_init_sensor(void)
{
    return vl53l0x_last_init_sensor;
}

const char *vl53l0x_get_last_init_stage(void)
{
    return vl53l0x_last_init_stage;
}

uint8_t vl53l0x_get_last_init_attempt(void)
{
    return vl53l0x_last_init_attempt;
}

uint32_t vl53l0x_get_last_hal_status(void)
{
    return (uint32_t)vl53l0x_last_hal_status;
}

uint32_t vl53l0x_get_last_i2c_error(void)
{
    return vl53l0x_last_i2c_error;
}

uint32_t vl53l0x_get_last_i2c_state(void)
{
    return (uint32_t)vl53l0x_last_i2c_state;
}

/* 선택한 센서의 측정 상태머신을 "한 단계만" 진행한다 (non-blocking).
 *
 * 폴링 for(;;)이 전혀 없으므로 즉시 리턴한다. 실제로 새 거리값이 distance_mm/valid에
 * 반영되는 시점은 상태머신이 READ_RESULT 단계까지 도달했을 때뿐이며, 그 전까지는
 * 이전에 측정된 값이 그대로 남아 있다(= 이 호출이 곧바로 새 측정값을 보장하지 않는다,
 * 예전의 blocking single-shot 버전과 달라진 부분). 매 실행 틱마다 반복 호출해서
 * 상태머신을 계속 앞으로 진행시켜야 한다.
 *
 * 반환값은 "이 센서에 대해 상태머신 스텝을 실제로 시도했는지"만 의미한다
 * (센서 id가 유효하고 ready == true일 때 true). 이번 스텝에서 I2C 트랜잭션이
 * 실패하거나 timeout이 나도 별도 처리 없이 true를 반환하며, 그 결과는
 * state.valid / vl53l0x_is_valid()에 반영된다. */
bool vl53l0x_update_sensor(vl53l0x_id_t sensor)
{
    if (sensor >= VL53L0X_COUNT || !vl53l0x_state[sensor].ready)
    {
        return false;
    }

    vl53l0x_meas_step(sensor);

    return true;
}

/* 새 거리 결과 이벤트를 한 번 꺼낸다. 측정 진행 중에는 false다. */
bool vl53l0x_take_new_measurement(vl53l0x_id_t sensor)
{
    bool has_new_measurement;

    if (sensor >= VL53L0X_COUNT)
    {
        return false;
    }

    has_new_measurement = vl53l0x_state[sensor].measurement_new;
    vl53l0x_state[sensor].measurement_new = false;

    return has_new_measurement;
}

/* I2C 실패 또는 측정 timeout 이벤트를 한 번 꺼낸다. */
bool vl53l0x_take_measurement_error(vl53l0x_id_t sensor)
{
    bool has_error;

    if (sensor >= VL53L0X_COUNT)
    {
        return false;
    }

    has_error = vl53l0x_state[sensor].measurement_error;
    vl53l0x_state[sensor].measurement_error = false;

    return has_error;
}

/* ready인 센서들의 측정 상태머신을 각각 한 단계씩 진행한다 (non-blocking, ready == false 인 센서는 건너뜀).
 * 새 거리값이 실제로 반영되는 시점은 센서마다 상태머신이 READ_RESULT에 도달하는 순간이며,
 * 이는 이 함수가 여러 번(=여러 실행 틱) 호출된 뒤일 수 있다. */
void vl53l0x_update(void)
{
    vl53l0x_id_t sensor;

    for (sensor = VL53L0X_FRONT; sensor < VL53L0X_COUNT; sensor++)
    {
        if (vl53l0x_state[sensor].ready)
        {
            vl53l0x_update_sensor(sensor);
        }
    }
}

/* 선택한 센서의 최근 거리값을 mm 단위로 반환 (초기화 전이거나 잘못된 id면 0) */
uint16_t vl53l0x_get_distance_mm(vl53l0x_id_t sensor)
{
    if (sensor >= VL53L0X_COUNT)
    {
        return 0;
    }

    return vl53l0x_state[sensor].distance_mm;
}

/* 선택한 센서의 초기화 성공 여부를 반환 */
bool vl53l0x_is_ready(vl53l0x_id_t sensor)
{
    if (sensor >= VL53L0X_COUNT)
    {
        return false;
    }

    return vl53l0x_state[sensor].ready;
}

/* 선택한 센서의 최근 측정값이 유효한지 반환 */
bool vl53l0x_is_valid(vl53l0x_id_t sensor)
{
    if (sensor >= VL53L0X_COUNT)
    {
        return false;
    }

    return vl53l0x_state[sensor].valid;
}

/* LEFT/FRONT/RIGHT(ready인 센서만) 각각 측정 상태머신을 한 단계씩 진행하고,
 * 현재 캐시된 값 기준으로 장애물 여부를 판정해 저장한다 (non-blocking, 내부 폴링 없음).
 *
 * ready == false인 센서는
 * 상태머신 자체를 진행시키지 않고 obstacle을 안전하게 false로만 둔 채 건너뛴다.
 * 판정 규칙 : valid == true && distance_mm <= threshold_mm 이면 obstacle = true.
 * distance_mm/valid는 해당 센서의 상태머신이 READ_RESULT에 도달한 틱에만 갱신되므로,
 * 이 함수를 20ms 등 짧은 주기로 반복 호출해도 매번 새 측정이 끝나는 것은 아니다 —
 * 그 사이에는 직전에 확정된 값 기준으로 obstacle이 재계산된다. */
void vl53l0x_obstacle_update(uint16_t threshold_mm)
{
    vl53l0x_id_t sensor;

    for (sensor = VL53L0X_FRONT; sensor < VL53L0X_COUNT; sensor++)
    {
        if (!vl53l0x_state[sensor].ready)
        {
            vl53l0x_state[sensor].obstacle = false;
            continue;
        }

        vl53l0x_update_sensor(sensor);

        vl53l0x_state[sensor].obstacle =
            vl53l0x_state[sensor].valid &&
            (vl53l0x_state[sensor].distance_mm <= threshold_mm);
    }
}

/* 선택한 센서의 최근 장애물 판정 결과를 반환 (vl53l0x_obstacle_update()가 채운 값) */
bool vl53l0x_is_obstacle(vl53l0x_id_t sensor)
{
    if (sensor >= VL53L0X_COUNT)
    {
        return false;
    }

    return vl53l0x_state[sensor].obstacle;
}
