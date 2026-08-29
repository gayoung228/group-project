#include "wheel_calibration.h"
#include "drive.h"
#include "encoder.h"
#include <stdio.h>

/* 각 PWM에서 회전이 안정될 시간과 평균 RPM을 모을 시간 */
#define CALIBRATION_SETTLE_MS       1000U
#define CALIBRATION_SAMPLE_MS       2000U

/* 상승 구간은 기동 특성, 하강 구간은 이미 회전 중일 때의 유지 특성을 보여준다. */
static const uint8_t calibration_outputs[] =
{
    80U, 85U, 90U, 95U, 100U, 95U, 90U, 85U, 80U
};

#define CALIBRATION_STEP_COUNT \
    (sizeof(calibration_outputs) / sizeof(calibration_outputs[0]))

typedef enum
{
    CALIBRATION_IDLE = 0,
    CALIBRATION_ARMED,
    CALIBRATION_SETTLING,
    CALIBRATION_SAMPLING
} calibration_state_t;

static calibration_state_t calibration_state = CALIBRATION_IDLE;
static uint8_t calibration_step = 0U;
static uint32_t calibration_phase_elapsed_ms = 0U;
static float left_rpm_sum = 0.0f;
static float right_rpm_sum = 0.0f;
static float left_rpm_min = 0.0f;
static float right_rpm_min = 0.0f;
static float left_rpm_max = 0.0f;
static float right_rpm_max = 0.0f;
static uint32_t calibration_sample_count = 0U;
static bool calibration_completed = false;
static bool calibration_sensor_fault = false;

static void calibration_apply_step(void)
{
    uint8_t output = calibration_outputs[calibration_step];

    calibration_phase_elapsed_ms = 0U;
    calibration_state = CALIBRATION_SETTLING;
    drive_set_direct_output((int16_t)output, (int16_t)output);

    printf("[CAL] Step %u/%u: PWM %u%% - settling 1 s\r\n",
           (unsigned)(calibration_step + 1U),
           (unsigned)CALIBRATION_STEP_COUNT,
           (unsigned)output);
}

void wheel_calibration_init(void)
{
    calibration_state = CALIBRATION_IDLE;
    calibration_step = 0U;
    calibration_phase_elapsed_ms = 0U;
    calibration_completed = false;
    calibration_sensor_fault = false;
}

void wheel_calibration_arm(void)
{
    if (calibration_state != CALIBRATION_IDLE)
    {
        return;
    }

    calibration_state = CALIBRATION_ARMED;
    calibration_completed = false;
    calibration_sensor_fault = false;

    printf("\r\n========== WHEEL-LIFT RPM CALIBRATION ==========\r\n");
    printf("[CAL][SAFETY] Vehicle stopped. Lift BOTH wheels off the floor.\r\n");
    printf("[CAL] Test: PWM 80->85->90->95->100->95->90->85->80%%\r\n");
    printf("[CAL] Each step: settle 1 s + measure 2 s. Total about 27 s.\r\n");
    printf("[CAL] Press C again to START, or X to cancel.\r\n");
    printf("================================================\r\n");
}

bool wheel_calibration_start(void)
{
    if (calibration_state != CALIBRATION_ARMED)
    {
        return false;
    }

    if (drive_is_ready() == false)
    {
        printf("[CAL][ERROR] Motor/encoder/gyro is not ready; calibration cancelled.\r\n");
        calibration_state = CALIBRATION_IDLE;
        return false;
    }

    calibration_step = 0U;
    calibration_completed = false;
    drive_reset_distance();

    printf("\r\n[CAL] START - keep hands and cables away from wheels.\r\n");
    calibration_apply_step();
    return true;
}

void wheel_calibration_update(uint32_t elapsed_time_ms)
{
    float left_rpm;
    float right_rpm;

    if ((calibration_state != CALIBRATION_SETTLING)
        && (calibration_state != CALIBRATION_SAMPLING))
    {
        return;
    }

    /* 직접 PWM 출력 중에도 엔코더와 자이로 측정은 계속 갱신한다. */
    drive_update(elapsed_time_ms);

    if (drive_is_ready() == false)
    {
        printf("\r\n[CAL][ERROR] Sensor update failed - emergency stop.\r\n");
        drive_stop();
        calibration_state = CALIBRATION_IDLE;
        calibration_sensor_fault = true;
        return;
    }

    calibration_phase_elapsed_ms += elapsed_time_ms;

    if (calibration_state == CALIBRATION_SETTLING)
    {
        if (calibration_phase_elapsed_ms < CALIBRATION_SETTLE_MS)
        {
            return;
        }

        calibration_state = CALIBRATION_SAMPLING;
        calibration_phase_elapsed_ms = 0U;
        left_rpm_sum = 0.0f;
        right_rpm_sum = 0.0f;
        left_rpm_min = encoder_get_rpm(ENCODER_LEFT);
        right_rpm_min = encoder_get_rpm(ENCODER_RIGHT);
        left_rpm_max = left_rpm_min;
        right_rpm_max = right_rpm_min;
        calibration_sample_count = 0U;
        printf("[CAL] PWM %u%% - measuring 2 s...\r\n",
               (unsigned)calibration_outputs[calibration_step]);
        return;
    }

    left_rpm = encoder_get_rpm(ENCODER_LEFT);
    right_rpm = encoder_get_rpm(ENCODER_RIGHT);
    left_rpm_sum += left_rpm;
    right_rpm_sum += right_rpm;
    calibration_sample_count++;

    if (left_rpm < left_rpm_min)   { left_rpm_min = left_rpm; }
    if (left_rpm > left_rpm_max)   { left_rpm_max = left_rpm; }
    if (right_rpm < right_rpm_min) { right_rpm_min = right_rpm; }
    if (right_rpm > right_rpm_max) { right_rpm_max = right_rpm; }

    if (calibration_phase_elapsed_ms < CALIBRATION_SAMPLE_MS)
    {
        return;
    }

    if (calibration_sample_count > 0U)
    {
        float left_average = left_rpm_sum / (float)calibration_sample_count;
        float right_average = right_rpm_sum / (float)calibration_sample_count;

        printf("[CAL][RESULT] PWM:%3u%% | "
               "L avg:%4d min:%4d max:%4d RPM | "
               "R avg:%4d min:%4d max:%4d RPM | diff:%4d\r\n",
               (unsigned)calibration_outputs[calibration_step],
               (int)(left_average + 0.5f),
               (int)(left_rpm_min + 0.5f),
               (int)(left_rpm_max + 0.5f),
               (int)(right_average + 0.5f),
               (int)(right_rpm_min + 0.5f),
               (int)(right_rpm_max + 0.5f),
               (int)(left_average - right_average));
    }

    calibration_step++;
    if (calibration_step < CALIBRATION_STEP_COUNT)
    {
        calibration_apply_step();
        return;
    }

    drive_stop();
    calibration_state = CALIBRATION_IDLE;
    calibration_completed = true;

    printf("\r\n[CAL] COMPLETE - motors stopped.\r\n");
    printf("[CAL] Save all [CAL][RESULT] lines.\r\n");
    printf("[CAL] Air test calibrates motor/encoder only; ground straight test is still required.\r\n");
}

void wheel_calibration_cancel(void)
{
    drive_stop();
    calibration_state = CALIBRATION_IDLE;
    calibration_completed = false;
    printf("\r\n[CAL] CANCELLED - motors stopped.\r\n");
}

bool wheel_calibration_is_armed(void)
{
    return (calibration_state == CALIBRATION_ARMED);
}

bool wheel_calibration_is_active(void)
{
    return (calibration_state == CALIBRATION_SETTLING)
        || (calibration_state == CALIBRATION_SAMPLING);
}

bool wheel_calibration_take_completed(void)
{
    bool completed = calibration_completed;
    calibration_completed = false;
    return completed;
}

bool wheel_calibration_take_sensor_fault(void)
{
    bool occurred = calibration_sensor_fault;
    calibration_sensor_fault = false;
    return occurred;
}
