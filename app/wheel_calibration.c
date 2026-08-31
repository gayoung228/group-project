#include "wheel_calibration.h"
#include "drive.h"
#include "encoder.h"
#include "heading_control.h"
#include "wheel.h"
#include "rover_config.h"
#include <stdio.h>

/* 각 PWM에서 회전이 안정될 시간과 평균 RPM을 모을 시간 */
#define CALIBRATION_SETTLE_MS       1000U
#define CALIBRATION_SAMPLE_MS       2000U

/* 상승 구간은 기동 특성, 하강 구간은 이미 회전 중일 때의 유지 특성을 보여준다. */
static const uint8_t calibration_outputs[] =
{
    70U, 75U, 80U, 85U, 90U, 85U, 80U, 75U, 70U
};

#define CALIBRATION_STEP_COUNT \
    (sizeof(calibration_outputs) / sizeof(calibration_outputs[0]))

/* 원시 PWM 표를 얻은 뒤 새 보정표가 목표 RPM을 실제로 만드는지 자동 확인한다. */
static const float calibration_targets[] =
{
    ROVER_DRIVE_RPM_MIN,
    ROVER_DRIVE_RPM_NORMAL,
    ROVER_DRIVE_RPM_MAX
};

#define CALIBRATION_TARGET_COUNT \
    (sizeof(calibration_targets) / sizeof(calibration_targets[0]))

typedef enum
{
    CALIBRATION_PWM_SWEEP = 0,
    CALIBRATION_TARGET_VERIFY
} calibration_mode_t;

typedef enum
{
    CALIBRATION_IDLE = 0,
    CALIBRATION_ARMED,
    CALIBRATION_SETTLING,
    CALIBRATION_SAMPLING
} calibration_state_t;

static calibration_state_t calibration_state = CALIBRATION_IDLE;
static calibration_mode_t calibration_mode = CALIBRATION_PWM_SWEEP;
static uint8_t calibration_step = 0U;
static uint32_t calibration_phase_elapsed_ms = 0U;
static float left_rpm_sum = 0.0f;
static float right_rpm_sum = 0.0f;
static float left_rpm_min = 0.0f;
static float right_rpm_min = 0.0f;
static float left_rpm_max = 0.0f;
static float right_rpm_max = 0.0f;
static int32_t left_output_sum = 0;
static int32_t right_output_sum = 0;
static uint32_t calibration_sample_count = 0U;
static bool calibration_completed = false;
static bool calibration_sensor_fault = false;

static void calibration_apply_step(void)
{
    calibration_phase_elapsed_ms = 0U;
    calibration_state = CALIBRATION_SETTLING;

    if (calibration_mode == CALIBRATION_PWM_SWEEP)
    {
        uint8_t output = calibration_outputs[calibration_step];

        drive_set_direct_output((int16_t)output, (int16_t)output);
        printf("[CAL] PWM step %u/%u: %u%% - settling 1 s\r\n",
               (unsigned)(calibration_step + 1U),
               (unsigned)CALIBRATION_STEP_COUNT,
               (unsigned)output);
    }
    else
    {
        float target_rpm = calibration_targets[calibration_step];

        drive_set_forward_target_rpm(target_rpm);
        /* 공중에서는 바퀴가 돌아도 차체 Yaw가 변하지 않는다. 작은 자이로 오차가
         * 좌우 목표 RPM 차이로 섞이지 않게 하고 바퀴 PID만 검증한다. */
        heading_set_enabled(false);
        printf("[CAL] VERIFY step %u/%u: target %u RPM - settling 1 s\r\n",
               (unsigned)(calibration_step + 1U),
               (unsigned)CALIBRATION_TARGET_COUNT,
               (unsigned)target_rpm);
    }
}

void wheel_calibration_init(void)
{
    calibration_state = CALIBRATION_IDLE;
    calibration_mode = CALIBRATION_PWM_SWEEP;
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
    printf("[CAL] Test: PWM 70->75->80->85->90->85->80->75->70%%\r\n");
    printf("[CAL] Then verify corrected targets: %u->%u->%u RPM.\r\n",
           (unsigned)ROVER_DRIVE_RPM_MIN,
           (unsigned)ROVER_DRIVE_RPM_NORMAL,
           (unsigned)ROVER_DRIVE_RPM_MAX);
    printf("[CAL] Each step: settle 1 s + measure 2 s. Total about 36 s.\r\n");
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
    calibration_mode = CALIBRATION_PWM_SWEEP;
    calibration_completed = false;
    drive_reset_distance();
    drive_reset_heading();

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
        left_output_sum = 0;
        right_output_sum = 0;
        calibration_sample_count = 0U;

        if (calibration_mode == CALIBRATION_PWM_SWEEP)
        {
            printf("[CAL] PWM %u%% - measuring 2 s...\r\n",
                   (unsigned)calibration_outputs[calibration_step]);
        }
        else
        {
            printf("[CAL] Target %u RPM - measuring corrected output for 2 s...\r\n",
                   (unsigned)calibration_targets[calibration_step]);
        }
        return;
    }

    left_rpm = encoder_get_rpm(ENCODER_LEFT);
    right_rpm = encoder_get_rpm(ENCODER_RIGHT);
    left_rpm_sum += left_rpm;
    right_rpm_sum += right_rpm;
    left_output_sum += wheel_get_output(WHEEL_LEFT);
    right_output_sum += wheel_get_output(WHEEL_RIGHT);
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

        if (calibration_mode == CALIBRATION_PWM_SWEEP)
        {
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
        else
        {
            int left_output_average = left_output_sum / (int32_t)calibration_sample_count;
            int right_output_average = right_output_sum / (int32_t)calibration_sample_count;

            printf("[CAL][VERIFY] target:%3u RPM | "
                   "L avg:%4d RPM pwm:%3d%% | "
                   "R avg:%4d RPM pwm:%3d%% | diff:%4d\r\n",
                   (unsigned)calibration_targets[calibration_step],
                   (int)(left_average + 0.5f),
                   left_output_average,
                   (int)(right_average + 0.5f),
                   right_output_average,
                   (int)(left_average - right_average));
        }
    }

    calibration_step++;
    if (((calibration_mode == CALIBRATION_PWM_SWEEP)
         && (calibration_step < CALIBRATION_STEP_COUNT))
        || ((calibration_mode == CALIBRATION_TARGET_VERIFY)
            && (calibration_step < CALIBRATION_TARGET_COUNT)))
    {
        calibration_apply_step();
        return;
    }

    if (calibration_mode == CALIBRATION_PWM_SWEEP)
    {
        drive_stop();
        drive_prepare_start();
        drive_reset_heading();
        calibration_mode = CALIBRATION_TARGET_VERIFY;
        calibration_step = 0U;

        printf("\r\n[CAL] PWM sweep complete. Starting wheel-only closed-loop verification.\r\n");
        printf("[CAL] Heading correction is disabled during wheel-lift verification.\r\n");
        calibration_apply_step();
        return;
    }

    drive_stop();
    calibration_state = CALIBRATION_IDLE;
    calibration_completed = true;

    printf("\r\n[CAL] COMPLETE - motors stopped.\r\n");
    printf("[CAL] Save all [CAL][RESULT] and [CAL][VERIFY] lines.\r\n");
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
