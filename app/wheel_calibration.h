#ifndef WHEEL_CALIBRATION_H
#define WHEEL_CALIBRATION_H

#include <stdbool.h>
#include <stdint.h>

/* 바퀴를 띄운 상태에서 좌우 모터의 PWM-RPM 특성을 기록하는 시험 기능이다. */
void wheel_calibration_init(void);
void wheel_calibration_arm(void);
bool wheel_calibration_start(void);
void wheel_calibration_update(uint32_t elapsed_time_ms);
void wheel_calibration_cancel(void);

bool wheel_calibration_is_armed(void);
bool wheel_calibration_is_active(void);
bool wheel_calibration_take_completed(void);
bool wheel_calibration_take_sensor_fault(void);

#endif
