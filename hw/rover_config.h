#ifndef ROVER_CONFIG_H
#define ROVER_CONFIG_H

/* ------------------------------------------------------------------
 * rover_config.h - 실물 로버 공통 제원
 *
 * 여러 모듈에 같은 숫자를 따로 적으면 한쪽만 수정되는 실수가 생긴다.
 * 엔코더 슬롯 수, 바퀴 지름, 실측 RPM 범위를 이 파일 한 곳에서 관리한다.
 * ------------------------------------------------------------------ */

/* HC-020K 슬롯 디스크 한 바퀴당 펄스 수 */
#define ROVER_ENCODER_SLOTS_PER_REV       20U

/* 실측 바퀴 지름 [mm] */
#define ROVER_WHEEL_DIAMETER_MM            66.0f

#define ROVER_PI                            3.141592f

/* 기어모터에서 실측한 제어 가능 범위 */
#define ROVER_DRIVE_RPM_MIN                90.0f
#define ROVER_DRIVE_RPM_NORMAL            120.0f
#define ROVER_DRIVE_RPM_MAX               150.0f
#define ROVER_MOTOR_MIN_OUTPUT              80
#define ROVER_MOTOR_NORMAL_OUTPUT           90
#define ROVER_MOTOR_MAX_OUTPUT             100

/* 엔코더 신호 오류로 판단할 물리적 상한. 정상 최대 RPM의 두 배로 둔다. */
#define ROVER_ENCODER_MAX_PLAUSIBLE_RPM   300.0f

#endif
