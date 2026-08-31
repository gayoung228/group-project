#ifndef ROVER_CONFIG_H
#define ROVER_CONFIG_H

/* ------------------------------------------------------------------
 * rover_config.h - 실물 로버 공통 제원
 *
 * 여러 모듈에 같은 숫자를 따로 적으면 한쪽만 수정되는 실수가 생긴다.
 * 엔코더 슬롯 수, 바퀴 지름, 실측 RPM 범위와 자율주행 튜닝값을
 * 이 파일 한 곳에서 관리한다.
 * ------------------------------------------------------------------ */

/* HC-020K 슬롯 디스크 한 바퀴당 펄스 수 */
#define ROVER_ENCODER_SLOTS_PER_REV       20U

/* 실측 바퀴 지름 [mm] */
#define ROVER_WHEEL_DIAMETER_MM            66.0f

#define ROVER_PI                            3.141592f

/* 공중 3회 반복 측정으로 정한 현실적인 공통 주행 속도.
 * 두 바퀴가 모두 도달할 수 있는 느린 쪽(LEFT)을 기준으로 한다. */
#define ROVER_DRIVE_RPM_MIN                65.0f
#define ROVER_DRIVE_RPM_NORMAL             70.0f
#define ROVER_DRIVE_RPM_MAX                78.0f

/* 좌우 모터의 공중 실측 최대 RPM. 방향 보정 목표의 물리적 상한이다. */
#define ROVER_LEFT_WHEEL_RPM_MAX            78.0f
#define ROVER_RIGHT_WHEEL_RPM_MAX          107.0f
#define ROVER_MOTOR_MIN_OUTPUT              70
#define ROVER_MOTOR_NORMAL_OUTPUT           90
#define ROVER_MOTOR_MAX_OUTPUT             100

/* 엔코더 신호 오류로 판단할 물리적 상한. 정상 최대 RPM의 두 배로 둔다. */
#define ROVER_ENCODER_MAX_PLAUSIBLE_RPM   300.0f

/* ==================================================================
 * 3방향 VL53L0X 거리센서
 * ================================================================== */

/* 센서 장착 중심각. FRONT=0도, LEFT=+45도, RIGHT=-45도 기준이다. */
#define ROVER_SIDE_SENSOR_ANGLE_DEG        45.0f

/* 같은 센서의 새 측정이 이 시간보다 오래 없으면 오래된 값으로 판단한다. */
#define ROVER_DISTANCE_STALE_MS            600U

/* 연속 통신 실패 및 부팅 시 정상 측정 확인에 사용할 횟수다. */
#define ROVER_DISTANCE_FAIL_COUNT            3U
#define ROVER_DISTANCE_READY_COUNT           3U

/* ==================================================================
 * 부드러운 장애물 회피
 *
 * 실차 시험에서 가장 자주 바꿀 값이다. 거리 단위는 mm, 각도는 deg,
 * 시간은 ms, 속도는 RPM이다.
 * ================================================================== */

/* 정면 장애물을 회피 이벤트로 확정하는 거리와 연속 측정 횟수 */
#define ROVER_AVOID_TRIGGER_MM              500U
#define ROVER_AVOID_TRIGGER_CONFIRM_COUNT     3U

/* 정면 장애물이 이 거리까지 가까워지면 최대 회전 강도에 도달한다. */
#define ROVER_AVOID_FULL_TURN_MM             250U

/* 일반 회피보다 우선하는 즉시 정지 거리 */
#define ROVER_AVOID_FRONT_EMERGENCY_MM      220U
#define ROVER_AVOID_SIDE_EMERGENCY_MM       180U

/* 좌우 중 실제 통로 후보로 인정할 최소 거리와 방향 선택 데드밴드 */
#define ROVER_AVOID_DIRECTION_OPEN_MM       350U
#define ROVER_AVOID_DIRECTION_DEADBAND_MM   100U

/* 회피 시작 즉시 확보할 Yaw, 최대 Yaw와 거리 기반 목표 Yaw 변화 속도 */
#define ROVER_AVOID_INITIAL_OUT_YAW_DEG      45.0f
#define ROVER_AVOID_MAX_OUT_YAW_DEG          75.0f
#define ROVER_AVOID_MIN_YAW_RATE_DEG_S         8.0f
#define ROVER_AVOID_MAX_YAW_RATE_DEG_S        40.0f
#define ROVER_AVOID_RETURN_YAW_RATE_DEG_S     20.0f

/* 회피 중 바퀴가 정지 마찰 구간으로 떨어지지 않는 폐루프 속도 */
#define ROVER_AVOID_BASE_RPM                  65.0f
#define ROVER_AVOID_MIN_RPM                   ROVER_DRIVE_RPM_MIN
#define ROVER_HEADING_DRIVE_DIFFERENTIAL_MAX 12.0f

/* 안쪽 45도 센서로 장애물을 따라갈 때 사용하는 거리 */
#define ROVER_AVOID_SIDE_TARGET_MM           400U
#define ROVER_AVOID_SIDE_DEADBAND_MM          75U
#define ROVER_AVOID_SIDE_TRACK_SEEN_MM       700U
#define ROVER_AVOID_SIDE_TRACK_CONFIRM_COUNT   3U

/* 안쪽 센서가 가까웠다가 이 거리 이상으로 바뀌면 장애물 끝 후보로 본다. */
#define ROVER_AVOID_EDGE_CLEAR_MM           1000U
#define ROVER_AVOID_EDGE_CONFIRM_COUNT         3U
#define ROVER_AVOID_EDGE_MIN_TRAVEL_MM       250.0f

/* 측면 장애물을 못 잡은 좁은 물체도 무한 추적하지 않게 하는 보조 조건 */
#define ROVER_AVOID_UNTRACKED_CLEAR_MM        700U
#define ROVER_AVOID_UNTRACKED_TRAVEL_MM       500.0f
#define ROVER_AVOID_UNTRACKED_MIN_YAW_DEG      25.0f

/* 측면 센서가 본 열린 방향을 정면으로 재확인하는 조건 */
#define ROVER_AVOID_OPEN_FRONT_MM            1000U
#define ROVER_AVOID_OPEN_CONFIRM_COUNT          3U

/* 초기 45도 회전이 끝난 뒤 원래 Yaw 복귀를 시작할 총 전진 거리 */
#define ROVER_AVOID_FORWARD_TRAVEL_MM          1000.0f

/* 고RPM 제자리 회전은 정지 관성이 있으므로 이 범위 안이면 도달로 인정한다. */
#define ROVER_HEADING_ROTATION_TOLERANCE_DEG   12.0f

/* 원래 방향 복귀 완료 조건 */
#define ROVER_AVOID_HEADING_TOLERANCE_DEG       ROVER_HEADING_ROTATION_TOLERANCE_DEG
#define ROVER_AVOID_HEADING_HOLD_MS            300U
#define ROVER_AVOID_FINAL_FRONT_CLEAR_MM        500U

/* 회피 상태가 고착되거나 너무 멀리 이탈할 때의 안전 한계 */
#define ROVER_AVOID_STATE_TIMEOUT_MS          12000U
#define ROVER_AVOID_TOTAL_TIMEOUT_MS          30000U
#define ROVER_AVOID_MAX_TRAVEL_MM              2500.0f

/* 측면/전방 비상정지 뒤 실행하는 짧은 탈출 동작 */
#define ROVER_ESCAPE_SETTLE_MS                  300U
#define ROVER_ESCAPE_TURN_DEG                    45.0f
#define ROVER_ESCAPE_REQUIRED_OPEN_MM           350U
#define ROVER_ESCAPE_ADVANCE_MM                  400.0f

#endif
