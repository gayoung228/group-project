# 2WD 인터페이스 정의서

## 1. Motor Interface

| 모듈 | 담당 기능 | 외부 제공 함수 | 입력 | 반환값 |
|---|---|---|---|---|
| Motor | 모터 초기화 | `motor_init()` | 없음 | `void` |
| Motor | 개별 모터 방향 설정 | `motor_set_direction()` | `motor_t motor`, `motor_direction_t direction` | `void` |
| Motor | 개별 모터 속도 설정 | `motor_set_speed()` | `motor_t motor`, `uint8_t speed` | `void` |
| Motor | 모터 방향 및 속도 동시 설정 | `motor_set_output()` | `motor_t motor`, `int16_t output` | `void` |
| Motor | 개별 모터 정지 | `motor_stop()` | `motor_t motor` | `void` |
| Motor | 좌·우 모터 전체 정지 | `motor_stop_all()` | 없음 | `void` |
| Motor | 개별 모터 속도 반환 | `motor_get_speed()` | `motor_t motor` | `uint8_t` |
| Motor | 개별 모터 방향 반환 | `motor_get_direction()` | `motor_t motor` | `motor_direction_t` |

### Motor Type

- `MOTOR_LEFT`: 왼쪽 모터
- `MOTOR_RIGHT`: 오른쪽 모터
- `MOTOR_DIRECTION_FORWARD`: 정방향
- `MOTOR_DIRECTION_REVERSE`: 역방향
- `MOTOR_DIRECTION_STOP`: 정지

`motor_set_output()`의 입력 범위는 `-100~100`이며 음수는 후진, 0은 정지, 양수는 전진을 의미한다.

## 2. Drive Interface

| 모듈 | 담당 기능 | 외부 제공 함수 | 입력 | 반환값 |
|---|---|---|---|---|
| Drive | 주행 모듈 초기화 | `drive_init()` | 없음 | `void` |
| Drive | 차량 주행 명령 실행 | `drive_control()` | `drive_command_t command` | `void` |
| Drive | 기본 주행 속도 설정 | `drive_set_speed()` | `uint8_t speed` | `void` |
| Drive | 좌·우 바퀴 속도 개별 설정 | `drive_set_wheel_speed()` | `int16_t left_speed`, `int16_t right_speed` | `void` |
| Drive | 차량 전진 | `drive_forward()` | 없음 | `void` |
| Drive | 차량 후진 | `drive_backward()` | 없음 | `void` |
| Drive | 차량 좌회전 | `drive_left()` | 없음 | `void` |
| Drive | 차량 우회전 | `drive_right()` | 없음 | `void` |
| Drive | 차량 정지 | `drive_stop()` | 없음 | `void` |
| Drive | 기본 속도 반환 | `drive_get_speed()` | 없음 | `uint8_t` |
| Drive | 왼쪽 적용 속도 반환 | `drive_get_left_speed()` | 없음 | `int16_t` |
| Drive | 오른쪽 적용 속도 반환 | `drive_get_right_speed()` | 없음 | `int16_t` |
| Drive | 현재 주행 명령 반환 | `drive_get_command()` | 없음 | `drive_command_t` |

### Drive Command

- `DRIVE_COMMAND_NONE`: 새로운 명령 없음
- `DRIVE_COMMAND_FORWARD`: 전진
- `DRIVE_COMMAND_BACKWARD`: 후진
- `DRIVE_COMMAND_LEFT`: 좌회전
- `DRIVE_COMMAND_RIGHT`: 우회전
- `DRIVE_COMMAND_STOP`: 정지

차량 단위 제어는 반드시 `drive` API를 사용한다.

```text
mission_control → drive_set_wheel_speed() → motor_set_output() → PWM / Direction
```

## 3. Encoder Interface

| 모듈 | 담당 기능 | 외부 제공 함수 | 입력 | 반환값 |
|---|---|---|---|---|
| Encoder | 엔코더 초기화 | `encoder_init()` | 없음 | `void` |
| Encoder | 카운트와 RPM 초기화 | `encoder_reset()` | 없음 | `void` |
| Encoder | 펄스 변화량과 RPM 갱신 | `encoder_update()` | `uint32_t elapsed_time_ms` | `void` |
| Encoder | 누적 펄스 수 반환 | `encoder_get_count()` | `encoder_id_t encoder` | `int32_t` |
| Encoder | 최근 펄스 변화량 반환 | `encoder_get_delta_count()` | `encoder_id_t encoder` | `int32_t` |
| Encoder | 현재 RPM 반환 | `encoder_get_rpm()` | `encoder_id_t encoder` | `float` |
| Encoder | 바퀴 회전 여부 반환 | `encoder_is_running()` | `encoder_id_t encoder` | `bool` |
| Encoder | 펄스 인터럽트 전달 | `encoder_on_pulse()` | `encoder_id_t encoder` | `void` |

### Encoder ID

- `ENCODER_LEFT`: 왼쪽 바퀴 엔코더
- `ENCODER_RIGHT`: 오른쪽 바퀴 엔코더

`encoder_on_pulse()`는 외부 인터럽트 방식에서 ISR 또는 HAL 콜백이 호출한다.

## 4. MPU6050 Interface

| 모듈 | 담당 기능 | 외부 제공 함수 | 입력 | 반환값 |
|---|---|---|---|---|
| MPU6050 | IMU 초기화 및 연결 확인 | `mpu6050_init()` | 없음 | `bool` |
| MPU6050 | 가속도·자이로 갱신 | `mpu6050_update()` | 없음 | `bool` |
| MPU6050 | 마지막 측정 데이터 반환 | `mpu6050_get_data()` | `mpu6050_data_t *data` | `bool` |
| MPU6050 | 자이로 영점 보정 | `mpu6050_calibrate_gyro()` | `uint16_t sample_count` | `bool` |
| MPU6050 | 센서 준비 여부 반환 | `mpu6050_is_ready()` | 없음 | `bool` |

### MPU6050 Data

| 멤버 | 의미 | 단위 |
|---|---|---|
| `accel_x_g`, `accel_y_g`, `accel_z_g` | X·Y·Z축 가속도 | g |
| `gyro_x_dps`, `gyro_y_dps`, `gyro_z_dps` | X·Y·Z축 각속도 | degree/s |
| `temperature_c` | 센서 내부 온도 | °C |

자이로 보정 중에는 차량을 움직이지 않는다. 가속도는 노면 판단, Z축 자이로는 방향 유지에 사용한다.

## 5. VL53L0X Interface

| 모듈 | 담당 기능 | 외부 제공 함수 | 입력 | 반환값 |
|---|---|---|---|---|
| VL53L0X | I2C 및 XSHUT 상태 초기화 | `vl53l0x_init()` | 없음 | `void` |
| VL53L0X | 개별 센서 초기화 및 주소 변경 | `vl53l0x_init_sensor()` | `vl53l0x_id_t sensor`, `uint8_t new_address` | `bool` |
| VL53L0X | 센서 3개 순차 초기화 | `vl53l0x_init_all()` | 없음 | `bool` |
| VL53L0X | 개별 센서 거리 갱신 | `vl53l0x_update_sensor()` | `vl53l0x_id_t sensor` | `bool` |
| VL53L0X | 전체 센서 거리 갱신 | `vl53l0x_update()` | 없음 | `void` |
| VL53L0X | 측정 거리 반환 | `vl53l0x_get_distance_mm()` | `vl53l0x_id_t sensor` | `uint16_t` |
| VL53L0X | 센서 준비 여부 반환 | `vl53l0x_is_ready()` | `vl53l0x_id_t sensor` | `bool` |
| VL53L0X | 거리값 유효 여부 반환 | `vl53l0x_is_valid()` | `vl53l0x_id_t sensor` | `bool` |

### VL53L0X ID 및 주소

| 센서 ID | 역할 | 7비트 I2C 주소 |
|---|---|---:|
| `VL53L0X_FRONT` | 전방 센서 | `0x30` |
| `VL53L0X_LEFT` | 좌측 센서 | `0x31` |
| `VL53L0X_RIGHT` | 우측 센서 | `0x32` |

기본 주소는 `0x29`이며 다음 순서로 초기화한다.

```text
모든 XSHUT LOW
→ FRONT HIGH 및 0x30 설정
→ LEFT HIGH 및 0x31 설정
→ RIGHT HIGH 및 0x32 설정
```

## 6. Odometry Interface

| 모듈 | 담당 기능 | 외부 제공 함수 | 입력 | 반환값 |
|---|---|---|---|---|
| Odometry | 거리 계산 모듈 초기화 | `odometry_init()` | 없음 | `void` |
| Odometry | 누적 거리 초기화 | `odometry_reset()` | 없음 | `void` |
| Odometry | 거리와 선속도 갱신 | `odometry_update()` | 없음 | `void` |
| Odometry | 바퀴 설정 적용 | `odometry_set_config()` | `const odometry_config_t *config` | `void` |
| Odometry | 왼쪽 누적 거리 반환 | `odometry_get_left_distance_mm()` | 없음 | `float` |
| Odometry | 오른쪽 누적 거리 반환 | `odometry_get_right_distance_mm()` | 없음 | `float` |
| Odometry | 시작점 기준 변위 반환 | `odometry_get_displacement_mm()` | 없음 | `float` |
| Odometry | 누적 병진 거리 반환 | `odometry_get_traveled_distance_mm()` | 없음 | `float` |
| Odometry | 왼쪽 선속도 반환 | `odometry_get_left_speed_mm_s()` | 없음 | `float` |
| Odometry | 오른쪽 선속도 반환 | `odometry_get_right_speed_mm_s()` | 없음 | `float` |

### Odometry Config

- `wheel_diameter_mm`: 바퀴 지름(mm)
- `pulses_per_revolution`: 바퀴 1회전당 엔코더 펄스 수

제자리 회전을 이동 거리에 포함하지 않도록 다음 식을 사용한다.

```c
delta_distance = (delta_left_distance + delta_right_distance) / 2.0f;
```

## 7. Heading Control Interface

| 모듈 | 담당 기능 | 외부 제공 함수 | 입력 | 반환값 |
|---|---|---|---|---|
| Heading Control | 방향 제어 초기화 | `heading_control_init()` | 없음 | `void` |
| Heading Control | 현재·목표 방향 초기화 | `heading_control_reset()` | `float initial_heading_deg` | `void` |
| Heading Control | 현재 방향과 보정값 갱신 | `heading_control_update()` | `float gyro_z_dps`, `float delta_time_sec` | `void` |
| Heading Control | 목표 방향 설정 | `heading_control_set_target()` | `float target_heading_deg` | `void` |
| Heading Control | 제어 설정 적용 | `heading_control_set_config()` | `const heading_control_config_t *config` | `void` |
| Heading Control | 방향 보정 활성화 | `heading_control_enable()` | 없음 | `void` |
| Heading Control | 방향 보정 비활성화 | `heading_control_disable()` | 없음 | `void` |
| Heading Control | 활성화 여부 반환 | `heading_control_is_enabled()` | 없음 | `bool` |
| Heading Control | 현재 방향 반환 | `heading_control_get_current()` | 없음 | `float` |
| Heading Control | 목표 방향 반환 | `heading_control_get_target()` | 없음 | `float` |
| Heading Control | 방향 오차 반환 | `heading_control_get_error()` | 없음 | `float` |
| Heading Control | 모터 보정값 반환 | `heading_control_get_correction()` | 없음 | `float` |

### Heading Control Config

- `kp`: 방향 오차 비례 제어 게인
- `max_correction`: 좌·우 모터 최대 보정값

장애물 회피 중 `heading_control_disable()`을 호출하더라도 현재 방향 적분을 위해 `heading_control_update()`는 계속 호출한다.

## 8. Road Detector Interface

| 모듈 | 담당 기능 | 외부 제공 함수 | 입력 | 반환값 |
|---|---|---|---|---|
| Road Detector | 노면 감지 초기화 | `road_detector_init()` | 없음 | `void` |
| Road Detector | 진동값과 상태 초기화 | `road_detector_reset()` | 없음 | `void` |
| Road Detector | IMU 기반 노면 상태 갱신 | `road_detector_update()` | `const mpu6050_data_t *imu_data` | `void` |
| Road Detector | 판단 임계값 설정 | `road_detector_set_config()` | `const road_detector_config_t *config` | `void` |
| Road Detector | 현재 노면 상태 반환 | `road_detector_get_state()` | 없음 | `road_state_t` |
| Road Detector | 현재 진동 크기 반환 | `road_detector_get_vibration()` | 없음 | `float` |

### Road State

- `ROAD_STATE_FLAT`: 평지
- `ROAD_STATE_SMALL_VIBRATION`: 작은 진동
- `ROAD_STATE_LARGE_VIBRATION`: 큰 진동

### Road Detector Config

- `small_vibration_threshold`: 작은 진동 판단 임계값
- `large_vibration_threshold`: 큰 진동 판단 임계값

`road_detector`는 노면 상태만 판단한다. AI 적용 시 API는 유지하고 `road_detector_update()` 내부 판단만 교체한다.

## 9. Speed Control Interface

| 모듈 | 담당 기능 | 외부 제공 함수 | 입력 | 반환값 |
|---|---|---|---|---|
| Speed Control | 속도제어 초기화 | `speed_control_init()` | 없음 | `void` |
| Speed Control | 목표 속도 초기화 | `speed_control_reset()` | 없음 | `void` |
| Speed Control | 노면별 목표 속도 갱신 | `speed_control_update()` | `road_state_t road_state` | `void` |
| Speed Control | 단계별 속도 설정 | `speed_control_set_config()` | `const speed_control_config_t *config` | `void` |
| Speed Control | 현재 목표 속도 반환 | `speed_control_get_target_speed()` | 없음 | `uint8_t` |

### Speed Control Config

| 멤버 | 노면 | 초기값 예시 |
|---|---|---:|
| `flat_speed` | 평지 | 100% |
| `small_vibration_speed` | 작은 진동 | 70% |
| `large_vibration_speed` | 큰 진동 | 40% |

```text
MPU6050 → road_detector → road_state_t → speed_control → 기본 목표 속도
```

## 10. Obstacle Avoidance Interface

| 모듈 | 담당 기능 | 외부 제공 함수 | 입력 | 반환값 |
|---|---|---|---|---|
| Obstacle Avoidance | 회피 상태 초기화 | `obstacle_avoidance_init()` | 없음 | `void` |
| Obstacle Avoidance | 회피 동작 초기화 | `obstacle_avoidance_reset()` | 없음 | `void` |
| Obstacle Avoidance | 회피 설정 적용 | `obstacle_avoidance_set_config()` | `const obstacle_avoidance_config_t *config` | `void` |
| Obstacle Avoidance | 회피 방향 선택 | `obstacle_avoidance_select_direction()` | `uint16_t left_distance_mm`, `uint16_t right_distance_mm` | `avoid_direction_t` |
| Obstacle Avoidance | 회피 시작 | `obstacle_avoidance_start()` | `avoid_direction_t direction`, `float current_heading_deg`, `float current_distance_mm` | `bool` |
| Obstacle Avoidance | 회피 상태 갱신 | `obstacle_avoidance_update()` | `float current_heading_deg`, `float current_distance_mm`, `uint16_t front_distance_mm` | `void` |
| Obstacle Avoidance | 현재 회피 상태 반환 | `obstacle_avoidance_get_state()` | 없음 | `avoid_state_t` |
| Obstacle Avoidance | 현재 회피 방향 반환 | `obstacle_avoidance_get_direction()` | 없음 | `avoid_direction_t` |
| Obstacle Avoidance | 왼쪽 회피 속도 반환 | `obstacle_avoidance_get_left_speed()` | 없음 | `int16_t` |
| Obstacle Avoidance | 오른쪽 회피 속도 반환 | `obstacle_avoidance_get_right_speed()` | 없음 | `int16_t` |
| Obstacle Avoidance | 회피 진행 여부 반환 | `obstacle_avoidance_is_running()` | 없음 | `bool` |
| Obstacle Avoidance | 회피 완료 여부 반환 | `obstacle_avoidance_is_completed()` | 없음 | `bool` |
| Obstacle Avoidance | 회피 실패 여부 반환 | `obstacle_avoidance_has_failed()` | 없음 | `bool` |

### Avoid Direction

- `AVOID_DIRECTION_NONE`: 방향 미결정
- `AVOID_DIRECTION_LEFT`: 왼쪽 회피
- `AVOID_DIRECTION_RIGHT`: 오른쪽 회피

### Avoid State

- `AVOID_STATE_IDLE`: 회피 대기
- `AVOID_STATE_STOP`: 장애물 앞 정지
- `AVOID_STATE_CHECK_SPACE`: 좌·우 공간 확인
- `AVOID_STATE_FIRST_TURN`: 첫 번째 90도 회전
- `AVOID_STATE_FORWARD`: 설정 거리만큼 우회 전진
- `AVOID_STATE_SECOND_TURN`: 기존 방향으로 두 번째 90도 회전
- `AVOID_STATE_COMPLETED`: 회피 완료
- `AVOID_STATE_FAILED`: 회피 실패

### Obstacle Avoidance Config

- `obstacle_distance_mm`: 장애물 판단 거리(mm)
- `bypass_distance_mm`: 우회 전진 거리(mm)
- `forward_speed`: 회피 전진 속도(%)
- `turn_speed`: 회전 속도(%)
- `turn_angle_deg`: 회전 목표 각도(degree)

## 11. Mission Control Interface

| 모듈 | 담당 기능 | 외부 제공 함수 | 입력 | 반환값 |
|---|---|---|---|---|
| Mission Control | 전체 임무 초기화 | `mission_control_init()` | 없음 | `void` |
| Mission Control | 현재 임무 초기화 | `mission_control_reset()` | 없음 | `void` |
| Mission Control | 목표 거리 설정 | `mission_control_set_target_distance()` | `uint32_t target_distance_mm` | `bool` |
| Mission Control | 자율주행 시작 | `mission_control_start()` | 없음 | `bool` |
| Mission Control | 상태 및 최종 출력 갱신 | `mission_control_update()` | 없음 | `void` |
| Mission Control | 현재 임무 중지 | `mission_control_stop()` | 없음 | `void` |
| Mission Control | 긴급 정지 | `mission_control_emergency_stop()` | 없음 | `void` |
| Mission Control | 현재 임무 상태 반환 | `mission_control_get_state()` | 없음 | `mission_state_t` |
| Mission Control | 목표 거리 반환 | `mission_control_get_target_distance()` | 없음 | `uint32_t` |
| Mission Control | 누적 거리 반환 | `mission_control_get_traveled_distance()` | 없음 | `uint32_t` |
| Mission Control | 남은 거리 반환 | `mission_control_get_remaining_distance()` | 없음 | `uint32_t` |
| Mission Control | 임무 완료 여부 반환 | `mission_control_is_completed()` | 없음 | `bool` |

### Mission State

- `MISSION_STATE_IDLE`: 목표 거리 미설정
- `MISSION_STATE_READY`: 목표 거리 설정 완료
- `MISSION_STATE_DRIVING`: 일반 자율주행
- `MISSION_STATE_OBSTACLE_AVOIDANCE`: 장애물 회피
- `MISSION_STATE_COMPLETED`: 목표 거리 도달
- `MISSION_STATE_STOPPED`: 외부 명령으로 중지
- `MISSION_STATE_ERROR`: 센서 또는 제어 오류

## 12. Status Indicator Interface

| 모듈 | 담당 기능 | 외부 제공 함수 | 입력 | 반환값 |
|---|---|---|---|---|
| Status Indicator | LED·부저 초기화 | `status_indicator_init()` | 없음 | `void` |
| Status Indicator | 표시 상태 설정 | `status_indicator_set_state()` | `status_indicator_state_t state` | `void` |
| Status Indicator | 점멸·부저 패턴 갱신 | `status_indicator_update()` | 없음 | `void` |
| Status Indicator | 모든 표시 끄기 | `status_indicator_off()` | 없음 | `void` |
| Status Indicator | 현재 표시 상태 반환 | `status_indicator_get_state()` | 없음 | `status_indicator_state_t` |

### Status Indicator State

- `STATUS_INDICATOR_OFF`: 표시 OFF
- `STATUS_INDICATOR_READY`: 출발 대기
- `STATUS_INDICATOR_NORMAL`: 정상 주행
- `STATUS_INDICATOR_CAUTION`: 작은 진동 및 감속
- `STATUS_INDICATOR_DANGER`: 큰 진동 또는 위험
- `STATUS_INDICATOR_COMPLETE`: 주행 완료
- `STATUS_INDICATOR_ERROR`: 오류

## 13. Debug UART Interface

| 모듈 | 담당 기능 | 외부 제공 함수 | 입력 | 반환값 |
|---|---|---|---|---|
| Debug UART | UART 및 버퍼 초기화 | `debug_uart_init()` | 없음 | `bool` |
| Debug UART | 문자열 전송 | `debug_uart_send()` | `const char *string` | `bool` |
| Debug UART | 원시 데이터 전송 | `debug_uart_send_data()` | `const uint8_t *data`, `size_t length` | `bool` |
| Debug UART | 형식 문자열 전송 | `debug_uart_send_format()` | `const char *format`, `...` | `bool` |
| Debug UART | 비동기 송신 상태 갱신 | `debug_uart_update()` | 없음 | `void` |
| Debug UART | 송신 중 여부 반환 | `debug_uart_is_busy()` | 없음 | `bool` |
| Debug UART | 송신 완료 인터럽트 전달 | `debug_uart_on_tx_complete()` | 없음 | `void` |

UART는 센서값, RPM, 거리, 진동값, 주행 상태를 PC에서 확인하는 용도로 사용한다.

## 14. Application Interface

| 모듈 | 담당 기능 | 외부 제공 함수 | 입력 | 반환값 |
|---|---|---|---|---|
| Application | 전체 모듈 초기화 | `ap_init()` | 없음 | `void` |
| Application | 메인 제어 루프 실행 | `ap_main()` | 없음 | `void` |

`main.c`는 CubeMX 초기화 이후 다음 함수만 호출한다.

```c
ap_init();
ap_main();
```

## 15. 전체 모듈 호출 관계

### 일반 주행

```text
MPU6050 가속도
→ road_detector_update()
→ road_state_t
→ speed_control_update()
→ 기본 목표 속도

MPU6050 Z축 자이로
→ heading_control_update()
→ 방향 보정값

기본 목표 속도 + 방향 보정값
→ mission_control_update()
→ drive_set_wheel_speed()
→ motor_set_output()
→ PWM / Direction
```

### 장애물 회피

```text
VL53L0X FRONT < 장애물 기준 거리
→ heading_control_disable()
→ LEFT / RIGHT 거리 비교
→ obstacle_avoidance_select_direction()
→ obstacle_avoidance_start()
→ obstacle_avoidance_update()
→ drive_set_wheel_speed()
→ 회피 완료
→ heading_control_enable()
```

### 목표 거리 주행

```text
Encoder L/R
→ encoder_update()
→ odometry_update()
→ 누적 병진 이동 거리
→ mission_control_update()
→ 목표 거리 도달
→ drive_stop()
```

## 16. 최종 제어 규칙

1. `motor`는 GPIO와 PWM만 제어한다.
2. `drive`는 좌·우 모터를 조합하여 차량 움직임을 만든다.
3. `road_detector`는 노면 상태만 판단한다.
4. `speed_control`은 기본 목표 속도만 결정한다.
5. `heading_control`은 방향 보정값만 계산한다.
6. `obstacle_avoidance`는 회피 상태와 회피용 속도만 제공한다.
7. `mission_control`만 최종 모터 출력을 결정한다.
8. `ap_main`은 각 모듈을 정해진 주기로 호출한다.

```text
긴급 정지
→ 목표 거리 도달
→ 장애물 회피
→ 일반 방향 유지 및 노면 속도제어
```

