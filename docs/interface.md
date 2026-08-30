# 주요 모듈 인터페이스

이 문서는 각 파일의 모든 함수를 나열하지 않고, 협업할 때 지켜야 할 경계와 주요
함수만 설명한다. 자세한 인자와 반환형은 같은 이름의 헤더 파일을 기준으로 한다.

## 계층별 역할

| 계층 | 대표 파일 | 책임 |
|---|---|---|
| App | `app/rover_app.c` | 입력을 읽고 제어 모듈을 연결하며 로그를 출력한다. |
| Mission | `mission_control.c` | 대기·직진·회피·복구·오류 중 현재 상태 하나를 보관한다. |
| Control | `proximity_monitor.c`, `heading_control.c`, `obstacle_avoidance.c`, `speed_bump_control.c` | 센서값을 검증하고 실제 주행 명령으로 바꾼다. |
| Driver | `drive.c`, `wheel.c`, `motor.c` | 최종 명령을 PWM과 방향 핀 출력으로 바꾼다. |
| Sensor | `encoder.c`, `mpu6050.c`, `vl53l0x.c`, `ir_remote.c` | 하드웨어에서 측정값과 명령을 읽는다. |

중요한 규칙은 모터를 여러 파일이 직접 만지지 않는 것이다.

```text
제어 모듈 → drive → wheel → motor → 실제 모터
```

부드러운 장애물 회피는 직접 PWM을 사용하지 않고 `drive_follow_heading()`에 기본
RPM과 목표 Yaw를 전달한다.

## 앱과 임무 상태

### `rover_app`

- `rover_app_init()` — 모든 모듈을 순서대로 초기화하고 준비 로그를 출력한다.
- `rover_app_run()` — 메인 반복문에서 계속 호출한다. 지연 없이 센서와 제어를 갱신한다.

### `mission_control`

주요 상태는 `IDLE`, `DRIVING`, `OBSTACLE_AVOIDANCE`,
`BUMP_HEADING_RECOVERY`, `STOPPED`, `ERROR`이다.

- `mission_control_start()` — 정상 직진 상태로 전환
- `mission_control_begin_avoidance()` — 장애물 회피 상태로 전환
- `mission_control_begin_bump_recovery()` — 방지턱 자세 복구 상태로 전환
- `mission_control_resume_driving()` — 특수 동작이 끝난 뒤 직진 복귀
- `mission_control_stop()` — 사용자 요청에 따른 정지
- `mission_control_emergency_stop()` — 센서·모터 오류에 따른 안전 정지

## 주행 출력

### `drive`

- `drive_init()` — 모터, 엔코더, 바퀴 제어, 방향 제어를 초기화
- `drive_forward()` / `drive_stop()` — 정상 주행 시작과 정지
- `drive_set_speed(speed)` — 80~100 주행 단계값을 대응 RPM으로 변환해 설정
- `drive_update(dt)` — 반드시 반복 호출하여 IMU·엔코더 피드백을 반영
- `drive_set_direct_output(left, right)` — 회전 등 특수 동작의 직접 출력
- `drive_follow_heading(rpm, yaw)` — 지정 RPM으로 전진하며 연속 목표 Yaw 추종
- `wheel_get_target_rpm(WHEEL_LEFT/RIGHT)` — 코드가 각 바퀴에 요구한 RPM
- `wheel_get_rpm(WHEEL_LEFT/RIGHT)` — 엔코더가 측정한 실제 RPM

`drive`는 최종 모터 명령의 단일 통로다. 직접 출력 모드가 켜진 동안에는 방향
제어기가 모터 출력을 덮어쓰지 않고 자세 측정만 갱신한다.

### `heading_control`

- `heading_start_forward()` — 현재 yaw를 기준으로 직진 제어 시작
- `heading_update(dt)` — yaw 오차와 바퀴 RPM을 이용해 좌우 목표를 보정
- `heading_update_measurement(dt)` — 모터 출력 없이 자세값만 갱신
- `heading_reset_reference()` — 현재 방향을 새로운 0도로 설정
- `heading_track_target(yaw)` — PID 상태를 지우지 않고 주행 중 목표 Yaw 갱신

## 센서

### `encoder`

TIM5 CH1·CH2 입력 캡처 인터럽트가 펄스 시각을 1 µs 단위로 저장한다.

```text
펄스 사이 시간 측정 → 순간 RPM 계산 → EMA 필터 → 실제 RPM
```

- `encoder_update()` — 새 펄스를 RPM으로 계산하고 정지 시간초과를 처리
- `encoder_get_rpm()` — 필터를 거친 RPM 크기 반환
- `encoder_get_count()` — 누적 펄스 수 반환
- `encoder_get_delta_count()` — 마지막 호출 이후 펄스 변화량 반환

공식은 `RPM = 60,000,000 / (펄스 간격 µs × 회전당 펄스 수)`이다. 기존처럼
100 ms 동안 몇 개가 들어왔는지 세지 않으므로 값이 30 RPM 단위로만 변하지 않는다.
EMA는 새 측정값 30%, 기존값 70%를 섞어 한 펄스의 흔들림을 줄인다.

### `mpu6050`

- `mpu6050_start()` — 통신 확인, 자이로 영점 보정, 자세 기준 설정
- `mpu6050_update_orientation(dt)` — X/Y/Z 자세 갱신
- `mpu6050_get_orientation()` — 기준점 대비 자세 반환

X·Y는 짧은 순간에 안정적인 자이로와 장시간 기준을 잡는 가속도계를 결합한
상보 필터를 사용한다. Z(yaw)는 중력으로 보정할 수 없으므로 자이로 적분값이다.

### `vl53l0x`, `proximity_monitor`, `ir_remote`

- `vl53l0x_update()` — LEFT/FRONT/RIGHT의 non-blocking 측정을 진행한다.
- `proximity_monitor_update()` — 새 결과를 중앙값 필터와 오류·신선도 상태에 반영한다.
- `proximity_monitor_get_snapshot()` — 세 거리, 측정 순번, 건강 상태를 한 번에 반환한다.
- `ir_remote_update()` — 캡처한 NEC 신호를 명령으로 해석한다.
- `ir_remote_get_command()` — 새 명령이 있을 때만 반환한다.

## 공통 물리 설정

바퀴 지름, 엔코더 슬롯 수, RPM 범위, 장애물 거리와 각도처럼 여러 모듈이
함께 알아야 하는 값은
`hw/rover_config.h`에서 한 번만 정의한다. 모터나 바퀴를 교체했을 때 여러 C 파일의
숫자를 따로 고치지 않는다.

회피 상태와 모든 튜닝값의 의미는 [smooth-obstacle-avoidance.md](smooth-obstacle-avoidance.md)를
기준으로 한다.
