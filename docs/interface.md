| 모듈         | 담당 기능                    | 외부 제공 함수         | 입력       | 반환값     |
| ------------ | --------------------------- | --------------------- | --------- | ---------- |
| Car Control  | 차량 제어 초기화             | `carControlInit()`    | 없음       |`void`     |
| Car Control  | 주기적인 차량상태 판단 및 제어| `carControlUpdate()`  | 없음       | `void`   | 
| Motor        | 모터 제어 초기화             | `motor_init()`         | 없음       | `void`   |
| Motor        | 좌/우 모터 속도 및 방향 설정  | `motor_set_speed()`     | 모터, 속도 | `void`   |
| Motor        | 특정 모터 정지               | `motor_stop()`         | 모터       | `void`   |
| Motor        | 좌/우 모터 전체 정지         | `motor_stop_all()`       | 없음      | `void`    |
| HC-SR04      | 초음파 센서 초기화           | `hcsr04_init()`         | 없음      | `void`    |
| HC-SR04      | 장애물 거리 측정             | `hcsr04_get_distance()`  | 없음      |`uint32_t` |
| IR Remote    | 리모컨 초기화                | `ir_remote_init()`     | 없음      | `void`    |
| IR Remote    | 주기적 신호 탐색            | `ir_remote_update()`   | 없음  |`RemoteCommand`|
| IR Remote    | true와 함께 전달            | `ir_remote_take_command()` | 없음  |`RemoteCommand`|
| IR Remote    | 원시 버튼 코드를 전달        | `ir_remote_on_raw_code()` | 없음  |`RemoteCommand`|
| Encoder      | 엔코더 초기화                | `encoder_init()`        | 없음      |`void`    |
| Encoder      | 좌/우 엔코더 카운트 읽기     | `encoder_get_count()`    | 휠         | `int32_t'|
| Encoder      | 엔코더 카운트 초기화         | `encoder_reset()`       | 휠         |`void`    |
| Encoder      | 엔코더 카운트 초기화         | `encoder_reset_all()`       | 휠         |`void`    |
| Debug UART   | 디버그 UART 초기화           | `debug_uart_init()`      | 없음      |`void`     |
| Debug UART   | 문자열 전송                 | `debug_uart_send()`      | 문자열      |`void`    |
| Debug UART   | 포맷 문자열 출력             | `debug_uart_printf()`    | 포맷, 가변 인자 |`void`|


---

- motor.h
speed:
-1000 ~ +1000

양수 → 정방향
음수 → 역방향
0    → 정지

---

## 1. Motor Interface
| 모듈 | 담당 기능 | 외부 제공 함수 | 입력 | 반환값 |
|---|---|---|---|---|
| Motor | 모터 초기화 | `motor_nit()` | 없음 | `void` |
| Motor | 개별 모터 방향 설정 | `motor_set_direction()` | `motor_t motor`, `motor_direction_t direction` | `void` |
| Motor | 개별 모터 속도 설정 | `motor_set_speed()` | `motor_t motor`, `uint8_t speed` | `void` |
| Motor | 모터 방향 및 속도 제어 | `motor_control()` | `motor_t motor`, `motor_direction_t direction`, `uint8_t speed` | `void` |
| Motor | 좌/우 모터 전체 정지 | `motor_stop_all()` | 없음 | `void` |


## 2. Drive Interface
| 모듈 | 담당 기능 | 외부 제공 함수 | 입력 | 반환값 |
|---|---|---|---|---|
| Drive | 주행 모듈 초기화 | `drive_init()` | 없음 | `void` |
| Drive | 차량 주행 명령 실행 | `drive_control()` | `drive_command_t command` | `void` |
| Drive | 기본 주행 속도 설정 | `drive_set_speed()` | `uint8_t speed` | `void` |
| Drive | 차량 정지 | `Drive_Stop()` | 없음 | `void` |

### Drive Command
- `DRIVE_CMD_STOP`: 정지
- `DRIVE_CMD_FORWARD`: 전진
- `DRIVE_CMD_BACKWARD`: 후진
- `DRIVE_CMD_LEFT`: 좌회전
- `DRIVE_CMD_RIGHT`: 우회전


## 3. IR Remote Interface
| 모듈 | 담당 기능 | 외부 제공 함수 | 입력 | 반환값 |
|---|---|---|---|---|
| IR Remote | IR 리모컨 모듈 초기화 | `ir_remote_init()` | 없음 | `void` |
| IR Remote | IR 원시 데이터 수신 | `ir_remote_on_raw_code()` | 없음 | `uint32_t` |
| IR Remote | IR 코드를 주행 명령으로 변환 | `ir_remote_take_command()` | `uint32_t raw_code` | `drive_command_t` |
| IR Remote | IR 입력에 해당하는 주행 명령 반환 | `IR_Remote_GetCommand()` | 없음 | `drive_command_t` |

### IR Command Mapping
| IR 입력 | 변환되는 주행 명령 |
|---|---|
| Forward | `DRIVE_CMD_FORWARD` |
| Backward | `DRIVE_CMD_BACKWARD` |
| Left | `DRIVE_CMD_LEFT` |
| Right | `DRIVE_CMD_RIGHT` |
| Stop | `DRIVE_CMD_STOP` |
| 입력 없음 / 알 수 없는 입력 | `DRIVE_CMD_NONE` |

## 4. 모듈 호출 관계
IR Remote 모듈은 Motor 모듈을 직접 제어하지 않는다.
주행 제어는 반드시 `drive_control()`을 통해 수행한다.
`IR Remote → drive_command_t → drive_control() → motor_control() → PWM / Direction`