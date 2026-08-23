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
| IR Remote    | ???                         | `ir_remote_update()`   | 없음  |`RemoteCommand`|
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