# TERRA Rover Pin Map

이 문서는 현재 `car_project.ioc`에 설정되어 있고 실제로 사용할 핀만 정리한다.
예약 핀과 사용하지 않는 HC-SR04 설정은 포함하지 않는다.

## 하드웨어 구성

- STM32 NUCLEO-F411RE
- TB6612FNG 듀얼 DC 모터 드라이버
- DC 기어 모터 2개
- HC-020K 휠 엔코더 2개
- MPU6050
- VL53L0X 정면 센서 1개 사용 중(좌·우 센서 핀은 확장용으로 예약)
- IR 리모컨 수신기
- ST-LINK Virtual COM Port

## 전체 핀맵

| 기능 | 코드 이름 | Nucleo 핀 | STM32 핀 | CubeMX 설정 | 연결 대상 |
|---|---|---:|---:|---|---|
| 왼쪽 모터 PWM | `MOTOR_L_PWM` | D6 | PB10 | TIM2_CH3 PWM | TB6612FNG PWMA |
| 왼쪽 모터 방향 1 | `MOTOR_L_IN1` | D4 | PB5 | GPIO Output Push-Pull | TB6612FNG AIN1 |
| 왼쪽 모터 방향 2 | `MOTOR_L_IN2` | D5 | PB4 | GPIO Output Push-Pull | TB6612FNG AIN2 |
| 오른쪽 모터 PWM | `MOTOR_R_PWM` | D9 | PC7 | TIM3_CH2 PWM | TB6612FNG PWMB |
| 오른쪽 모터 방향 1 | `MOTOR_R_IN1` | D7 | PA8 | GPIO Output Push-Pull | TB6612FNG BIN1 |
| 오른쪽 모터 방향 2 | `MOTOR_R_IN2` | D8 | PA9 | GPIO Output Push-Pull | TB6612FNG BIN2 |
| 모터 드라이버 활성화 | `MOTOR_STBY` | D3 | PB3 | GPIO Output Push-Pull | TB6612FNG STBY |
| 왼쪽 엔코더 펄스 | — | A0 | PA0 | TIM5_CH1 Input Capture | 왼쪽 HC-020K OUT |
| 오른쪽 엔코더 펄스 | — | A1 | PA1 | TIM5_CH2 Input Capture | 오른쪽 HC-020K OUT |
| I2C Clock | — | D15 | PB8 | I2C1_SCL | MPU6050 및 VL53L0X SCL |
| I2C Data | — | D14 | PB9 | I2C1_SDA | MPU6050 및 VL53L0X SDA |
| 왼쪽 거리 센서 종료 제어 | `TOF_LEFT_XSHUT` | A2 | PA4 | GPIO Output Open-Drain | 확장용 예약 |
| 정면 거리 센서 종료 제어 | `TOF_FRONT_XSHUT` | D2 | PA10 | GPIO Output Open-Drain | 정면 VL53L0X XSHUT |
| 오른쪽 거리 센서 종료 제어 | `TOF_RIGHT_XSHUT` | A3 | PB0 | GPIO Output Open-Drain | 확장용 예약 |
| IR 리모컨 수신 신호 | — | D10 | PB6 | TIM4_CH1 Input Capture | IR 수신기 OUT |
| 디버그 UART 송신 | `USART_TX` | D1 | PA2 | USART2_TX | ST-LINK Virtual COM Port |
| 디버그 UART 수신 | `USART_RX` | D0 | PA3 | USART2_RX | ST-LINK Virtual COM Port |

## TB6612FNG 연결

```text
NUCLEO D6  / PB10 → PWMA
NUCLEO D4  / PB5  → AIN1
NUCLEO D5  / PB4  → AIN2

NUCLEO D9  / PC7  → PWMB
NUCLEO D7  / PA8  → BIN1
NUCLEO D8  / PA9  → BIN2

NUCLEO D3  / PB3  → STBY
NUCLEO 3.3V        → VCC
NUCLEO GND         → GND

모터 배터리 (+)    → VM
모터 배터리 (-)    → GND

AO1 / AO2          → 왼쪽 모터
BO1 / BO2          → 오른쪽 모터
```

코드와 배선에서 다음 이름을 일관되게 사용한다.

```text
Motor A = LEFT
Motor B = RIGHT
```

## 엔코더 연결

```text
왼쪽 HC-020K OUT  → NUCLEO A0 / PA0 / TIM5_CH1
오른쪽 HC-020K OUT → NUCLEO A1 / PA1 / TIM5_CH2
엔코더 GND         → 공통 GND
```

TIM5 설정:

```text
Clock Source       : Internal Clock
Prescaler          : 83
Counter Period     : 4294967295
Channel 1          : Input Capture Direct Mode
Channel 2          : Input Capture Direct Mode
Polarity           : Rising Edge
Input Filter       : 4
TIM5 Interrupt     : Enabled
```

두 엔코더는 서로 다른 바퀴의 단일 펄스 신호이므로 TIM5의 Encoder Mode가 아니라 CH1과 CH2의 Input Capture를 사용한다.

## IR 리모컨 연결

```text
IR 수신기 OUT → NUCLEO D10 / PB6 / TIM4_CH1
IR 수신기 VCC → NUCLEO 3.3V
IR 수신기 GND → NUCLEO GND
```

TIM4 설정:

```text
Clock Source       : Internal Clock
Prescaler          : 83
Counter Period     : 65535
Channel 1          : Input Capture Direct Mode
Polarity           : Falling Edge
Input Prescaler    : No Division
Input Filter       : 4
TIM4 Interrupt     : Enabled
Interrupt Priority : 0
```

TIM4의 입력 클럭은 84MHz이다. Prescaler가 83이므로 타이머 카운트 1은
`1us`를 뜻하며, IR 수신기의 하강 에지 사이 시간을 마이크로초 단위로 측정할 수 있다.

CubeMX는 TIM4와 인터럽트 핸들러를 생성하지만 입력 캡처 측정을 자동으로 시작하지는
않는다. 리모컨 드라이버 초기화에서 다음 함수를 한 번 호출해야 한다.

```c
HAL_TIM_IC_Start_IT(&htim4, TIM_CHANNEL_1);
```

인터럽트에서는 캡처 시간과 수신 플래그만 저장한다. 버튼 코드 해석, UART 출력,
모터 명령 처리는 메인 제어 흐름에서 수행하여 엔코더 TIM5 인터럽트를 방해하지 않도록 한다.

## I2C 센서 연결

MPU6050과 정면 VL53L0X는 하나의 I2C1 버스를 공유한다. 좌·우 VL53L0X를
추가할 때도 같은 두 선에 병렬로 연결하고 XSHUT으로 주소를 나눈다.

```text
PB8 / D15 → MPU6050 SCL
          → VL53L0X FRONT SCL

PB9 / D14 → MPU6050 SDA
          → VL53L0X FRONT SDA
```

I2C1 설정:

```text
Mode            : I2C
Clock Speed     : 100 kHz
Addressing Mode : 7-bit
SCL             : PB8
SDA             : PB9
```

MPU6050의 `AD0`는 GND에 연결하여 7비트 주소 `0x68`을 사용한다. 초기 센서 확인 단계에서는 `INT` 핀을 연결하지 않고 폴링 방식으로 값을 읽는다.

## VL53L0X XSHUT 연결

```text
PA4  / A2 → VL53L0X LEFT XSHUT
PA10 / D2 → VL53L0X FRONT XSHUT
PB0  / A3 → VL53L0X RIGHT XSHUT
```

세 XSHUT 핀은 확장을 고려해 CubeMX에 설정되어 있다. 현재는 FRONT만 연결한다.
공통 설정은 다음과 같다.

```text
GPIO Mode        : Output Open-Drain
Initial Level    : Low
Pull-up/down     : No Pull
Maximum Speed    : Low
```

나중에 VL53L0X 3개를 모두 연결하면 전원을 켤 때 기본 I2C 주소가 같으므로
다음 순서로 초기화한다.

```text
모든 XSHUT LOW
    ↓
LEFT만 활성화하고 새 주소 설정
    ↓
FRONT를 활성화하고 새 주소 설정
    ↓
RIGHT를 활성화하고 새 주소 설정
```

권장 7비트 주소:

```text
MPU6050        : 0x68
VL53L0X LEFT   : 0x30
VL53L0X FRONT  : 0x31
VL53L0X RIGHT  : 0x32
```

VL53L0X 주소는 전원을 껐다 켜면 기본값으로 돌아오기 때문에 부팅할 때마다 다시 설정한다.

## 모터 PWM 설정

```text
TIM2_CH3 / PB10 / MOTOR_L_PWM
Prescaler      : 0
Counter Period : 4199
PWM Mode       : PWM Mode 1
Initial Pulse  : 0

TIM3_CH2 / PC7 / MOTOR_R_PWM
Prescaler      : 0
Counter Period : 4199
PWM Mode       : PWM Mode 1
Initial Pulse  : 0
```

타이머 클럭이 84 MHz이므로 PWM 주파수는 20 kHz이다.

## 디버그 UART 설정

```text
USART2
TX        : PA2 / D1
RX        : PA3 / D0
Baud Rate : 115200
Data Bits : 8
Parity    : None
Stop Bits : 1
```

## 전원 연결

```text
모터 배터리
    → TB6612FNG VM
    → 좌우 DC 기어 모터

NUCLEO 3.3V
    → TB6612FNG VCC
    → 3.3V 입력을 지원하는 센서 모듈 VCC

STM32 GND
    = TB6612FNG GND
    = 엔코더 GND
    = MPU6050 GND
    = VL53L0X GND
    = 모터 배터리 (-)
```

## 주의 사항

- 모터 전원을 NUCLEO 보드의 3.3V 또는 5V 핀에서 공급하지 않는다.
- TB6612FNG의 `VCC`에는 3.3V, `VM`에는 모터용 배터리를 연결한다.
- 모든 장치의 GND를 공통으로 연결한다.
- HC-020K OUT 전압을 측정한 뒤 STM32 핀에 연결한다. 특히 PA0에 5V 신호를 직접 입력하지 않는다.
- 센서 모듈의 SDA, SCL, XSHUT 풀업 전압이 3.3V인지 확인한다.
- XSHUT을 Open-Drain으로 사용하므로 센서 모듈에 적절한 풀업이 있는지 확인한다.
- 배선을 변경할 때는 전원을 먼저 끈다.
- 센서를 처음부터 모두 연결하지 않고 모터, 엔코더, MPU6050, VL53L0X 순서로 하나씩 확인한다.
- CubeMX 핀을 변경하면 이 문서도 함께 수정한다.
