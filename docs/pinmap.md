ctrl + shift + v 하시면 잘보입니다.
# 하드웨어 리스트

## 1차 구현: IR 리모컨 조종 미니카

### 차체 및 구동부

- [x] 2WD 차체 섀시
- [x] 구동 바퀴 × 2
- [x] 보조 바퀴(캐스터 휠) × 1
- [x] DC 기어모터 × 2
- [x] 전원 스위치
- [x] 모터용 배터리
- [x] 배터리 홀더

### 제어 및 전원

- [x] STM32 Nucleo-F411RE 보드
- [x] TB6612FNG 듀얼 DC 모터 드라이버
- [x] 18650 배터리 충전·승압 모듈
  - 입력: USB Type-C DC 5V
  - 출력: DC 5V / 9V / 12V
  - STM32 전원에는 5V 출력 사용
- [x] 18650 배터리 셀

### 조종 및 회로 구성

- [x] IR 수신 센서
- [x] IR 리모컨
- [x] 300홀 브레드보드
- [x] 점퍼선
- [x] 나사, 너트, 양면테이프, 케이블 타이 등 조립 재료

## 2차 확장 기능

### 폐루프 속도 제어

- [x] HC-020K 광학식 엔코더 센서 × 2
- [x] 엔코더 슬롯 디스크 × 2
- [ ] 엔코더 출력 전압 확인 및 필요 시 레벨 시프터 또는 저항 분압 회로

### 장애물 감지

- [x] HC-SR04 초음파 센서
- [ ] Echo 출력용 저항 분압 회로

## 전원 역할

```text
AA 배터리
→ TB6612FNG VM
→ 좌·우 DC 기어모터

18650 배터리 + 충전·승압 모듈 5V 출력
→ STM32 Nucleo-F411RE 전원

STM32 GND = TB6612FNG GND = 모터 배터리 (-)
```

## 주의 사항

- TB6612FNG의 `VCC`는 STM32의 3.3V에 연결한다.
- TB6612FNG의 `VM`에는 모터용 배터리를 연결한다.
- 18650 충전·승압 모듈의 9V·12V 출력은 STM32에 연결하지 않는다.
- HC-020K와 HC-SR04의 출력 신호가 5V일 경우 STM32 GPIO에 직접 연결하지 않는다.

# Pin Map

## 1차 구현: IR 리모컨 조종 미니카

| 기능 | Nucleo 핀 | STM32 핀 | CubeMX 설정 | 연결 대상 |
|---|---|---|---|---|
| 좌측 모터 PWM | D6 | PB10 | TIM2_CH3 PWM | TB6612FNG PWMA |
| 우측 모터 PWM | D9 | PC7 | TIM3_CH2 PWM | TB6612FNG PWMB |
| 좌측 모터 방향 1 | D4 | PB5 | GPIO Output | TB6612FNG AIN1 |
| 좌측 모터 방향 2 | D5 | PB4 | GPIO Output | TB6612FNG AIN2 |
| 우측 모터 방향 1 | D7 | PA8 | GPIO Output | TB6612FNG BIN1 |
| 우측 모터 방향 2 | D8 | PA9 | GPIO Output | TB6612FNG BIN2 |
| 모터 드라이버 활성화 | D3 | PB3 | GPIO Output | TB6612FNG STBY |
| IR 리모컨 수신 | D2 | PA10 | TIM1_CH3 Input Capture | IR 수신기 OUT |
| UART TX | D1 | PA2 | USART2_TX | ST-LINK Virtual COM |
| UART RX | D0 | PA3 | USART2_RX | ST-LINK Virtual COM |

## 2차 확장 기능 예약

| 기능 | Nucleo 핀 | STM32 핀 | 예정 설정 | 연결 대상 |
|---|---|---|---|---|
| 좌측 엔코더 펄스 | A0 | PA0 | TIM5_CH1 Input Capture | 좌측 HC-020K OUT |
| 우측 엔코더 펄스 | A1 | PA1 | TIM5_CH2 Input Capture | 우측 HC-020K OUT |
| 초음파 Trigger | A2 | PA4 | GPIO Output | HC-SR04 Trig |
| 초음파 Echo | D10 | PB6 | TIM4_CH1 Input Capture | HC-SR04 Echo |
| I²C Clock | D15 | PB8 | I2C1_SCL | OLED / MPU6050 SCL |
| I²C Data | D14 | PB9 | I2C1_SDA | OLED / MPU6050 SDA |
| SPI MOSI | D11 | PA7 | SPI1 예약 | 향후 무선 모듈 |
| SPI MISO | D12 | PA6 | SPI1 예약 | 향후 무선 모듈 |
| SPI Clock | D13 | PA5 | SPI1 예약 | 향후 무선 모듈 |

## CubeMX 초기 설정

```text
SYS
- Debug: Serial Wire

USART2
- Mode: Asynchronous
- PA2: USART2_TX
- PA3: USART2_RX

TIM2
- Channel 3: PWM Generation
- PB10: MOTOR_L_PWM

TIM3
- Channel 2: PWM Generation
- PC7: MOTOR_R_PWM

GPIO Output
- PB5: MOTOR_L_IN1
- PB4: MOTOR_L_IN2
- PA8: MOTOR_R_IN1
- PA9: MOTOR_R_IN2
- PB3: MOTOR_STBY

TIM1
- Channel 3: Input Capture
- PA10: IR_REMOTE_IN
- TIM1 Capture/Compare Interrupt 활성화
```

## 전원 연결

```text
Nucleo 3.3V → TB6612FNG VCC
Nucleo GND  → TB6612FNG GND

AA 배터리 (+) → TB6612FNG VM
AA 배터리 (-) → TB6612FNG GND

STM32 GND = TB6612FNG GND = 모터 배터리 (-)
```

## 주의 사항

- TB6612FNG의 `VM`에는 모터용 배터리를 연결하고, Nucleo 보드의 5V·3.3V 핀으로 모터를 구동하지 않는다.
- TB6612FNG의 `VCC`는 3.3V에 연결한다.
- HC-SR04의 Echo와 HC-020K의 OUT은 5V 출력형일 수 있으므로, STM32 GPIO 연결 전 3.3V 호환 여부를 확인한다.
- 5V 출력일 경우 저항 분압 또는 레벨 시프터를 사용한다.
- CubeMX의 `.ioc` 파일은 하드웨어·통합 담당자만 수정한다.
- 핀을 변경할 경우 이 문서도 함께 갱신한다.

## 참고

- [NUCLEO-F411RE 제품 페이지](https://www.st.com/en/evaluation-tools/nucleo-f411re.html)
- [STM32 Nucleo-64 보드 사용자 매뉴얼](https://www.st.com/resource/en/user_manual/dm00105823-stm32-nucleo%20-64-boards-mb1136-stmicroelectronics.pdf)
- [STM32F411RE 데이터시트](https://www.st.com/resource/en/datasheet/stm32f411re.pdf)
