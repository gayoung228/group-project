# TERRA 자율주행 로버

STM32F411RE 기반 2륜 로버 프로젝트입니다. 엔코더로 좌우 바퀴 속도를 측정하고,
MPU6050으로 진행 방향을 보정하며, VL53L0X 3개와 IR 리모컨으로 장애물 회피와 원격
명령을 수행합니다.

## 현재 동작

- IR 리모컨으로 주행 시작·정지, 속도 변경, 기준 방향 재설정
- 자이로와 엔코더를 함께 사용한 직진 보정
- 좌·정면·우 거리로 넓은 방향을 선택하고 Yaw/RPM 폐루프로 부드럽게 벽 회피
- 과속방지턱의 오르막·내리막 및 자세 이탈 대응
- 벽 회피와 낮은 방지턱 통과를 순서·횟수 제한 없이 독립적으로 반복
- 정지한 바퀴를 감지하면 한 번 재시동하고, 실패하면 안전 정지
- UART `c` 명령으로 좌·우 모터의 공중 PWM-RPM 특성 자동 측정
- 주행 불능 오류 시 긴급정지·하드웨어 재시동·Z/START 재출발 안내
- 센서 초기화 결과와 주행 상태를 USART2 로그로 출력

## 시연 영상

### PID 기반 직진 보정
엔코더로 측정한 좌우 바퀴 속도와 MPU6050의 방향 정보를 이용해
직진 주행을 보정하는 영상입니다.
<img width="400" height="400" alt="PID_OFF_ON" src="https://github.com/user-attachments/assets/76a44b00-babc-426e-9e57-252335361766" />

### 장애물 판단 및 회피
좌·정면·우 거리센서로 장애물을 감지하고,
더 넓은 방향을 선택해 회피하는 영상입니다.
<img width="400" height="400" alt="장애물 판단" src="https://github.com/user-attachments/assets/70ce4938-a274-4442-9284-2226801bbdbe" />

### 과속방지턱 통과
과속방지턱의 오르막과 내리막을 감지하고,
상태에 따라 주행을 제어하는 영상입니다.
<img width="400" height="439" alt="방지턱" src="https://github.com/user-attachments/assets/3bbfaba0-c09e-4694-9a69-8f9ec7e2008b" />

### 전체 시연
<img width="400" height="225" alt="전체" src="https://github.com/user-attachments/assets/f70b4072-c0fa-42ff-9821-10619242b45c" />


## 코드 구조

```text
Core/                 CubeMX가 생성하는 MCU 초기화·인터럽트 코드
app/
  ap_main.c           실행할 앱을 고르는 가장 얇은 진입점
  rover_app.c         리모컨·센서·제어 모듈을 연결하는 로버 앱
  tests/              센서별 단독 시험 앱
hw/
  driver/             모터, 엔코더, IMU, 거리센서 등 하드웨어 드라이버
  control/            방향, 장애물, 방지턱, 임무 상태 제어
  rover_config.h      바퀴 치수, RPM, 회피 거리·각도 등 공통 튜닝 설정
docs/                 핀맵, 배선, 인터페이스, 시험 절차
```

전체 흐름은 다음과 같습니다.

```text
IR·거리·IMU 입력
       ↓
rover_app ──→ mission_control(현재 주행 상태)
       ↓
proximity → obstacle / heading / speed_bump 제어
       ↓
drive(최종 모터 명령의 단일 통로)
       ↓
wheel → motor              encoder ──┘ 피드백
```

## 빌드

터미널에서 STM32 GCC와 Ninja가 PATH에 잡혀 있다면 다음 명령으로 기본 로버
펌웨어를 빌드합니다.

```bash
cmake --preset Debug -DROVER_FIRMWARE_MODE=ROVER
cmake --build --preset Debug
```

센서 하나만 시험할 때는 모드를 바꿉니다.

```bash
cmake --preset Debug -DROVER_FIRMWARE_MODE=HEADING_TEST
cmake --preset Debug -DROVER_FIRMWARE_MODE=IR_TEST
cmake --preset Debug -DROVER_FIRMWARE_MODE=VL53_TEST
```

모드를 고른 뒤 `cmake --build --preset Debug`를 실행합니다. 단독 시험이 끝나면
반드시 `ROVER` 모드로 되돌린 뒤 실제 로버 펌웨어를 빌드합니다.

## 문서

- [핀맵](docs/pinmap.md)
- [배선 주의사항](docs/wiring.md)
- [현재 구현 기능](docs/features.md)
- [주요 모듈 인터페이스](docs/interface.md)
- [시험 절차](docs/test-plan.md)
- [3센서 부드러운 장애물 회피 설계](docs/smooth-obstacle-avoidance.md)

모터를 처음 시험할 때는 바퀴를 바닥에서 띄우고, 비상 정지 수단을 준비합니다.
