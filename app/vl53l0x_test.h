#ifndef VL53L0X_TEST_H
#define VL53L0X_TEST_H

/* VL53L0X 테스트를 단독으로 실행하는 대표 진입점. vl53l0x_test_update()를
 * while(1)에서 반복 호출해 LEFT/FRONT/RIGHT 거리·valid·obstacle 상태를
 * USART2로 계속 출력한다. 이 함수는 반환하지 않는다
 * (heading_drive_test 등 다른 상위 루프와 같이 쓰지 않는, 완전히 독립된 진입점).
 *
 * 이 함수는 초기화를 하지 않는다. 반드시 vl53l0x_test_init()을 먼저 호출한
 * 뒤에 이 함수를 호출해야 한다(예: ap_init()에서 vl53l0x_test_init(),
 * ap_main()에서 vl53l0x_test_run()). */
void vl53l0x_test_run(void);

/* heading_drive_test 등 다른 상위 실행 루프에 얹어 쓰기 위한 non-blocking 진입점.
 * vl53l0x_init_all()을 한 번 호출해 LEFT/FRONT/RIGHT 센서를 초기화한다.
 * 호출자의 init 함수(예: heading_drive_test_init())에서 한 번만 호출한다. */
void vl53l0x_test_init(void);

/* 호출자의 while(1) 안에서 매 루프마다 한 번씩 호출한다. vl53l0x_obstacle_update()로
 * 상태머신을 한 단계만 진행시키고, 200ms 주기로만 LEFT/FRONT/RIGHT 거리·valid·
 * obstacle 상태를 USART2에 출력한다. vl53l0x_test_run()과 달리 즉시 반환한다
 * (non-blocking) - 호출자의 루프를 막지 않는다. MPU6050과 같은 I2C1 버스를
 * 쓰므로, 반드시 호출자의 foreground 루프에서 순서대로 호출해야 한다
 * (타이머 ISR 등 별도 실행 컨텍스트에서 호출하지 말 것). */
void vl53l0x_test_update(void);

#endif
