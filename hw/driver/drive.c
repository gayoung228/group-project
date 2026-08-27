#include "drive.h"
#include "motor.h"

static int16_t drive_read_motor_speed(motor_t motor) {
    return (uint16_t)motor_get_speed(motor);
}

/* 주행 모듈 초기화 */
void drive_init(void) { 
    motor_init(); 
}

/* 좌우 바퀴를 같은 방향으로 돌려 전진한다. */
void drive_forward(uint8_t speed) {
    motor_control(MOTOR_LEFT, MOTOR_FORWARD, speed);
    motor_control(MOTOR_RIGHT, MOTOR_FORWARD, speed);
}

/* 좌우 바퀴를 같은 방향으로 돌려 후진한다. */
void drive_backward(uint8_t speed) {
    motor_control(MOTOR_LEFT, MOTOR_REVERSE, speed);
    motor_control(MOTOR_RIGHT, MOTOR_REVERSE, speed);
}

/* 왼쪽 바퀴는 뒤로, 오른쪽 바퀴는 앞으로 돌려 제자리에서 좌회전한다. */
void drive_turn_left(uint8_t speed) {
    motor_control(MOTOR_LEFT, MOTOR_REVERSE, speed);
    motor_control(MOTOR_RIGHT, MOTOR_FORWARD, speed);
}

/* 왼쪽 바퀴는 앞으로, 오른쪽 바퀴는 뒤로 돌려 제자리에서 우회전한다. */
void drive_turn_right(uint8_t speed) {
    motor_control(MOTOR_LEFT, MOTOR_FORWARD, speed);
    motor_control(MOTOR_RIGHT, MOTOR_REVERSE, speed);
}

/* 좌우 바퀴를 모두 멈춘다. */
void drive_stop(void) { 
    motor_stop_all(); 
}

/* 현재 왼쪽 바퀴에 적용된 속도를 반환 (전진 +, 후진 -, 정지 0) */
int16_t drive_get_left_speed(void) {
    return drive_read_motor_speed(MOTOR_LEFT);
}

/* 현재 오른쪽 바퀴에 적용된 속도를 반환 (전진 +, 후진 -, 정지 0) */
int16_t drive_get_right_speed(void) {
    return drive_read_motor_speed(MOTOR_RIGHT);
}

/* 현재 설정된 기본 주행 속도를 반환 */
uint8_t drive_get_speed(void) {
    return (drive_get_left_speed() + drive_get_right_speed()) / 2;
}