#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h>

#define MOTOR_LEFT     0
#define MOTOR_RIGHT    1

void motor_init(void);

void motor_set_speed(uint8_t motor, int16_t speed);

void motor_stop(uint8_t motor);

void motor_stop_all(void);

#endif