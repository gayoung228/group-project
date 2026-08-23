#ifndef ENCODER_H
#define ENCODER_H

#include <stdint.h>

#define ENCODER_LEFT  0
#define ENCODER_RIGHT 1

void encoder_init(void);

int32_t encoder_get_count(uint8_t wheel);

void encoder_reset(uint8_t wheel);

void encoder_reset_all(void);

#endif /* ENCODER_H */