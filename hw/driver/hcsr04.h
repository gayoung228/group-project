#ifndef HCSR04_H
#define HCSR04_H

#include <stdint.h>

#define HCSR04_DISTANCE_ERROR UINT32_MAX

void hcsr04_init(void);

uint32_t hcsr04_get_distance(void);

#endif 