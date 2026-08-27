#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum{
    ENCODER_LEFT,     // 왼쪽 바퀴 엔코더
    ENCODER_RIGHT     // 오른쪽 바퀴 엔코더
} encoder_id_t;

void encoder_init(void);  // 좌우 엔코더 타이머 또는 인터럽트를 초기화
   
void encoder_reset(void);  // 좌우 엔코더 카운트와 RPM 계산값을 초기화

void encoder_update(uint32_t elapsed_time_ms);  // 측정 시간 동안의 펄스 변화량을 이용해 RPM을 계산

int32_t encoder_get_count(encoder_id_t encoder);  // 선택한 엔코더의 부호 있는 누적 펄스 수를 반환

int32_t encoder_get_delta_count(encoder_id_t encoder);  // 마지막 측정 구간에서 발생한 펄스 변화량을 반환

float encoder_get_rpm(encoder_id_t encoder);  // 선택한 바퀴의 현재 회전 속도를 RPM으로 반환

bool encoder_is_running(encoder_id_t encoder);  // 선택한 바퀴에서 최근 펄스가 발생했는지 반환

void encoder_on_pulse(encoder_id_t encoder);  // 외부 인터럽트 방식에서 엔코더 펄스 발생을 전달

#endif