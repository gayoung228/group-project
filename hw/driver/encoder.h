#ifndef ENCODER_H
#define ENCODER_H

#include <stdbool.h>
#include <stdint.h>

typedef enum{
    ENCODER_LEFT,     // 왼쪽 바퀴 엔코더
    ENCODER_RIGHT     // 오른쪽 바퀴 엔코더
} encoder_id_t;

bool encoder_init(void);  // 좌우 엔코더 입력 캡처를 시작하고 결과를 반환

// TIM5 입력 캡처를 정지했다가 다시 시작하고 측정 상태를 초기화한다.
bool encoder_restart(void);

bool encoder_is_ready(encoder_id_t encoder);  // 선택한 엔코더 채널 초기화 결과
   
void encoder_reset(void);  // 좌우 엔코더 카운트와 RPM 계산값을 초기화

// 최신 펄스 간격을 RPM으로 변환하고 EMA 필터를 적용
void encoder_update(uint32_t elapsed_time_ms);

int32_t encoder_get_count(encoder_id_t encoder);  // 선택한 엔코더의 부호 있는 누적 펄스 수를 반환

int32_t encoder_get_delta_count(encoder_id_t encoder);  // 마지막 측정 구간에서 발생한 펄스 변화량을 반환

float encoder_get_rpm(encoder_id_t encoder);  // 선택한 바퀴의 필터링된 RPM 크기를 반환

bool encoder_is_running(encoder_id_t encoder);  // 선택한 바퀴에서 최근 펄스가 발생했는지 반환

// 입력 캡처 인터럽트에서 펄스와 TIM5 캡처값(1us 단위)을 전달
void encoder_on_pulse(encoder_id_t encoder, uint32_t capture_tick);

#endif
