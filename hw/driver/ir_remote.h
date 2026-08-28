#ifndef IR_REMOTE_H
#define IR_REMOTE_H

#include "main.h"   /* TIM_HandleTypeDef */
#include <stdbool.h>
#include <stdint.h>

/* NEC IR 수신 상태머신을 초기화하고 TIM4 CH1 입력 캡처 인터럽트를 시작한다. */
bool ir_remote_init(void);

/* 메인 루프에서 매 틱 호출한다. 링버퍼에 쌓인 캡처 간격을 꺼내
 * NEC 프레임(Leader/비트/inverse 검증/Repeat/140ms 창)으로 해석한다.
 * non-blocking - 내부에 대기/지연이 없다. */
void ir_remote_update(void);

/* 이번 호출 이전에 새로 확정된 유효 명령이 있으면 addr/cmd를 채우고 true를
 * 반환한다. 반환 즉시 내부적으로 소비되어, 같은 버튼을 계속 누르고 있어도
 * (NEC Repeat 프레임) 다시 true가 되지 않는다 - 버튼을 뗐다가 다시 눌러
 * 새 32비트 프레임이 와야 다시 true가 된다. */
bool ir_remote_get_command(uint8_t *addr, uint8_t *cmd);

/* hw/driver/encoder.c의 HAL_TIM_IC_CaptureCallback()이 어떤 타이머든 매 호출마다
 * htim을 그대로 넘겨서 부르는 진입점이다. 이 함수 안에서 TIM4가 맞는지
 * 판별하고, 맞으면 HAL_TIM_ReadCapturedValue()로 캡처값을 읽어 내부 링버퍼에
 * 저장한다(TIM4가 아니면 즉시 반환). encoder.c는 이 함수를 통해서만 IR과
 * 연결되고, TIM4/캡처 레지스터 등 IR 관련 지식을 전혀 가질 필요가 없다.
 * ISR에서 호출되므로 짧게 유지한다(printf/모터제어/NEC해석 없음). */
void ir_remote_capture_callback(TIM_HandleTypeDef *htim);

#endif
