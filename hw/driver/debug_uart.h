#ifndef DEBUG_UART_H
#define DEBUG_UART_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

bool debug_uart_init(void);  // 디버그 출력용 UART와 송수신 버퍼를 초기화

bool debug_uart_send(const char *string);  // 널 종료 문자열을 UART로 전송

// 지정된 길이의 원시 데이터를 UART로 전송
bool debug_uart_send_data(const uint8_t *data, size_t length);  

// printf 형식으로 문자열을 생성하여 UART로 전송
bool debug_uart_send_format(
    const char *format,
    ...
);  

void debug_uart_update(void);  // 현재 blocking 구현에서는 호환용 빈 함수

bool debug_uart_is_busy(void);  // UART HAL 송신 상태를 반환

void debug_uart_on_tx_complete(void);  // 향후 비동기 전환을 위한 호환 함수

#endif
