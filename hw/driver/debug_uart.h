#ifndef DEBUG_UART_H
#define DEBUG_UART_H

void debug_uart_init(void);

void debug_uart_send(const char *message);

void debug_uart_printf(const char *format, ...);

#endif 