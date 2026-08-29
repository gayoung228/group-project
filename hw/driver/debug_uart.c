#include "main.h"
#include "debug_uart.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

/* ------------------------------------------------------------------
 * debug_uart.c - ST-LINK Virtual COM Port 공용 로그 출력
 *
 * printf의 _write를 한 번의 UART 전송으로 처리한다. 기존처럼 글자마다 HAL을
 * 호출하지 않아 제어 루프가 로그 출력 때문에 오래 지연되는 일을 줄인다.
 * ------------------------------------------------------------------ */

#define DEBUG_UART_TIMEOUT_MS       100U
#define DEBUG_UART_FORMAT_BUFFER    256U

extern UART_HandleTypeDef huart2;

bool debug_uart_init(void)
{
    return (huart2.gState != HAL_UART_STATE_RESET);
}

bool debug_uart_send_data(const uint8_t *data, size_t length)
{
    if ((data == NULL) || (length == 0U) || (length > UINT16_MAX))
    {
        return false;
    }

    return (HAL_UART_Transmit(&huart2,
                              (uint8_t *)data,
                              (uint16_t)length,
                              DEBUG_UART_TIMEOUT_MS) == HAL_OK);
}

bool debug_uart_send(const char *string)
{
    if (string == NULL)
    {
        return false;
    }

    return debug_uart_send_data((const uint8_t *)string, strlen(string));
}

bool debug_uart_send_format(const char *format, ...)
{
    char buffer[DEBUG_UART_FORMAT_BUFFER];
    va_list args;
    int length;

    if (format == NULL)
    {
        return false;
    }

    va_start(args, format);
    length = vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    if (length < 0)
    {
        return false;
    }
    if ((size_t)length >= sizeof(buffer))
    {
        length = (int)sizeof(buffer) - 1;
    }

    return debug_uart_send_data((const uint8_t *)buffer, (size_t)length);
}

void debug_uart_update(void)
{
    /* 현재는 blocking 전송이라 주기적으로 처리할 큐가 없다. */
}

bool debug_uart_is_busy(void)
{
    return (huart2.gState == HAL_UART_STATE_BUSY_TX)
        || (huart2.gState == HAL_UART_STATE_BUSY_TX_RX);
}

void debug_uart_on_tx_complete(void)
{
    /* 향후 DMA/인터럽트 기반 송신으로 바꿀 때 사용할 확장 지점이다. */
}

/* putchar 계열 출력용 단일 문자 경로 */
int __io_putchar(int ch)
{
    uint8_t value = (uint8_t)ch;
    (void)debug_uart_send_data(&value, 1U);
    return ch;
}

/* syscalls.c의 weak _write를 덮어써 printf 문자열을 한 번에 전송한다. */
int _write(int file, char *ptr, int length)
{
    (void)file;

    if ((ptr == NULL) || (length <= 0))
    {
        return 0;
    }

    return debug_uart_send_data((const uint8_t *)ptr, (size_t)length)
         ? length
         : -1;
}
