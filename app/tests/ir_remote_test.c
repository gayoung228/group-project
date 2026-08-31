#include "main.h"
#include "ir_remote_test.h"
#include "ir_remote.h"
#include "drive.h"
#include <stdio.h>

#ifdef TEST_IR_REMOTE

/* ------------------------------------------------------------------
 * app/tests/ir_remote_test.c - NEC IR 리모컨 단독 테스트
 *
 * hw/driver/ir_remote.c가 이미 해석해 둔 Addr/Cmd를 UART로 출력하고,
 * 실측한 "시작/정지" 버튼(Addr=0x00 Cmd=0xC2)이 새 명령으로
 * 확인되면 drive_forward()를 한 번만 호출해 실제로 전진하는지 확인한다.
 *
 * 이 파일은 NEC 디코딩이나 주행 로직을 새로 만들지 않는다 - 전부 기존
 * hw/driver/ir_remote.c, hw/driver/drive.c의 공개 API만 사용한다.
 *
 * 터미널 설정: 115200 baud, 8 data bits, no parity, 1 stop bit
 * ------------------------------------------------------------------ */

/* drive_update() 호출 주기 [ms] (heading_drive_test.c와 동일한 관례) */
#define IR_TEST_CONTROL_PERIOD_MS   20U

/* 현재 NEC 디코더에서 실측한 "시작/정지" 버튼 값 */
#define IR_TEST_START_ADDR   0x00U
#define IR_TEST_START_CMD    0xC2U

/* IR 리모컨 테스트에 필요한 모듈들을 초기화한다. */
void ir_remote_test_init(void)
{
    drive_init();
    ir_remote_init();

    printf("\r\n=========================================\r\n");
    printf(" IR Remote Test (start button -> forward)\r\n");
    printf("=========================================\r\n");
    printf(" start button (addr=0x00 cmd=0xC2)\r\n");
    printf("-----------------------------------------\r\n");
}

/* IR 명령을 기다리며 UART로 확인하는 테스트 메인 루프를 실행한다. */
void ir_remote_test_run(void)
{
    uint32_t control_tick = HAL_GetTick();
    uint32_t now;
    uint32_t elapsed;

    while (1)
    {
        uint8_t addr;
        uint8_t cmd;

        /* non-blocking: 링버퍼에 쌓인 캡처 간격을 NEC 프레임으로 해석한다 */
        ir_remote_update();

        if (ir_remote_get_command(&addr, &cmd))
        {
            printf("[IR] addr=0x%02X cmd=0x%02X\r\n", addr, cmd);

            if ((addr == IR_TEST_START_ADDR) && (cmd == IR_TEST_START_CMD))
            {
                printf(">>> START button confirmed - drive_forward()\r\n");
                drive_forward(DRIVE_SPEED_NORMAL);
            }
        }

        now = HAL_GetTick();
        if ((now - control_tick) >= IR_TEST_CONTROL_PERIOD_MS)
        {
            elapsed      = now - control_tick;
            control_tick = now;
            drive_update(elapsed);
        }
    }
}

#endif /* TEST_IR_REMOTE */
