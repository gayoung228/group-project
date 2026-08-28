#include "main.h"
#include "vl53l0x_test.h"
#include "vl53l0x.h"
#include <stdio.h>

#ifdef TEST_VL53L0X

/* ============================================================================
 * VL53L0X 단독/통합 테스트 모듈
 *
 * vl53l0x.c의 non-blocking 상태머신(vl53l0x_init_all / vl53l0x_obstacle_update /
 * getter)을 그대로 재사용하며, 이 파일에서는 거리 측정·판정 로직을 다시 만들지
 * 않는다. 여기서 하는 일은 (1) 매 호출마다 상태머신을 한 단계 진행시키는 것과
 * (2) 200ms에 한 번만 그 결과를 USART2로 출력하는 것뿐이다.
 * ============================================================================ */

/* 장애물 판정 기준 [mm] = 20cm */
#define VL53L0X_TEST_OBSTACLE_THRESHOLD_MM  200U

/* USART2 출력 주기 [ms] */
#define VL53L0X_TEST_PRINT_PERIOD_MS        200U

static uint32_t vl53l0x_test_print_tick = 0;


/* ready가 아니거나 마지막 측정이 invalid면 실제 거리값처럼 오해되지 않도록 "--"를 채운다. */
static void vl53l0x_test_format_distance(vl53l0x_id_t sensor, char *buf, size_t buf_size)
{
    if (!vl53l0x_is_ready(sensor) || !vl53l0x_is_valid(sensor))
    {
        snprintf(buf, buf_size, "--");
    }
    else
    {
        snprintf(buf, buf_size, "%u", (unsigned int)vl53l0x_get_distance_mm(sensor));
    }
}

/* ready가 아니면 '-', ready면 valid 여부를 '1'/'0'으로 표시한다. */
static char vl53l0x_test_valid_char(vl53l0x_id_t sensor)
{
    if (!vl53l0x_is_ready(sensor))
    {
        return '-';
    }

    return vl53l0x_is_valid(sensor) ? '1' : '0';
}

/* LEFT/FRONT/RIGHT 거리·valid·obstacle 상태를 한 줄로 출력한다. */
static void vl53l0x_test_print_status(void)
{
    char dist_l[8];
    char dist_f[8];
    char dist_r[8];

    vl53l0x_test_format_distance(VL53L0X_LEFT,  dist_l, sizeof(dist_l));
    vl53l0x_test_format_distance(VL53L0X_FRONT, dist_f, sizeof(dist_f));
    vl53l0x_test_format_distance(VL53L0X_RIGHT, dist_r, sizeof(dist_r));

    printf("VL53  | L=%s F=%s R=%s | VALID(L/F/R)=%c/%c/%c | OBS(L/F/R)=%d/%d/%d\r\n",
           dist_l, dist_f, dist_r,
           vl53l0x_test_valid_char(VL53L0X_LEFT),
           vl53l0x_test_valid_char(VL53L0X_FRONT),
           vl53l0x_test_valid_char(VL53L0X_RIGHT),
           vl53l0x_is_obstacle(VL53L0X_LEFT)  ? 1 : 0,
           vl53l0x_is_obstacle(VL53L0X_FRONT) ? 1 : 0,
           vl53l0x_is_obstacle(VL53L0X_RIGHT) ? 1 : 0);
}

void vl53l0x_test_init(void)
{
    /* VL53L0X_MULTI_SENSOR_ENABLE == 0이면 FRONT만 ready가 되고,
     * 1이면 LEFT/FRONT/RIGHT가 각각 독립 상태머신으로 ready가 된다. */
    vl53l0x_init_all();

    vl53l0x_test_print_tick = HAL_GetTick();
}

void vl53l0x_test_update(void)
{
    /* non-blocking : 상태머신을 한 단계만 진행시키고 바로 반환한다. */
    vl53l0x_obstacle_update(VL53L0X_TEST_OBSTACLE_THRESHOLD_MM);

    if ((HAL_GetTick() - vl53l0x_test_print_tick) >= VL53L0X_TEST_PRINT_PERIOD_MS)
    {
        vl53l0x_test_print_tick += VL53L0X_TEST_PRINT_PERIOD_MS;
        vl53l0x_test_print_status();
    }
}

/* VL53L0X 테스트를 단독으로 실행하는 대표 진입점. vl53l0x_test_update()를
 * while(1)에서 반복 호출하기만 한다 - 이 함수는 반환하지 않는다.
 *
 * 초기화는 하지 않는다. 반드시 이 함수를 부르기 전에 vl53l0x_test_init()을
 * 먼저 호출해야 한다(예: ap_init()에서 vl53l0x_test_init(), ap_main()에서
 * vl53l0x_test_run()). heading_drive_test.c 등 다른 상위 루프에 얹어 쓸 때는
 * 이 함수를 부르지 말고 vl53l0x_test_init()/vl53l0x_test_update()를 각자의
 * init/루프 안에서 직접 호출할 것. */
void vl53l0x_test_run(void)
{
    for (;;)
    {
        vl53l0x_test_update();
    }
}

#endif /* TEST_VL53L0X */
