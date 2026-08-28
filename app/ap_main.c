#include "ap_main.h"
#include "obstacle_avoidance_test.h"

/* ------------------------------------------------------------------
 * ap_main.c - 애플리케이션 진입점
 *
 * Core/Src/main.c는 이 파일의 ap_init(), ap_main()만 호출한다.
 * 실제 주행 기능은 obstacle_avoidance_test 모듈에 맡긴다.
 *
 * 이렇게 진입점을 작게 유지하면 시험 기능이 바뀌더라도
 * CubeMX가 생성한 main.c와 다른 센서 드라이버를 건드릴 필요가 없다.
 * 리모컨 입력도 이 주행 모듈 안에서 UART 명령과 함께 처리한다.
 * ------------------------------------------------------------------ */

/* 프로그램 시작 시 한 번 호출되는 애플리케이션 초기화 함수 */
void ap_init(void)
{
    obstacle_avoidance_test_init();
}

/* 초기화 후 진입하는 애플리케이션 실행 함수 */
void ap_main(void)
{
    obstacle_avoidance_test_run();
}
