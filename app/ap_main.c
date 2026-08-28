#include "ap_main.h"
// #include "obstacle_avoidance_test.h"
#include "ir_remote_test.h"

/* ------------------------------------------------------------------
 * ap_main.c - 애플리케이션 진입점
 *
 * Core/Src/main.c는 이 파일의 ap_init(), ap_main()만 호출한다.
 * 실제 시험 기능은 시험 모듈 하나에 맡긴다.
 *
 * 이렇게 진입점을 작게 유지하면 시험 기능이 바뀌더라도
 * CubeMX가 생성한 main.c와 다른 센서 드라이버를 건드릴 필요가 없다.
 *
 * 지금은 IR 리모컨 단독 테스트를 위해 ir_remote_test 모듈을 사용한다
 * (임시). 원래대로 되돌리려면 include와 아래 두 호출만
 * obstacle_avoidance_test_init()/obstacle_avoidance_test_run()으로
 * 바꾸면 된다(obstacle_avoidance_test.c는 손대지 않아도 됨,
 * TEST_OBSTACLE_AVOIDANCE로 계속 보존됨).
 * ------------------------------------------------------------------ */

/* 프로그램 시작 시 한 번 호출되는 애플리케이션 초기화 함수 */
void ap_init(void)
{
    // obstacle_avoidance_test_init();
    ir_remote_test_init();
}

/* 초기화 후 진입하는 애플리케이션 실행 함수 */
void ap_main(void)
{
    // obstacle_avoidance_test_run();
    ir_remote_test_run();
}
