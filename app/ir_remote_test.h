#ifndef IR_REMOTE_TEST_H
#define IR_REMOTE_TEST_H

/* IR 리모컨 테스트에 필요한 모듈들을 초기화한다. 프로그램 시작 시 한 번 호출한다. */
void ir_remote_test_init(void);

/* IR 명령을 기다리며 UART로 확인하는 테스트 메인 루프를 실행한다.
 * 이 함수는 반환하지 않는다(heading_drive_test_run()과 동일한 구조). */
void ir_remote_test_run(void);

#endif
