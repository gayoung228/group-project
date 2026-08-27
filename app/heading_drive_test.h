#ifndef HEADING_DRIVE_TEST_H
#define HEADING_DRIVE_TEST_H

/* 자이로 직진 유지 시험에 필요한 드라이버와 상태를 초기화한다. */
void heading_drive_test_init(void);

/* UART 명령과 주기 제어를 처리하는 시험 메인 루프를 실행한다. */
void heading_drive_test_run(void);

#endif
