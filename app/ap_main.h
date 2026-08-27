#ifndef AP_MAIN_H
#define AP_MAIN_H

/* 모든 애플리케이션 모듈을 초기화한다. 프로그램 시작 시 한 번 호출한다. */
void ap_init(void);

/* 차량 애플리케이션의 메인 루프를 실행한다. */
void ap_main(void);

#endif
