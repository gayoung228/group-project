#ifndef ROVER_APP_H
#define ROVER_APP_H

// 자율주행 로버에 필요한 드라이버와 제어 상태를 초기화한다.
void rover_app_init(void);

// UART/IR 명령과 주기 제어를 처리한다. 이 함수는 반환하지 않는다.
void rover_app_run(void);

#endif
