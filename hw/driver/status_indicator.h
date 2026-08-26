#ifndef STATUS_INDICATOR_H
#define STATUS_INDICATOR_H

typedef enum{
    STATUS_INDICATOR_OFF,        // 모든 LED와 부저를 끈 상태
    STATUS_INDICATOR_READY,      // 출발 대기 상태
    STATUS_INDICATOR_NORMAL,     // 정상 노면 주행 상태
    STATUS_INDICATOR_CAUTION,    // 작은 진동 감지 및 감속 상태
    STATUS_INDICATOR_DANGER,     // 큰 진동 또는 위험 상태
    STATUS_INDICATOR_COMPLETE,   // 목표 거리 주행 완료 상태
    STATUS_INDICATOR_ERROR       // 센서 또는 주행 오류 상태
} status_indicator_state_t;

void status_indicator_init(void);  // 상태 LED와 부저 출력 핀을 초기화

// 시스템 상태에 맞는 LED와 부저 출력 상태를 설정
void status_indicator_set_state(status_indicator_state_t state);    

void status_indicator_update(void);  // 점멸 주기와 부저 패턴을 갱신

void status_indicator_off(void);  // 모든 LED와 부저를 즉시 끔

status_indicator_state_t status_indicator_get_state(void);  // 현재 표시 중인 상태를 반환

#endif