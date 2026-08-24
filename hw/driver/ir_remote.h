#ifndef IR_REMOTE_H
#define IR_REMOTE_H

#include <stdint.h>

#include "drive_command.h"

#ifdef __cplusplus
extern "C" {
#endif

// docs/interface.md가 명시한 반환형 이름(RemoteCommand)을 그대로 노출하되,
// car_control 등 다른 모듈과 공유하는 drive_command_t와 완전히 호환되도록 typedef만 추가한다.
typedef drive_command_t RemoteCommand;

void ir_remote_init(void);

// 주기적 신호 탐색. 새로 확정된 명령을 반환하고 소비 (없으면 DRIVE_CMD_NONE 반환)
RemoteCommand ir_remote_update(void);

// 보관된 명령을 반환하고 소비한 상태로 되돌린다. 없으면 DRIVE_CMD_NONE 반환
RemoteCommand ir_remote_take_command(void);

// IR 디코더가 해석한 원시 버튼 코드를 전달 및 검증
void ir_remote_on_raw_code(uint32_t raw_code);

#ifdef __cplusplus
}
#endif

#endif
