#ifndef IR_REMOTE_H
#define IR_REMOTE_H

#include <stdbool.h>
#include <stdint.h>

#include "drive_command.h"

void ir_remote_init(void);
void ir_remote_update(void);

// 새 명령이 있으면 true와 함께 전달하고, 없으면 false 반환
bool ir_remote_take_command(drive_command_t *command);

// IR 디코더가 해석한 원시 버튼 코드를 전달 
void ir_remote_on_raw_code(uint32_t raw_code);

#endif