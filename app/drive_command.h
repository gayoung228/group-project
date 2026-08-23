#ifndef DRIVE_COMMAND_H
#define DRIVE_COMMAND_H

typedef enum
{
    // DRIVE_CMD_NONE: 새 리모컨 입력 없음
    DRIVE_CMD_NONE = 0,
    // DRIVE_CMD_STOP: 정지 버튼을 눌렀음
    DRIVE_CMD_STOP,
    DRIVE_CMD_FORWARD,
    DRIVE_CMD_BACKWARD,
    DRIVE_CMD_LEFT,
    DRIVE_CMD_RIGHT
} drive_command_t;

#endif