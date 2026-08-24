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
    DRIVE_CMD_RIGHT,
    // DRIVE_CMD_SPEED_UP: 속도 증가 버튼을 눌렀음
    DRIVE_CMD_SPEED_UP,
    // DRIVE_CMD_SPEED_DOWN: 속도 감소 버튼을 눌렀음
    DRIVE_CMD_SPEED_DOWN
} drive_command_t;

#endif