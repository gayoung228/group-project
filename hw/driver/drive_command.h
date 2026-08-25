#ifndef DRIVE_COMMAND_H
#define DRIVE_COMMAND_H

typedef enum{ 
    DRIVE_CMD_NONE = 0, // 새로운 입력이 없는 상태
    DRIVE_CMD_FORWARD,
    DRIVE_CMD_BACKWARD,
    DRIVE_CMD_LEFT,
    DRIVE_CMD_RIGHT,
    DRIVE_CMD_STOP, 
    DRIVE_CMD_SPEED_UP,
    DRIVE_CMD_SPEED_DOWN
} drive_command_t;

#endif