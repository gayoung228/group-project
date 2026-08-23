#ifndef CAR_CONTROL_H
#define CAR_CONTROL_H

#include "drive_command.h"

void carControlInit(void);
void carControlUpdate(void);
void carControlExecute(const DriveCommand *command);

#endif 