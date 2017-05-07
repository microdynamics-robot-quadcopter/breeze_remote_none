#ifndef __STM32F10X_ALGORITHM_CONTROL_H__
#define __STM32F10X_ALGORITHM_CONTROL_H__

#include "stm32f10x.h"

extern int Throttle;
extern int Roll;
extern int Pitch;
extern int Yaw;
extern int8_t ClibraFlag;

void LoadRCdata(void);
void controlClibra(void);
void RockerUnlockcrazepony(void);
void KeyLockcrazepony(void);
void IMUcalibrate(void);
void Remotecalibrate(void);

#endif
