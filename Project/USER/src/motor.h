#ifndef __MOTOR_H
#define __MOTOR_H
#include "headfile.h"
#include "common.h"
#include "zf_pwm.h"


void ServoInit(void);
void MotorInit(void);
void MotorControlL(uint16 duty);
void MotorControlR(uint16 duty);
void ServoControl(int32 duty);


#endif
