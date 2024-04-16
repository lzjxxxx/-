#ifndef __CONTROL_H
#define __CONTROL_H
#include "common.h"
#include "zf_adc.h"
#include "zf_pwm.h"
#include "My_ADC.h"
#include "encoder.h"
#include "motor.h"
int8 fastSqrt(int8 x);
void ServoCorrection(int32 error);

int circle_judge();
int  cross_judge();
uint8 circle_ready();
uint8 ramp_judge(uint16 dl1a_distance_mm);
float direction();
float direction_cir();
float direction_cross();
void PidControlServo();
void circle_control();
void cross_control();
void ramp_control();
void tof_get(uint16 dl1a_distance_mm);
void SpeedComputerL(int32 EncodeNow, uint8 ms);
void SpeedComputerR(int32 EncodeNow, uint8 ms);
void ComputerADC(float error, uint8 isrms);
void PidControlServo();
void PID_motor(uint16 speed_set, float speed_now_L, float speed_now_R);
float fabs(float x);
#endif