#ifndef __ENCODER_H
#define __ENCODER_H
#include "headfile.h"
#include "common.h"
#include "zf_pwm.h"
#include "zf_tim.h"
#include "board.h"
//������������
//#define SPEEDL_PLUSE   CTIM0_P34
//#define SPEEDR_PLUSE   CTIM3_P04
//���巽������
//#define SPEEDL_DIR     P35
//#define SPEEDR_DIR     P53

int16 ReadEncoderL();
int16 ReadEncoderR();
void EncoderInitL();
void EncoderInitR();

#endif
