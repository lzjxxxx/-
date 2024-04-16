#include "motor.h"
#include "zf_pwm.h"
#define ServoCenter 270

uint32 ServoDuty;
    int32 Duty;
//
//PWM1 P66             PWM2 P62
//DIR1 P64右正转1      DIR2 P60左正转0
//右电机阈值810，低于此阈值不会起转
void MotorInit(void)
{
    pwm_init(PWMA_CH2P_P62, 10000, 0);      //
    pwm_init(PWMA_CH4P_P66, 10000, 0);      // 
}
void ServoInit(void)
{
     pwm_init(PWMB_CH1_P74, 50, 270);      //初始化PWMB  使用引脚P7.4  输出PWM频率10000HZ   占空比为百分之 pwm_duty / PWM_DUTY_MAX * 100
}
void MotorControlR(uint16 duty)
{
    if(duty>=2500)
    {
        duty = 2500;
    }
//    else if(duty<=1000)
//    {
//        duty = 1000;
//    } 
    
     pwm_duty(PWMA_CH4P_P66,duty);
    
}
void MotorControlL(uint16 duty)
{
    if(duty>=2500)
    {
        duty = 2500;
    }
//    else if(duty<=1000)
//    {
//        duty = 1000;
//    } 
   pwm_duty(PWMA_CH2P_P62,duty);
}
void ServoControl(int32 duty)
{

  //限制幅值
    //中值0
    if (duty >= 80)//右转限幅
    {
        duty = 80;
    }      
    else if (duty <= -80)
    {
        duty = -80; 
    }
    Duty = duty+ServoCenter;
    pwm_duty(PWMB_CH1_P74,Duty);//舵机
}