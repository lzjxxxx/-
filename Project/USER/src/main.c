/*********************************************************************************************************************
电机初始化
舵机初始化-50Hz
ADC初始化
编码器初始化
差和比控制
PID循迹
定时器初始化
目前还差：滤波、环岛、陀螺仪与TOF应用
环岛：入环：水平电感瞬间下降，横向电感加倍，
主要条件：左右垂直电感出现显著差异，中间电感上升
通过一个阈值判断，入环仅用ML，MR电感循迹
 ********************************************************************************************************************/

#include "headfile.h"

void showoled();
uint8 motor_flag = 0;
uint8 servo_flag = 0;
uint8 tof_flag = 0;
uint8 imu_flag = 0;
uint8 circle_return = 0;
uint8 oled_flag = 0;
//特殊标志
uint8 cross_flag = 0;//十字
uint8 straight_flag = 0;//直行
uint8 circle_flag = 1;//圆环
int16 circle_z = 0;//z轴积分
uint8 cnt_circl = 0;
uint8 circle_finish = 0;
extern int EncodeNowL;
extern int EncodeNowR;
extern uint16 motor_duty_L;
extern uint16 motor_duty_R;
extern void PID_motor(uint16 speed_set, float speed_now_L, float speed_now_R);
extern float motor_kp_L;
extern float motor_kp_R;
extern float SpeedNowR;
extern float SpeedNowL;
extern uint16 time0_t;
extern int servo_duty_delta;
extern int16 diangan[4];
extern float  servo_kp;
extern float  servo_kd;
extern float AdcNow;
extern uint16 TofNow;
extern int32 Duty;
extern int16 error;
extern uint8 ramp_flag;
extern float  servo_error[2];
extern float direction();
extern float L,R,LM,RM;
void main()
{
    
    
	board_init();	    // 初始化寄存器,勿删除此句代码。相当于龙邱sysint()
    delay_ms(200);
//    while(imu660ra_init())				//六轴陀螺仪初始化不能放到函数调用
//	{
//        P52 = !P52;
//		delay_ms(50);
//	}
//    while(dl1a_init())				//tof
//	{
//        P52 = !P52;
//		delay_ms(50);
//	}
    ServoInit();
    ips200_init();//显示屏
    ctimer_count_init(CTIM0_P34);           //初始化计时器映射到P3.4
    adc_init(ADC_P10, ADC_SYSclk_DIV_2);	//初始化ADC,P1.0通道 ，ADC时钟频率：SYSclk/2      
    ctimer_count_init(TIM_0);//编码器计数
    ctimer_count_init(TIM_3); //编码器计数
    pit_timer_ms(TIM_2,1);				    //使用TIMER2作为周期中断
    MotorInit();//电机
    AdcInit();//ADC
    EncoderInitL();//编码器
    EncoderInitR();//编码器
    EnableGlobalIRQ();	//开启总中断
    delay_ms(200);
    while(1)
	{
        //MotorControlL(725);
        
        //ServoControl(6000);
//        static int i = 10 ;
//        ServoControl(i);
//        if(P73 == 0)
//        {
//            i+=1;
//        }
//        if(P72 == 0)
//        {
//            i-=1;
//          
//        }
//        ips200_showint16(6*15, 8, i);
   // MotorControlR(1200);
        P77 = 0;
        P52 = !P52;
        P64 = 1;//正转右
        if(motor_flag==1)
        {
            motor_flag = 0;
            EncodeNowL = ReadEncoderL();
            EncodeNowR = ReadEncoderR();
            SpeedComputerL(EncodeNowL, 10);
            SpeedComputerR(EncodeNowR, 10);
            PID_motor(1000,-SpeedNowL,SpeedNowR);
        }
        if(servo_flag==1)
        {    
            servo_flag = 0;
            //PidControlServo();
            if(cross_judge())
            {    
                ServoControl(0);
                cross_control();
                delay_ms(20);
                
            }
            //修改思路：入环判定条件较为简单，第一次判定成功之后编码器积分，然后第二次判断
            //到之后打角，然后入环用中间两电感循迹，然后出环之后再两次判断到都会直行
            else if(circle_judge()&&(circle_flag)&&(!circle_finish))
            {
                circle_return = 1;//开始入环
            }
            else if(!circle_judge()||!circle_flag)//圆环
            {
                PidControlServo();
                
            }
            switch(circle_return)//circle_return 任何时候都会是有效数值
            {
                case 1:
                    imu_flag = 1;
                    ServoControl(70);//打死
                    PID_motor(600,-SpeedNowL,SpeedNowR);
                    circle_return = 3;
                    delay_ms(800);
                    
                    if(imu_flag)
                    {
                        imu660ra_get_gyro();
                        circle_z += (int16)(-imu660ra_gyro_z);//累加积分
                        if (circle_z >= 200)
                        {
                            circle_z = 200;
                        }
                        if (fabs(circle_z) >= 200)
                        {
                            circle_flag = 0;
                            circle_finish = 1;
                            
                            cnt_circl += 1;

                            if(cnt_circl == 2)
                            {
                                circle_return = 2;//二次检测到入环区
                            }
                        }
                    }
                    break;
                case 2://出环
                    //PidControlServo();
                    ServoControl(0);
                    PID_motor(600,-SpeedNowL,SpeedNowR);
                    cross_control();
                    delay_ms(100);  
                    break;
                case 3:
                    circle_control();
                default:
                    PidControlServo();
            }
            
        }      
        if (P71 == 0)
        {
            showoled();
        }
        if(tof_flag == 1)
        {
            dl1a_get_distance();//获取距离
            tof_get(dl1a_distance_mm);
            if(ramp_judge(TofNow))
            {
                PID_motor(1000,-SpeedNowL,SpeedNowR);
                ramp_control();
            }
            
        }
    }
}

void showoled()
{
        ips200_showstr(0, 0, "ADC1");
		ips200_showstr(0, 1, "ADC2");
        ips200_showstr(0, 2, "ADC3");
        ips200_showstr(0, 3, "ADC4");
        ips200_showstr(0, 4, "circle_z");
        ips200_showstr(0, 5, "kp");
        ips200_showstr(0, 6, "tof");
        ips200_showstr(0, 7, "acc_z");
        ips200_showstr(0, 8, "circle_fl");
        ips200_showstr(0, 9, "circlenow");
        ips200_showstr(0, 10, "error");
        ips200_showfloat(6*15, 0, L,4,4);
        ips200_showfloat(6*15, 1, LM,4,4);
        ips200_showfloat(6*15, 2, RM,4,4);
        ips200_showfloat(6*15, 3, R,4,4);
//        ips200_showfloat(6*15, 0, diangan[0],4,4);
//        ips200_showfloat(6*15, 1, diangan[1],4,4);
//        ips200_showfloat(6*15, 2, diangan[2],4,4);
//        ips200_showfloat(6*15, 3, diangan[3],4,4);
        ips200_showint16(6*15, 4,  circle_z);
        ips200_showint16(6*15, 5,  servo_kp);
        ips200_showint16(6*15, 6,  TofNow);
        ips200_showint16(6*15, 7,  imu660ra_acc_z);
        ips200_showint16(6*15, 8,  circle_judge());
        ips200_showuint8(6*15, 9,  circle_return);
        ips200_showfloat(6*15, 10, direction(),10,5);
//            ips200_showint16(6*15, 8,  SpeedNowL);
//            ips200_showuint16(6*15, 9, SpeedNowR);

}
