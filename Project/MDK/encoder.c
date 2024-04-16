#include "encoder.h"

//定义脉冲引脚
#define SPEEDL_PLUSE   CTIM0_P34
#define SPEEDR_PLUSE   CTIM3_P04
//定义方向引脚
#define SPEEDL_DIR     P35
#define SPEEDR_DIR     P53

int16 templ_pluse = 0;
int16 tempr_pluse = 0;

int16 ReadEncoderL()
{
    WTST = 0;               //设置程序代码等待参数，赋值为0可将CPU执行程序的速度设置为最快
    
	DisableGlobalIRQ();		//关闭总中断确保不会被其他中断干扰打断，提高计数的准确性，但是其他的中断任务也会被关闭
	
    //sys_clk可选值:35000000，30000000, 27000000. 24000000, 22118400, 20000000, 18432000, 12000000, 11059200, 6000000, 5529600。
    //设置系统频率，此频率需要跟STC-ISP软件中的 <输入用户程序运行的IRC频率>选项的频率一致。
    //如果频率设置不对，将会导致串口的数据不正常,PWM的工作不正常等等。
    sys_clk = 35000000;     //设置系统频率为35MHz
	ctimer_count_init(SPEEDL_PLUSE);	//初始化定时器0作为外部计数

	EnableGlobalIRQ();					//开启总中断
    //读取采集到的编码器脉冲数
    templ_pluse = ctimer_count_read(SPEEDL_PLUSE);//读取编码器值


    //计数器清零
    ctimer_count_clean(SPEEDL_PLUSE);
 

    //采集方向信息
    if(1 == SPEEDL_DIR)    
    {
        templ_pluse = -templ_pluse;
    }
//    else//左侧电机返回0            
//    {
//        templ_pluse = templ_pluse;
//    }
    //EnableGlobalIRQ();					//开启总中断
    return templ_pluse;
}
int16 ReadEncoderR()
{
    WTST = 0;               //设置程序代码等待参数，赋值为0可将CPU执行程序的速度设置为最快
    
	DisableGlobalIRQ();		
    sys_clk = 35000000;     //设置系统频率为35MHz
	ctimer_count_init(SPEEDR_PLUSE);	//初始化定时器0作为外部计数
	//EnableGlobalIRQ();					//开启总中断
    //读取采集到的编码器脉冲数
    tempr_pluse = ctimer_count_read(SPEEDR_PLUSE);//读取编码器值
    //计数器清零
    ctimer_count_clean(SPEEDR_PLUSE);
 

    //采集方向信息
    if(1 == SPEEDR_DIR)    
    {
        tempr_pluse = -tempr_pluse;
    }
//    else//右侧电机返回0            
//    {
//        tempr_pluse = tempr_pluse;
//    }
    EnableGlobalIRQ();					//开启总中断
    return tempr_pluse;
}
void EncoderInitL()
{
    ctimer_count_init(SPEEDL_PLUSE);
    SPEEDL_DIR = 1;
}
void EncoderInitR()
{
    ctimer_count_init(SPEEDR_PLUSE);
    SPEEDR_DIR = 1;
}