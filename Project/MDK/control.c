#include "control.h"
#define kp_motor 100
#define ki_motor 50
#define kd_motor 0.1
#define kp_servo 8
#define ki_servo 4
#define kd_servo 0.1
#define L_Max   3795
#define L_Min   700
#define R_Max   3780
#define R_Min   740
#define LM_Max  3780
#define LM_Min  725
#define RM_Max  3800
#define RM_Min  80
#define circle_duty_flag 1200
#define ramp_threshold 160
float error;
float A,MyB,C,MyP;
float L,R,LM,RM;
extern int16 diangan[4];
float SpeedNowL;
float SpeedNowR;
float AdcNow = 0;
uint16 TofNow = 0;
const int8 sqrtTable[26] = {
    0, 1, 1, 1, 2, 2, 2, 2, 2, 3, 3, 3, 3, 3, 3, 3, 3, 4, 4, 4, 4, 4, 4, 4, 4, 5
};
int8 fastSqrt(int8 x)
{
    if (x<=0)
    {
        x = fabs(x); 
        if ( x >= 25) 
        {
            x = 25;  // ��������Χ�����
        }
        return (-sqrtTable[x]);
    }
    else
    {
        if ( x >= 25) 
        {
            x = 25;  // ��������Χ�����
        }
        return (sqrtTable[x]);
    }
    
    
}
float fabs(float x)
{
    if(x<0)
    {
        x = -x;
    }
    return x;
}
void SpeedComputerL(int32 EncodeNow, uint8 isrms)
{
    uint8 i = 0, j = 0;
    int16 templ = 0.0;
    static uint8 SpCount = 0, k = 0;
    static int16 SpeedArryL[10] = {0.0};
    if (SpCount == isrms)
    {

        SpeedArryL[k++] = (float)(EncodeNow);
        if(k == 10)
        {
            for (i = 10; i >= 1; i--)
            {
                for(j = 0; j< (i-1); j++)
                {
                    if (SpeedArryL[j] > SpeedArryL[j+1])
                    {
                        templ = SpeedArryL[j];
                        SpeedArryL[j] = SpeedArryL[j+1];
                        SpeedArryL[j+1] = templ;
                    
                    }
                }
               
            }
            templ = 0;
            for (i = 2;i < 8; i++)
            {
                templ += SpeedArryL[i];
            }
            templ = templ/6;
            //һ�׵�ͨ�˲�
            /*Y(n) = q*X(n) + (1-q)*Y(n-1)
            q�˲�ϵ��(0-1)   X(n)���β���ֵ   Y(n-1)�ϴ��˲����ֵ
            qԽ�󣬴ﵽĿ��ֵ���ٶ�Խ�죬�������߲�ƽ����qԽС�������Խƽ����������ӦԽ��*/
            SpeedNowL = (float)(((float)0.7 * templ) + (SpeedNowL * (float)0.3));
            k = 0;
        }
            SpCount = 0;
    }
    SpCount++; 
}

void SpeedComputerR(int32 EncodeNow, uint8 isrms)
{
    
    uint8 i = 0, j = 0;
    int16 tempr = 0.0;
    static uint8 SpCount = 0, k = 0;
    static int16 SpeedArryR[10] = {0.0};
    
    if (SpCount == isrms)
    {

        SpeedArryR[k++] = (float)(EncodeNow);
        SpeedArryR[k++] = EncodeNow;
        if(k == 10)
        {
            for (i = 10; i > 1; i--)
            {
                for(j = 0; j< (i-1); j++)
                {
                    if (SpeedArryR[j] > SpeedArryR[j+1])
                    {
                        tempr = SpeedArryR[j];
                        SpeedArryR[j] = SpeedArryR[j+1];
                        SpeedArryR[j+1] = tempr;
                    
                    }
                }
               
            }
            tempr = 0;
            for (i = 2;i < 8; i++)
            {
                tempr += SpeedArryR[i];
            }
            tempr = tempr/6;
            //һ�׵�ͨ�˲�
            /*Y(n) = q*X(n) + (1-q)*Y(n-1)
            q�˲�ϵ��(0-1)   X(n)���β���ֵ   Y(n-1)�ϴ��˲����ֵ
            qԽ�󣬴ﵽĿ��ֵ���ٶ�Խ�죬�������߲�ƽ����qԽС�������Խƽ����������ӦԽ��*/
            SpeedNowR = (float)(((float)0.7 * tempr) + (SpeedNowR * (float)0.3));
            k = 0;
        }
            SpCount = 0;
    }
    SpCount++;
}
float direction()
{
    //����A / B A/BA/B����ǿLR������е����ã�����B / C B/CB/C����߳�����������򡱣���ֵԽ�󣬾������ʱ�������������ݲࡣ
    A = 1;
    MyB = 1;
    C = 1;
    //MyP = 6;
    MyP = 10;
    AdcRead();
    L = (float)diangan[0];
    
    R = (float)diangan[3];
    LM = (float)diangan[1];
    RM = (float)diangan[2];
//    L = ((float)diangan[0]/(float)(L_Max))*100;
//    
//    R = ((float)diangan[3]/(float)(R_Max))*100;
//    LM = ((float)diangan[1]/(float)(LM_Max))*100;
//    RM = ((float)diangan[2]/(float)(RM_Max))*100;
    error = ((A*(L-R)+MyB*(LM-RM))/(A*(L+R)+fabs(C*(LM-RM))))*MyP;//��ͱȼ����ƫ��
//    if (error<0)error = -(error*error);
//    else error = error*error;
    return error;
}
float direction_cir()
{
    A = 1;
    MyB = 1;
    C = 1;
    MyP = 10.0;
    AdcRead();
    LM = (float)diangan[1];
    RM = (float)diangan[2];
//    LM = ((float)diangan[1]/(float)(LM_Max-LM_Min))*100;
//    RM = ((float)diangan[2]/(float)(RM_Max-RM_Min))*100;
    error = (MyB*(LM-RM))/(C*(LM+RM))*MyP;//��ͱȼ����ƫ��
    //error = (L-R)/(L+R)*10.0;//���ֵʼ��С��1
	//�޷�
    return error;
}

float direction_cross()
{
    A = 1;
    MyB = 0.75;
    C = 1;
    MyP = 10.0;
    AdcRead();
    L = (float)diangan[0];
    R = (float)diangan[3];
    error = (L-R)/(L+R)*10.0;//���ֵʼ��С��1
    
    return error;
}
void ComputerADC(float error, uint8 isrms)
{
    uint8 i = 0;
    int16 tempadc = 0;
    static uint8 SpCount = 0, k = 0;
    int16 min = 0;
    int16 max = 0;
    static int16 AdcArry[10] = {0.0};
    if (SpCount == isrms)
    {
        AdcArry[k++] = error;
        if(k == 10)
        {
            for (i = 0; i < 10; i++)
            {
                if(AdcArry[i]>max)
                {
                    max = AdcArry[i];
                }
                if(AdcArry[i]<min)
                {
                    min = AdcArry[i];
                }
                tempadc += AdcArry[i];
               
            }
             tempadc = tempadc - max -min;
             AdcNow = (int16)(tempadc/8);
        }
        SpCount = 0;
    }     
    SpCount++; 
}
//�������
void ServoCorrection(int32 error)
{

    ServoControl(error);
}

float  servo_error[2] = {0};
float  servo_kp;
float  servo_kd;
int16  servo_duty_delta;
void PidControlServo()//����Ŀ��Ƕ���ʵ�ʽǶ�
{
    ComputerADC(direction(), 1);//��������Ϊfloat����
//    servo_kp = 15;
//    servo_kd = 0;
//    if(fabs(AdcNow)<=1.5)
//    {
//       servo_kp = 1;
//       servo_kd = 10;
//    }
    if((fabs(AdcNow)<=3.5))
    {
       servo_kp = 7.5;
       servo_kd = 3;
    }
    else if((fabs(AdcNow)>3.5)&&(fabs(AdcNow)<=5.5))
    {
       servo_kp = 7.5+(fabs(AdcNow)-3.5)*1;//���10
       servo_kd = 4;
    }
    else if((fabs(AdcNow)>5.5)&&(fabs(AdcNow)<=8.5))
    {
       servo_kp = 9.5+(fabs(AdcNow)-5.5)*1;//���12
       servo_kd = 5;
    }
    else if(fabs(AdcNow)>8.5)
    {
       servo_kp = 10;
       servo_kd = 6;
    }
    servo_error[0] = AdcNow;    //��Ⱥʹ��������Ϊ����
    if (servo_error[0]>= 100)
    {
        servo_error[0] = 100;
    
    }
    else if(servo_error[0] <= -100)
    {
    
        servo_error[0] = -100;
    }
    servo_duty_delta = (servo_error[0]*servo_kp + (servo_error[0]-servo_error[1])*servo_kd);    //���λ��ʽPD����
    //servo_duty_delta /= 1.8;    //�ɸ������������ֵ��С��ע����������
    servo_error[1] = servo_error[0];    //����
    ServoControl(servo_duty_delta);    //ִ�ж��PWM���
}
float  speed_error_L[2] = {0};
float  speed_error_R[2] = {0};
float  motor_kp_L = 1.65;
float  motor_kp_R = 1.5;
float  motor_ki_L = 0;
float  motor_ki_R = 0;
uint16  motor_duty_L ;
uint16  motor_duty_R ;
uint16 speed_set_L;
uint16 speed_set_R;
extern uint8 circle_return;
void PID_motor(uint16 speed_set, float speed_now_L, float speed_now_R)
{
//    if( (circle_return == 1)||(circle_return == 3))
//    {
//        speed_set_L = speed_set;    //Ŀ���ٶȣ��漰���ٿ����а���ʽ�޸�
//        speed_set_R = speed_set;
    //}
//    speed_set_L = speed_set;    //Ŀ���ٶȣ��漰���ٿ����а���ʽ�޸�
//        speed_set_R = speed_set;
    speed_set_L = speed_set+((uint16)direction())*10;    //Ŀ���ٶȣ��漰���ٿ����а���ʽ�޸�
    speed_set_R = speed_set-((uint16)direction())*10;
    speed_error_L[0] = (float)(speed_set_L - speed_now_L);    //�ٶ�ƫ�����
    speed_error_R[0] = (float)(speed_set_R - speed_now_R);
    motor_duty_L += (uint16)(motor_kp_L * (speed_error_L[0]-speed_error_L[1]) + motor_ki_L * speed_error_L[0] );    //�������ʽPI���㣬PI�����͹��ˣ���DҲ����
    motor_duty_R += (uint16)(motor_kp_R * (speed_error_R[0]-speed_error_R[1]) + motor_ki_R * speed_error_R[0] );
    speed_error_L[1] = speed_error_L[0];    //����
    speed_error_R[1] = speed_error_R[0];
    MotorControlL(motor_duty_L);    //ִ�е��PWM���
    MotorControlR(motor_duty_R);
}
uint8 circle_ready()//Ԥ�뻷
{
    if(L>=3800)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

int circle_judge()
{
    uint8 circle_Flag = 0;
    if((L>=3500)&&((L-LM)>=1600))//��
    {
        circle_Flag = 1;
        
    }
//    else if((LM/RM) >= 50&&(LM>=1000))//��ƫ
//    {
//        circle_Flag = 1;
//    }
//    else if((LM>=3000)&&(RM>=2000)&&(R>=2400))//��ƫ
//    {
//        circle_Flag = 1;
//    }
    return circle_Flag;

}

float  servo_kp_cir = 25;
float  servo_kd_cir = 0 ;
void circle_control()
{
    //AdcRead();
    ComputerADC(direction_cir(), 1);//��������Ϊfloat����
    servo_error[0] = AdcNow;    //��Ⱥʹ��������Ϊ����
    if (servo_error[0]>= 100)
    {
        servo_error[0] = 100;
    
    }
    else if(servo_error[0] <= -100)
    {
    
        servo_error[0] = -100;
    }
    servo_duty_delta = (servo_error[0]*servo_kp_cir + (servo_error[0]-servo_error[1])*servo_kd_cir);    //���λ��ʽPD����
    //servo_duty_delta /= 2;    //�ɸ������������ֵ��С��ע����������
    servo_error[1] = servo_error[0];    //����
    ServoControl(servo_duty_delta);    //ִ�ж��PWM���
}
int cross_judge()
{
    if((L/R<=1.8)&&(R/L<=1.8)&&(L>=2500||R>=2500))
//    if(((L/R<=1.8)&&(R/L<=1.8)&&(RM/LM<=1.8)&&(LM/RM)<=1.8)||((LM>=3300)&&(RM>=3300)&&(L+R)>=4500))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

float  servo_kp_cro = 1;
float  servo_kd_cro = 0 ;
void cross_control()
{
    ComputerADC(direction_cross(), 1);//��������Ϊfloat����
    servo_error[0] = AdcNow;    //��Ⱥʹ��������Ϊ����
    if (servo_error[0]>= 100)
    {
        servo_error[0] = 100;
    
    }
    else if(servo_error[0] <= -100)
    {
    
        servo_error[0] = -100;
    }
    servo_duty_delta = (servo_error[0]*servo_kp_cir + (servo_error[0]-servo_error[1])*servo_kd_cir);    //���λ��ʽPD����
    servo_error[1] = servo_error[0];    //����
    ServoControl(servo_duty_delta);    //ִ�ж��PWM���
}

/*tof���*/
void tof_get(uint16 dl1a_distance_mm)
{
    uint8 i = 0;
    uint16 temptof = 0;
    static uint8 SpCount = 0, k = 0;
    uint16 min = 0;
    uint16 max = 0;
    static uint16 TofArry[10] = {0.0};
    TofArry[k++] = dl1a_distance_mm;
    if(k == 10)
    {
        for (i = 0; i < 10; i++)
        {
            if(TofArry[i]>max)
            {
                max = TofArry[i];
            }
            if(TofArry[i]<min)
            {
               min = TofArry[i];
            }
            temptof += TofArry[i];
               
        }
        temptof = temptof - max -min;
        TofNow =  (uint16)(temptof/8);
    }      
}
uint8 ramp_flag;
uint8 ramp_judge(dl1a_distance_mm)
{
    
    if((dl1a_distance_mm<=ramp_threshold)&&(RM<=1000)&&(LM<=1000)&&(R<=1000)&&(LM>=300)&&(R>=200))
    {
        ramp_flag = 1;
        
    }
    else
    {
        ramp_flag = 0;
    }
    return ramp_flag;

}
float  servo_kp_ramp = 15;
float  servo_kd_ramp = 0 ;

void ramp_control()
{
    ComputerADC(direction(), 1);
    servo_error[0] = AdcNow;    
    if (servo_error[0]>= 100)
    {
        servo_error[0] = 100;
    
    }
    else if(servo_error[0] <= -100)
    {
    
        servo_error[0] = -100;
    }
    servo_duty_delta = (servo_error[0]*servo_kp_ramp + (servo_error[0]-servo_error[1])*servo_kd_ramp);    //���λ��ʽPD����
    //servo_duty_delta /= 2;    //�ɸ������������ֵ��С��ע����������
    servo_error[1] = servo_error[0];    //����
    ServoControl(servo_duty_delta);    //ִ�ж��PWM���
}    
