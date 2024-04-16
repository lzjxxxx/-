#include "encoder.h"

//������������
#define SPEEDL_PLUSE   CTIM0_P34
#define SPEEDR_PLUSE   CTIM3_P04
//���巽������
#define SPEEDL_DIR     P35
#define SPEEDR_DIR     P53

int16 templ_pluse = 0;
int16 tempr_pluse = 0;

int16 ReadEncoderL()
{
    WTST = 0;               //���ó������ȴ���������ֵΪ0�ɽ�CPUִ�г�����ٶ�����Ϊ���
    
	DisableGlobalIRQ();		//�ر����ж�ȷ�����ᱻ�����жϸ��Ŵ�ϣ���߼�����׼ȷ�ԣ������������ж�����Ҳ�ᱻ�ر�
	
    //sys_clk��ѡֵ:35000000��30000000, 27000000. 24000000, 22118400, 20000000, 18432000, 12000000, 11059200, 6000000, 5529600��
    //����ϵͳƵ�ʣ���Ƶ����Ҫ��STC-ISP����е� <�����û��������е�IRCƵ��>ѡ���Ƶ��һ�¡�
    //���Ƶ�����ò��ԣ����ᵼ�´��ڵ����ݲ�����,PWM�Ĺ����������ȵȡ�
    sys_clk = 35000000;     //����ϵͳƵ��Ϊ35MHz
	ctimer_count_init(SPEEDL_PLUSE);	//��ʼ����ʱ��0��Ϊ�ⲿ����

	EnableGlobalIRQ();					//�������ж�
    //��ȡ�ɼ����ı�����������
    templ_pluse = ctimer_count_read(SPEEDL_PLUSE);//��ȡ������ֵ


    //����������
    ctimer_count_clean(SPEEDL_PLUSE);
 

    //�ɼ�������Ϣ
    if(1 == SPEEDL_DIR)    
    {
        templ_pluse = -templ_pluse;
    }
//    else//���������0            
//    {
//        templ_pluse = templ_pluse;
//    }
    //EnableGlobalIRQ();					//�������ж�
    return templ_pluse;
}
int16 ReadEncoderR()
{
    WTST = 0;               //���ó������ȴ���������ֵΪ0�ɽ�CPUִ�г�����ٶ�����Ϊ���
    
	DisableGlobalIRQ();		
    sys_clk = 35000000;     //����ϵͳƵ��Ϊ35MHz
	ctimer_count_init(SPEEDR_PLUSE);	//��ʼ����ʱ��0��Ϊ�ⲿ����
	//EnableGlobalIRQ();					//�������ж�
    //��ȡ�ɼ����ı�����������
    tempr_pluse = ctimer_count_read(SPEEDR_PLUSE);//��ȡ������ֵ
    //����������
    ctimer_count_clean(SPEEDR_PLUSE);
 

    //�ɼ�������Ϣ
    if(1 == SPEEDR_DIR)    
    {
        tempr_pluse = -tempr_pluse;
    }
//    else//�Ҳ�������0            
//    {
//        tempr_pluse = tempr_pluse;
//    }
    EnableGlobalIRQ();					//�������ж�
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