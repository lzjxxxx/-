#include "imu660.h"
void Imu660Init()
{
    while(imu660ra_init())				//���������ǳ�ʼ��
	{
		delay_ms(50);
		P52 = !P52;
	}
}
