/*********************************************************************************************************************
 * COPYRIGHT NOTICE
 * Copyright (c) 2018,��ɿƼ�
 * All rights reserved.
 * ��������QQȺ��һȺ��179029047(����)  ��Ⱥ��244861897(����)  ��Ⱥ��824575535
 *
 * �����������ݰ�Ȩ������ɿƼ����У�δ��������������ҵ��;��
 * ��ӭ��λʹ�ò������������޸�����ʱ���뱣����ɿƼ��İ�Ȩ������
 *
 * @file       		main
 * @company	   		�ɶ���ɿƼ����޹�˾
 * @author     		��ɿƼ�(QQ3184284598)
 * @version    		�鿴doc��version�ļ� �汾˵��
 * @Software 		MDK FOR C251 V5.60
 * @Taobao   		https://seekfree.taobao.com/
 * @date       		2019-11-15
 * @note		
					���߶��壺
					------------------------------------ 
					1.14��IPSģ��ܽ�       ��Ƭ���ܽ�
					SCL                 	�鿴SEEKFREE_IPS200_SPI.h�ļ��ڵ�IPS200_SCL		�궨��     Ӳ��SPI���Ų��������л�
					SDA                 	�鿴SEEKFREE_IPS200_SPI.h�ļ��ڵ�IPS200_SDA		�궨��     Ӳ��SPI���Ų��������л�
					RES                 	�鿴SEEKFREE_IPS200_SPI.h�ļ��ڵ�IPS200_REST_PIN�궨��    
					DC                  	�鿴SEEKFREE_IPS200_SPI.h�ļ��ڵ�IPS200_DC_PIN	�궨��  
					CS                  	�鿴SEEKFREE_IPS200_SPI.h�ļ��ڵ�IPS200_CS		�궨��     Ӳ��SPI���Ų��������л�
					BL  					�鿴SEEKFREE_IPS200_SPI.h�ļ��ڵ�IPS200_BL_PIN	�궨��
					
					��Դ����
					VCC 3.3V��Դ
					GND ��Դ��
					���ֱ���240*320
					------------------------------------ 
 ********************************************************************************************************************/
 



#include "SEEKFREE_IPS200_SPI.h"
#include "SEEKFREE_FONT.h"
#include "SEEKFREE_PRINTF.h"
#include "zf_spi.h"
#include "zf_delay.h"




//-------------------------------------------------------------------------------------------------------------------
//  ���º�����ʹ�����SPIͨ�ţ���Ƚ�Ӳ��SPI�����SPI���Ÿ���������ʹ��������ͨIO
//-------------------------------------------------------------------------------------------------------------------



//-------------------------------------------------------------------------------------------------------------------
//  @brief      �ڲ����ã��û��������
//  @param      void 		    
//  @return     				
//  @since      v1.0
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
void ips200_writeIndex_simspi(uint8 dat)			//д����
{	
	uint8 i = 8;
	IPS200_CS_SIMSPI(0);
    IPS200_DC_SIMSPI(0);

	IPS200_SCL_SIMSPI(0);

	while(i--)
	{
		if(dat&0x80){IPS200_SDA_SIMSPI(1);}
		else{IPS200_SDA_SIMSPI(0);}
		IPS200_SCL_SIMSPI(1);

		IPS200_SCL_SIMSPI(0);
		dat<<=1;
	}	
	IPS200_CS_SIMSPI(1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �ڲ����ã��û��������
//  @param      void 		    
//  @return     				
//  @since      v1.0
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
void ips200_writeData_simspi(uint8 dat)			//д����
{
	uint8 i = 8;
	IPS200_CS_SIMSPI(0);
    IPS200_DC_SIMSPI(1);

	IPS200_SCL_SIMSPI(0);

	while(i--)
	{
		if(dat&0x80){IPS200_SDA_SIMSPI(1);}
		else{IPS200_SDA_SIMSPI(0);}
		IPS200_SCL_SIMSPI(1);

		IPS200_SCL_SIMSPI(0);
		dat<<=1;
	}	
	IPS200_CS_SIMSPI(1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �ڲ����ã��û��������
//  @param      void 		    
//  @return     				
//  @since      v1.0
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
void ips200_writeDatai_16bit_simspi(uint16 dat)	//��Һ����дһ��16λ����
{
    uint8 i=16;
	IPS200_CS_SIMSPI(0);
    IPS200_DC_SIMSPI(1);
    IPS200_SCL_SIMSPI(0);
	while(i--)
	{
	  	if(dat&0x8000){IPS200_SDA_SIMSPI(1);}
	  	else{IPS200_SDA_SIMSPI(0);}
	  	IPS200_SCL_SIMSPI(1);
		
	  	IPS200_SCL_SIMSPI(0);
	  	dat<<=1;
	}
	IPS200_CS_SIMSPI(1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      1.14�� IPSҺ����ʼ��
//  @return     void
//  @since      v1.0
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
void ips200_init_simspi(void)
{
    
	IPS200_BL_SIMSPI(1);
	IPS200_DC_SIMSPI(0);
	IPS200_REST_SIMSPI(0);
	
    delay_ms(200);
    
    IPS200_REST_SIMSPI(1);
    delay_ms(100);
	ips200_writeIndex_simspi(0x11);
    delay_ms(120);
    ips200_writeIndex_simspi(0x36);
    
    #if (IPS200_DISPLAY_DIR==0)    
		ips200_writeData_simspi(0x00);
    #elif (IPS200_DISPLAY_DIR==1)    
		ips200_writeData_simspi(0xC0);
    #elif (IPS200_DISPLAY_DIR==2)    
		ips200_writeData_simspi(0x70);
    #else                            	
		ips200_writeData_simspi(0xA0);
    #endif
    ips200_writeIndex_simspi(0x3A); 
    ips200_writeData_simspi(0x05);

    ips200_writeIndex_simspi(0xB2);
    ips200_writeData_simspi(0x0C);
    ips200_writeData_simspi(0x0C);
    ips200_writeData_simspi(0x00);
    ips200_writeData_simspi(0x33);
    ips200_writeData_simspi(0x33); 

    ips200_writeIndex_simspi(0xB7); 
    ips200_writeData_simspi(0x35);  

    ips200_writeIndex_simspi(0xBB);
    ips200_writeData_simspi(0x29);

//    ips200_writeIndex_simspi(0xC0);
//    ips200_writeData_simspi(0x2C);

    ips200_writeIndex_simspi(0xC2);
    ips200_writeData_simspi(0x01);

    ips200_writeIndex_simspi(0xC3);
    ips200_writeData_simspi(0x19);   

    ips200_writeIndex_simspi(0xC4);
    ips200_writeData_simspi(0x20);  

    ips200_writeIndex_simspi(0xC5);
    ips200_writeData_simspi(0x1A); 
	
    ips200_writeIndex_simspi(0xC6); 
    ips200_writeData_simspi(0x0F);    

    ips200_writeIndex_simspi(0xD0); 
    ips200_writeData_simspi(0xA4);
    ips200_writeData_simspi(0xA1);

    ips200_writeIndex_simspi(0xE0);
    ips200_writeData_simspi(0xD0);
    ips200_writeData_simspi(0x08);
    ips200_writeData_simspi(0x0E);
    ips200_writeData_simspi(0x09);
    ips200_writeData_simspi(0x09);
    ips200_writeData_simspi(0x05);
    ips200_writeData_simspi(0x31);
    ips200_writeData_simspi(0x33);
    ips200_writeData_simspi(0x48);
    ips200_writeData_simspi(0x17);
    ips200_writeData_simspi(0x14);
    ips200_writeData_simspi(0x15);
    ips200_writeData_simspi(0x31);
    ips200_writeData_simspi(0x34);

    ips200_writeIndex_simspi(0xE1);
    ips200_writeData_simspi(0xD0);
    ips200_writeData_simspi(0x08);
    ips200_writeData_simspi(0x0E);
    ips200_writeData_simspi(0x09);
    ips200_writeData_simspi(0x09);
    ips200_writeData_simspi(0x15);
    ips200_writeData_simspi(0x31);
    ips200_writeData_simspi(0x33);
    ips200_writeData_simspi(0x48);
    ips200_writeData_simspi(0x17);
    ips200_writeData_simspi(0x14);
    ips200_writeData_simspi(0x15);
    ips200_writeData_simspi(0x31);
    ips200_writeData_simspi(0x34);

    ips200_writeIndex_simspi(0x21); 

    ips200_writeIndex_simspi(0x29);
    ips200_clear_simspi(IPS200_BGCOLOR);
}



void ips200_set_region_simspi(uint16 x1,uint16 y1,uint16 x2,uint16 y2)
{
	#if(IPS200_DISPLAY_DIR==0)
	{
		ips200_writeIndex_simspi(0x2a);//�е�ַ����
		ips200_writeDatai_16bit_simspi(x1);
		ips200_writeDatai_16bit_simspi(x2);
		ips200_writeIndex_simspi(0x2b);//�е�ַ����
		ips200_writeDatai_16bit_simspi(y1);
		ips200_writeDatai_16bit_simspi(y2);
		ips200_writeIndex_simspi(0x2c);//������д
	}
	#elif(IPS200_DISPLAY_DIR==1)
	{
		ips200_writeIndex_simspi(0x2a);//�е�ַ����
		ips200_writeDatai_16bit_simspi(x1);
		ips200_writeDatai_16bit_simspi(x2);
		ips200_writeIndex_simspi(0x2b);//�е�ַ����
		ips200_writeDatai_16bit_simspi(y1);
		ips200_writeDatai_16bit_simspi(y2);
		ips200_writeIndex_simspi(0x2c);//������д
	}
	#elif(IPS200_DISPLAY_DIR==2)
	{
		ips200_writeIndex_simspi(0x2a);//�е�ַ����
		ips200_writeDatai_16bit_simspi(x1);
		ips200_writeDatai_16bit_simspi(x2);
		ips200_writeIndex_simspi(0x2b);//�е�ַ����
		ips200_writeDatai_16bit_simspi(y1);
		ips200_writeDatai_16bit_simspi(y2);
		ips200_writeIndex_simspi(0x2c);//������д
	}
	#else
	{
		ips200_writeIndex_simspi(0x2a);//�е�ַ����
		ips200_writeDatai_16bit_simspi(x1);
		ips200_writeDatai_16bit_simspi(x2);
		ips200_writeIndex_simspi(0x2b);//�е�ַ����
		ips200_writeDatai_16bit_simspi(y1);
		ips200_writeDatai_16bit_simspi(y2);
		ips200_writeIndex_simspi(0x2c);//������д
	}
	#endif
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����������
//  @param      color     		��ɫ����
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_clear(YELLOW);// ȫ������Ϊ��ɫ
//-------------------------------------------------------------------------------------------------------------------
void ips200_clear_simspi(uint16 color)
{
    uint16 i,j;
	ips200_set_region_simspi(0,0,IPS200_X_MAX-1,IPS200_Y_MAX-1);
    for(i=0;i<IPS200_X_MAX;i++)
    {
        for (j=0;j<IPS200_Y_MAX;j++)
	   	{
        	ips200_writeDatai_16bit_simspi(color);	 			 
	    }
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ������
//  @param      x     	        ����x��������
//  @param      y     	        ����y��������
//  @param      dat       	    ��Ҫ��ʾ����ɫ
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_drawpoint(0,0,RED);  //����0,0��һ����ɫ�ĵ�
//-------------------------------------------------------------------------------------------------------------------
void ips200_drawpoint_simspi(uint16 x,uint16 y,uint16 color)
{
    ips200_set_region_simspi(x,y,x,y);
    ips200_writeDatai_16bit_simspi(color);
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ�ַ�
//  @param      x     	        ����x�������� ������Χ 0 -��IPS200_X_MAX-1��
//  @param      y     	        ����y�������� ������Χ 0 -��IPS200_Y_MAX/16-1��
//  @param      dat       	    ��Ҫ��ʾ���ַ�
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_showchar(0,0,'x');//����0,0дһ���ַ�x
//-------------------------------------------------------------------------------------------------------------------
void ips200_showchar_simspi(uint16 x,uint16 y,const int8 dat)
{
	uint8 i,j;
	uint8 temp;
    
	for(i=0; i<16; i++)
	{
		ips200_set_region_simspi(x,y+i,x+7,y+i);
		temp = tft_ascii[dat-32][i];//��32��Ϊ��ȡģ�Ǵӿո�ʼȡ�� �ո���ascii�������32
		for(j=0; j<8; j++)
		{
			if(temp&0x01)	
            {
                ips200_writeDatai_16bit_simspi(IPS200_PENCOLOR);
            }
			else			ips200_writeDatai_16bit_simspi(IPS200_BGCOLOR);
			temp>>=1;
		}
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ�ַ���
//  @param      x     	        ����x��������  ������Χ 0 -��IPS200_X_MAX-1��
//  @param      y     	        ����y��������  ������Χ 0 -��IPS200_Y_MAX/16-1��
//  @param      dat       	    ��Ҫ��ʾ���ַ���
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_showstr(0,0,"seekfree");
//-------------------------------------------------------------------------------------------------------------------
void ips200_showstr_simspi(uint16 x,uint16 y,const int8 dat[])
{
	uint16 j;
	
	j = 0;
	while(dat[j] != '\0')
	{
		ips200_showchar_simspi(x+8*j,y*16,dat[j]);
		j++;
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ8λ�з���
//  @param      x     	        ����x��������  ������Χ 0 -��IPS200_X_MAX-1��
//  @param      y     	        ����y��������  ������Χ 0 -��IPS200_Y_MAX/16-1��
//  @param      dat       	    ��Ҫ��ʾ�ı�������������int8
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_showint8(0,0,x);//xΪint8����
//-------------------------------------------------------------------------------------------------------------------
void ips200_showint8_simspi(uint16 x,uint16 y,int8 dat)
{
	uint8 a[3];
	uint8 i;
	if(dat<0)
	{
		ips200_showchar_simspi(x,y*16,'-');
		dat = -dat;
	}
	else	ips200_showchar_simspi(x,y*16,' ');
	
	a[0] = dat/100;
	a[1] = dat/10%10;
	a[2] = dat%10;
	i = 0;
	while(i<3)
	{
		ips200_showchar_simspi((uint8)(x+(8*(i+1))), (uint8)(y*16),(int8)('0' + a[i]));
		i++;
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ8λ�޷���
//  @param      x     	        ����x�������� ������Χ 0 -��IPS200_X_MAX-1��
//  @param      y     	        ����y�������� ������Χ 0 -��IPS200_Y_MAX/16-1��
//  @param      dat       	    ��Ҫ��ʾ�ı�������������uint8
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_showuint8(0,0,x);//xΪuint8����
//-------------------------------------------------------------------------------------------------------------------
void ips200_showuint8_simspi(uint16 x,uint16 y,uint8 dat)
{
	uint8 a[3];
	uint8 i;
	
	a[0] = dat/100;
	a[1] = dat/10%10;
	a[2] = dat%10;
	i = 0;
	while(i<3)
	{
		ips200_showchar_simspi((uint8)(x+(8*(i+1))), (uint8)(y*16),(int8)('0' + a[i]));
		i++;
	}
	
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ16λ�з���
//  @param      x     	        ����x�������� ������Χ 0 -��IPS200_X_MAX-1��
//  @param      y     	        ����y�������� ������Χ 0 -��IPS200_Y_MAX/16-1��
//  @param      dat       	    ��Ҫ��ʾ�ı�������������int16
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_showint16(0,0,x);//xΪint16����
//-------------------------------------------------------------------------------------------------------------------
void ips200_showint16_simspi(uint16 x,uint16 y,int16 dat)
{
	uint8 a[5];
	uint8 i;
	if(dat<0)
	{
		ips200_showchar_simspi(x,y*16,'-');
		dat = -dat;
	}
	else	ips200_showchar_simspi(x,y*16,' ');

	a[0] = dat/10000;
	a[1] = dat/1000%10;
	a[2] = dat/100%10;
	a[3] = dat/10%10;
	a[4] = dat%10;
	
	i = 0;
	while(i<5)
	{
		ips200_showchar_simspi((uint8)(x+(8*(i+1))), (uint8)(y*16),(int8)('0' + a[i]));
		i++;
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ16λ�޷���
//  @param      x     	        ����x�������� ������Χ 0 -��IPS200_X_MAX-1��
//  @param      y     	        ����y�������� ������Χ 0 -��IPS200_Y_MAX/16-1��
//  @param      dat       	    ��Ҫ��ʾ�ı�������������uint16
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_showuint16(0,0,x);//xΪuint16����
//-------------------------------------------------------------------------------------------------------------------
void ips200_showuint16_simspi(uint16 x,uint16 y,uint16 dat)
{
	uint8 a[5];
	uint8 i;

	a[0] = dat/10000;
	a[1] = dat/1000%10;
	a[2] = dat/100%10;
	a[3] = dat/10%10;
	a[4] = dat%10;
	
	i = 0;
	while(i<5)
	{
		ips200_showchar_simspi((uint8)(x+(8*(i+1))), (uint8)(y*16),(int8)('0' + a[i]));
		i++;
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ32λ�з���(ȥ������������Ч��0)
//  @param      x     	        ����x�������� ������Χ 0 -��IPS200_X_MAX-1��
//  @param      y     	        ����y�������� ������Χ 0 -��IPS200_Y_MAX/16-1��
//  @param      dat       	    ��Ҫ��ʾ�ı�������������uint32
//  @param      num       	    ��Ҫ��ʾ��λ�� ���10λ  ������������
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_showuint32(0,0,x,3);//x����Ϊint32 uint16 int16 uint8 int8����
//  Sample usage:               ��������ʾһ�� ��-����   ������ʾһ���ո�
//-------------------------------------------------------------------------------------------------------------------
void ips200_showint32_simspi(uint16 x,uint16 y,int32 dat,uint8 num)
{
    int8    buff[34];
    uint8   length;
    
    if(10<num)      num = 10;
    
    num++;
    if(0>dat)   length = zf_sprintf( &buff[0],"%d",dat);//����
    else
    {
        buff[0] = ' ';
        length = zf_sprintf( &buff[1],"%d",dat);
        length++;
    }
    while(length < num)
    {
        buff[length] = ' ';
        length++;
    }
    buff[num] = '\0';

    ips200_showstr_simspi(x, y, buff);	//��ʾ����
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ������(ȥ������������Ч��0)
//  @param      x     	        ����x�������� ������Χ 0 -��IPS200_X_MAX-1��
//  @param      y     	        ����y�������� ������Χ 0 -��IPS200_Y_MAX/16-1��
//  @param      dat       	    ��Ҫ��ʾ�ı�������������float��double
//  @param      num       	    ����λ��ʾ����   ���10λ  
//  @param      pointnum        С��λ��ʾ����   ���6λ
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_showfloat(0,0,x,2,3);//��ʾ������   ������ʾ2λ   С����ʾ��λ
//  @note                       �ر�ע�⵱����С��������ʾ��ֵ����д���ֵ��һ����ʱ��
//                              ���������ڸ��������ȶ�ʧ���⵼�µģ��Ⲣ������ʾ���������⣬
//                              �й���������飬�����аٶ�ѧϰ   ���������ȶ�ʧ���⡣
//                              ��������ʾһ�� ��-����   ������ʾһ���ո�
//-------------------------------------------------------------------------------------------------------------------
void ips200_showfloat_simspi(uint16 x,uint16 y,double dat,uint8 num,uint8 pointnum)
{
    uint8   length;
	int8    buff[34];
	int8    start,end,point;

	if(6<pointnum)  pointnum = 6;
    if(10<num)      num = 10;
        
    if(0>dat)   length = zf_sprintf( &buff[0],"%f",dat);//����
    else
    {
        length = zf_sprintf( &buff[1],"%f",dat);
        length++;
    }
    point = length - 7;         //����С����λ��
    start = point - num - 1;    //������ʼλ
    end = point + pointnum + 1; //�������λ
    while(0>start)//����λ����  ĩβӦ�����ո�
    {
        buff[end] = ' ';
        end++;
        start++;
    }
    
    if(0>dat)   buff[start] = '-';
    else        buff[start] = ' ';
    
    buff[end] = '\0';

    ips200_showstr_simspi(x, y, buff);	//��ʾ����
}




//-------------------------------------------------------------------------------------------------------------------
//  ���º�����ʹ��Ӳ��SPIͨ�ţ���Ƚ����IIC���ٶȱ����IIC�졣
//-------------------------------------------------------------------------------------------------------------------




//-------------------------------------------------------------------------------------------------------------------
//  @brief      �ڲ����ã��û��������
//  @param      void 		    
//  @return     				
//  @since      v1.0
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
void ips200_writeIndex(uint8 dat)			//д����
{	
	IPS200_CS(0);
    IPS200_DC(0);

	spi_mosi(dat);
	IPS200_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �ڲ����ã��û��������
//  @param      void 		    
//  @return     				
//  @since      v1.0
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
void ips200_writeData(uint8 dat)			//д����
{
	IPS200_CS(0);
    IPS200_DC(1);

	spi_mosi(dat);
	IPS200_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      �ڲ����ã��û��������
//  @param      void 		    
//  @return     				
//  @since      v1.0
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
void ips200_writedata_16bit(uint16 dat)	//��Һ����дһ��16λ����
{
    uint8 dat1[2];
    dat1[0] = dat >> 8;
    dat1[1] = (uint8)dat;
	IPS200_CS(0);
    IPS200_DC(1);
	spi_mosi(dat1[0]);	//д������  ��λ��ǰ  ��λ�ں�
	spi_mosi(dat1[1]);
	IPS200_CS(1);
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      1.14�� IPSҺ����ʼ��
//  @return     void
//  @since      v1.0
//  Sample usage:               
//-------------------------------------------------------------------------------------------------------------------
void ips200_init(void)
{
    spi_init(IPS200_SPIN_PIN, IPS200_SCL_PIN, IPS200_SDA_PIN, IPS200_SDA_IN_PIN, 0, MASTER, SPI_SYSclk_DIV_4);

	IPS200_BL(1);
	IPS200_DC(0);
	IPS200_REST(0);
	
    delay_ms(200);
    
    IPS200_REST(1);
    delay_ms(100);
	ips200_writeIndex(0x11);
	delay_ms(100);
    ips200_writeIndex(0x36);
    
    #if	(IPS200_DISPLAY_DIR==0)    
		ips200_writeData(0x00);
    #elif (IPS200_DISPLAY_DIR==1)    
		ips200_writeData(0xC0);
    #elif (IPS200_DISPLAY_DIR==2)    
		ips200_writeData(0x70);
    #else                            
		ips200_writeData(0xA0);
    #endif
	
    ips200_writeIndex(0x3A); 
    ips200_writeData(0x05);

    ips200_writeIndex(0xB2);
    ips200_writeData(0x0C);
    ips200_writeData(0x0C);
    ips200_writeData(0x00);
    ips200_writeData(0x33);
    ips200_writeData(0x33); 

    ips200_writeIndex(0xB7); 
    ips200_writeData(0x35);  

    ips200_writeIndex(0xBB);
    ips200_writeData(0x29);

//    ips200_writeIndex(0xC0);
//    ips200_writeData(0x2C);

    ips200_writeIndex(0xC2);
    ips200_writeData(0x01);

    ips200_writeIndex(0xC3);
    ips200_writeData(0x19);   

    ips200_writeIndex(0xC4);
    ips200_writeData(0x20);  

	ips200_writeIndex(0xC5);
    ips200_writeData(0x1A);
	
    ips200_writeIndex(0xC6); 
    ips200_writeData(0x0F);    

    ips200_writeIndex(0xD0); 
    ips200_writeData(0xA4);
    ips200_writeData(0xA1);

    ips200_writeIndex(0xE0);
    ips200_writeData(0xD0);
    ips200_writeData(0x08);
    ips200_writeData(0x0E);
    ips200_writeData(0x09);
    ips200_writeData(0x09);
    ips200_writeData(0x05);
    ips200_writeData(0x31);
    ips200_writeData(0x33);
    ips200_writeData(0x48);
    ips200_writeData(0x17);
    ips200_writeData(0x14);
    ips200_writeData(0x15);
    ips200_writeData(0x31);
    ips200_writeData(0x34);

    ips200_writeIndex(0xE1);
    ips200_writeData(0xD0);
    ips200_writeData(0x08);
    ips200_writeData(0x0E);
    ips200_writeData(0x09);
    ips200_writeData(0x09);
    ips200_writeData(0x15);
    ips200_writeData(0x31);
    ips200_writeData(0x33);
    ips200_writeData(0x48);
    ips200_writeData(0x17);
    ips200_writeData(0x14);
    ips200_writeData(0x15);
    ips200_writeData(0x31);
    ips200_writeData(0x34);

    ips200_writeIndex(0x21); 

    ips200_writeIndex(0x29);
    ips200_clear(IPS200_BGCOLOR);
}



void ips200_set_region(uint16 x1,uint16 y1,uint16 x2,uint16 y2)
{
	#if(IPS200_DISPLAY_DIR==0)
	{
		ips200_writeIndex(0x2a);//�е�ַ����
		ips200_writedata_16bit(x1);
		ips200_writedata_16bit(x2);
		ips200_writeIndex(0x2b);//�е�ַ����
		ips200_writedata_16bit(y1);
		ips200_writedata_16bit(y2);
		ips200_writeIndex(0x2c);//������д
	}
	#elif(IPS200_DISPLAY_DIR==1)
	{
		ips200_writeIndex(0x2a);//�е�ַ����
		ips200_writedata_16bit(x1);
		ips200_writedata_16bit(x2);
		ips200_writeIndex(0x2b);//�е�ַ����
		ips200_writedata_16bit(y1);
		ips200_writedata_16bit(y2);
		ips200_writeIndex(0x2c);//������д
	}
	#elif(IPS200_DISPLAY_DIR==2)
	{
		ips200_writeIndex(0x2a);//�е�ַ����
		ips200_writedata_16bit(x1);
		ips200_writedata_16bit(x2);
		ips200_writeIndex(0x2b);//�е�ַ����
		ips200_writedata_16bit(y1);
		ips200_writedata_16bit(y2);
		ips200_writeIndex(0x2c);//������д
	}
	#else
	{
		ips200_writeIndex(0x2a);//�е�ַ����
		ips200_writedata_16bit(x1);
		ips200_writedata_16bit(x2);
		ips200_writeIndex(0x2b);//�е�ַ����
		ips200_writedata_16bit(y1);
		ips200_writedata_16bit(y2);
		ips200_writeIndex(0x2c);//������д
	}
	#endif
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����������
//  @param      color     		��ɫ����
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_clear(YELLOW);// ȫ������Ϊ��ɫ
//-------------------------------------------------------------------------------------------------------------------
void ips200_clear(uint16 color)
{
    uint16 i,j;
	ips200_set_region(0,0,IPS200_X_MAX-1,IPS200_Y_MAX-1);
    for(i=0;i<IPS200_X_MAX;i++)
    {
        for (j=0;j<IPS200_Y_MAX;j++)
	   	{
        	ips200_writedata_16bit(color);	 			 
	    }
    }
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ������
//  @param      x     	        ����x��������
//  @param      y     	        ����y��������
//  @param      dat       	    ��Ҫ��ʾ����ɫ
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_drawpoint(0,0,RED);  //����0,0��һ����ɫ�ĵ�
//-------------------------------------------------------------------------------------------------------------------
void ips200_drawpoint(uint16 x,uint16 y,uint16 color)
{
    ips200_set_region(x,y,x,y);
    ips200_writedata_16bit(color);
}



//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ�ַ�
//  @param      x     	        ����x�������� ������Χ 0 -��IPS200_X_MAX-1��
//  @param      y     	        ����y�������� ������Χ 0 -��IPS200_Y_MAX/16-1��
//  @param      dat       	    ��Ҫ��ʾ���ַ�
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_showchar(0,0,'x');//����0,0дһ���ַ�x
//-------------------------------------------------------------------------------------------------------------------
void ips200_showchar(uint16 x,uint16 y,const int8 dat)
{
	uint8 i,j;
	uint8 temp;
    
	for(i=0; i<16; i++)
	{
		ips200_set_region(x,y+i,x+7,y+i);
		temp = tft_ascii[dat-32][i];//��32��Ϊ��ȡģ�Ǵӿո�ʼȡ�� �ո���ascii�������32
		for(j=0; j<8; j++)
		{
			if(temp&0x01)	
            {
                ips200_writedata_16bit(IPS200_PENCOLOR);
            }
			else			ips200_writedata_16bit(IPS200_BGCOLOR);
			temp>>=1;
		}
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ�ַ���
//  @param      x     	        ����x��������  ������Χ 0 -��IPS200_X_MAX-1��
//  @param      y     	        ����y��������  ������Χ 0 -��IPS200_Y_MAX/16-1��
//  @param      dat       	    ��Ҫ��ʾ���ַ���
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_showstr(0,0,"seekfree");
//-------------------------------------------------------------------------------------------------------------------
void ips200_showstr(uint16 x,uint16 y,const int8 dat[])
{
	uint16 j;
	
	j = 0;
	while(dat[j] != '\0')
	{
		ips200_showchar(x+8*j,y*16,dat[j]);
		j++;
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ8λ�з���
//  @param      x     	        ����x��������  ������Χ 0 -��IPS200_X_MAX-1��
//  @param      y     	        ����y��������  ������Χ 0 -��IPS200_Y_MAX/16-1��
//  @param      dat       	    ��Ҫ��ʾ�ı�������������int8
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_showint8(0,0,x);//xΪint8����
//-------------------------------------------------------------------------------------------------------------------
void ips200_showint8(uint16 x,uint16 y,int8 dat)
{
	uint8 a[3];
	uint8 i;
	if(dat<0)
	{
		ips200_showchar(x,y*16,'-');
		dat = -dat;
	}
	else	ips200_showchar(x,y*16,' ');
	
	a[0] = dat/100;
	a[1] = dat/10%10;
	a[2] = dat%10;
	i = 0;
	while(i<3)
	{
		ips200_showchar((uint16)(x+(8*(i+1))), (uint16)(y*16),(int8)('0' + a[i]));
		i++;
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ8λ�޷���
//  @param      x     	        ����x�������� ������Χ 0 -��IPS200_X_MAX-1��
//  @param      y     	        ����y�������� ������Χ 0 -��IPS200_Y_MAX/16-1��
//  @param      dat       	    ��Ҫ��ʾ�ı�������������uint8
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_showuint8(0,0,x);//xΪuint8����
//-------------------------------------------------------------------------------------------------------------------
void ips200_showuint8(uint16 x,uint16 y,uint8 dat)
{
	uint8 a[3];
	uint8 i;
	
	a[0] = dat/100;
	a[1] = dat/10%10;
	a[2] = dat%10;
	i = 0;
	while(i<3)
	{
		ips200_showchar((uint16)(x+(8*(i+1))), (uint16)(y*16),(int8)('0' + a[i]));
		i++;
	}
	
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ16λ�з���
//  @param      x     	        ����x�������� ������Χ 0 -��IPS200_X_MAX-1��
//  @param      y     	        ����y�������� ������Χ 0 -��IPS200_Y_MAX/16-1��
//  @param      dat       	    ��Ҫ��ʾ�ı�������������int16
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_showint16(0,0,x);//xΪint16����
//-------------------------------------------------------------------------------------------------------------------
void ips200_showint16(uint16 x,uint16 y,int16 dat)
{
	uint8 a[5];
	uint8 i;
	if(dat<0)
	{
		ips200_showchar(x,y*16,'-');
		dat = -dat;
	}
	else	ips200_showchar(x,y*16,' ');

	a[0] = dat/10000;
	a[1] = dat/1000%10;
	a[2] = dat/100%10;
	a[3] = dat/10%10;
	a[4] = dat%10;
	
	i = 0;
	while(i<5)
	{
		ips200_showchar((uint16)(x+(8*(i+1))), (uint16)(y*16),(int8)('0' + a[i]));
		i++;
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ16λ�޷���
//  @param      x     	        ����x�������� ������Χ 0 -��IPS200_X_MAX-1��
//  @param      y     	        ����y�������� ������Χ 0 -��IPS200_Y_MAX/16-1��
//  @param      dat       	    ��Ҫ��ʾ�ı�������������uint16
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_showuint16(0,0,x);//xΪuint16����
//-------------------------------------------------------------------------------------------------------------------
void ips200_showuint16(uint16 x,uint16 y,uint16 dat)
{
	uint8 a[5];
	uint8 i;

	a[0] = dat/10000;
	a[1] = dat/1000%10;
	a[2] = dat/100%10;
	a[3] = dat/10%10;
	a[4] = dat%10;
	
	i = 0;
	while(i<5)
	{
		ips200_showchar((uint8)(x+(8*(i+1))), (uint8)(y*16),(int8)('0' + a[i]));
		i++;
	}
}

//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ32λ�з���(ȥ������������Ч��0)
//  @param      x     	        ����x�������� ������Χ 0 -��IPS200_X_MAX-1��
//  @param      y     	        ����y�������� ������Χ 0 -��IPS200_Y_MAX/16-1��
//  @param      dat       	    ��Ҫ��ʾ�ı�������������uint32
//  @param      num       	    ��Ҫ��ʾ��λ�� ���10λ  ������������
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_showuint32(0,0,x,3);//x����Ϊint32 uint16 int16 uint8 int8����
//  Sample usage:               ��������ʾһ�� ��-����   ������ʾһ���ո�
//-------------------------------------------------------------------------------------------------------------------
void ips200_showint32(uint16 x,uint16 y,int32 dat,uint8 num)
{
    int8    buff[34];
    uint8   length;
    
    if(10<num)      num = 10;
    
    num++;
    if(0>dat)   length = zf_sprintf( &buff[0],"%d",dat);//����
    else
    {
        buff[0] = ' ';
        length = zf_sprintf( &buff[1],"%d",dat);
        length++;
    }
    while(length < num)
    {
        buff[length] = ' ';
        length++;
    }
    buff[num] = '\0';

    ips200_showstr(x, y, buff);	//��ʾ����
}


//-------------------------------------------------------------------------------------------------------------------
//  @brief      Һ����ʾ������(ȥ������������Ч��0)
//  @param      x     	        ����x�������� ������Χ 0 -��IPS200_X_MAX-1��
//  @param      y     	        ����y�������� ������Χ 0 -��IPS200_Y_MAX/16-1��
//  @param      dat       	    ��Ҫ��ʾ�ı�������������float��double
//  @param      num       	    ����λ��ʾ����   ���10λ  
//  @param      pointnum        С��λ��ʾ����   ���6λ
//  @return     void
//  @since      v1.0
//  Sample usage:               ips200_showfloat(0,0,x,2,3);//��ʾ������   ������ʾ2λ   С����ʾ��λ
//  @note                       �ر�ע�⵱����С��������ʾ��ֵ����д���ֵ��һ����ʱ��
//                              ���������ڸ��������ȶ�ʧ���⵼�µģ��Ⲣ������ʾ���������⣬
//                              �й���������飬�����аٶ�ѧϰ   ���������ȶ�ʧ���⡣
//                              ��������ʾһ�� ��-����   ������ʾһ���ո�
//-------------------------------------------------------------------------------------------------------------------
void ips200_showfloat(uint16 x,uint16 y,double dat,uint8 num,uint8 pointnum)
{
    uint8   length;
	int8    buff[34];
	int8    start,end,point;

	if(6<pointnum)  pointnum = 6;
    if(10<num)      num = 10;
        
    if(0>dat)   length = zf_sprintf( &buff[0],"%f",dat);//����
    else
    {
        length = zf_sprintf( &buff[1],"%f",dat);
        length++;
    }
    point = length - 7;         //����С����λ��
    start = point - num - 1;    //������ʼλ
    end = point + pointnum + 1; //�������λ
    while(0>start)//����λ����  ĩβӦ�����ո�
    {
        buff[end] = ' ';
        end++;
        start++;
    }
    
    if(0>dat)   buff[start] = '-';
    else        buff[start] = ' ';
    
    buff[end] = '\0';

    ips200_showstr(x, y, buff);	//��ʾ����
}
