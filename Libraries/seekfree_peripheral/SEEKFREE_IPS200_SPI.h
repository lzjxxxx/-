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
					���ֱ���135*240
					------------------------------------ 
 ********************************************************************************************************************/
 


#ifndef _SEEKFREE_IPS200_H
#define _SEEKFREE_IPS200_H

#include "common.h"
#include "board.h"

//--------------------���SPI--------------------

//���˸Ĺ�����
#define	IPS200_SCL_SIMSPI_PIN 		P25		//����SPI_SCK����
#define	IPS200_SDA_SIMSPI_PIN		P23   	//����SPI_MOSI����
#define IPS200_REST_SIMSPI_PIN  	P20
#define IPS200_DC_SIMSPI_PIN 	 	P21   	//Һ������λ���Ŷ���
#define IPS200_CS_SIMSPI_PIN    	P22   	//����SPI_CS����
#define IPS200_BL_SIMSPI_PIN    	P27     //Һ���������Ŷ��� 
	


#define IPS200_SCL_SIMSPI(x)	   (IPS200_SCL_SIMSPI_PIN = x)
#define IPS200_SDA_SIMSPI(x)	   (IPS200_SDA_SIMSPI_PIN = x)
#define IPS200_REST_SIMSPI(x)      (IPS200_REST_SIMSPI_PIN = x)
#define IPS200_DC_SIMSPI(x)        (IPS200_DC_SIMSPI_PIN = x)
#define IPS200_CS_SIMSPI(x)        (IPS200_CS_SIMSPI_PIN = x)
#define IPS200_BL_SIMSPI(x)        (IPS200_BL_SIMSPI_PIN = x)


//--------------------Ӳ��SPI--------------------


#define IPS200_SPIN_PIN     SPI_CH2           //����ʹ�õ�SPI��
#define	IPS200_SCL_PIN 		SPI_CH2_SCLK_P25	//����SPI_SCK����
#define	IPS200_SDA_PIN		SPI_CH2_MOSI_P23   //����SPI_MOSI����
#define	IPS200_SDA_IN_PIN	SPI_CH2_MISO_P24   //����SPI_MISO����  IPS��Ļû��MISO���ţ�����������Ȼ��Ҫ���壬��spi�ĳ�ʼ��ʱ��Ҫʹ��
#define IPS200_REST_PIN  	P20
#define IPS200_DC_PIN 	 	P21   	//Һ������λ���Ŷ���
#define IPS200_CS_PIN    	P22   	//����SPI_CS����
#define IPS200_BL_PIN    	P27     //Һ���������Ŷ��� 


#define IPS200_REST(x)      (IPS200_REST_PIN = x)
#define IPS200_DC(x)        (IPS200_DC_PIN = x)
#define IPS200_CS(x)        (IPS200_CS_PIN = x)
#define IPS200_BL(x)        (IPS200_BL_PIN = x)

//-----------------------------------------------    


//-------������ɫ��SEEKFREE_FONT.h�ļ��ж���----------
//#define RED          	    0xF800	//��ɫ
//#define BLUE         	    0x001F  //��ɫ
//#define YELLOW       	    0xFFE0	//��ɫ
//#define GREEN        	    0x07E0	//��ɫ
//#define WHITE        	    0xFFFF	//��ɫ
//#define BLACK        	    0x0000	//��ɫ 
//#define GRAY  			0X8430 	//��ɫ
//#define BROWN 			0XBC40 	//��ɫ
//#define PURPLE    		0XF81F	//��ɫ
//#define PINK    		    0XFE19	//��ɫ


//����д�ֱʵ���ɫ
#define IPS200_PENCOLOR    RED

//���屳����ɫ
#define IPS200_BGCOLOR     WHITE



#define IPS200_W   240
#define IPS200_H   320

//������ʾ����
//0 ����ģʽ
//1 ����ģʽ  ��ת180
//2 ����ģʽ
//3 ����ģʽ  ��ת180
#define IPS200_DISPLAY_DIR 0

#if (0==IPS200_DISPLAY_DIR || 1==IPS200_DISPLAY_DIR)
#define	IPS200_X_MAX	IPS200_W	//Һ��X�����
#define IPS200_Y_MAX	IPS200_H    //Һ��Y�����
     
#elif (2==IPS200_DISPLAY_DIR || 3==IPS200_DISPLAY_DIR)
#define	IPS200_X_MAX	IPS200_H	//Һ��X�����
#define IPS200_Y_MAX	IPS200_W    //Һ��Y�����
     
#else
#error "IPS200_DISPLAY_DIR �������"
     
#endif

//--------------------���SPI--------------------

void ips200_init_simspi(void);
void ips200_clear_simspi(uint16 color);
void ips200_drawpoint_simspi(uint16 x,uint16 y,uint16 color);
void ips200_showchar_simspi(uint16 x,uint16 y,const int8 dat);
void ips200_showstr_simspi(uint16 x,uint16 y,const int8 dat[]);
void ips200_showint8_simspi(uint16 x,uint16 y,int8 dat);
void ips200_showuint8_simspi(uint16 x,uint16 y,uint8 dat);
void ips200_showint16_simspi(uint16 x,uint16 y,int16 dat);
void ips200_showuint16_simspi(uint16 x,uint16 y,uint16 dat);
void ips200_showint32_simspi(uint16 x,uint16 y,int32 dat,uint8 num);
void ips200_showfloat_simspi(uint16 x,uint16 y,double dat,uint8 num,uint8 pointnum);

//--------------------Ӳ��SPI--------------------

void ips200_init(void);
void ips200_clear(uint16 color);
void ips200_drawpoint(uint16 x,uint16 y,uint16 color);
void ips200_showchar(uint16 x,uint16 y,const int8 dat);
void ips200_showstr(uint16 x,uint16 y,const int8 dat[]);
void ips200_showint8(uint16 x,uint16 y,int8 dat);
void ips200_showuint8(uint16 x,uint16 y,uint8 dat);
void ips200_showint16(uint16 x,uint16 y,int16 dat);
void ips200_showuint16(uint16 x,uint16 y,uint16 dat);
void ips200_showint32(uint16 x,uint16 y,int32 dat,uint8 num);
void ips200_showfloat(uint16 x,uint16 y,double dat,uint8 num,uint8 pointnum);


//-----------------------------------------------    


#endif
