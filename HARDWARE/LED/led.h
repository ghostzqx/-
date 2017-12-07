#ifndef __LED_H
#define __LED_H
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


//LED�˿ڶ���
#define LED0 PBout(14)	// DS0
#define LED1 PBout(13)	// DS1
#define M8P_RESET PCout(9)	// M8P reset control
#define POWERCTRL PCout(4)	// BoardPower control
#define POWERIN PCin(1)	// BoardPower ̽��

void LED_Init(void);//��ʼ��		 				    
#endif
