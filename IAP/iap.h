#ifndef __IAP_H__
#define __IAP_H__
#include "sys.h"  
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//IAP ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/7/21
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
//////////////////////////////////////////////////////////////////////////////////    
typedef  void (*iapfun)(void);				//����һ���������͵Ĳ���.   
#define FLASH_BOOT_ADDR		(FLASH_BASE)  	//��һ��Ӧ�ó�����ʼ��ַ(�����FLASH)��4 5 6����
#define FLASH_APP1_ADDR		(FLASH_BASE+0x10000)  	//��һ��Ӧ�ó�����ʼ��ַ(�����FLASH)��4 5 6����
#define FLASH_APP2_ADDR		(FLASH_BASE+0x60000)  	//�ڶ���Ӧ�ó�����ʼ��ַ(�����FLASH)��7 8 9����
#define FLASH_APP_SWITCH_ADDR		(FLASH_BASE+0xC0000)  	//�ж��Ƿ���г��������л�(�����FLASH)
#define FLASH_APP1_SECTOR_LENGTH		(0x5ffff)  	//��һ��Ӧ�ó�����ʼ��ַ(�����FLASH)��320KB
#define FLASH_APP2_SECTOR_LENGTH		(0x5ffff)  	//��һ��Ӧ�ó�����ʼ��ַ(�����FLASH)��320KB
											//����0X08000000~0X0800FFFF�Ŀռ�ΪBootloaderʹ��(��64KB)	   
void iap_load_app(u32 appxaddr);			//��ת��APP����ִ��
void iap_write_appbin(u32 appxaddr,u8 *appbuf,u32 applen);	//��ָ����ַ��ʼ,д��bin
void app_load_iap(u32 appxaddr);
#endif







































