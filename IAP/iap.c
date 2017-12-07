#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "stmflash.h"
#include "iap.h" 
#include "core_cm4.h"
#include "stdlib.h"
#include "rt_heap.h"

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

iapfun jump2app; 
//u32 iapbuf[512]; 	//2K�ֽڻ��� 
//appxaddr:Ӧ�ó������ʼ��ַ
//appbuf:Ӧ�ó���CODE.
//appsize:Ӧ�ó����С(�ֽ�).
void iap_write_appbin(u32 appxaddr,u8 *appbuf,u32 appsize)
{
	u32 t;
	u32 iapbuf[512]; 	//2K�ֽڻ��� 
	u16 i=0;
	u32 temp;
	u32 fwaddr=appxaddr;//��ǰд��ĵ�ַ
	u8 *dfu=appbuf;
	if(iapbuf==NULL)
		return;
	for(t=0;t<appsize;t+=4)
	{
		temp=(u32)dfu[3]<<24;
		temp|=(u32)dfu[2]<<16;
		temp|=(u32)dfu[1]<<8;
		temp|=(u32)dfu[0]; 
		dfu+=4;//ƫ��4���ֽ�
		iapbuf[i++]=temp;
		if(i==512)
		{
			i=0; 
			STMFLASH_Write(fwaddr,iapbuf,512);
			fwaddr+=2048;//ƫ��2048  512*4=2048
		}
	} 
	if(i)STMFLASH_Write(fwaddr,iapbuf,i);//������һЩ�����ֽ�д��ȥ.  
}

//��ת��Ӧ�ó����
//appxaddr:�û�������ʼ��ַ.
void iap_load_app(u32 appxaddr)
{
	RCC_DeInit();//
	////NVIC_DeInit();//�����ǲ�����ѭ����nvic�Ķ���ֹ��?
	__set_PRIMASK(1);//���ж�
	__disable_irq(); //���жϣ����APPû����ucos����ʼ����Ҫ���жϣ���ucos�������������Ὺ�ж�
	__set_CONTROL(0);
	if(((*(vu32*)appxaddr)&0x2FFE0000)==0x20000000)	//���ջ����ַ�Ƿ�Ϸ�.
	{ 
		jump2app=(iapfun)*(vu32*)(appxaddr+4);		//�û��������ڶ�����Ϊ����ʼ��ַ(��λ��ַ)		
		MSR_MSP(*(vu32*)appxaddr);					//��ʼ��APP��ջָ��(�û��������ĵ�һ�������ڴ��ջ����ַ)
		jump2app();									//��ת��APP.
	}
}

/***********************************************************************
�������ܣ���ת��IAP����
***********************************************************************/
void app_load_iap(u32 appxaddr)
{
	vu32 IapSpInitVal,IapJumpAddr;
	void (*pIapFun)(void);
	RCC_DeInit();
	//NVIC_DeInit();
	__set_PRIMASK(1);//���ж�
	__disable_irq(); //���жϣ���
		//���佫�����Ѳ������ж������ָ��
	// APP����תǰ���жϣ���ת��IAP��IAP��ʼ����Ҫ���ж�
	IapSpInitVal = *(vu32 *)appxaddr;
	IapJumpAddr = *(vu32 *)(appxaddr + 4);
	__set_CONTROL(0);
	//�����û����߳�ģʽ �������жϺ�ſ��Իص���Ȩ���߳�ģʽ
	//APP��ʹ��ϵͳ��ucos����Ҫ�д˹��̷�������IAP���޷��ٴ�����APP
	__set_MSP (IapSpInitVal);
	pIapFun = (void (*)(void))IapJumpAddr;
	(*pIapFun) ();
}












