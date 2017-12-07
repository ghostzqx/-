#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "stmflash.h"
#include "iap.h" 
#include "core_cm4.h"
#include "stdlib.h"
#include "rt_heap.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//IAP 代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/7/21
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 

iapfun jump2app; 
//u32 iapbuf[512]; 	//2K字节缓存 
//appxaddr:应用程序的起始地址
//appbuf:应用程序CODE.
//appsize:应用程序大小(字节).
void iap_write_appbin(u32 appxaddr,u8 *appbuf,u32 appsize)
{
	u32 t;
	u32 iapbuf[512]; 	//2K字节缓存 
	u16 i=0;
	u32 temp;
	u32 fwaddr=appxaddr;//当前写入的地址
	u8 *dfu=appbuf;
	if(iapbuf==NULL)
		return;
	for(t=0;t<appsize;t+=4)
	{
		temp=(u32)dfu[3]<<24;
		temp|=(u32)dfu[2]<<16;
		temp|=(u32)dfu[1]<<8;
		temp|=(u32)dfu[0]; 
		dfu+=4;//偏移4个字节
		iapbuf[i++]=temp;
		if(i==512)
		{
			i=0; 
			STMFLASH_Write(fwaddr,iapbuf,512);
			fwaddr+=2048;//偏移2048  512*4=2048
		}
	} 
	if(i)STMFLASH_Write(fwaddr,iapbuf,i);//将最后的一些内容字节写进去.  
}

//跳转到应用程序段
//appxaddr:用户代码起始地址.
void iap_load_app(u32 appxaddr)
{
	RCC_DeInit();//
	////NVIC_DeInit();//看看是不是做循环把nvic的都禁止了?
	__set_PRIMASK(1);//关中断
	__disable_irq(); //关中断；如果APP没有用ucos，初始化后要开中断；用ucos，在启动任务后会开中断
	__set_CONTROL(0);
	if(((*(vu32*)appxaddr)&0x2FFE0000)==0x20000000)	//检查栈顶地址是否合法.
	{ 
		jump2app=(iapfun)*(vu32*)(appxaddr+4);		//用户代码区第二个字为程序开始地址(复位地址)		
		MSR_MSP(*(vu32*)appxaddr);					//初始化APP堆栈指针(用户代码区的第一个字用于存放栈顶地址)
		jump2app();									//跳转到APP.
	}
}

/***********************************************************************
函数功能：跳转到IAP函数
***********************************************************************/
void app_load_iap(u32 appxaddr)
{
	vu32 IapSpInitVal,IapJumpAddr;
	void (*pIapFun)(void);
	RCC_DeInit();
	//NVIC_DeInit();
	__set_PRIMASK(1);//关中断
	__disable_irq(); //关中断（）
		//补充将所有已产生的中断清除的指令
	// APP如跳转前关中断，跳转到IAP后，IAP初始化后要打开中断
	IapSpInitVal = *(vu32 *)appxaddr;
	IapJumpAddr = *(vu32 *)(appxaddr + 4);
	__set_CONTROL(0);
	//进入用户级线程模式 进入软中断后才可以回到特权级线程模式
	//APP如使用系统如ucos必须要有此过程否则跳到IAP后，无法再次跳到APP
	__set_MSP (IapSpInitVal);
	pIapFun = (void (*)(void))IapJumpAddr;
	(*pIapFun) ();
}












