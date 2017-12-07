#include "timer.h"
#include "led.h"
#include "usart.h"	
#include "delay.h"
#include "gnss.h"
#include "sim800c.h"
#include "dma.h"
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//定时器 驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2014/5/4
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 
extern OS_FLAG_GRP	TimeupEventFlags;		//定义定时事件标志组


//////////int i,rxlen;
//////////extern u8 timeupflag;//GPRS数据定时处理（上传至各服务器）标志
////////////extern u8 updatastep;//数据定时上传至地图服务器步骤控制
////////////extern u8 rtkstep;//数据定时上传至差分服务器步骤控制
////////////extern nmea_msg gpsx; 											//GPS信息
////////////extern u8 updatareconnect;//数据上传服务器重新连接步骤控制
////////////extern u8 rtkreconnect;//差分服务器重新连接步骤控制
//////////extern u8 GPSGPGGA[200];
//////////extern u8 id[10];
//////////extern u8 dataserverip[30],dataserverport[30],rtkserverip[30],rtkserverport[30];
//////////extern u8 rtkmountpoint[30];
//////////extern u8 userpassencoded[50];
////////////extern u8 connectsta;//数据上传服务器连接状态标志
////////////extern u8 connectsta2;//差分服务器连接状态标志
////////////extern u8 transcount;//用于计数通信模块是否传输正常
////////////extern u16 timex;

////////////extern u8 rtkcount;//用于计数差分数据是否接收正正常
////////////extern u8 rtkntripdone;//用于进行rtkntrip连接后，指示每秒发送GGA语句。
//////////extern u8 atomflag;//用于原子操作
//////////extern u8 sendpoll[10];////用于定时轮询发送
//////////extern statemachine rtklink;
//////////extern statemachine uplink;
////////////定时器7中断服务程序		    
////////////void TIM7_IRQHandler(void)
////////////{ 	
////////////	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)//是更新中断
////////////	{	
////////////		if(GPS_NEW_DATA==0)
////////////		{
////////////			TIM_ClearITPendingBit(TIM7, TIM_IT_Update  );  //清除TIM7更新中断标志    
////////////			TIM_Cmd(TIM7, DISABLE);  //关闭TIM7
////////////			//printf("in TIM7 step1\r\n");
////////////			GPS_NEW_DATA=1;
////////////		}
////////////		else
////////////		{
////////////			//printf("in TIM7 step2\r\n");
////////////		USART6_RX_STA|=1<<15;	//标记接收完成
////////////		TIM_ClearITPendingBit(TIM7, TIM_IT_Update  );  //清除TIM7更新中断标志    
////////////		TIM_Cmd(TIM7, DISABLE);  //关闭TIM7
////////////		rxlen=USART6_RX_STA&0X3FFF;
////////////		for(i=0;i<rxlen;i++)
////////////			{
////////////				GPS_RX_BUF[i]=USART6_RX_BUF[i];
////////////				//USART1_TX_BUF[i]=USART6_RX_BUF[i];
////////////			}
////////////		//USART1_TX_BUF[i]=0;
////////////		GPS_RX_BUF[i]=0;
////////////		GPS_DATA_READY=1;
////////////		}
////////////		//USART1_printf("%s\r\n",USART1_TX_BUF);
////////////			//delay_ms(100);
////////////			//USART1_printf("\032");
////////////			//delay_ms(500);
////////////			//USART1_printf("AT+CIPSEND\r\n");
////////////			//delay_ms(100);
////////////			//GPS_DATA_READY=0;
////////////		//USART6_RX_STA=0;
////////////			//TIM_SetCounter(TIM7,0);
////////////			//USART1_printf("in TIM7,%d\r\n",i++);
////////////	}	    
////////////}
////////////////定时器7中断服务程序		    
//////////////void TIM7_IRQHandler(void)
//////////////{ 	
//////////////	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)//是更新中断
//////////////	{	
////////////////		if(GPS_NEW_DATA==0)
////////////////		{
////////////////			TIM_ClearITPendingBit(TIM7, TIM_IT_Update  );  //清除TIM7更新中断标志    
////////////////			TIM_Cmd(TIM7, DISABLE);  //关闭TIM7
////////////////			//printf("in TIM7 step1\r\n");
////////////////			GPS_NEW_DATA=1;
////////////////		}
////////////////		else
//////////////		{
//////////////			//printf("in TIM7 step2\r\n");
//////////////		USART6_RX_STA|=1<<15;	//标记接收完成
//////////////		TIM_ClearITPendingBit(TIM7, TIM_IT_Update  );  //清除TIM7更新中断标志    
//////////////		TIM_Cmd(TIM7, DISABLE);  //关闭TIM7
//////////////		}
//////////////		//USART1_printf("%s\r\n",USART1_TX_BUF);
//////////////			//delay_ms(100);
//////////////			//USART1_printf("\032");
//////////////			//delay_ms(500);
//////////////			//USART1_printf("AT+CIPSEND\r\n");
//////////////			//delay_ms(100);
//////////////			//GPS_DATA_READY=0;
//////////////		//USART6_RX_STA=0;
//////////////			//TIM_SetCounter(TIM7,0);
//////////////			//USART1_printf("in TIM7,%d\r\n",i++);
//////////////	}	    
//////////////}
////////////定时器3中断服务程序		    
//////////void TIM3_IRQHandler(void)
//////////{ 	
//////////	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)//是更新中断
//////////	{
//////////		TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE ); //使能指定的TIM5中断,不允许更新中断
//////////		
//////////		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIM4更新中断标志    

//////////		TIM_SetCounter(TIM3,0);	//计数器清空			
//////////		TIM_Cmd(TIM3, DISABLE); //关闭定时器6
////////////		atomflag=1;


///////////*			if(1==uplink.reconnectstep)
//////////			{
//////////				//sprintf(USART3_TX_DMABUF,"AT+CIPCLOSE=0\r\n");
//////////				//MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输
//////////				USART3_printf("AT+CIPCLOSE=0\r\n");
//////////				uplink.reconnectstep=2;
//////////				TIM_SetCounter(TIM3,0);	//计数器清空			
//////////				TIM_Cmd(TIM3, ENABLE); //再次使能定时器3，
//////////			}
//////////			else if(2==uplink.reconnectstep)
//////////			{
//////////				uplink.reconnectstep=3;
//////////				//sprintf(USART3_TX_DMABUF,"AT+CIPSTART=0,\"TCP\",\"%s\",\"%s\"\r\n",dataserverip,dataserverport);
//////////				//MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输
//////////				USART3_printf("AT+CIPSTART=0,\"TCP\",\"%s\",\"%s\"\r\n",dataserverip,dataserverport);
//////////				TIM_SetCounter(TIM3,0);	//计数器清空			
//////////				TIM_Cmd(TIM3, ENABLE); //再次使能定时器3，
//////////			}
//////////			else if(3==uplink.reconnectstep)
//////////			{
//////////				uplink.reconnectstep=0;
//////////				TIM_SetCounter(TIM3,0);	//计数器清空			
//////////				TIM_Cmd(TIM3, DISABLE); //关闭定时器6
//////////				uplink.connectsta=0;
//////////				uplink.losscount=0;
//////////				uplink.timex=2;
//////////				atomflag=1;
//////////			}
//////////			else
//////////			{
//////////				uplink.reconnectstep=0;
//////////				TIM_SetCounter(TIM3,0);	//计数器清空			
//////////				TIM_Cmd(TIM3, DISABLE); //关闭定时器6
//////////				atomflag=1;
//////////			}
//////////*/

//////////		TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM5中断,不允许更新中断

//////////	}	    
//////////}


////////////定时器7中断服务程序		    
//////////void TIM7_IRQHandler(void)
//////////{ 	
//////////	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)//是更新中断
//////////	{	
//////////		TIM_ClearITPendingBit(TIM7, TIM_IT_Update  );  //清除TIM7更新中断标志    
//////////		if(1==rtklink.ntripnmeastep)
//////////		{
//////////			sprintf(USART3_TX_DMABUF,"AT+CIPSEND=1\r\n");
//////////			MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输
//////////			//USART3_printf("AT+CIPSEND=1\r\n");
//////////			rtklink.ntripnmeastep=2;
//////////			//TIM_Cmd(TIM7, DISABLE); //再次使能定时器3，
//////////			//TIM_SetAutoreload(TIM7,750);
//////////			TIM_SetCounter(TIM7,0);	//计数器清空			
//////////			TIM_Cmd(TIM7, ENABLE); //再次使能定时器3，
//////////		}
//////////		else if(2==rtklink.ntripnmeastep)
//////////		{
//////////			rtklink.ntripnmeastep=4;
//////////			if(rtkmountpoint!=NULL&&GPSGPGGA!=NULL)
//////////			{
//////////				sprintf(USART3_TX_DMABUF,"GET /%s HTTP/1.1\r\nAccept: rtk/rtcm, dgps/rtcm\r\nUser-Agent: NTRIP Survey-Controller-15.0\r\n\r\n%s\r\n\032",rtkmountpoint,GPSGPGGA);
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输
//////////				//USART3_printf("GET /%s HTTP/1.1\r\nAccept: rtk/rtcm, dgps/rtcm\r\nUser-Agent: NTRIP Survey-Controller-15.0\r\n\r\n%s\r\n",rtkmountpoint,GPSGPGGA);
//////////			}
//////////			else
//////////			{
//////////				sprintf(USART3_TX_DMABUF,"GET /%s HTTP/1.1\r\nAccept: rtk/rtcm, dgps/rtcm\r\nUser-Agent: NTRIP Survey-Controller-15.0\r\n\r\n$GNGGA,000000.00,0.0,N,0.0,E,0,0,255.0,49.0,M,0.0,M,,0000*42\r\n\032",rtkmountpoint);
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输
//////////				//USART3_printf("GET /%s HTTP/1.1\r\nAccept: rtk/rtcm, dgps/rtcm\r\nUser-Agent: NTRIP Survey-Controller-15.0\r\n\r\n$GNGGA,000000.00,0.0,N,0.0,E,0,0,255.0,49.0,M,0.0,M,,0000*42\r\n",rtkmountpoint);
//////////			}
//////////			//TIM_Cmd(TIM7, DISABLE); //再次使能定时器3，
//////////			//TIM_SetAutoreload(TIM7,750);
//////////			TIM_SetCounter(TIM7,0);	//计数器清空			
//////////			TIM_Cmd(TIM7, ENABLE); //再次使能定时器3，
//////////		}
//////////////		else if(3==rtklink.ntripnmeastep)
//////////////		{
//////////////			rtklink.ntripnmeastep=4;
//////////////			sprintf(USART3_TX_DMABUF,"\032");
//////////////			MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输
//////////////			//USART3_printf("\032");
//////////////			
//////////////			//TIM_Cmd(TIM7, DISABLE); //再次使能定时器3，
//////////////			//TIM_SetAutoreload(TIM7,1250);
//////////////			TIM_SetCounter(TIM7,0);	//计数器清空			
//////////////			TIM_Cmd(TIM7, ENABLE); //关闭定时器6
//////////////			//atomflag=1;
//////////////			//USART1_printf("rtd NMEA请求动作完毕..\r\n");
//////////////		}
//////////		else if(4==rtklink.ntripnmeastep)
//////////		{
//////////			rtklink.ntripnmeastep=0;
//////////			//TIM_SetAutoreload(TIM7,500);
//////////			TIM_SetCounter(TIM7,0);	//计数器清空			
//////////			TIM_Cmd(TIM7, DISABLE); //关闭定时器6
//////////			atomflag=1;
//////////			USART1_printf("rtd NMEA请求动作完毕..\r\n");
//////////		}
//////////		else
//////////		{
//////////			rtklink.ntripnmeastep=0;
//////////			//TIM_SetAutoreload(TIM7,500);
//////////			TIM_SetCounter(TIM7,0);	//计数器清空			
//////////			TIM_Cmd(TIM7, DISABLE); //关闭定时器6
//////////			atomflag=1;
//////////		}				
//////////		
//////////		
//////////		////TIM_Cmd(TIM7, DISABLE);  //关闭TIM7
//////////	}	    
//////////}
////////// 
////////////void TIM4_IRQHandler(void)
////////////{
////////////	//USART1_printf("in tim4 RTK_NEW_DATA=%d",RTK_NEW_DATA);
////////////	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//是更新中断
////////////	{	
////////////		if(RTK_NEW_DATA==0)
////////////		{
////////////			TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //清除TIM4更新中断标志    
////////////			TIM_Cmd(TIM4, DISABLE);  //关闭TIM4
////////////			RTK_NEW_DATA=1;
////////////		}
////////////		else
////////////		{
////////////		USART3_RX_STA|=1<<15;	//标记接收完成
////////////		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //清除TIM4更新中断标志    
////////////		TIM_Cmd(TIM4, DISABLE);  //关闭TIM4
////////////		//rxlen=USART3_RX_STA&0X3FFF;
////////////		//USART3_RX_BUF[rxlen]=0;
////////////		//USART1_printf("\r\nRTK test:rxlen=%d,%d\r\n",USART3_RX_STA,rxlen);
////////////		//RTK_RX_BUF[i]=0;
////////////		//USART1_printf("%s\r\n",USART3_RX_BUF);
////////////		RTK_DATA_READY=1;
////////////		}
////////////	}	    
////////////}

////////////void TIM4_IRQHandler(void)
////////////{
////////////	//USART1_printf("in tim4 RTK_NEW_DATA=%d",RTK_NEW_DATA);
////////////	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//是更新中断
////////////	{	
////////////		{
////////////		USART3_RX_STA|=1<<15;	//标记接收完成
////////////		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //清除TIM4更新中断标志    
////////////		TIM_Cmd(TIM4, DISABLE);  //关闭TIM4
////////////		//rxlen=USART3_RX_STA&0X3FFF;
////////////		//USART3_RX_BUF[rxlen]=0;
////////////		//USART1_printf("\r\nRTK test:rxlen=%d,%d\r\n",USART3_RX_STA,rxlen);
////////////		//RTK_RX_BUF[i]=0;
////////////		//USART1_printf("%s\r\n",USART3_RX_BUF);
//////////////		RTK_DATA_READY=1;
////////////		}
////////////	}	    
////////////}

//////////void TIM4_IRQHandler(void)
//////////{
//////////	//USART1_printf("in tim4 RTK_NEW_DATA=%d",RTK_NEW_DATA);
//////////	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//是更新中断
//////////	{
//////////		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //清除TIM4更新中断标志    
//////////		

//////////			if(1==rtklink.ntripauthstep)
//////////			{
//////////				rtklink.ntripauthstep=2;
//////////				sprintf(USART3_TX_DMABUF,"AT+CIPSEND=1\r\n");
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输
//////////				//USART3_printf("AT+CIPSEND=1\r\n");
//////////				//TIM_Cmd(TIM4, DISABLE); //再次使能定时器6
//////////				//TIM_SetAutoreload(TIM4,1499);
//////////				TIM_SetCounter(TIM4,0);	//计数器清空			
//////////				TIM_Cmd(TIM4, ENABLE); //再次使能定时器6
//////////			}
//////////			else if(2==rtklink.ntripauthstep)
//////////			{
//////////				rtklink.ntripauthstep=4;
//////////				sprintf(USART3_TX_DMABUF,"GET /%s HTTP/1.0\r\nUser-Agent: NTRIP GNSSInternetRadio/1.4.10\r\nAuthorization: Basic %s\r\n\r\n\032",rtkmountpoint,userpassencoded);
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输
//////////				//USART3_printf("GET /%s HTTP/1.0\r\nUser-Agent: NTRIP GNSSInternetRadio/1.4.10\r\nAuthorization: Basic %s\r\n\r\n",rtkmountpoint,userpassencoded);
//////////				//TIM_Cmd(TIM4, DISABLE); //再次使能定时器6
//////////				//TIM_SetAutoreload(TIM4,1499);
//////////				TIM_SetCounter(TIM4,0);	//计数器清空			
//////////				TIM_Cmd(TIM4, ENABLE); //再次使能定时器6
//////////			}
//////////////			else if(3==rtklink.ntripauthstep)
//////////////			{
//////////////				rtklink.ntripauthstep=4;
//////////////				sprintf(USART3_TX_DMABUF,"\032");
//////////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输
//////////////				//USART3_printf("\032");
//////////////				//USART1_printf("rtd ntrip连接动作完毕..\r\n");
//////////////				//rtklink.connectsta=STATE_AUTHORIZED;
//////////////				TIM_SetCounter(TIM4,0);	//计数器清空			
//////////////				TIM_Cmd(TIM4, ENABLE); //关闭定时器6
//////////////				//atomflag=1;
//////////////			}
//////////			else if(4==rtklink.ntripauthstep)
//////////			{
//////////				rtklink.ntripauthstep=0;
//////////				//USART3_printf("\032");
//////////				USART1_printf("rtd ntrip连接动作完毕..\r\n");
//////////				rtklink.connectsta=STATE_AUTHORIZED;
//////////				TIM_SetCounter(TIM4,0);	//计数器清空			
//////////				TIM_Cmd(TIM4, DISABLE); //关闭定时器6
//////////				atomflag=1;
//////////			}
//////////			else
//////////			{
//////////				rtklink.ntripauthstep=0;
//////////				TIM_SetCounter(TIM4,0);	//计数器清空			
//////////				TIM_Cmd(TIM4, DISABLE); //关闭定时器6
//////////				atomflag=1;
//////////			}
//////////	}	    
//////////}

//////////void TIM6_DAC_IRQHandler(void)
//////////{
//////////	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)//是更新中断
//////////	{	 			   
//////////			if(1==uplink.updatastep)
//////////			{
//////////				sprintf(USART3_TX_DMABUF,"AT+CIPSEND=0\r\n");
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输

//////////				//USART3_printf("AT+CIPSEND=0\r\n");
//////////				uplink.updatastep=2;
//////////				TIM_Cmd(TIM6, DISABLE);
//////////				TIM_SetAutoreload(TIM6,750-1);
//////////				TIM_SetCounter(TIM6,0);	//计数器清空			
//////////				TIM_Cmd(TIM6, ENABLE); //再次使能定时器3，
//////////			}
//////////			else if(2==uplink.updatastep)
//////////			{
//////////				uplink.updatastep=4;
//////////				if(id!=NULL&&GPSGPGGA!=NULL)
//////////				{
//////////					sprintf(USART3_TX_DMABUF,"%s,%s\r\n\032",id,GPSGPGGA);
//////////					MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输
//////////					//USART3_printf("%s,%s\r\n",id,GPSGPGGA);
//////////				}
//////////				else
//////////				{
//////////					sprintf(USART3_TX_DMABUF,"%s,$GNGGA,000000.00,0.0,N,0.0,E,0,0,255.0,49.0,M,0.0,M,,0000*42\r\n\032",id);
//////////					MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输
//////////					//USART3_printf("%s,$GNGGA,000000.00,0.0,N,0.0,E,0,0,255.0,49.0,M,0.0,M,,0000*42\r\n",id);
//////////				}
//////////				TIM_Cmd(TIM6, DISABLE);
//////////				TIM_SetAutoreload(TIM6,1250-1);
//////////				TIM_SetCounter(TIM6,0);	//计数器清空			
//////////				TIM_Cmd(TIM6, ENABLE); //再次使能定时器3，
//////////			}
//////////////			else if(3==uplink.updatastep)
//////////////			{
//////////////				uplink.updatastep=4;
//////////////				sprintf(USART3_TX_DMABUF,"\032");
//////////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输
//////////////				//USART3_printf("\032");
//////////////				//TIM_Cmd(TIM6, DISABLE);
//////////////				//TIM_SetAutoreload(TIM6,1250-1);
//////////////				TIM_SetCounter(TIM6,0);	//计数器清空			
//////////////				TIM_Cmd(TIM6, ENABLE); //关闭定时器6
//////////////				//TIM_Cmd(TIM6, DISABLE); //关闭定时器6
//////////////				//atomflag=1;
//////////////			}
//////////			else if(4==uplink.updatastep)
//////////			{
//////////				uplink.updatastep=0;
//////////				TIM_SetCounter(TIM6,0);	//计数器清空			
//////////				//TIM_SetAutoreload(TIM6,300-1);
//////////				TIM_Cmd(TIM6, DISABLE); //关闭定时器6
//////////				atomflag=1;
//////////			}
//////////			else
//////////			{
//////////				uplink.updatastep=0;
//////////				TIM_SetCounter(TIM6,0);	//计数器清空			
//////////				//TIM_SetAutoreload(TIM6,300-1);
//////////				TIM_Cmd(TIM6, DISABLE); //关闭定时器6
//////////				atomflag=1;
//////////			}


//////////		TIM_ClearITPendingBit(TIM6, TIM_IT_Update  );  //清除TIM6更新中断标志    
//////////		//TIM_Cmd(TIM6, DISABLE);  //关闭TIM6 
//////////	}	    


//////////}




//////////void TIM5_IRQHandler(void)
//////////{
//////////	int i;
//////////	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)//是更新中断
//////////	{	 			   
//////////		TIM_ClearITPendingBit(TIM5, TIM_IT_Update  );  //清除TIM5更新中断标志    
//////////		if(1==atomflag)//发送原子操作
//////////		{
//////////			if(1==sendpoll[5])//重连操作1，关闭
//////////			{
//////////				sendpoll[5]=0;
//////////				atomflag=0;
//////////				memset(USART3_TX_DMABUF,0,USART_SEND_LEN);
//////////				sprintf(USART3_TX_DMABUF,"AT+CIPCLOSE=0\r\n");
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输
//////////				uplink.connectsta=0;//查询1状态
//////////				uplink.losscount=0;
//////////				uplink.timex=0;
//////////				delay_ms(20);
//////////				atomflag=1;
//////////				
//////////				//TIM_SetCounter(TIM3,0);	//计数器清空			
//////////				//TIM_Cmd(TIM3, ENABLE);
//////////				
//////////////				uplink.reconnectstep=1;
//////////////				TIM_SetCounter(TIM3,0);	//计数器清空			
//////////////				TIM_Cmd(TIM3, ENABLE);

////////////				sim800c_send_cmd("AT+CIPCLOSE=0","CLOSE OK",20);//关闭连接
////////////				USART3_printf("AT+CIPSTART=0,\"TCP\",\"%s\",\"%s\"\r\n",dataserverip,dataserverport);
////////////				uplink.connectsta=0;
////////////				uplink.losscount=0;
////////////				atomflag=1;
//////////				
//////////			}
//////////			else if(1==sendpoll[7])//重连操作2，连接
//////////			{
//////////				sendpoll[7]=0;
//////////				atomflag=0;
//////////				memset(USART3_TX_DMABUF,0,USART_SEND_LEN);
//////////				sprintf(USART3_TX_DMABUF,"AT+CIPSTART=0,\"TCP\",\"%s\",\"%s\"\r\n",dataserverip,dataserverport);
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输
//////////				uplink.connectsta=5;//查询2状态
//////////				uplink.losscount=0;
//////////				uplink.timex=0;
//////////				delay_ms(20);
//////////				atomflag=1;
//////////				//TIM_SetCounter(TIM3,0);	//计数器清空			
//////////				//TIM_Cmd(TIM3, ENABLE);
//////////			}
//////////			else if(1==sendpoll[6])//重连操作1 差分服务器 关闭
//////////			{
//////////				sendpoll[6]=0;
//////////				atomflag=0;

//////////////				sim800c_send_cmd("AT+CIPCLOSE=1","CLOSE OK",5);//关闭连接
//////////////				USART3_printf("AT+CIPSTART=1,\"TCP\",\"%s\",\"%s\"\r\n",rtkserverip,rtkserverport);
//////////////				rtklink.connectsta=STATE_QUERY;
//////////////				rtklink.losscount=0;
//////////////				rtklink.timex=2;
//////////////				atomflag=1;
//////////				
//////////				memset(USART3_TX_DMABUF,0,USART_SEND_LEN);
//////////				sprintf(USART3_TX_DMABUF,"AT+CIPCLOSE=1\r\n");
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输
//////////				//USART3_printf("AT+CIPCLOSE=1\r\n");
//////////				rtklink.connectsta=0;//查询1状态
//////////				rtklink.losscount=0;
//////////				rtklink.timex=2;
//////////				delay_ms(20);
//////////				atomflag=1;
//////////				//TIM_SetCounter(TIM12,0);	//计数器清空			
//////////				//TIM_Cmd(TIM12, ENABLE);
//////////				
////////////				rtklink.reconnectstep=1;
////////////				TIM_SetCounter(TIM12,0);	//计数器清空			
////////////				TIM_Cmd(TIM12, ENABLE);
//////////			}
//////////			else if(1==sendpoll[8])//重连操作2 差分服务器 重新连接
//////////			{
//////////				sendpoll[8]=0;
//////////				atomflag=0;
//////////				memset(USART3_TX_DMABUF,0,USART_SEND_LEN);
//////////				sprintf(USART3_TX_DMABUF,"AT+CIPSTART=1,\"TCP\",\"%s\",\"%s\"\r\n",rtkserverip,rtkserverport);
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输
//////////				//USART3_printf("AT+CIPSTART=1,\"TCP\",\"%s\",\"%s\"\r\n",rtkserverip,rtkserverport);
//////////				rtklink.connectsta=5;//查询5状态
//////////				rtklink.losscount=0;
//////////				rtklink.timex=2;
//////////				delay_ms(20);
//////////				atomflag=1;
//////////				//TIM_SetCounter(TIM12,0);	//计数器清空			
//////////				//TIM_Cmd(TIM12, ENABLE);
//////////			}
//////////			else if(1==sendpoll[0])//数据上传
//////////			{
//////////				sendpoll[0]=0;
//////////				atomflag=0;
//////////				uplink.updatastep=1;
//////////				TIM_Cmd(TIM6, DISABLE);
//////////				TIM_SetAutoreload(TIM6,150-1);
//////////				TIM_SetCounter(TIM6,0);	//计数器清空			
//////////				TIM_Cmd(TIM6, ENABLE);
//////////			}
//////////			else if(1==sendpoll[1])//发送NTRIP认证请求
//////////			{
//////////				sendpoll[1]=0;
//////////				atomflag=0;
//////////				rtklink.ntripauthstep=1;
//////////				//TIM_Cmd(TIM4, DISABLE);
//////////				//TIM_SetAutoreload(TIM4,30000-1);
//////////				TIM_SetCounter(TIM4,0);	//计数器清空			
//////////				TIM_Cmd(TIM4, ENABLE);
//////////			}
//////////			else if(1==sendpoll[2])//发送NMEA请求
//////////			{
//////////				sendpoll[2]=0;
//////////				atomflag=0;
//////////				rtklink.ntripnmeastep=1;
//////////				TIM_SetCounter(TIM7,0);	//计数器清空			
//////////				TIM_Cmd(TIM7, ENABLE);
//////////			}
//////////			else if(1==sendpoll[3])//发送IPSTATUS=0
//////////			{
//////////				sendpoll[3]=0;
//////////				atomflag=0;
//////////				sprintf(USART3_TX_DMABUF,"AT+CIPSTATUS=0\r\n");
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输
//////////				atomflag=1;
//////////				//TIM_SetCounter(TIM3,0);	//计数器清空			
//////////				//TIM_Cmd(TIM3, ENABLE);
//////////			}
//////////			else if(1==sendpoll[4])//发送IPSTATUS=1
//////////			{
//////////				sendpoll[4]=0;
//////////				atomflag=0;
//////////				sprintf(USART3_TX_DMABUF,"AT+CIPSTATUS=1\r\n");
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//开始一次DMA传输
//////////				atomflag=1;
//////////				//TIM_SetCounter(TIM12,0);	//计数器清空			
//////////				//TIM_Cmd(TIM12, ENABLE);
//////////			}
//////////		
//////////		}
//////////	}	    
//////////}

//////////void TIM8_BRK_TIM12_IRQHandler(void)
//////////{
//////////	int i;
//////////	if (TIM_GetITStatus(TIM12, TIM_IT_Update) != RESET)//是更新中断
//////////	{	 	
//////////		TIM_ClearITPendingBit(TIM12, TIM_IT_Update  );  //清除TIM5更新中断标志    
//////////		
//////////////		TIM_ITConfig(TIM12,TIM_IT_Update,DISABLE ); //使能指定的TIM5中断,不允许更新中断

//////////////			TIM_SetCounter(TIM12,0);	//计数器清空			
//////////////			TIM_Cmd(TIM12, DISABLE); //关闭定时器6
//////////////			atomflag=1;
//////////////		TIM_ITConfig(TIM12,TIM_IT_Update,ENABLE ); //使能指定的TIM5中断,不允许更新中断

//////////		if(1==rtklink.reconnectstep)
//////////		{
//////////			USART3_printf("AT+CIPCLOSE=1\r\n");
//////////			rtklink.reconnectstep=2;
//////////			TIM_SetCounter(TIM12,0);	//计数器清空			
//////////			TIM_Cmd(TIM12, ENABLE); //再次使能定时器3，
//////////		}
//////////		else if(2==rtklink.reconnectstep)
//////////		{
//////////			rtklink.reconnectstep=3;
//////////			USART3_printf("AT+CIPSTART=1,\"TCP\",\"%s\",\"%s\"\r\n",rtkserverip,rtkserverport);
//////////			TIM_SetCounter(TIM12,0);	//计数器清空			
//////////			TIM_Cmd(TIM12, ENABLE); //再次使能定时器3，
//////////		}
//////////		else if(3==rtklink.reconnectstep)
//////////		{
//////////			rtklink.reconnectstep=0;
//////////			TIM_SetCounter(TIM12,0);	//计数器清空			
//////////			TIM_Cmd(TIM12, DISABLE); //关闭定时器6
//////////			rtklink.connectsta=0;
//////////			rtklink.losscount=0;
//////////			rtklink.timex=2;
//////////			atomflag=1;
//////////		}
//////////		else
//////////		{
//////////			rtklink.reconnectstep=0;
//////////			TIM_SetCounter(TIM12,0);	//计数器清空			
//////////			TIM_Cmd(TIM12, DISABLE); //关闭定时器6
//////////			atomflag=1;
//////////		}

//////////		
//////////	}	    
//////////}



////////////通用定时器7中断初始化
////////////这里时钟选择为APB1的2倍，而APB1为42M
////////////arr：自动重装值。
////////////psc：时钟预分频数
////////////定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
////////////Ft=定时器工作频率,单位:Mhz 
////////////通用定时器中断初始化
////////////这里始终选择为APB1的2倍，而APB1为36M
////////////arr：自动重装值。
////////////psc：时钟预分频数		 
//////////void TIM7_Int_Init(u16 arr,u16 psc)
//////////{	
//////////	NVIC_InitTypeDef NVIC_InitStructure;
//////////	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//////////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);//TIM7时钟使能    
//////////	
//////////	//定时器TIM7初始化
//////////	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
//////////	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
//////////	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
//////////	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
//////////	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
////////// 
//////////	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE ); //使能指定的TIM7中断,允许更新中断
//////////	
//////////	//TIM_Cmd(TIM7,ENABLE);//开启定时器7
//////////	
//////////	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
//////////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级0
//////////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级2
//////////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//////////	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
//////////	
//////////}

//////////void TIM4_Int_Init(u16 arr,u16 psc)
//////////{	
//////////	NVIC_InitTypeDef NVIC_InitStructure;
//////////	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//////////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//TIM4时钟使能    
//////////	
//////////	//定时器TIM4初始化
//////////	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
//////////	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
//////////	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
//////////	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
//////////	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
////////// 
//////////	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //使能指定的TIM4中断,允许更新中断
//////////	
//////////	//TIM_Cmd(TIM4,ENABLE);//开启定时器7
//////////	
//////////	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
//////////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级0
//////////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//子优先级2
//////////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//////////	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
//////////	
//////////}
////////////通用定时器中断初始化
////////////这里始终选择为APB1的2倍，而APB1为42M
////////////arr：自动重装值。
////////////psc：时钟预分频数		 
//////////void TIM6_Int_Init(u16 arr,u16 psc)
//////////{	
//////////	NVIC_InitTypeDef NVIC_InitStructure;
//////////	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//////////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);//TIM6时钟使能    
//////////	
//////////	//定时器TIM6初始化
//////////	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
//////////	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
//////////	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
//////////	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
//////////	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
////////// 
//////////	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE ); //使能指定的TIM6中断,允许更新中断
//////////   
////////////	TIM_Cmd(TIM6,ENABLE);//使能定时器6
//////////	
//////////	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
//////////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级0
//////////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级1
//////////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//////////	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
//////////	
//////////}


////////////通用定时器5中断初始化
////////////这里始终选择为APB1的2倍，而APB1为42M，定时时间=(arr+1)*(psc+1)/84 us
////////////arr：自动重装值。定时器2是32位的arr
////////////psc：时钟预分频数		 
//////////void TIM5_Int_Init(u32 arr,u16 psc)
//////////{	
//////////	NVIC_InitTypeDef NVIC_InitStructure;
//////////	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//////////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);//TIM5时钟使能    
//////////	
//////////	//定时器TIM5初始化
//////////	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
//////////	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
//////////	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
//////////	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
//////////	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
////////// 
//////////	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE ); //使能指定的TIM5中断,允许更新中断
//////////   
//////////	TIM_Cmd(TIM5,ENABLE);//使能定时器5
//////////	
//////////	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
//////////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级0
//////////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级1
//////////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//////////	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
//////////	
//////////}

////////////通用定时器中断初始化
////////////这里始终选择为APB1的2倍，而APB1为36M
////////////arr：自动重装值。
////////////psc：时钟预分频数		 
//////////void TIM3_Int_Init(u16 arr,u16 psc)
//////////{	
//////////	NVIC_InitTypeDef NVIC_InitStructure;
//////////	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//////////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//TIM5时钟使能    
//////////	
//////////	//定时器TIM7初始化
//////////	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
//////////	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
//////////	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
//////////	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
//////////	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
////////// 
//////////	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM4中断,允许更新中断
//////////   
//////////	TIM_Cmd(TIM3,ENABLE);//使能定时器4
//////////	
//////////	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//////////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级0
//////////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级1
//////////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//////////	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
//////////	
//////////}


////////////通用定时器中断初始化
////////////这里始终选择为APB1的2倍，而APB1为36M
////////////arr：自动重装值。
////////////psc：时钟预分频数		 
//////////void TIM12_Int_Init(u16 arr,u16 psc)
//////////{	
//////////	NVIC_InitTypeDef NVIC_InitStructure;
//////////	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//////////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);//TIM5时钟使能    
//////////	
//////////	//定时器TIM7初始化
//////////	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
//////////	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
//////////	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
//////////	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
//////////	TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
////////// 
//////////	TIM_ITConfig(TIM12,TIM_IT_Update,ENABLE ); //使能指定的TIM4中断,允许更新中断
//////////   
//////////	//TIM_Cmd(TIM12,ENABLE);//使能定时器4
//////////	
//////////	NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
//////////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级0
//////////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级1
//////////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//////////	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
//////////	
//////////}

//通用定时器2中断初始化
//这里始终选择为APB1的2倍，而APB1为42M，定时时间=(arr+1)*(psc+1)/84 us
//arr：自动重装值。定时器2是32位的arr
//psc：时钟预分频数		 
void TIM2_Int_Init(u32 arr,u16 psc)
{	
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//TIM2时钟使能    
	
	//定时器TIM2初始化
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //根据指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //使能指定的TIM2中断,允许更新中断
   
	TIM_Cmd(TIM2,ENABLE);//使能定时器2
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//抢占优先级0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//子优先级1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
	
}

void TIM2_IRQHandler(void)
{
	OS_ERR err;
	
#ifdef SYSTEM_SUPPORT_OS	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntEnter();    
#endif
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//是更新中断
	{	 			   
		//timeupflag=1;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //清除TIM2更新中断标志    
		//TIM_Cmd(TIM2, DISABLE);  //关闭TIM2 
		OSFlagPost((OS_FLAG_GRP*)&TimeupEventFlags,
								 (OS_FLAGS	  )0xff,
								 (OS_OPT	  )OS_OPT_POST_FLAG_SET,
								 (OS_ERR*     )&err);

	}	    

#ifdef SYSTEM_SUPPORT_OS	 	//如果时钟节拍数定义了,说明要使用ucosII了.
	OSIntExit();  											 
#endif

}


