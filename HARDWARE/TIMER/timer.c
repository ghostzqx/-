#include "timer.h"
#include "led.h"
#include "usart.h"	
#include "delay.h"
#include "gnss.h"
#include "sim800c.h"
#include "dma.h"
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//��ʱ�� ��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/4
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 
extern OS_FLAG_GRP	TimeupEventFlags;		//���嶨ʱ�¼���־��


//////////int i,rxlen;
//////////extern u8 timeupflag;//GPRS���ݶ�ʱ�����ϴ���������������־
////////////extern u8 updatastep;//���ݶ�ʱ�ϴ�����ͼ�������������
////////////extern u8 rtkstep;//���ݶ�ʱ�ϴ�����ַ������������
////////////extern nmea_msg gpsx; 											//GPS��Ϣ
////////////extern u8 updatareconnect;//�����ϴ��������������Ӳ������
////////////extern u8 rtkreconnect;//��ַ������������Ӳ������
//////////extern u8 GPSGPGGA[200];
//////////extern u8 id[10];
//////////extern u8 dataserverip[30],dataserverport[30],rtkserverip[30],rtkserverport[30];
//////////extern u8 rtkmountpoint[30];
//////////extern u8 userpassencoded[50];
////////////extern u8 connectsta;//�����ϴ�����������״̬��־
////////////extern u8 connectsta2;//��ַ���������״̬��־
////////////extern u8 transcount;//���ڼ���ͨ��ģ���Ƿ�������
////////////extern u16 timex;

////////////extern u8 rtkcount;//���ڼ�����������Ƿ����������
////////////extern u8 rtkntripdone;//���ڽ���rtkntrip���Ӻ�ָʾÿ�뷢��GGA��䡣
//////////extern u8 atomflag;//����ԭ�Ӳ���
//////////extern u8 sendpoll[10];////���ڶ�ʱ��ѯ����
//////////extern statemachine rtklink;
//////////extern statemachine uplink;
////////////��ʱ��7�жϷ������		    
////////////void TIM7_IRQHandler(void)
////////////{ 	
////////////	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)//�Ǹ����ж�
////////////	{	
////////////		if(GPS_NEW_DATA==0)
////////////		{
////////////			TIM_ClearITPendingBit(TIM7, TIM_IT_Update  );  //���TIM7�����жϱ�־    
////////////			TIM_Cmd(TIM7, DISABLE);  //�ر�TIM7
////////////			//printf("in TIM7 step1\r\n");
////////////			GPS_NEW_DATA=1;
////////////		}
////////////		else
////////////		{
////////////			//printf("in TIM7 step2\r\n");
////////////		USART6_RX_STA|=1<<15;	//��ǽ������
////////////		TIM_ClearITPendingBit(TIM7, TIM_IT_Update  );  //���TIM7�����жϱ�־    
////////////		TIM_Cmd(TIM7, DISABLE);  //�ر�TIM7
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
////////////////��ʱ��7�жϷ������		    
//////////////void TIM7_IRQHandler(void)
//////////////{ 	
//////////////	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)//�Ǹ����ж�
//////////////	{	
////////////////		if(GPS_NEW_DATA==0)
////////////////		{
////////////////			TIM_ClearITPendingBit(TIM7, TIM_IT_Update  );  //���TIM7�����жϱ�־    
////////////////			TIM_Cmd(TIM7, DISABLE);  //�ر�TIM7
////////////////			//printf("in TIM7 step1\r\n");
////////////////			GPS_NEW_DATA=1;
////////////////		}
////////////////		else
//////////////		{
//////////////			//printf("in TIM7 step2\r\n");
//////////////		USART6_RX_STA|=1<<15;	//��ǽ������
//////////////		TIM_ClearITPendingBit(TIM7, TIM_IT_Update  );  //���TIM7�����жϱ�־    
//////////////		TIM_Cmd(TIM7, DISABLE);  //�ر�TIM7
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
////////////��ʱ��3�жϷ������		    
//////////void TIM3_IRQHandler(void)
//////////{ 	
//////////	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET)//�Ǹ����ж�
//////////	{
//////////		TIM_ITConfig(TIM3,TIM_IT_Update,DISABLE ); //ʹ��ָ����TIM5�ж�,����������ж�
//////////		
//////////		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //���TIM4�����жϱ�־    

//////////		TIM_SetCounter(TIM3,0);	//���������			
//////////		TIM_Cmd(TIM3, DISABLE); //�رն�ʱ��6
////////////		atomflag=1;


///////////*			if(1==uplink.reconnectstep)
//////////			{
//////////				//sprintf(USART3_TX_DMABUF,"AT+CIPCLOSE=0\r\n");
//////////				//MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����
//////////				USART3_printf("AT+CIPCLOSE=0\r\n");
//////////				uplink.reconnectstep=2;
//////////				TIM_SetCounter(TIM3,0);	//���������			
//////////				TIM_Cmd(TIM3, ENABLE); //�ٴ�ʹ�ܶ�ʱ��3��
//////////			}
//////////			else if(2==uplink.reconnectstep)
//////////			{
//////////				uplink.reconnectstep=3;
//////////				//sprintf(USART3_TX_DMABUF,"AT+CIPSTART=0,\"TCP\",\"%s\",\"%s\"\r\n",dataserverip,dataserverport);
//////////				//MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����
//////////				USART3_printf("AT+CIPSTART=0,\"TCP\",\"%s\",\"%s\"\r\n",dataserverip,dataserverport);
//////////				TIM_SetCounter(TIM3,0);	//���������			
//////////				TIM_Cmd(TIM3, ENABLE); //�ٴ�ʹ�ܶ�ʱ��3��
//////////			}
//////////			else if(3==uplink.reconnectstep)
//////////			{
//////////				uplink.reconnectstep=0;
//////////				TIM_SetCounter(TIM3,0);	//���������			
//////////				TIM_Cmd(TIM3, DISABLE); //�رն�ʱ��6
//////////				uplink.connectsta=0;
//////////				uplink.losscount=0;
//////////				uplink.timex=2;
//////////				atomflag=1;
//////////			}
//////////			else
//////////			{
//////////				uplink.reconnectstep=0;
//////////				TIM_SetCounter(TIM3,0);	//���������			
//////////				TIM_Cmd(TIM3, DISABLE); //�رն�ʱ��6
//////////				atomflag=1;
//////////			}
//////////*/

//////////		TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM5�ж�,����������ж�

//////////	}	    
//////////}


////////////��ʱ��7�жϷ������		    
//////////void TIM7_IRQHandler(void)
//////////{ 	
//////////	if (TIM_GetITStatus(TIM7, TIM_IT_Update) != RESET)//�Ǹ����ж�
//////////	{	
//////////		TIM_ClearITPendingBit(TIM7, TIM_IT_Update  );  //���TIM7�����жϱ�־    
//////////		if(1==rtklink.ntripnmeastep)
//////////		{
//////////			sprintf(USART3_TX_DMABUF,"AT+CIPSEND=1\r\n");
//////////			MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����
//////////			//USART3_printf("AT+CIPSEND=1\r\n");
//////////			rtklink.ntripnmeastep=2;
//////////			//TIM_Cmd(TIM7, DISABLE); //�ٴ�ʹ�ܶ�ʱ��3��
//////////			//TIM_SetAutoreload(TIM7,750);
//////////			TIM_SetCounter(TIM7,0);	//���������			
//////////			TIM_Cmd(TIM7, ENABLE); //�ٴ�ʹ�ܶ�ʱ��3��
//////////		}
//////////		else if(2==rtklink.ntripnmeastep)
//////////		{
//////////			rtklink.ntripnmeastep=4;
//////////			if(rtkmountpoint!=NULL&&GPSGPGGA!=NULL)
//////////			{
//////////				sprintf(USART3_TX_DMABUF,"GET /%s HTTP/1.1\r\nAccept: rtk/rtcm, dgps/rtcm\r\nUser-Agent: NTRIP Survey-Controller-15.0\r\n\r\n%s\r\n\032",rtkmountpoint,GPSGPGGA);
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����
//////////				//USART3_printf("GET /%s HTTP/1.1\r\nAccept: rtk/rtcm, dgps/rtcm\r\nUser-Agent: NTRIP Survey-Controller-15.0\r\n\r\n%s\r\n",rtkmountpoint,GPSGPGGA);
//////////			}
//////////			else
//////////			{
//////////				sprintf(USART3_TX_DMABUF,"GET /%s HTTP/1.1\r\nAccept: rtk/rtcm, dgps/rtcm\r\nUser-Agent: NTRIP Survey-Controller-15.0\r\n\r\n$GNGGA,000000.00,0.0,N,0.0,E,0,0,255.0,49.0,M,0.0,M,,0000*42\r\n\032",rtkmountpoint);
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����
//////////				//USART3_printf("GET /%s HTTP/1.1\r\nAccept: rtk/rtcm, dgps/rtcm\r\nUser-Agent: NTRIP Survey-Controller-15.0\r\n\r\n$GNGGA,000000.00,0.0,N,0.0,E,0,0,255.0,49.0,M,0.0,M,,0000*42\r\n",rtkmountpoint);
//////////			}
//////////			//TIM_Cmd(TIM7, DISABLE); //�ٴ�ʹ�ܶ�ʱ��3��
//////////			//TIM_SetAutoreload(TIM7,750);
//////////			TIM_SetCounter(TIM7,0);	//���������			
//////////			TIM_Cmd(TIM7, ENABLE); //�ٴ�ʹ�ܶ�ʱ��3��
//////////		}
//////////////		else if(3==rtklink.ntripnmeastep)
//////////////		{
//////////////			rtklink.ntripnmeastep=4;
//////////////			sprintf(USART3_TX_DMABUF,"\032");
//////////////			MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����
//////////////			//USART3_printf("\032");
//////////////			
//////////////			//TIM_Cmd(TIM7, DISABLE); //�ٴ�ʹ�ܶ�ʱ��3��
//////////////			//TIM_SetAutoreload(TIM7,1250);
//////////////			TIM_SetCounter(TIM7,0);	//���������			
//////////////			TIM_Cmd(TIM7, ENABLE); //�رն�ʱ��6
//////////////			//atomflag=1;
//////////////			//USART1_printf("rtd NMEA���������..\r\n");
//////////////		}
//////////		else if(4==rtklink.ntripnmeastep)
//////////		{
//////////			rtklink.ntripnmeastep=0;
//////////			//TIM_SetAutoreload(TIM7,500);
//////////			TIM_SetCounter(TIM7,0);	//���������			
//////////			TIM_Cmd(TIM7, DISABLE); //�رն�ʱ��6
//////////			atomflag=1;
//////////			USART1_printf("rtd NMEA���������..\r\n");
//////////		}
//////////		else
//////////		{
//////////			rtklink.ntripnmeastep=0;
//////////			//TIM_SetAutoreload(TIM7,500);
//////////			TIM_SetCounter(TIM7,0);	//���������			
//////////			TIM_Cmd(TIM7, DISABLE); //�رն�ʱ��6
//////////			atomflag=1;
//////////		}				
//////////		
//////////		
//////////		////TIM_Cmd(TIM7, DISABLE);  //�ر�TIM7
//////////	}	    
//////////}
////////// 
////////////void TIM4_IRQHandler(void)
////////////{
////////////	//USART1_printf("in tim4 RTK_NEW_DATA=%d",RTK_NEW_DATA);
////////////	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//�Ǹ����ж�
////////////	{	
////////////		if(RTK_NEW_DATA==0)
////////////		{
////////////			TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //���TIM4�����жϱ�־    
////////////			TIM_Cmd(TIM4, DISABLE);  //�ر�TIM4
////////////			RTK_NEW_DATA=1;
////////////		}
////////////		else
////////////		{
////////////		USART3_RX_STA|=1<<15;	//��ǽ������
////////////		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //���TIM4�����жϱ�־    
////////////		TIM_Cmd(TIM4, DISABLE);  //�ر�TIM4
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
////////////	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//�Ǹ����ж�
////////////	{	
////////////		{
////////////		USART3_RX_STA|=1<<15;	//��ǽ������
////////////		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //���TIM4�����жϱ�־    
////////////		TIM_Cmd(TIM4, DISABLE);  //�ر�TIM4
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
//////////	if (TIM_GetITStatus(TIM4, TIM_IT_Update) != RESET)//�Ǹ����ж�
//////////	{
//////////		TIM_ClearITPendingBit(TIM4, TIM_IT_Update  );  //���TIM4�����жϱ�־    
//////////		

//////////			if(1==rtklink.ntripauthstep)
//////////			{
//////////				rtklink.ntripauthstep=2;
//////////				sprintf(USART3_TX_DMABUF,"AT+CIPSEND=1\r\n");
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����
//////////				//USART3_printf("AT+CIPSEND=1\r\n");
//////////				//TIM_Cmd(TIM4, DISABLE); //�ٴ�ʹ�ܶ�ʱ��6
//////////				//TIM_SetAutoreload(TIM4,1499);
//////////				TIM_SetCounter(TIM4,0);	//���������			
//////////				TIM_Cmd(TIM4, ENABLE); //�ٴ�ʹ�ܶ�ʱ��6
//////////			}
//////////			else if(2==rtklink.ntripauthstep)
//////////			{
//////////				rtklink.ntripauthstep=4;
//////////				sprintf(USART3_TX_DMABUF,"GET /%s HTTP/1.0\r\nUser-Agent: NTRIP GNSSInternetRadio/1.4.10\r\nAuthorization: Basic %s\r\n\r\n\032",rtkmountpoint,userpassencoded);
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����
//////////				//USART3_printf("GET /%s HTTP/1.0\r\nUser-Agent: NTRIP GNSSInternetRadio/1.4.10\r\nAuthorization: Basic %s\r\n\r\n",rtkmountpoint,userpassencoded);
//////////				//TIM_Cmd(TIM4, DISABLE); //�ٴ�ʹ�ܶ�ʱ��6
//////////				//TIM_SetAutoreload(TIM4,1499);
//////////				TIM_SetCounter(TIM4,0);	//���������			
//////////				TIM_Cmd(TIM4, ENABLE); //�ٴ�ʹ�ܶ�ʱ��6
//////////			}
//////////////			else if(3==rtklink.ntripauthstep)
//////////////			{
//////////////				rtklink.ntripauthstep=4;
//////////////				sprintf(USART3_TX_DMABUF,"\032");
//////////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����
//////////////				//USART3_printf("\032");
//////////////				//USART1_printf("rtd ntrip���Ӷ������..\r\n");
//////////////				//rtklink.connectsta=STATE_AUTHORIZED;
//////////////				TIM_SetCounter(TIM4,0);	//���������			
//////////////				TIM_Cmd(TIM4, ENABLE); //�رն�ʱ��6
//////////////				//atomflag=1;
//////////////			}
//////////			else if(4==rtklink.ntripauthstep)
//////////			{
//////////				rtklink.ntripauthstep=0;
//////////				//USART3_printf("\032");
//////////				USART1_printf("rtd ntrip���Ӷ������..\r\n");
//////////				rtklink.connectsta=STATE_AUTHORIZED;
//////////				TIM_SetCounter(TIM4,0);	//���������			
//////////				TIM_Cmd(TIM4, DISABLE); //�رն�ʱ��6
//////////				atomflag=1;
//////////			}
//////////			else
//////////			{
//////////				rtklink.ntripauthstep=0;
//////////				TIM_SetCounter(TIM4,0);	//���������			
//////////				TIM_Cmd(TIM4, DISABLE); //�رն�ʱ��6
//////////				atomflag=1;
//////////			}
//////////	}	    
//////////}

//////////void TIM6_DAC_IRQHandler(void)
//////////{
//////////	if (TIM_GetITStatus(TIM6, TIM_IT_Update) != RESET)//�Ǹ����ж�
//////////	{	 			   
//////////			if(1==uplink.updatastep)
//////////			{
//////////				sprintf(USART3_TX_DMABUF,"AT+CIPSEND=0\r\n");
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����

//////////				//USART3_printf("AT+CIPSEND=0\r\n");
//////////				uplink.updatastep=2;
//////////				TIM_Cmd(TIM6, DISABLE);
//////////				TIM_SetAutoreload(TIM6,750-1);
//////////				TIM_SetCounter(TIM6,0);	//���������			
//////////				TIM_Cmd(TIM6, ENABLE); //�ٴ�ʹ�ܶ�ʱ��3��
//////////			}
//////////			else if(2==uplink.updatastep)
//////////			{
//////////				uplink.updatastep=4;
//////////				if(id!=NULL&&GPSGPGGA!=NULL)
//////////				{
//////////					sprintf(USART3_TX_DMABUF,"%s,%s\r\n\032",id,GPSGPGGA);
//////////					MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����
//////////					//USART3_printf("%s,%s\r\n",id,GPSGPGGA);
//////////				}
//////////				else
//////////				{
//////////					sprintf(USART3_TX_DMABUF,"%s,$GNGGA,000000.00,0.0,N,0.0,E,0,0,255.0,49.0,M,0.0,M,,0000*42\r\n\032",id);
//////////					MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����
//////////					//USART3_printf("%s,$GNGGA,000000.00,0.0,N,0.0,E,0,0,255.0,49.0,M,0.0,M,,0000*42\r\n",id);
//////////				}
//////////				TIM_Cmd(TIM6, DISABLE);
//////////				TIM_SetAutoreload(TIM6,1250-1);
//////////				TIM_SetCounter(TIM6,0);	//���������			
//////////				TIM_Cmd(TIM6, ENABLE); //�ٴ�ʹ�ܶ�ʱ��3��
//////////			}
//////////////			else if(3==uplink.updatastep)
//////////////			{
//////////////				uplink.updatastep=4;
//////////////				sprintf(USART3_TX_DMABUF,"\032");
//////////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����
//////////////				//USART3_printf("\032");
//////////////				//TIM_Cmd(TIM6, DISABLE);
//////////////				//TIM_SetAutoreload(TIM6,1250-1);
//////////////				TIM_SetCounter(TIM6,0);	//���������			
//////////////				TIM_Cmd(TIM6, ENABLE); //�رն�ʱ��6
//////////////				//TIM_Cmd(TIM6, DISABLE); //�رն�ʱ��6
//////////////				//atomflag=1;
//////////////			}
//////////			else if(4==uplink.updatastep)
//////////			{
//////////				uplink.updatastep=0;
//////////				TIM_SetCounter(TIM6,0);	//���������			
//////////				//TIM_SetAutoreload(TIM6,300-1);
//////////				TIM_Cmd(TIM6, DISABLE); //�رն�ʱ��6
//////////				atomflag=1;
//////////			}
//////////			else
//////////			{
//////////				uplink.updatastep=0;
//////////				TIM_SetCounter(TIM6,0);	//���������			
//////////				//TIM_SetAutoreload(TIM6,300-1);
//////////				TIM_Cmd(TIM6, DISABLE); //�رն�ʱ��6
//////////				atomflag=1;
//////////			}


//////////		TIM_ClearITPendingBit(TIM6, TIM_IT_Update  );  //���TIM6�����жϱ�־    
//////////		//TIM_Cmd(TIM6, DISABLE);  //�ر�TIM6 
//////////	}	    


//////////}




//////////void TIM5_IRQHandler(void)
//////////{
//////////	int i;
//////////	if (TIM_GetITStatus(TIM5, TIM_IT_Update) != RESET)//�Ǹ����ж�
//////////	{	 			   
//////////		TIM_ClearITPendingBit(TIM5, TIM_IT_Update  );  //���TIM5�����жϱ�־    
//////////		if(1==atomflag)//����ԭ�Ӳ���
//////////		{
//////////			if(1==sendpoll[5])//��������1���ر�
//////////			{
//////////				sendpoll[5]=0;
//////////				atomflag=0;
//////////				memset(USART3_TX_DMABUF,0,USART_SEND_LEN);
//////////				sprintf(USART3_TX_DMABUF,"AT+CIPCLOSE=0\r\n");
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����
//////////				uplink.connectsta=0;//��ѯ1״̬
//////////				uplink.losscount=0;
//////////				uplink.timex=0;
//////////				delay_ms(20);
//////////				atomflag=1;
//////////				
//////////				//TIM_SetCounter(TIM3,0);	//���������			
//////////				//TIM_Cmd(TIM3, ENABLE);
//////////				
//////////////				uplink.reconnectstep=1;
//////////////				TIM_SetCounter(TIM3,0);	//���������			
//////////////				TIM_Cmd(TIM3, ENABLE);

////////////				sim800c_send_cmd("AT+CIPCLOSE=0","CLOSE OK",20);//�ر�����
////////////				USART3_printf("AT+CIPSTART=0,\"TCP\",\"%s\",\"%s\"\r\n",dataserverip,dataserverport);
////////////				uplink.connectsta=0;
////////////				uplink.losscount=0;
////////////				atomflag=1;
//////////				
//////////			}
//////////			else if(1==sendpoll[7])//��������2������
//////////			{
//////////				sendpoll[7]=0;
//////////				atomflag=0;
//////////				memset(USART3_TX_DMABUF,0,USART_SEND_LEN);
//////////				sprintf(USART3_TX_DMABUF,"AT+CIPSTART=0,\"TCP\",\"%s\",\"%s\"\r\n",dataserverip,dataserverport);
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����
//////////				uplink.connectsta=5;//��ѯ2״̬
//////////				uplink.losscount=0;
//////////				uplink.timex=0;
//////////				delay_ms(20);
//////////				atomflag=1;
//////////				//TIM_SetCounter(TIM3,0);	//���������			
//////////				//TIM_Cmd(TIM3, ENABLE);
//////////			}
//////////			else if(1==sendpoll[6])//��������1 ��ַ����� �ر�
//////////			{
//////////				sendpoll[6]=0;
//////////				atomflag=0;

//////////////				sim800c_send_cmd("AT+CIPCLOSE=1","CLOSE OK",5);//�ر�����
//////////////				USART3_printf("AT+CIPSTART=1,\"TCP\",\"%s\",\"%s\"\r\n",rtkserverip,rtkserverport);
//////////////				rtklink.connectsta=STATE_QUERY;
//////////////				rtklink.losscount=0;
//////////////				rtklink.timex=2;
//////////////				atomflag=1;
//////////				
//////////				memset(USART3_TX_DMABUF,0,USART_SEND_LEN);
//////////				sprintf(USART3_TX_DMABUF,"AT+CIPCLOSE=1\r\n");
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����
//////////				//USART3_printf("AT+CIPCLOSE=1\r\n");
//////////				rtklink.connectsta=0;//��ѯ1״̬
//////////				rtklink.losscount=0;
//////////				rtklink.timex=2;
//////////				delay_ms(20);
//////////				atomflag=1;
//////////				//TIM_SetCounter(TIM12,0);	//���������			
//////////				//TIM_Cmd(TIM12, ENABLE);
//////////				
////////////				rtklink.reconnectstep=1;
////////////				TIM_SetCounter(TIM12,0);	//���������			
////////////				TIM_Cmd(TIM12, ENABLE);
//////////			}
//////////			else if(1==sendpoll[8])//��������2 ��ַ����� ��������
//////////			{
//////////				sendpoll[8]=0;
//////////				atomflag=0;
//////////				memset(USART3_TX_DMABUF,0,USART_SEND_LEN);
//////////				sprintf(USART3_TX_DMABUF,"AT+CIPSTART=1,\"TCP\",\"%s\",\"%s\"\r\n",rtkserverip,rtkserverport);
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����
//////////				//USART3_printf("AT+CIPSTART=1,\"TCP\",\"%s\",\"%s\"\r\n",rtkserverip,rtkserverport);
//////////				rtklink.connectsta=5;//��ѯ5״̬
//////////				rtklink.losscount=0;
//////////				rtklink.timex=2;
//////////				delay_ms(20);
//////////				atomflag=1;
//////////				//TIM_SetCounter(TIM12,0);	//���������			
//////////				//TIM_Cmd(TIM12, ENABLE);
//////////			}
//////////			else if(1==sendpoll[0])//�����ϴ�
//////////			{
//////////				sendpoll[0]=0;
//////////				atomflag=0;
//////////				uplink.updatastep=1;
//////////				TIM_Cmd(TIM6, DISABLE);
//////////				TIM_SetAutoreload(TIM6,150-1);
//////////				TIM_SetCounter(TIM6,0);	//���������			
//////////				TIM_Cmd(TIM6, ENABLE);
//////////			}
//////////			else if(1==sendpoll[1])//����NTRIP��֤����
//////////			{
//////////				sendpoll[1]=0;
//////////				atomflag=0;
//////////				rtklink.ntripauthstep=1;
//////////				//TIM_Cmd(TIM4, DISABLE);
//////////				//TIM_SetAutoreload(TIM4,30000-1);
//////////				TIM_SetCounter(TIM4,0);	//���������			
//////////				TIM_Cmd(TIM4, ENABLE);
//////////			}
//////////			else if(1==sendpoll[2])//����NMEA����
//////////			{
//////////				sendpoll[2]=0;
//////////				atomflag=0;
//////////				rtklink.ntripnmeastep=1;
//////////				TIM_SetCounter(TIM7,0);	//���������			
//////////				TIM_Cmd(TIM7, ENABLE);
//////////			}
//////////			else if(1==sendpoll[3])//����IPSTATUS=0
//////////			{
//////////				sendpoll[3]=0;
//////////				atomflag=0;
//////////				sprintf(USART3_TX_DMABUF,"AT+CIPSTATUS=0\r\n");
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����
//////////				atomflag=1;
//////////				//TIM_SetCounter(TIM3,0);	//���������			
//////////				//TIM_Cmd(TIM3, ENABLE);
//////////			}
//////////			else if(1==sendpoll[4])//����IPSTATUS=1
//////////			{
//////////				sendpoll[4]=0;
//////////				atomflag=0;
//////////				sprintf(USART3_TX_DMABUF,"AT+CIPSTATUS=1\r\n");
//////////				MYDMA_Enable(DMA1_Stream3,strlen(USART3_TX_DMABUF));//��ʼһ��DMA����
//////////				atomflag=1;
//////////				//TIM_SetCounter(TIM12,0);	//���������			
//////////				//TIM_Cmd(TIM12, ENABLE);
//////////			}
//////////		
//////////		}
//////////	}	    
//////////}

//////////void TIM8_BRK_TIM12_IRQHandler(void)
//////////{
//////////	int i;
//////////	if (TIM_GetITStatus(TIM12, TIM_IT_Update) != RESET)//�Ǹ����ж�
//////////	{	 	
//////////		TIM_ClearITPendingBit(TIM12, TIM_IT_Update  );  //���TIM5�����жϱ�־    
//////////		
//////////////		TIM_ITConfig(TIM12,TIM_IT_Update,DISABLE ); //ʹ��ָ����TIM5�ж�,����������ж�

//////////////			TIM_SetCounter(TIM12,0);	//���������			
//////////////			TIM_Cmd(TIM12, DISABLE); //�رն�ʱ��6
//////////////			atomflag=1;
//////////////		TIM_ITConfig(TIM12,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM5�ж�,����������ж�

//////////		if(1==rtklink.reconnectstep)
//////////		{
//////////			USART3_printf("AT+CIPCLOSE=1\r\n");
//////////			rtklink.reconnectstep=2;
//////////			TIM_SetCounter(TIM12,0);	//���������			
//////////			TIM_Cmd(TIM12, ENABLE); //�ٴ�ʹ�ܶ�ʱ��3��
//////////		}
//////////		else if(2==rtklink.reconnectstep)
//////////		{
//////////			rtklink.reconnectstep=3;
//////////			USART3_printf("AT+CIPSTART=1,\"TCP\",\"%s\",\"%s\"\r\n",rtkserverip,rtkserverport);
//////////			TIM_SetCounter(TIM12,0);	//���������			
//////////			TIM_Cmd(TIM12, ENABLE); //�ٴ�ʹ�ܶ�ʱ��3��
//////////		}
//////////		else if(3==rtklink.reconnectstep)
//////////		{
//////////			rtklink.reconnectstep=0;
//////////			TIM_SetCounter(TIM12,0);	//���������			
//////////			TIM_Cmd(TIM12, DISABLE); //�رն�ʱ��6
//////////			rtklink.connectsta=0;
//////////			rtklink.losscount=0;
//////////			rtklink.timex=2;
//////////			atomflag=1;
//////////		}
//////////		else
//////////		{
//////////			rtklink.reconnectstep=0;
//////////			TIM_SetCounter(TIM12,0);	//���������			
//////////			TIM_Cmd(TIM12, DISABLE); //�رն�ʱ��6
//////////			atomflag=1;
//////////		}

//////////		
//////////	}	    
//////////}



////////////ͨ�ö�ʱ��7�жϳ�ʼ��
////////////����ʱ��ѡ��ΪAPB1��2������APB1Ϊ42M
////////////arr���Զ���װֵ��
////////////psc��ʱ��Ԥ��Ƶ��
////////////��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
////////////Ft=��ʱ������Ƶ��,��λ:Mhz 
////////////ͨ�ö�ʱ���жϳ�ʼ��
////////////����ʼ��ѡ��ΪAPB1��2������APB1Ϊ36M
////////////arr���Զ���װֵ��
////////////psc��ʱ��Ԥ��Ƶ��		 
//////////void TIM7_Int_Init(u16 arr,u16 psc)
//////////{	
//////////	NVIC_InitTypeDef NVIC_InitStructure;
//////////	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//////////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);//TIM7ʱ��ʹ��    
//////////	
//////////	//��ʱ��TIM7��ʼ��
//////////	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
//////////	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
//////////	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
//////////	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
//////////	TIM_TimeBaseInit(TIM7, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
////////// 
//////////	TIM_ITConfig(TIM7,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM7�ж�,��������ж�
//////////	
//////////	//TIM_Cmd(TIM7,ENABLE);//������ʱ��7
//////////	
//////////	NVIC_InitStructure.NVIC_IRQChannel = TIM7_IRQn;
//////////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�0
//////////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�2
//////////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//////////	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
//////////	
//////////}

//////////void TIM4_Int_Init(u16 arr,u16 psc)
//////////{	
//////////	NVIC_InitTypeDef NVIC_InitStructure;
//////////	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//////////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);//TIM4ʱ��ʹ��    
//////////	
//////////	//��ʱ��TIM4��ʼ��
//////////	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
//////////	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
//////////	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
//////////	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
//////////	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
////////// 
//////////	TIM_ITConfig(TIM4,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM4�ж�,��������ж�
//////////	
//////////	//TIM_Cmd(TIM4,ENABLE);//������ʱ��7
//////////	
//////////	NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
//////////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�0
//////////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;		//�����ȼ�2
//////////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//////////	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
//////////	
//////////}
////////////ͨ�ö�ʱ���жϳ�ʼ��
////////////����ʼ��ѡ��ΪAPB1��2������APB1Ϊ42M
////////////arr���Զ���װֵ��
////////////psc��ʱ��Ԥ��Ƶ��		 
//////////void TIM6_Int_Init(u16 arr,u16 psc)
//////////{	
//////////	NVIC_InitTypeDef NVIC_InitStructure;
//////////	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//////////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, ENABLE);//TIM6ʱ��ʹ��    
//////////	
//////////	//��ʱ��TIM6��ʼ��
//////////	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
//////////	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
//////////	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
//////////	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
//////////	TIM_TimeBaseInit(TIM6, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
////////// 
//////////	TIM_ITConfig(TIM6,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM6�ж�,��������ж�
//////////   
////////////	TIM_Cmd(TIM6,ENABLE);//ʹ�ܶ�ʱ��6
//////////	
//////////	NVIC_InitStructure.NVIC_IRQChannel = TIM6_DAC_IRQn;
//////////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�0
//////////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�1
//////////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//////////	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
//////////	
//////////}


////////////ͨ�ö�ʱ��5�жϳ�ʼ��
////////////����ʼ��ѡ��ΪAPB1��2������APB1Ϊ42M����ʱʱ��=(arr+1)*(psc+1)/84 us
////////////arr���Զ���װֵ����ʱ��2��32λ��arr
////////////psc��ʱ��Ԥ��Ƶ��		 
//////////void TIM5_Int_Init(u32 arr,u16 psc)
//////////{	
//////////	NVIC_InitTypeDef NVIC_InitStructure;
//////////	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//////////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);//TIM5ʱ��ʹ��    
//////////	
//////////	//��ʱ��TIM5��ʼ��
//////////	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
//////////	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
//////////	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
//////////	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
//////////	TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
////////// 
//////////	TIM_ITConfig(TIM5,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM5�ж�,��������ж�
//////////   
//////////	TIM_Cmd(TIM5,ENABLE);//ʹ�ܶ�ʱ��5
//////////	
//////////	NVIC_InitStructure.NVIC_IRQChannel = TIM5_IRQn;
//////////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�0
//////////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�1
//////////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//////////	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
//////////	
//////////}

////////////ͨ�ö�ʱ���жϳ�ʼ��
////////////����ʼ��ѡ��ΪAPB1��2������APB1Ϊ36M
////////////arr���Զ���װֵ��
////////////psc��ʱ��Ԥ��Ƶ��		 
//////////void TIM3_Int_Init(u16 arr,u16 psc)
//////////{	
//////////	NVIC_InitTypeDef NVIC_InitStructure;
//////////	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//////////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);//TIM5ʱ��ʹ��    
//////////	
//////////	//��ʱ��TIM7��ʼ��
//////////	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
//////////	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
//////////	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
//////////	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
//////////	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
////////// 
//////////	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM4�ж�,��������ж�
//////////   
//////////	TIM_Cmd(TIM3,ENABLE);//ʹ�ܶ�ʱ��4
//////////	
//////////	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;
//////////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�0
//////////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�1
//////////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//////////	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
//////////	
//////////}


////////////ͨ�ö�ʱ���жϳ�ʼ��
////////////����ʼ��ѡ��ΪAPB1��2������APB1Ϊ36M
////////////arr���Զ���װֵ��
////////////psc��ʱ��Ԥ��Ƶ��		 
//////////void TIM12_Int_Init(u16 arr,u16 psc)
//////////{	
//////////	NVIC_InitTypeDef NVIC_InitStructure;
//////////	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

//////////	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, ENABLE);//TIM5ʱ��ʹ��    
//////////	
//////////	//��ʱ��TIM7��ʼ��
//////////	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
//////////	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
//////////	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
//////////	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
//////////	TIM_TimeBaseInit(TIM12, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
////////// 
//////////	TIM_ITConfig(TIM12,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM4�ж�,��������ж�
//////////   
//////////	//TIM_Cmd(TIM12,ENABLE);//ʹ�ܶ�ʱ��4
//////////	
//////////	NVIC_InitStructure.NVIC_IRQChannel = TIM8_BRK_TIM12_IRQn;
//////////	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�0
//////////	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�1
//////////	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
//////////	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
//////////	
//////////}

//ͨ�ö�ʱ��2�жϳ�ʼ��
//����ʼ��ѡ��ΪAPB1��2������APB1Ϊ42M����ʱʱ��=(arr+1)*(psc+1)/84 us
//arr���Զ���װֵ����ʱ��2��32λ��arr
//psc��ʱ��Ԥ��Ƶ��		 
void TIM2_Int_Init(u32 arr,u16 psc)
{	
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);//TIM2ʱ��ʹ��    
	
	//��ʱ��TIM2��ʼ��
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM2, &TIM_TimeBaseStructure); //����ָ���Ĳ�����ʼ��TIMx��ʱ�������λ
 
	TIM_ITConfig(TIM2,TIM_IT_Update,ENABLE ); //ʹ��ָ����TIM2�ж�,��������ж�
   
	TIM_Cmd(TIM2,ENABLE);//ʹ�ܶ�ʱ��2
	
	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0 ;//��ռ���ȼ�0
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;		//�����ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	
}

void TIM2_IRQHandler(void)
{
	OS_ERR err;
	
#ifdef SYSTEM_SUPPORT_OS	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntEnter();    
#endif
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET)//�Ǹ����ж�
	{	 			   
		//timeupflag=1;
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update  );  //���TIM2�����жϱ�־    
		//TIM_Cmd(TIM2, DISABLE);  //�ر�TIM2 
		OSFlagPost((OS_FLAG_GRP*)&TimeupEventFlags,
								 (OS_FLAGS	  )0xff,
								 (OS_OPT	  )OS_OPT_POST_FLAG_SET,
								 (OS_ERR*     )&err);

	}	    

#ifdef SYSTEM_SUPPORT_OS	 	//���ʱ�ӽ�����������,˵��Ҫʹ��ucosII��.
	OSIntExit();  											 
#endif

}


