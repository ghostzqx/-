#include "delay.h" 			 
#include "usart.h" 			 
#include "sim800c.h" 
#include "string.h"	 
#include "sys.h"
#include "GlobalVariables.h"
#include "iwdg.h"
//#include "dma.h"
//���ʹ��ucos,����������ͷ�ļ�����.
#define SIMPOWER PCout(13)	// SIM POWER CONTROL

//ATK-SIM800C �������(���Ų��ԡ����Ų��ԡ�GPRS���ԡ���������)���ô���
//SIM800C���������,�����յ���Ӧ��
//str:�ڴ���Ӧ����
//����ֵ:0,û�еõ��ڴ���Ӧ����
//����,�ڴ�Ӧ������λ��(str��λ��)
u8* sim800c_check_cmd(u8 *str)
{
	char *strx=0;
	if(USART3_RX_STA&0X8000)  //���յ�һ��������
	{ 
		USART3_RX_BUF[USART3_RX_STA&0X7FFF]=0;//��ӽ�����
		strx=strstr((const char*)USART3_RX_BUF,(const char*)str);
	} 
	return (u8*)strx;
}
//��SIM800C��������
//cmd:���͵������ַ���(����Ҫ��ӻس���),��cmd<0XFF��ʱ��,��������(���緢��0X1A),���ڵ�ʱ�����ַ���.
//ack:�ڴ���Ӧ����,���Ϊ��,���ʾ����Ҫ�ȴ�Ӧ��
//waittime:�ȴ�ʱ��(��λ:10ms)
//����ֵ:0,���ͳɹ�(�õ����ڴ���Ӧ����)
//       1,����ʧ��
u8 sim800c_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
{
	u8 res=0; 
	USART3_RX_STA=0;
	if((u32)cmd<=0XFF)
	{
		while((USART3->SR&0X40)==0);//�ȴ���һ�����ݷ������  
		USART3->DR=(u32)cmd;
	}else USART3_printf("%s\r\n",cmd);  //��������
	
	if(ack&&waittime)		        //��Ҫ�ȴ�Ӧ��
	{
		while(--waittime)	        //�ȴ�����ʱ
		{ 
			delay_ms(10);
			if(USART3_RX_STA&0X8000)//���յ��ڴ���Ӧ����
			{
				if(sim800c_check_cmd(ack)){USART3_RX_STA=0;break;}//�õ���Ч���� 
				USART3_RX_STA=0;
			}
		}
		if(waittime==0)res=1; 
	}
	return res;
}


///////////////////////////////////////////////////////////////////////////////////////
//BLE101������ѯ/���������
//cmd����ѯ/��������
//ack��������Ϣ
//
//res:����0���ɹ�������1��ʧ��
//////////////////////////////////////////////////////////////////////////////////////
//u8 SIM800C_AT_CMD(u8 *cmd,u8 *ack)
//{
//	u8 t=0;
//	u8 res=1;
//	u8 retry=10;	
//	USART3_RX_STA=0;
//		
//	u3_printf("%s\r\n",cmd);		//����AT����ָ��
//	if(ack&&retry)
//	{
//		while(retry--)
//		{
//			for(t=0;t<10;t++) 			//��ȴ�50ms,������HC05ģ��Ļ�Ӧ
//			{
//				if(USART3_RX_STA&0X8000)break;
//				delay_ms(5);
//			}	
//		
//			if(USART3_RX_STA&0X8000)	//���յ�һ��������
//			{
//				USART3_RX_BUF[USART3_RX_STA&0X7FFF]=0;				
//			
//				if(strstr((const char*)USART3_RX_BUF,(const char*)ack))
//				{
//					res=0;
//					printf("%s\r\n",(const char*)USART3_RX_BUF);	//�����յ���ָ���ֵ���䵽����1��ʾ
//					USART3_RX_STA=0;			 
//					break;
//				}
//				USART3_RX_STA=0;			 
//			}			
//		}	
//		if(retry==0)res=1;	//���ʧ��

//	}

//	return res;	
//	
//} 

////sim800c GPRS׼��
////����ֵ:0,����
////����,�������
u8 sim800c_gprs_prepare(void)
{
//////	sim800c_send_cmd("AT+CIPCLOSE","CLOSE OK",100);	//�ر�����
////	if(sim800c_send_cmd("AT+CIPSHUT","SHUT OK",500)) return 8;		//�ر��ƶ����� 
////	if(sim800c_send_cmd("AT+CGCLASS=\"B\"","OK",1000))return 1;				//����GPRS�ƶ�̨���ΪB,֧�ְ����������ݽ��� 
////	if(sim800c_send_cmd("AT+CGDCONT=1,\"IP\",\"CMIOT\"","OK",1000))return 2;//����PDP������,��������Э��,��������Ϣ
////	if(sim800c_send_cmd("AT+CGATT=1","OK",500))return 3;					//����GPRSҵ��
//////	if(sim800c_send_cmd("AT+CGATT=1",0,0))return 3;					//����GPRSҵ��
////	if(sim800c_send_cmd("AT+CIPCSGP=1,\"CMIOT\"","OK",500))return 4;	 	//����ΪGPRS����ģʽ

	if(sim800c_send_cmd("AT+CIPSHUT",0,500)) return 8;		//�ر��ƶ����� 
	delay_ms(2000);

	if(sim800c_send_cmd("AT+CIPMUX=1","OK",500))return 9;	 				//���ö�IP����
	delay_ms(1000);
	if(sim800c_send_cmd("AT+CIPHEAD=1","OK",500))return 5;	 				//���ý���������ʾIPͷ(�����ж�������Դ)
	delay_ms(1000);
	if(sim800c_send_cmd("AT+CSTT","OK",500))return 6;	 				//����APN���û���������
	delay_ms(1000);
	if(sim800c_send_cmd("AT+CIICR","OK",500))return 7;	 				//����GPRS��CSD��������
	delay_ms(1000);
	if(sim800c_send_cmd("AT+CIFSR",0,500))return 10;	 				//��ȡ����IP��ַ����Ӫ�̷��䣩
	delay_ms(1000);
	if(sim800c_send_cmd("AT+CIPQSEND=1","OK",200)) return 11;//�촫;
	delay_ms(500);

	return 0;
} 

u8 OSsim800c_gprs_prepare(OS_ERR* err)
{
	if(sim800c_send_cmd("AT+CIPSHUT",0,500)) return 8;		//�ر��ƶ����� 
	OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,err);

	if(sim800c_send_cmd("AT+CIPMUX=1","OK",500))return 9;	 				//���ö�IP����
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+CIPHEAD=1","OK",500))return 5;	 				//���ý���������ʾIPͷ(�����ж�������Դ)
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+CSTT","OK",500))return 6;	 				//����APN���û���������
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+CIICR","OK",500))return 7;	 				//����GPRS��CSD��������
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+CIFSR",0,500))return 10;	 				//��ȡ����IP��ַ����Ӫ�̷��䣩
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+CIPQSEND=1","OK",200)) return 11;//�촫;
	OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,err);

	return 0;
} 

u8 OSsim7X00_gprs_prepare(OS_ERR* err)
{

	if(sim800c_send_cmd("AT+CSQ","OK",500))return 1;	 				//�ź�ǿ��
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("ATE0","OK",500)) return 8;		//ȡ������
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+NETCLOSE",0,500)) return 8;		//�ر��ƶ����� 
	OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,err);

	if(sim800c_send_cmd("AT+CREG?","OK",500))return 1;	 				//GPRS����ʹ�ܲ�ѯ
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+CPSI?",0,500))return 1;	 				//GPRS����ʹ��
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+CGREG?","OK",500))return 1;	 				//GPRS����ʹ��
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);

	
//	if(sim800c_send_cmd("AT+CIPHEAD=1","OK",500))return 5;	 				//���ý���������ʾIPͷ(�����ж�������Դ)
//	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
//	if(sim800c_send_cmd("AT+CGSOCKCONT=1,\"IP\",\"CMNET\"","OK",500))return 6;	 				//����APN
//	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
//	if(sim800c_send_cmd("AT+CSOCKSETPN=1","OK",500))return 7;	 				//����APN
//	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
//	if(sim800c_send_cmd("AT+CIPMODE=0","OK",500))return 7;	 				//����APN
//	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+NETOPEN","+NETOPEN: 0",500))return 10;	 				//��ȡ����IP��ַ����Ӫ�̷��䣩+NETOPEN: 0
	OSTimeDlyHMSM(0,0,5,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+CIPCCFG=,,,,1,,",0,500))return 10;	 				//�����յ�����ϢͷΪ"+RECEIVE,<link num>,<data length>"
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
//	if(sim800c_send_cmd("AT+IPADDR",0,500))return 10;	 				//��ȡ����IP��ַ����Ӫ�̷��䣩
//	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);

	return 0;
} 


const u8 *modetbl[2]={"TCP","UDP"};//����ģʽ
////tcp/udp����
////����������,��ά������
////mode:0:TCP����;1,UDP����)
////ipaddr:ip��ַ
////port:�˿� 
//u8 sim800c_tcpudp_connect(u8 mode,u8* ipaddr,u8* port,int id)
u8 sim800c_tcpudp_connect(u8 mode,u8* ipaddr,u32 port,int id)
{ 
	u8 p[100],*p1,*p2,*p3;
	u8 key;
	u16 timex=0;
	u8 count=0;
	const u8* cnttbl[3]={"��������","���ӳɹ�","���ӹر�"};
	u8 connectsta=0;			//0,��������;1,���ӳɹ�;2,���ӹر�; 
	u8 hbeaterrcnt=0;			//�������������,����5�������ź���Ӧ��,����������
	u8 oldsta=0XFF;
	
	if(mode!=0 && mode!=1) return 1;//ģʽ����
	sprintf((char*)p,"AT+CIPSTART=%d,\"%s\",\"%s\",\"%d\"",id,modetbl[mode],ipaddr,port);
	if(sim800c_send_cmd(p,"OK",500))return 2;		//��������
	return 0;
}

u8 sim7X00_tcpudp_connect(u8 mode,u8* ipaddr,u32 port,int id)
{ 
	u8 p[100],*p1,*p2,*p3;
	u8 key;
	u16 timex=0;
	u8 count=0;
	//const u8* cnttbl[3]={"��������","���ӳɹ�","���ӹر�"};
	u8 connectsta=0;			//0,��������;1,���ӳɹ�;2,���ӹر�; 
	u8 hbeaterrcnt=0;			//�������������,����5�������ź���Ӧ��,����������
	u8 oldsta=0XFF;
	
	if(mode!=0 && mode!=1) return 1;//ģʽ����
	sprintf((char*)p,"AT+CIPOPEN=%d,\"%s\",\"%s\",%u",id,modetbl[mode],ipaddr,port);
	if(sim800c_send_cmd(p,"OK",500))return 2;		//��������
	return 0;
}

void sim_powerup()
{
	IWDG_Feed();
	SIMPOWER=0;
	delay_ms(2000);
	IWDG_Feed();
	SIMPOWER=1;
	delay_ms(2000);
	IWDG_Feed();
	SIMPOWER=0;
	delay_ms(2000);
	IWDG_Feed();
	SIMPOWER=1;
	delay_ms(300);
	SIMPOWER=0;
	delay_ms(10000);
	IWDG_Feed();
}
void OSsim_powerup(OS_ERR* err)
{
	
	IWDG_Feed();
	SIMPOWER=0;
	OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,err);
	IWDG_Feed();
	SIMPOWER=1;
	OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,err);
	IWDG_Feed();
	SIMPOWER=0;
	OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,err);
	IWDG_Feed();
	SIMPOWER=1;
	OSTimeDlyHMSM(0,0,0,300,OS_OPT_TIME_HMSM_STRICT,err);
	SIMPOWER=0;
	OSTimeDlyHMSM(0,0,10,0,OS_OPT_TIME_HMSM_STRICT,err);
	IWDG_Feed();
}

void OSsim_init(OS_ERR* err)
{
//	sim800c_send_cmd("ATE0","OK",200);//������;
//	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	while(1)
	{
		IWDG_Feed();
		//if(0==sim800c_gprs_prepare())
		if(0==OSsim7X00_gprs_prepare(err))
			break;
	}

	while(1)
	{
		//if(0==sim800c_tcpudp_connect(0,CfgDataServerip,CfgDataServerport,0))
		if(0==sim7X00_tcpudp_connect(0,CfgDataServerip,CfgDataServerport,0))
			break;
		OSTimeDlyHMSM(0,0,3,0,OS_OPT_TIME_HMSM_STRICT,err);
		IWDG_Feed();
	}

	//USART3_printf("AT+CIPSTART=0,\"TCP\",\"%s\",\"%s\"\r\n",dataserverip,dataserverport);
	//delay_ms(5000);
	uplink.connectsta=1;//
	IWDG_Feed();
	//USART3_printf("AT+CIPSTART=1,\"TCP\",\"%s\",\"%s\"\r\n",rtkserverip,rtkserverport);
	//delay_ms(5000);
	if(rtkflag==1)
	{
		while(1)
		{
			//if(0==sim800c_tcpudp_connect(0,CfgDifferServerip,CfgDifferServerport,1))
			if(0==sim7X00_tcpudp_connect(0,CfgDifferServerip,CfgDifferServerport,1))
				break;
			OSTimeDlyHMSM(0,0,3,0,OS_OPT_TIME_HMSM_STRICT,err);
			IWDG_Feed();
		}
	rtklink.connectsta=5;//STATE_QUERY;//STATE_CONNECTED;
	}
}


void sim_init()
{
	sim800c_send_cmd("ATE1","OK",200);//����;
	delay_ms(1000);
	while(1)
	{
		IWDG_Feed();
		if(0==sim800c_gprs_prepare())
			break;
	}

	while(1)
	{
		if(0==sim800c_tcpudp_connect(0,CfgDataServerip,CfgDataServerport,0))
			break;
		delay_ms(3000);
		IWDG_Feed();
	}

	//USART3_printf("AT+CIPSTART=0,\"TCP\",\"%s\",\"%s\"\r\n",dataserverip,dataserverport);
	//delay_ms(5000);
	uplink.connectsta=1;//
	IWDG_Feed();
	//USART3_printf("AT+CIPSTART=1,\"TCP\",\"%s\",\"%s\"\r\n",rtkserverip,rtkserverport);
	//delay_ms(5000);
	if(rtkflag==1)
	{
		while(1)
		{
			if(0==sim800c_tcpudp_connect(0,CfgDifferServerip,CfgDifferServerport,1))
				break;
			delay_ms(3000);
			IWDG_Feed();
		}
	rtklink.connectsta=5;//STATE_QUERY;//STATE_CONNECTED;
	}
}

void sim_reinit()
{
	IWDG_Feed();//ι��
	while(1)
	{
		IWDG_Feed();//ι��
		if(0==sim800c_gprs_prepare())
			break;
	}
	USART3_printf("AT+CIPSTART=0,\"TCP\",\"%s\",\"%d\"\r\n",CfgDataServerip,CfgDataServerport);
	delay_ms(2000);
	uplink.connectsta=5;//0;//
	IWDG_Feed();
	if(rtkflag==1)
	{
		USART3_printf("AT+CIPSTART=1,\"TCP\",\"%s\",\"%d\"\r\n",CfgDifferServerip,CfgDifferServerport);
		delay_ms(2000);
		IWDG_Feed();//ι��
		rtklink.connectsta=5;//STATE_QUERY;
		//connectsta2=0;//
		//rtkconnectedflag=1;
	}
}

//д��ת��
//���������г���<Ctrl+Z>��0x1a��ת��Ϊ0x03 0x1a
//0x1BתΪ0x03 0x1B,0x03תΪ0x03 0x03?
int GPRSWriteTrans(u8 dataout[],u16 length,u8 datain[])
{
	u16 datainptr,dataoutptr;
//	u8* datainptr=datain;
//	u8* dataoutptr=dataout;
	
	if(length>2048||datain==NULL||dataout==NULL)
		return -1;
	
	for(datainptr=0,dataoutptr=0;datainptr<length;datainptr++,dataoutptr++)
	{
		if(datain[datainptr]==0x1A||datain[datainptr]==0x1B||datain[datainptr]==0x03)
		{
			dataout[dataoutptr++]=0x03;
			dataout[dataoutptr]=datain[datainptr];
		}
		else
			dataout[dataoutptr]=datain[datainptr];
	}
	
	if(dataoutptr>2048)
		return -1;

	return dataoutptr;
	
}
