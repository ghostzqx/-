#include "delay.h" 			 
#include "usart.h" 			 
#include "sim800c.h" 
#include "string.h"	 
#include "sys.h"
#include "GlobalVariables.h"
#include "iwdg.h"
//#include "dma.h"
//如果使用ucos,则包括下面的头文件即可.
#define SIMPOWER PCout(13)	// SIM POWER CONTROL

//ATK-SIM800C 各项测试(拨号测试、短信测试、GPRS测试、蓝牙测试)共用代码
//SIM800C发送命令后,检测接收到的应答
//str:期待的应答结果
//返回值:0,没有得到期待的应答结果
//其他,期待应答结果的位置(str的位置)
u8* sim800c_check_cmd(u8 *str)
{
	char *strx=0;
	if(USART3_RX_STA&0X8000)  //接收到一次数据了
	{ 
		USART3_RX_BUF[USART3_RX_STA&0X7FFF]=0;//添加结束符
		strx=strstr((const char*)USART3_RX_BUF,(const char*)str);
	} 
	return (u8*)strx;
}
//向SIM800C发送命令
//cmd:发送的命令字符串(不需要添加回车了),当cmd<0XFF的时候,发送数字(比如发送0X1A),大于的时候发送字符串.
//ack:期待的应答结果,如果为空,则表示不需要等待应答
//waittime:等待时间(单位:10ms)
//返回值:0,发送成功(得到了期待的应答结果)
//       1,发送失败
u8 sim800c_send_cmd(u8 *cmd,u8 *ack,u16 waittime)
{
	u8 res=0; 
	USART3_RX_STA=0;
	if((u32)cmd<=0XFF)
	{
		while((USART3->SR&0X40)==0);//等待上一次数据发送完成  
		USART3->DR=(u32)cmd;
	}else USART3_printf("%s\r\n",cmd);  //发送命令
	
	if(ack&&waittime)		        //需要等待应答
	{
		while(--waittime)	        //等待倒计时
		{ 
			delay_ms(10);
			if(USART3_RX_STA&0X8000)//接收到期待的应答结果
			{
				if(sim800c_check_cmd(ack)){USART3_RX_STA=0;break;}//得到有效数据 
				USART3_RX_STA=0;
			}
		}
		if(waittime==0)res=1; 
	}
	return res;
}


///////////////////////////////////////////////////////////////////////////////////////
//BLE101蓝牙查询/设置命令函数
//cmd：查询/设置命令
//ack：返回信息
//
//res:返回0：成功，返回1：失败
//////////////////////////////////////////////////////////////////////////////////////
//u8 SIM800C_AT_CMD(u8 *cmd,u8 *ack)
//{
//	u8 t=0;
//	u8 res=1;
//	u8 retry=10;	
//	USART3_RX_STA=0;
//		
//	u3_printf("%s\r\n",cmd);		//发送AT测试指令
//	if(ack&&retry)
//	{
//		while(retry--)
//		{
//			for(t=0;t<10;t++) 			//最长等待50ms,来接收HC05模块的回应
//			{
//				if(USART3_RX_STA&0X8000)break;
//				delay_ms(5);
//			}	
//		
//			if(USART3_RX_STA&0X8000)	//接收到一次数据了
//			{
//				USART3_RX_BUF[USART3_RX_STA&0X7FFF]=0;				
//			
//				if(strstr((const char*)USART3_RX_BUF,(const char*)ack))
//				{
//					res=0;
//					printf("%s\r\n",(const char*)USART3_RX_BUF);	//将接收到的指令返回值传输到串口1显示
//					USART3_RX_STA=0;			 
//					break;
//				}
//				USART3_RX_STA=0;			 
//			}			
//		}	
//		if(retry==0)res=1;	//检测失败

//	}

//	return res;	
//	
//} 

////sim800c GPRS准备
////返回值:0,正常
////其他,错误代码
u8 sim800c_gprs_prepare(void)
{
//////	sim800c_send_cmd("AT+CIPCLOSE","CLOSE OK",100);	//关闭连接
////	if(sim800c_send_cmd("AT+CIPSHUT","SHUT OK",500)) return 8;		//关闭移动场景 
////	if(sim800c_send_cmd("AT+CGCLASS=\"B\"","OK",1000))return 1;				//设置GPRS移动台类别为B,支持包交换和数据交换 
////	if(sim800c_send_cmd("AT+CGDCONT=1,\"IP\",\"CMIOT\"","OK",1000))return 2;//设置PDP上下文,互联网接协议,接入点等信息
////	if(sim800c_send_cmd("AT+CGATT=1","OK",500))return 3;					//附着GPRS业务
//////	if(sim800c_send_cmd("AT+CGATT=1",0,0))return 3;					//附着GPRS业务
////	if(sim800c_send_cmd("AT+CIPCSGP=1,\"CMIOT\"","OK",500))return 4;	 	//设置为GPRS连接模式

	if(sim800c_send_cmd("AT+CIPSHUT",0,500)) return 8;		//关闭移动场景 
	delay_ms(2000);

	if(sim800c_send_cmd("AT+CIPMUX=1","OK",500))return 9;	 				//设置多IP连接
	delay_ms(1000);
	if(sim800c_send_cmd("AT+CIPHEAD=1","OK",500))return 5;	 				//设置接收数据显示IP头(方便判断数据来源)
	delay_ms(1000);
	if(sim800c_send_cmd("AT+CSTT","OK",500))return 6;	 				//设置APN、用户名、密码
	delay_ms(1000);
	if(sim800c_send_cmd("AT+CIICR","OK",500))return 7;	 				//发起GPRS或CSD无线连接
	delay_ms(1000);
	if(sim800c_send_cmd("AT+CIFSR",0,500))return 10;	 				//获取本地IP地址（运营商分配）
	delay_ms(1000);
	if(sim800c_send_cmd("AT+CIPQSEND=1","OK",200)) return 11;//快传;
	delay_ms(500);

	return 0;
} 

u8 OSsim800c_gprs_prepare(OS_ERR* err)
{
	if(sim800c_send_cmd("AT+CIPSHUT",0,500)) return 8;		//关闭移动场景 
	OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,err);

	if(sim800c_send_cmd("AT+CIPMUX=1","OK",500))return 9;	 				//设置多IP连接
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+CIPHEAD=1","OK",500))return 5;	 				//设置接收数据显示IP头(方便判断数据来源)
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+CSTT","OK",500))return 6;	 				//设置APN、用户名、密码
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+CIICR","OK",500))return 7;	 				//发起GPRS或CSD无线连接
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+CIFSR",0,500))return 10;	 				//获取本地IP地址（运营商分配）
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+CIPQSEND=1","OK",200)) return 11;//快传;
	OSTimeDlyHMSM(0,0,0,500,OS_OPT_TIME_HMSM_STRICT,err);

	return 0;
} 

u8 OSsim7X00_gprs_prepare(OS_ERR* err)
{

	if(sim800c_send_cmd("AT+CSQ","OK",500))return 1;	 				//信号强度
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("ATE0","OK",500)) return 8;		//取消回显
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+NETCLOSE",0,500)) return 8;		//关闭移动场景 
	OSTimeDlyHMSM(0,0,2,0,OS_OPT_TIME_HMSM_STRICT,err);

	if(sim800c_send_cmd("AT+CREG?","OK",500))return 1;	 				//GPRS网络使能查询
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+CPSI?",0,500))return 1;	 				//GPRS网络使能
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+CGREG?","OK",500))return 1;	 				//GPRS网络使能
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);

	
//	if(sim800c_send_cmd("AT+CIPHEAD=1","OK",500))return 5;	 				//设置接收数据显示IP头(方便判断数据来源)
//	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
//	if(sim800c_send_cmd("AT+CGSOCKCONT=1,\"IP\",\"CMNET\"","OK",500))return 6;	 				//设置APN
//	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
//	if(sim800c_send_cmd("AT+CSOCKSETPN=1","OK",500))return 7;	 				//设置APN
//	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
//	if(sim800c_send_cmd("AT+CIPMODE=0","OK",500))return 7;	 				//设置APN
//	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+NETOPEN","+NETOPEN: 0",500))return 10;	 				//获取本地IP地址（运营商分配）+NETOPEN: 0
	OSTimeDlyHMSM(0,0,5,0,OS_OPT_TIME_HMSM_STRICT,err);
	if(sim800c_send_cmd("AT+CIPCCFG=,,,,1,,",0,500))return 10;	 				//配置收到的消息头为"+RECEIVE,<link num>,<data length>"
	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);
//	if(sim800c_send_cmd("AT+IPADDR",0,500))return 10;	 				//获取本地IP地址（运营商分配）
//	OSTimeDlyHMSM(0,0,1,0,OS_OPT_TIME_HMSM_STRICT,err);

	return 0;
} 


const u8 *modetbl[2]={"TCP","UDP"};//连接模式
////tcp/udp测试
////带心跳功能,以维持连接
////mode:0:TCP测试;1,UDP测试)
////ipaddr:ip地址
////port:端口 
//u8 sim800c_tcpudp_connect(u8 mode,u8* ipaddr,u8* port,int id)
u8 sim800c_tcpudp_connect(u8 mode,u8* ipaddr,u32 port,int id)
{ 
	u8 p[100],*p1,*p2,*p3;
	u8 key;
	u16 timex=0;
	u8 count=0;
	const u8* cnttbl[3]={"正在连接","连接成功","连接关闭"};
	u8 connectsta=0;			//0,正在连接;1,连接成功;2,连接关闭; 
	u8 hbeaterrcnt=0;			//心跳错误计数器,连续5次心跳信号无应答,则重新连接
	u8 oldsta=0XFF;
	
	if(mode!=0 && mode!=1) return 1;//模式不对
	sprintf((char*)p,"AT+CIPSTART=%d,\"%s\",\"%s\",\"%d\"",id,modetbl[mode],ipaddr,port);
	if(sim800c_send_cmd(p,"OK",500))return 2;		//发起连接
	return 0;
}

u8 sim7X00_tcpudp_connect(u8 mode,u8* ipaddr,u32 port,int id)
{ 
	u8 p[100],*p1,*p2,*p3;
	u8 key;
	u16 timex=0;
	u8 count=0;
	//const u8* cnttbl[3]={"正在连接","连接成功","连接关闭"};
	u8 connectsta=0;			//0,正在连接;1,连接成功;2,连接关闭; 
	u8 hbeaterrcnt=0;			//心跳错误计数器,连续5次心跳信号无应答,则重新连接
	u8 oldsta=0XFF;
	
	if(mode!=0 && mode!=1) return 1;//模式不对
	sprintf((char*)p,"AT+CIPOPEN=%d,\"%s\",\"%s\",%u",id,modetbl[mode],ipaddr,port);
	if(sim800c_send_cmd(p,"OK",500))return 2;		//发起连接
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
//	sim800c_send_cmd("ATE0","OK",200);//不回显;
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
	sim800c_send_cmd("ATE1","OK",200);//回显;
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
	IWDG_Feed();//喂狗
	while(1)
	{
		IWDG_Feed();//喂狗
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
		IWDG_Feed();//喂狗
		rtklink.connectsta=5;//STATE_QUERY;
		//connectsta2=0;//
		//rtkconnectedflag=1;
	}
}

//写入转义
//发送内容中出现<Ctrl+Z>即0x1a则转义为0x03 0x1a
//0x1B转为0x03 0x1B,0x03转为0x03 0x03?
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
