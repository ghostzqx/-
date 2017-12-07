#ifndef __SIM800C_H
#define __SIM800C_H	 
#include "sys.h" 
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif



//////////////////////////////////////////////////////////////////////////////////	
//
//	SIM800Cģ����������	   
//
////////////////////////////////////////////////////////////////////////////////// 	
typedef enum{
	STATE_CONNECTED = 1,
	STATE_RECONNECT = 2,
	STATE_QUERY = 0,
	STATE_AUTHORIZED = 3,
}CONNECTSTA_ENUM;

typedef struct{
	u8 connectsta;//����״̬��־
	u8 losscount;//���ڼ���ͨ��ģ���Ƿ�������
	u16 querytimes;//�Ѳ�ѯ����
	u32 timex;//���ڼ���
	u8 updatastep;//�����ϴ��������
	u8 reconnectstep;//�������Ӳ������
	u8 ntripauthstep;//ntrip��֤�������
	u8 ntripnmeastep;//ntrip NMEA���������
	u8 reconnecttimes;//�������Ӵ���
}statemachine;


u8* sim800c_check_cmd(u8 *str);
u8 sim800c_send_cmd(u8 *cmd,u8 *ack,u16 waittime);

u8 SIM800C_AT_CMD(u8 *cmd,u8 *ack);
u8 sim800c_gprs_prepare(void);
u8 OSsim800c_gprs_prepare(OS_ERR* err);
u8 sim800c_tcpudp_connect(u8 mode,u8* ipaddr,u32 port,int id);
u8 OSsim7X00_gprs_prepare(OS_ERR* err);
u8 sim7X00_tcpudp_connect(u8 mode,u8* ipaddr,u32 port,int id);
void sim_powerup(void);
void OSsim_powerup(OS_ERR* err);
void sim_init(void);
void OSsim_init(OS_ERR* err);
void sim_reinit(void);
int GPRSWriteTrans(u8 dataout[],u16 length,u8 datain[]);//GPRSת��




#endif  
















