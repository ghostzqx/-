#ifndef __GPS_H
#define __GPS_H	 
#include "sys.h"  
#include "protocol808.h"
#include "GlobalVariables.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//ATK-NEO-6M GPSģ����������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//�޸�����:2014/10/26
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved							  
////////////////////////////////////////////////////////////////////////////////// 	   
typedef struct
{
	double pos[3];//��γ��
	double vel[3];//������
	double time;
	int solq;//��λ����Ƿ����
	double pdop;//pdopֵ
	double hdop;//hdopֵ
	double pacc;//ˮƽ��λ���Ⱦ�����
}UM220data;


#define PI          (3.1415926535897932)  /* pi */
#define D2R         (PI/180.0)          /* deg to rad */
#define R2D         (180.0/PI)          /* rad to deg */
#define NMEAMAXFIELD	(64)  
#define NMEAMAXSENTENCE	(10)  
#define KNOTS2KMH	(1.852)		//�ڵ�����/Сʱ

int Base64Encode( char *OrgString, char *Base64String, int OrgStringLen );
int GPRS_RTCM23_Analysis(u8 *gprsrtcm,u8 *buf,unsigned int len);
int decode_nmea(char *buff, ClientPos *nmea);
int decode_nmeas(char *buff, ClientPos *nmea);
int GPRS_Config_Analysis(u8 *configmsgin,u8 *configmsgout,unsigned int len);
void NMEA_GPGGA_Analysis1(u8 *gpsx,u8 *buf);



#endif  

 



