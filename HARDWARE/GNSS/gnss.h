#ifndef __GPS_H
#define __GPS_H	 
#include "sys.h"  
#include "protocol808.h"
#include "GlobalVariables.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F407开发板
//ATK-NEO-6M GPS模块驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//修改日期:2014/10/26
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved							  
////////////////////////////////////////////////////////////////////////////////// 	   
typedef struct
{
	double pos[3];//经纬高
	double vel[3];//东北天
	double time;
	int solq;//定位结果是否可用
	double pdop;//pdop值
	double hdop;//hdop值
	double pacc;//水平定位精度均方差
}UM220data;


#define PI          (3.1415926535897932)  /* pi */
#define D2R         (PI/180.0)          /* deg to rad */
#define R2D         (180.0/PI)          /* rad to deg */
#define NMEAMAXFIELD	(64)  
#define NMEAMAXSENTENCE	(10)  
#define KNOTS2KMH	(1.852)		//节到公里/小时

int Base64Encode( char *OrgString, char *Base64String, int OrgStringLen );
int GPRS_RTCM23_Analysis(u8 *gprsrtcm,u8 *buf,unsigned int len);
int decode_nmea(char *buff, ClientPos *nmea);
int decode_nmeas(char *buff, ClientPos *nmea);
int GPRS_Config_Analysis(u8 *configmsgin,u8 *configmsgout,unsigned int len);
void NMEA_GPGGA_Analysis1(u8 *gpsx,u8 *buf);



#endif  

 



