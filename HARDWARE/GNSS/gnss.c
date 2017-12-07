#include "gnss.h" 
#include "led.h" 
#include "delay.h" 								   
#include "usart.h" 								   
#include "stdio.h"	 
#include "stdarg.h"	 
#include "string.h"	 
#include "math.h"
#include "stdlib.h"
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

int Base64Encode( char *OrgString, char *Base64String, int OrgStringLen ) 
{
	// OrgString ????????
	// Base64String ???????????
	// OrgStringLen ????????
	static char Base64Encode[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
	int Base64StringLen = 0;

	while( OrgStringLen > 0 )
	{
		*Base64String ++ = Base64Encode[(OrgString[0] >> 2 ) & 0x3f];
		if( OrgStringLen > 2 )
		{
		 *Base64String ++ = Base64Encode[((OrgString[0] & 3) << 4) | (OrgString[1] >> 4)];
		 *Base64String ++ = Base64Encode[((OrgString[1] & 0xF) << 2) | (OrgString[2] >> 6)];
		 *Base64String ++ = Base64Encode[OrgString[2] & 0x3F];
		}
		else
		{
		 switch( OrgStringLen )
		 {
		 case 1:
			*Base64String ++ = Base64Encode[(OrgString[0] & 3) << 4 ];
			*Base64String ++ = '=';
			*Base64String ++ = '=';
			break;
		 case 2:
			*Base64String ++ = Base64Encode[((OrgString[0] & 3) << 4) | (OrgString[1] >> 4)];
			*Base64String ++ = Base64Encode[((OrgString[1] & 0x0F) << 2) | (OrgString[2] >> 6)];
			*Base64String ++ = '=';
			break;
		 }
		}
		
		OrgString +=3;
		OrgStringLen -=3;
		Base64StringLen +=4;
	}

	*Base64String = 0;
	return Base64StringLen;
}


//解析GPRS收到的RTCM语句，例句如下,80为RTCM长度
//+RECEIVE,1,80:
//fAB@k}q\||OCLAV@bBbn@]}ORb|oBY~OVK@}KtV@@Izt@Gy_XZAzwh{B\EW`}{jH@qFd{@s}HzB@@o

int GPRS_RTCM23_Analysis(u8 *gprsrtcm,u8 *buf,unsigned int len)
{
	u8 *p1,*plen,*poshead,dx;
	char val[10];
	u8 *p2=gprsrtcm;
	unsigned int posx,len1;
	unsigned int rtcmlen;//要拷贝的长度

	plen=strstr((char *)p2,"RECEIVE,1");//最后一个,的位置为pen+10
	p1=(u8*)strstr((const char *)plen,"\r\n");//冒号的位置--7600直接取0D的位置
	poshead=p1+2;//RTCM起始位置
	len1=p1-plen-10;//收到的长度
	if(p1!=NULL&&plen!=NULL&&(len1>0)&&(len1<10))
	{
		memcpy(val,plen+10,len1);
	}
	else
		return -1;
	rtcmlen=atoi(val);//
	
	
	if(poshead!=NULL&&rtcmlen<len&&rtcmlen>0)
	{
		memcpy(buf,poshead,rtcmlen);
		return rtcmlen;
	}
	
	return -1;
}

////解析GPRS收到的配置数据
//xxx+RECEIVE,0,n\r\ndata
int GPRS_Config_Analysis(u8 *configmsgin,u8 *configmsgout,unsigned int len)
{
	u8 *p1,*plen,*poshead,dx;
	char val[10];
	u8 *p2;
	unsigned int posx,len1;
	unsigned int rtcmlen;//要拷贝的长度

	if(configmsgin==NULL||configmsgout==NULL)
		return -1;
	
	p2=configmsgin;//指向最开始
	plen=strstr((char *)p2,"RECEIVE,0");//最后一个,的位置为pen+10
	if(plen!=NULL)
		p1=(u8*)strstr((char *)plen,"\r\n");//冒号的位置--7600直接取0D的位置
	else
		return -1;
	if(p1!=NULL)
	{
		poshead=p1+2;//RTCM起始位置
		len1=p1-plen-10;//收到的字符数的长度
	}
	else
		return -1;
	if((len1>0)&&(len1<10))
	{
		memcpy(val,plen+10,len1);
	}
	else
		return -1;
	rtcmlen=atoi(val);//
	
	if(poshead!=NULL&&rtcmlen<(len)&&rtcmlen>0)
	{
		memcpy(configmsgout,poshead,rtcmlen);//configmsgout=poshead;
		return rtcmlen;
	}
	
	return -1;
}
/* convert ddmm.mm in nmea format to deg -------------------------------------*/
static double dmm2deg(double dmm)
{
    return floor(dmm/100.0)+fmod(dmm,100.0)/60.0;
}
/* convert time in nmea format to time ---------------------------------------*/
//static void septime(double t, double *t1, double *t2, double *t3)
//{
//    *t1=floor(t/10000.0);
//    t-=*t1*10000.0;
//    *t2=floor(t/100.0);
//    *t3=t-*t2*100.0;
//}
/* convert time & date in nmea format to time ---------------------------------------*/
//t1:hh时 ,dd日
//t2:mm分 ,mm月
//t3:ss秒 ,yy年
static void sepdatetime(double t, u16 *t1, u16 *t2, u16 *t3)
{
	*t1=(u16)(t/10000.0);
	t-=*t1*10000.0;
	*t2=(u16)(t/100.0);
	*t3=(u16)(t-*t2*100.0);
}
/* decode nmea gprmc: recommended minumum data for gps -----------------------*/
static int decode_nmearmc(char **val, int n, ClientPos *nmea)
{
    double tod=0.0,lat=0.0,lon=0.0,vel=0.0,dir=0.0,date=0.0,ang=0.0,ep[6];
//    double pos[3]={0};
    char act=' ',ns='N',ew='E',mew='E',mode='A';
    int i;
	u16 time[6];
	//u8 timestr[13];
    
    
    for (i=0;i<n;i++) {
        switch (i) {
            case  0: tod =atof(val[i]); break; /* time in utc (hhmmss) */
            case  1: act =*val[i];      break; /* A=active,V=void */
            case  2: lat =atof(val[i]); break; /* latitude (ddmm.mmm) */
            case  3: ns  =*val[i];      break; /* N=north,S=south */
            case  4: lon =atof(val[i]); break; /* longitude (dddmm.mmm) */
            case  5: ew  =*val[i];      break; /* E=east,W=west */
            case  6: vel =atof(val[i]); break; /* speed (knots) */
            case  7: dir =atof(val[i]); break; /* track angle (deg) */
            case  8: date=atof(val[i]); break; /* date (ddmmyy) */
            case  9: ang =atof(val[i]); break; /* magnetic variation */
            case 10: mew =*val[i];      break; /* E=east,W=west */
            case 11: mode=*val[i];      break; /* mode indicator (>nmea 2) */
                                      /* A=autonomous,D=differential */
                                      /* E=estimated,N=not valid,S=simulator */
        }
    }
		if(act=='A')//模块故障
			nmea->Alarm &=~(0x10);
		else
			nmea->Alarm |=(0x10);
		nmea->Latitude=(u32)(dmm2deg(lat)*1e6);//
		nmea->Longitude=(u32)(dmm2deg(lon)*1e6);
		//nmea->Height=0;
		if(ns=='S')
			nmea->Status |= 0x4;
		else
			nmea->Status &= ~(0x4);
		if(ew=='W')
			nmea->Status |= 0x8;
		else
			nmea->Status &= ~(0x8);
		nmea->Status |= (0xC0000);//GPS+BEIDOU
		
		nmea->Vel=(u16)(vel*KNOTS2KMH*10);
		nmea->Direction=(u16)dir;
		if((vel*KNOTS2KMH)>CfgMaxSpeed)//超速
			nmea->Alarm |=0x2;
		else
			nmea->Alarm &=~(0x2);
		if(POWERIN==0)
			nmea->Alarm|=0x100;//主电源掉电
		else
			nmea->Alarm&=~(0x100);
		
		
		sepdatetime(date,&(time[2]),&(time[1]),&(time[0]));
		sepdatetime(tod,&(time[3]),&(time[4]),&(time[5]));
		sprintf(nmea->Time,"%02d%02d%02d%02d%02d%02d",time[0],time[1],time[2],time[3],time[4],time[5]);
    return 1;
}
/* decode nmea gpgga: fix information ----------------------------------------*/
static int decode_nmeagga(char **val, int n, ClientPos *nmea)
{
    double tod=0.0,lat=0.0,lon=0.0,hdop=0.0,alt=0.0,msl=0.0,ep[6],tt;
//    double pos[3]={0};
    char ns='N',ew='E',ua=' ',um=' ';
    int i,solq=0,nrcv=0;
    
    
    for (i=0;i<n;i++) {
        switch (i) {
//            case  0: tod =atof(val[i]); break; /* time in utc (hhmmss) */
//            case  1: lat =atof(val[i]); break; /* latitude (ddmm.mmm) */
//            case  2: ns  =*val[i];      break; /* N=north,S=south */
//            case  3: lon =atof(val[i]); break; /* longitude (dddmm.mmm) */
//            case  4: ew  =*val[i];      break; /* E=east,W=west */
            case  5: solq=atoi(val[i]); break; /* fix quality */
//            case  6: nrcv=atoi(val[i]); break; /* # of satellite tracked */
//            case  7: hdop=atof(val[i]); break; /* hdop */
//            case  8: alt =atof(val[i]); break; /* altitude in msl */
//            case  9: ua  =*val[i];      break; /* unit (M) */
//            case 10: msl =atof(val[i]); break; /* height of geoid */
//            case 11: um  =*val[i];      break; /* unit (M) */
        }
    }
		
		switch(solq)
		{
			case 0://失败
				nmea->Status &=~(0x1c00000);//22-24位
				break;
			case 1://单点
				nmea->Status &=~(0x1c00000);//22-24位
				nmea->Status |=(0x400000);//22-24位
				break;
			case 2://RTD
				nmea->Status &=~(0x1c00000);//22-24位
				nmea->Status |=(0x800000);//22-24位
				break;
			case 4://RTK固定
				nmea->Status &=~(0x1c00000);//22-24位
				nmea->Status |=(0x1000000);//22-24位
				break;
			case 5://RTK浮点
				nmea->Status &=~(0x1c00000);//22-24位
				nmea->Status |=(0x1400000);//22-24位
				break;
			case 6://惯导
				nmea->Status &=~(0x1c00000);//22-24位
				nmea->Status |=(0x1800000);//22-24位
				break;
			default:
				nmea->Status &=~(0x1c00000);//22-24位
				break;
		
		}
//////    if ((ns!='N'&&ns!='S')||(ew!='E'&&ew!='W')) {
////////        trace(2,"invalid nmea gpgga format\n");
//////        return 0;
//////    }
////////		if(solq!=0)
//////		{
//////			nmea->pos[0]=(ns=='N'?1.0:-1.0)*dmm2deg(lat);//*D2R;
//////			nmea->pos[1]=(ew=='E'?1.0:-1.0)*dmm2deg(lon);//*D2R;
//////			nmea->pos[2]=alt+msl;
//////		}
//////    nmea->time=tod;
////////		nmea->solq=solq;
////////		if(nmea->solq==1)
//////			nmea->solq=(hdop<3&&nrcv>5)?solq:0;//只取hdop在2.5以下，卫星数在5以上的定位结果
////////    septime(tod,ep+3,ep+4,ep+5);
////////    pos2ecef(pos,sol->rr);
    
    return 1;
}
/////* decode nmea gpgsa: fix information ----------------------------------------*/
////static int decode_nmeagsa(char **val, int n, UM220data *nmea)
////{
////	double pdop=128.0,hdop=128.0,vdop=128.0;
//////    double pos[3]={0};
////	char Smode='M';
////	int fs=1;
////	int i;
////			
////    
////	for (i=0;i<n;i++) {
////			switch (i) {
////					case  0: Smode =*val[i]; break; /* time in utc (hhmmss) */
////					case  1: fs =atoi(val[i]); break; /* latitude (ddmm.mmm) */
////					case  14: pdop =atof(val[i]); break; /* longitude (dddmm.mmm) */
////					case  15: hdop =atof(val[i]); break; /* E=east,W=west */
////					case  16: vdop =atoi(val[i]); break; /* fix quality */
////			}
////	}
////	
////	nmea->pdop=pdop;
////	nmea->hdop=hdop;
////	
////	return 1;
////}
/////* decode nmea gpacc: fix information ----------------------------------------*/
////static int decode_nmeaacc(char **val, int n, UM220data *nmea)
////{
////	double tod,pacc=128.0,vacc=128.0,cacc=128.0;
//////    double pos[3]={0};
////	 char act=' ';
////	int fs=1;
////	int i;
////			
////    
////	for (i=0;i<n;i++) {
////			switch (i) {
//////					case  0: tod =atof(val[i]);; break; /* time in utc (hhmmss) */
//////					case  1: act =*(val[i]); break; /* latitude (ddmm.mmm) */
////					case  2: pacc =atof(val[i]); break; /* longitude (dddmm.mmm) */
//////					case  3: vacc =atof(val[i]); break; /* E=east,W=west */
//////					case  4: cacc =atoi(val[i]); break; /* fix quality */
////			}
////	}
////	
////	nmea->pacc=pacc;
////	
////	return 1;
////}

/* decode nmea ---------------------------------------------------------------*/
int decode_nmea(char *buff, ClientPos *nmea){
	char *p,*q,*val[NMEAMAXFIELD];
	int n=0;

	//	/* parse fields */
	for (p=buff;*p&&n<NMEAMAXFIELD;p=q+1) {
			if ((q=strchr(p,','))!=NULL||(q=strchr(p,'*'))!=NULL) {
					val[n++]=p; *q='\0';
			}
			else break;
	}
	/* decode nmea sentence */
	if (!strcmp(val[0],"$GPRMC")||!strcmp(val[0],"$GNRMC")||!strcmp(val[0],"$BDRMC")) {
			return decode_nmearmc(val+1,n-1,nmea);
	}
	else if (!strcmp(val[0],"$GPGGA")||!strcmp(val[0],"$GNGGA")||!strcmp(val[0],"$BDGGA")) {
			return decode_nmeagga(val+1,n-1,nmea);
	}
////	else if (!strcmp(val[0],"$GPGSA")||!strcmp(val[0],"$GNGSA")||!strcmp(val[0],"$BDGSA")) {
////			return decode_nmeagsa(val+1,n-1,nmea);
////	}
////	else if (!strcmp(val[0],"$NAVACC")) {
////			return decode_nmeaacc(val+1,n-1,nmea);
////	}
	return 0;
}
/* decode nmeas ---------------------------------------------------------------*/
int decode_nmeas(char *buff, ClientPos *nmea)
{
	char *p,*q;
	char *val[NMEAMAXSENTENCE];
//	char *h,*i;
	int n=0,i=0;
	
	/* parse sentences */
	for (p=buff;*p&&n<NMEAMAXSENTENCE;p=q+1) {
		if ((q=strstr(p,"\n"))!=NULL) {//0a
			val[n++]=p;*q='\0';//*(q+1)='\0';
		}
		else break;
	}
////////	for (p=buff;*p&&n<NMEAMAXSENTENCE;p=q+2) {
////////		if ((q=strstr(p,"\r\n"))!=NULL) {//0d0a
////////			val[n++]=p;*q='\0';//*(q+1)='\0';
////////		}
////////		else break;
////////	}
	for (i=0;i<n&&n>0;i++)
	{
		decode_nmea(val[i],nmea);
	}
	
	return 0;
}

//gpsx:nmea信息结构体
//buf:接收到的GPS数据缓冲区首地址
void NMEA_GPGGA_Analysis1(u8 *gpsx,u8 *buf)
{
	u8 *p1,*p2,dx;			 
	u8 posx;    
	p1=(u8*)strstr((const char *)buf,"$GNGGA");
//	USART1_printf("%s\r\n",p1);

	//ublox
////	if(p1!=NULL)
////		p2=(u8*)strstr((const char *)p1,"\r\n");
////	if(p2!=NULL)
////	{
////		*p2='\0';
////		strcpy(gpsx,p1);
//////		USART1_printf("%s\r\n",p1);
////	}
	
	//海积
	if(p1!=NULL)
		p2=(u8*)strstr((const char *)p1,"*");
	if(p2!=NULL)
	{
		*(p2+3)='\0';
		if(strlen(p1)<200)
			strcpy(gpsx,p1);
//		USART1_printf("%s\r\n",p1);

	}
}
