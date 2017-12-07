#include "dma.h"
//#include "stm32f4xx_hal_dma.h"
//#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK STM32F429开发板
//DMA驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2016/1/13
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
//DMAx的各通道配置
//这里的传输形式是固定的,这点要根据不同的情况来修改
//从存储器->外设模式/8位数据宽度/存储器增量模式
//DMA_Streamx:DMA数据流,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMA通道选择,@ref DMA_channel DMA_CHANNEL_0~DMA_CHANNEL_7
//void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx)
//{ 
//	if((u32)DMA_Streamx>(u32)DMA2)//得到当前stream是属于DMA2还是DMA1
//	{
//        __HAL_RCC_DMA2_CLK_ENABLE();//DMA2时钟使能	
//	}else 
//	{
//        __HAL_RCC_DMA1_CLK_ENABLE();//DMA1时钟使能 
//	}
//    
//    __HAL_LINKDMA(&UART1_Handler,hdmatx,UART1TxDMA_Handler);    //将DMA与USART1联系起来(发送DMA)
//    
//    //Tx DMA配置
//    UART1TxDMA_Handler.Instance=DMA_Streamx;                            //数据流选择
//    UART1TxDMA_Handler.Init.Channel=chx;                                //通道选择
//    UART1TxDMA_Handler.Init.Direction=DMA_MEMORY_TO_PERIPH;             //存储器到外设
//    UART1TxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //外设非增量模式
//    UART1TxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //存储器增量模式
//    UART1TxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //外设数据长度:8位
//    UART1TxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //存储器数据长度:8位
//    UART1TxDMA_Handler.Init.Mode=DMA_NORMAL;                            //外设普通模式
//    UART1TxDMA_Handler.Init.Priority=DMA_PRIORITY_HIGH;                 //高等优先级
//    UART1TxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
//    UART1TxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
//    UART1TxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //存储器突发单次传输
//    UART1TxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //外设突发单次传输
//	

//	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn,1,2);       //抢占优先级为2，子优先级为0	
//	NVIC_EnableIRQ(DMA2_Stream7_IRQn);	
//	
//    HAL_DMA_DeInit(&UART1TxDMA_Handler);   	
//    HAL_DMA_Init(&UART1TxDMA_Handler);
//} 
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u8 chx,u32 par,u32 mar,u16 ndtr)
{ 
	DMA_TypeDef *DMAx;
	u8 streamx;
	if((u32)DMA_Streamx>(u32)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
		DMAx=DMA2;
		RCC->AHB1ENR|=1<<22;//DMA2时钟使能 
	}else 
	{
		DMAx=DMA1; 
 		RCC->AHB1ENR|=1<<21;//DMA1时钟使能 
	}
	while(DMA_Streamx->CR&0X01);//等待DMA可配置 
	streamx=(((u32)DMA_Streamx-(u32)DMAx)-0X10)/0X18;		//得到stream号
 	if(streamx>=6)DMAx->HIFCR|=0X3D<<(6*(streamx-6)+16);	//清空之前该stream上的所有中断标志
	else if(streamx>=4)DMAx->HIFCR|=0X3D<<6*(streamx-4);    //清空之前该stream上的所有中断标志
	else if(streamx>=2)DMAx->LIFCR|=0X3D<<(6*(streamx-2)+16);//清空之前该stream上的所有中断标志
	else DMAx->LIFCR|=0X3D<<6*streamx;						//清空之前该stream上的所有中断标志
		
	
	DMA_Streamx->PAR=par;		//DMA外设地址
	DMA_Streamx->M0AR=mar;		//DMA 存储器0地址
	DMA_Streamx->NDTR=ndtr;		//DMA 存储器0地址
	
	DMA_Streamx->CR=0;			//先全部复位CR寄存器值
//	DMA_Streamx->CR|=1<<4;		//传输完成中断使能
	DMA_Streamx->CR|=1<<6;		//存储器到外设模式
	DMA_Streamx->CR|=0<<8;		//非循环模式(即使用普通模式)
	DMA_Streamx->CR|=0<<9;		//外设非增量模式
	DMA_Streamx->CR|=1<<10;		//存储器增量模式
	DMA_Streamx->CR|=0<<11;		//外设数据长度:8位
	DMA_Streamx->CR|=0<<13;		//存储器数据长度:8位
	DMA_Streamx->CR|=1<<16;		//中等优先级
	DMA_Streamx->CR|=0<<21;		//外设突发单次传输
	DMA_Streamx->CR|=0<<23;		//存储器突发单次传输

	DMA_Streamx->CR|=(u32)chx<<25;//通道选择
//		
//	NVIC_EnableIRQ(DMA2_Stream7_IRQn);
//	NVIC_SetPriority(DMA2_Stream7_IRQn,2);       //抢占优先级为2，子优先级为0	
//	
	//DMA_Streamx->FCR=0X21;	//FIFO控制寄存器
} 
//开启一次DMA传输
//huart:串口句柄
//pData：传输的数据指针
//Size:传输的数据量
//void MYDMA_USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
//{
//    HAL_DMA_Start_IT(huart->hdmatx, (u32)pData, (uint32_t)&huart->Instance->DR, Size);//开启DMA传输
//    huart->Instance->CR3 |= USART_CR3_DMAT;//使能串口DMA发送
//}	

void MYDMA_Config_uartrec(DMA_Stream_TypeDef *DMA_Streamx,u8 chx,u32 par,u32 mar,u16 ndtr)
{ 
	DMA_TypeDef *DMAx;
	u8 streamx;
	if((u32)DMA_Streamx>(u32)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
		DMAx=DMA2;
		RCC->AHB1ENR|=1<<22;//DMA2时钟使能 
	}else 
	{
		DMAx=DMA1; 
 		RCC->AHB1ENR|=1<<21;//DMA1时钟使能 
	}
	while(DMA_Streamx->CR&0X01);//等待DMA可配置 
	streamx=(((u32)DMA_Streamx-(u32)DMAx)-0X10)/0X18;		//得到stream号
 	if(streamx>=6)DMAx->HIFCR|=0X3D<<(6*(streamx-6)+16);	//清空之前该stream上的所有中断标志
	else if(streamx>=4)DMAx->HIFCR|=0X3D<<6*(streamx-4);    //清空之前该stream上的所有中断标志
	else if(streamx>=2)DMAx->LIFCR|=0X3D<<(6*(streamx-2)+16);//清空之前该stream上的所有中断标志
	else DMAx->LIFCR|=0X3D<<6*streamx;						//清空之前该stream上的所有中断标志
		
	
	DMA_Streamx->PAR=par;		//DMA外设地址
	DMA_Streamx->M0AR=mar;		//DMA 存储器0地址
	DMA_Streamx->NDTR=ndtr;		//DMA 存储器0地址
	
	DMA_Streamx->CR=0;			//先全部复位CR寄存器值
	DMA_Streamx->CR|=0<<4;		//传输完成中断使能
	DMA_Streamx->CR|=0<<6;		//外设到存储器模式
	DMA_Streamx->CR|=0<<8;		//非循环模式(即使用普通模式)
	DMA_Streamx->CR|=0<<9;		//外设非增量模式
	DMA_Streamx->CR|=1<<10;		//存储器增量模式
	DMA_Streamx->CR|=0<<11;		//外设数据长度:8位
	DMA_Streamx->CR|=0<<13;		//存储器数据长度:8位
	DMA_Streamx->CR|=1<<16;		//中等优先级
	DMA_Streamx->CR|=0<<21;		//外设突发单次传输
	DMA_Streamx->CR|=0<<23;		//存储器突发单次传输

	DMA_Streamx->CR|=(u32)chx<<25;//通道选择
	DMA_Streamx->CR|=1<<0;//开启
//		
//	NVIC_EnableIRQ(DMA2_Stream7_IRQn);
//	NVIC_SetPriority(DMA2_Stream7_IRQn,2);       //抢占优先级为2，子优先级为0	
//	
	//DMA_Streamx->FCR=0X21;	//FIFO控制寄存器
} 



void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{	

	DMA_TypeDef *DMAx;
	u8 streamx;
	if((u32)DMA_Streamx>(u32)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
		DMAx=DMA2;
		RCC->AHB1ENR|=1<<22;//DMA2时钟使能 
	}else 
	{
		DMAx=DMA1; 
 		RCC->AHB1ENR|=1<<21;//DMA1时钟使能 
	}
	while(DMA_Streamx->CR&0X01);//等待DMA可配置 
	streamx=(((u32)DMA_Streamx-(u32)DMAx)-0X10)/0X18;		//得到stream号
 	if(streamx>=6)DMAx->HIFCR|=0X3D<<(6*(streamx-6)+16);	//清空之前该stream上的所有中断标志
	else if(streamx>=4)DMAx->HIFCR|=0X3D<<6*(streamx-4);    //清空之前该stream上的所有中断标志
	else if(streamx>=2)DMAx->LIFCR|=0X3D<<(6*(streamx-2)+16);//清空之前该stream上的所有中断标志
	else DMAx->LIFCR|=0X3D<<6*streamx;						//清空之前该stream上的所有中断标志
//////	if((DMA1->LISR) & (1<<27))
//////	{
//////		DMA1->LIFCR |= 15<<24;
//////	}
//	if(DMA_GetFlagStatus(DMA_Streamx,DMA_FLAG_TCIF7)!=RESET)//等待DMA1_Steam3传输完成
//	{ 
//		DMA_ClearFlag(DMA_Streamx,DMA_FLAG_TCIF7);//清除DMA1_Steam3传输完成标志
//	}	
	DMA_Streamx->CR&=~(1<<0); 	//关闭DMA传输 
	while(DMA_Streamx->CR&0X1);	//确保DMA可以被设置  
	DMA_Streamx->NDTR=ndtr;		//DMA 存储器0地址 
	DMA_Streamx->CR|=1<<0;		//开启DM 

}	


void MYDMA_Disable_uartrec(DMA_Stream_TypeDef *DMA_Streamx)
{	
	DMA_Streamx->CR&=~(1<<0); 	//关闭DMA传输 
}	

void MYDMA_Enable_uartrec(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
////	if((DMA2->LISR) & (1<<21))
////	{
////		DMA2->LIFCR |= 15<<18;
////	}

	DMA_TypeDef *DMAx;
	u8 streamx;
	if((u32)DMA_Streamx>(u32)DMA2)//得到当前stream是属于DMA2还是DMA1
	{
		DMAx=DMA2;
		RCC->AHB1ENR|=1<<22;//DMA2时钟使能 
	}else 
	{
		DMAx=DMA1; 
 		RCC->AHB1ENR|=1<<21;//DMA1时钟使能 
	}
	while(DMA_Streamx->CR&0X01);//等待DMA可配置 
	streamx=(((u32)DMA_Streamx-(u32)DMAx)-0X10)/0X18;		//得到stream号
 	if(streamx>=6)DMAx->HIFCR|=0X3D<<(6*(streamx-6)+16);	//清空之前该stream上的所有中断标志
	else if(streamx>=4)DMAx->HIFCR|=0X3D<<6*(streamx-4);    //清空之前该stream上的所有中断标志
	else if(streamx>=2)DMAx->LIFCR|=0X3D<<(6*(streamx-2)+16);//清空之前该stream上的所有中断标志
	else DMAx->LIFCR|=0X3D<<6*streamx;						//清空之前该stream上的所有中断标志

	
	while(DMA_Streamx->CR&0X1);	//确保DMA可以被设置  
	DMA_Streamx->NDTR=ndtr;		//DMA 存储器0地址 
	DMA_Streamx->CR|=1<<0;		//开启DM 

}	


//中断处理函数

void DMA2_Stream7_IRQHandler(void)
{	
	if((DMA2->HISR) & (1<<27))
	{
		DMA2->HIFCR |= 15<<24; 
	}
}	



















