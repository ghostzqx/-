#include "dma.h"
//#include "stm32f4xx_hal_dma.h"
//#include "usart.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F429������
//DMA��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2016/1/13
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	
//DMAx�ĸ�ͨ������
//����Ĵ�����ʽ�ǹ̶���,���Ҫ���ݲ�ͬ��������޸�
//�Ӵ洢��->����ģʽ/8λ���ݿ��/�洢������ģʽ
//DMA_Streamx:DMA������,DMA1_Stream0~7/DMA2_Stream0~7
//chx:DMAͨ��ѡ��,@ref DMA_channel DMA_CHANNEL_0~DMA_CHANNEL_7
//void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx)
//{ 
//	if((u32)DMA_Streamx>(u32)DMA2)//�õ���ǰstream������DMA2����DMA1
//	{
//        __HAL_RCC_DMA2_CLK_ENABLE();//DMA2ʱ��ʹ��	
//	}else 
//	{
//        __HAL_RCC_DMA1_CLK_ENABLE();//DMA1ʱ��ʹ�� 
//	}
//    
//    __HAL_LINKDMA(&UART1_Handler,hdmatx,UART1TxDMA_Handler);    //��DMA��USART1��ϵ����(����DMA)
//    
//    //Tx DMA����
//    UART1TxDMA_Handler.Instance=DMA_Streamx;                            //������ѡ��
//    UART1TxDMA_Handler.Init.Channel=chx;                                //ͨ��ѡ��
//    UART1TxDMA_Handler.Init.Direction=DMA_MEMORY_TO_PERIPH;             //�洢��������
//    UART1TxDMA_Handler.Init.PeriphInc=DMA_PINC_DISABLE;                 //���������ģʽ
//    UART1TxDMA_Handler.Init.MemInc=DMA_MINC_ENABLE;                     //�洢������ģʽ
//    UART1TxDMA_Handler.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;    //�������ݳ���:8λ
//    UART1TxDMA_Handler.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;       //�洢�����ݳ���:8λ
//    UART1TxDMA_Handler.Init.Mode=DMA_NORMAL;                            //������ͨģʽ
//    UART1TxDMA_Handler.Init.Priority=DMA_PRIORITY_HIGH;                 //�ߵ����ȼ�
//    UART1TxDMA_Handler.Init.FIFOMode=DMA_FIFOMODE_DISABLE;              
//    UART1TxDMA_Handler.Init.FIFOThreshold=DMA_FIFO_THRESHOLD_FULL;      
//    UART1TxDMA_Handler.Init.MemBurst=DMA_MBURST_SINGLE;                 //�洢��ͻ�����δ���
//    UART1TxDMA_Handler.Init.PeriphBurst=DMA_PBURST_SINGLE;              //����ͻ�����δ���
//	

//	HAL_NVIC_SetPriority(DMA2_Stream7_IRQn,1,2);       //��ռ���ȼ�Ϊ2�������ȼ�Ϊ0	
//	NVIC_EnableIRQ(DMA2_Stream7_IRQn);	
//	
//    HAL_DMA_DeInit(&UART1TxDMA_Handler);   	
//    HAL_DMA_Init(&UART1TxDMA_Handler);
//} 
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u8 chx,u32 par,u32 mar,u16 ndtr)
{ 
	DMA_TypeDef *DMAx;
	u8 streamx;
	if((u32)DMA_Streamx>(u32)DMA2)//�õ���ǰstream������DMA2����DMA1
	{
		DMAx=DMA2;
		RCC->AHB1ENR|=1<<22;//DMA2ʱ��ʹ�� 
	}else 
	{
		DMAx=DMA1; 
 		RCC->AHB1ENR|=1<<21;//DMA1ʱ��ʹ�� 
	}
	while(DMA_Streamx->CR&0X01);//�ȴ�DMA������ 
	streamx=(((u32)DMA_Streamx-(u32)DMAx)-0X10)/0X18;		//�õ�stream��
 	if(streamx>=6)DMAx->HIFCR|=0X3D<<(6*(streamx-6)+16);	//���֮ǰ��stream�ϵ������жϱ�־
	else if(streamx>=4)DMAx->HIFCR|=0X3D<<6*(streamx-4);    //���֮ǰ��stream�ϵ������жϱ�־
	else if(streamx>=2)DMAx->LIFCR|=0X3D<<(6*(streamx-2)+16);//���֮ǰ��stream�ϵ������жϱ�־
	else DMAx->LIFCR|=0X3D<<6*streamx;						//���֮ǰ��stream�ϵ������жϱ�־
		
	
	DMA_Streamx->PAR=par;		//DMA�����ַ
	DMA_Streamx->M0AR=mar;		//DMA �洢��0��ַ
	DMA_Streamx->NDTR=ndtr;		//DMA �洢��0��ַ
	
	DMA_Streamx->CR=0;			//��ȫ����λCR�Ĵ���ֵ
//	DMA_Streamx->CR|=1<<4;		//��������ж�ʹ��
	DMA_Streamx->CR|=1<<6;		//�洢��������ģʽ
	DMA_Streamx->CR|=0<<8;		//��ѭ��ģʽ(��ʹ����ͨģʽ)
	DMA_Streamx->CR|=0<<9;		//���������ģʽ
	DMA_Streamx->CR|=1<<10;		//�洢������ģʽ
	DMA_Streamx->CR|=0<<11;		//�������ݳ���:8λ
	DMA_Streamx->CR|=0<<13;		//�洢�����ݳ���:8λ
	DMA_Streamx->CR|=1<<16;		//�е����ȼ�
	DMA_Streamx->CR|=0<<21;		//����ͻ�����δ���
	DMA_Streamx->CR|=0<<23;		//�洢��ͻ�����δ���

	DMA_Streamx->CR|=(u32)chx<<25;//ͨ��ѡ��
//		
//	NVIC_EnableIRQ(DMA2_Stream7_IRQn);
//	NVIC_SetPriority(DMA2_Stream7_IRQn,2);       //��ռ���ȼ�Ϊ2�������ȼ�Ϊ0	
//	
	//DMA_Streamx->FCR=0X21;	//FIFO���ƼĴ���
} 
//����һ��DMA����
//huart:���ھ��
//pData�����������ָ��
//Size:�����������
//void MYDMA_USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
//{
//    HAL_DMA_Start_IT(huart->hdmatx, (u32)pData, (uint32_t)&huart->Instance->DR, Size);//����DMA����
//    huart->Instance->CR3 |= USART_CR3_DMAT;//ʹ�ܴ���DMA����
//}	

void MYDMA_Config_uartrec(DMA_Stream_TypeDef *DMA_Streamx,u8 chx,u32 par,u32 mar,u16 ndtr)
{ 
	DMA_TypeDef *DMAx;
	u8 streamx;
	if((u32)DMA_Streamx>(u32)DMA2)//�õ���ǰstream������DMA2����DMA1
	{
		DMAx=DMA2;
		RCC->AHB1ENR|=1<<22;//DMA2ʱ��ʹ�� 
	}else 
	{
		DMAx=DMA1; 
 		RCC->AHB1ENR|=1<<21;//DMA1ʱ��ʹ�� 
	}
	while(DMA_Streamx->CR&0X01);//�ȴ�DMA������ 
	streamx=(((u32)DMA_Streamx-(u32)DMAx)-0X10)/0X18;		//�õ�stream��
 	if(streamx>=6)DMAx->HIFCR|=0X3D<<(6*(streamx-6)+16);	//���֮ǰ��stream�ϵ������жϱ�־
	else if(streamx>=4)DMAx->HIFCR|=0X3D<<6*(streamx-4);    //���֮ǰ��stream�ϵ������жϱ�־
	else if(streamx>=2)DMAx->LIFCR|=0X3D<<(6*(streamx-2)+16);//���֮ǰ��stream�ϵ������жϱ�־
	else DMAx->LIFCR|=0X3D<<6*streamx;						//���֮ǰ��stream�ϵ������жϱ�־
		
	
	DMA_Streamx->PAR=par;		//DMA�����ַ
	DMA_Streamx->M0AR=mar;		//DMA �洢��0��ַ
	DMA_Streamx->NDTR=ndtr;		//DMA �洢��0��ַ
	
	DMA_Streamx->CR=0;			//��ȫ����λCR�Ĵ���ֵ
	DMA_Streamx->CR|=0<<4;		//��������ж�ʹ��
	DMA_Streamx->CR|=0<<6;		//���赽�洢��ģʽ
	DMA_Streamx->CR|=0<<8;		//��ѭ��ģʽ(��ʹ����ͨģʽ)
	DMA_Streamx->CR|=0<<9;		//���������ģʽ
	DMA_Streamx->CR|=1<<10;		//�洢������ģʽ
	DMA_Streamx->CR|=0<<11;		//�������ݳ���:8λ
	DMA_Streamx->CR|=0<<13;		//�洢�����ݳ���:8λ
	DMA_Streamx->CR|=1<<16;		//�е����ȼ�
	DMA_Streamx->CR|=0<<21;		//����ͻ�����δ���
	DMA_Streamx->CR|=0<<23;		//�洢��ͻ�����δ���

	DMA_Streamx->CR|=(u32)chx<<25;//ͨ��ѡ��
	DMA_Streamx->CR|=1<<0;//����
//		
//	NVIC_EnableIRQ(DMA2_Stream7_IRQn);
//	NVIC_SetPriority(DMA2_Stream7_IRQn,2);       //��ռ���ȼ�Ϊ2�������ȼ�Ϊ0	
//	
	//DMA_Streamx->FCR=0X21;	//FIFO���ƼĴ���
} 



void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{	

	DMA_TypeDef *DMAx;
	u8 streamx;
	if((u32)DMA_Streamx>(u32)DMA2)//�õ���ǰstream������DMA2����DMA1
	{
		DMAx=DMA2;
		RCC->AHB1ENR|=1<<22;//DMA2ʱ��ʹ�� 
	}else 
	{
		DMAx=DMA1; 
 		RCC->AHB1ENR|=1<<21;//DMA1ʱ��ʹ�� 
	}
	while(DMA_Streamx->CR&0X01);//�ȴ�DMA������ 
	streamx=(((u32)DMA_Streamx-(u32)DMAx)-0X10)/0X18;		//�õ�stream��
 	if(streamx>=6)DMAx->HIFCR|=0X3D<<(6*(streamx-6)+16);	//���֮ǰ��stream�ϵ������жϱ�־
	else if(streamx>=4)DMAx->HIFCR|=0X3D<<6*(streamx-4);    //���֮ǰ��stream�ϵ������жϱ�־
	else if(streamx>=2)DMAx->LIFCR|=0X3D<<(6*(streamx-2)+16);//���֮ǰ��stream�ϵ������жϱ�־
	else DMAx->LIFCR|=0X3D<<6*streamx;						//���֮ǰ��stream�ϵ������жϱ�־
//////	if((DMA1->LISR) & (1<<27))
//////	{
//////		DMA1->LIFCR |= 15<<24;
//////	}
//	if(DMA_GetFlagStatus(DMA_Streamx,DMA_FLAG_TCIF7)!=RESET)//�ȴ�DMA1_Steam3�������
//	{ 
//		DMA_ClearFlag(DMA_Streamx,DMA_FLAG_TCIF7);//���DMA1_Steam3������ɱ�־
//	}	
	DMA_Streamx->CR&=~(1<<0); 	//�ر�DMA���� 
	while(DMA_Streamx->CR&0X1);	//ȷ��DMA���Ա�����  
	DMA_Streamx->NDTR=ndtr;		//DMA �洢��0��ַ 
	DMA_Streamx->CR|=1<<0;		//����DM 

}	


void MYDMA_Disable_uartrec(DMA_Stream_TypeDef *DMA_Streamx)
{	
	DMA_Streamx->CR&=~(1<<0); 	//�ر�DMA���� 
}	

void MYDMA_Enable_uartrec(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
////	if((DMA2->LISR) & (1<<21))
////	{
////		DMA2->LIFCR |= 15<<18;
////	}

	DMA_TypeDef *DMAx;
	u8 streamx;
	if((u32)DMA_Streamx>(u32)DMA2)//�õ���ǰstream������DMA2����DMA1
	{
		DMAx=DMA2;
		RCC->AHB1ENR|=1<<22;//DMA2ʱ��ʹ�� 
	}else 
	{
		DMAx=DMA1; 
 		RCC->AHB1ENR|=1<<21;//DMA1ʱ��ʹ�� 
	}
	while(DMA_Streamx->CR&0X01);//�ȴ�DMA������ 
	streamx=(((u32)DMA_Streamx-(u32)DMAx)-0X10)/0X18;		//�õ�stream��
 	if(streamx>=6)DMAx->HIFCR|=0X3D<<(6*(streamx-6)+16);	//���֮ǰ��stream�ϵ������жϱ�־
	else if(streamx>=4)DMAx->HIFCR|=0X3D<<6*(streamx-4);    //���֮ǰ��stream�ϵ������жϱ�־
	else if(streamx>=2)DMAx->LIFCR|=0X3D<<(6*(streamx-2)+16);//���֮ǰ��stream�ϵ������жϱ�־
	else DMAx->LIFCR|=0X3D<<6*streamx;						//���֮ǰ��stream�ϵ������жϱ�־

	
	while(DMA_Streamx->CR&0X1);	//ȷ��DMA���Ա�����  
	DMA_Streamx->NDTR=ndtr;		//DMA �洢��0��ַ 
	DMA_Streamx->CR|=1<<0;		//����DM 

}	


//�жϴ�����

void DMA2_Stream7_IRQHandler(void)
{	
	if((DMA2->HISR) & (1<<27))
	{
		DMA2->HIFCR |= 15<<24; 
	}
}	



















