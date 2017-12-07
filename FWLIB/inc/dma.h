#ifndef __DMA_H
#define __DMA_H
#include "sys.h"
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


//void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart);
//void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u32 chx);
void MYDMA_Config(DMA_Stream_TypeDef *DMA_Streamx,u8 chx,u32 par,u32 mar,u16 ndtr);
void MYDMA_Config_uartrec(DMA_Stream_TypeDef *DMA_Streamx,u8 chx,u32 par,u32 mar,u16 ndtr);
void MYDMA_Disable_uartrec(DMA_Stream_TypeDef *DMA_Streamx);
void MYDMA_Enable_uartrec(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
//void MYDMA_USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size);
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
#endif
