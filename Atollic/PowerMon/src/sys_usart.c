/*
 * sys_usart.c
 *
 *  Created on: Mar 20, 2017
 *      Author: Aravindan
 */

#include "sys_usart.h"
#include "stm32f10x_usart.h"

void myUSART3_Init(void)
{
	USART_InitTypeDef USART_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

	USART_InitStruct.USART_BaudRate = 115200;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No ;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	USART_Cmd(USART3, ENABLE);
	USART_Init(USART3,&USART_InitStruct);

	/* Enable RXNE interrupt */
	//USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	/* Enable USART3 global interrupt */
	NVIC_EnableIRQ(USART3_IRQn);

}

void USART3_Send(uint8_t data)
{
	while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
	USART_SendData(USART3,data);
}


void USART3_IRQHandler (void)
{

}


