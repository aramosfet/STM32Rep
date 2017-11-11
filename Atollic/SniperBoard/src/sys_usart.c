/*
 * sys_usart.c
 *
 *  Created on: Mar 20, 2017
 *      Author: Aravindan
 */

#include "sys_usart.h"
#include "stm32f10x_usart.h"
#include "sys_timer.h"
#include <stddef.h>
#include <stdio.h>
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


void USART1_IRQHandler (void)
{

}

void myUSART1_Init(void)
{
	USART_InitTypeDef USART_InitStruct;
	GPIO_InitTypeDef GPIO_InitStruct;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

	USART_InitStruct.USART_BaudRate = 115200;
	USART_InitStruct.USART_WordLength = USART_WordLength_8b;
	USART_InitStruct.USART_StopBits = USART_StopBits_1;
	USART_InitStruct.USART_Parity = USART_Parity_No ;
	USART_InitStruct.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_InitStruct.USART_HardwareFlowControl = USART_HardwareFlowControl_None;

	USART_Cmd(USART1, ENABLE);
	USART_Init(USART1,&USART_InitStruct);

	/* Enable RXNE interrupt */
	//USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);
	/* Enable USART3 global interrupt */
	NVIC_EnableIRQ(USART1_IRQn);

}

void USART3_IRQHandler (void)
{

}

void USART1_Send(uint8_t data)
{
	while(USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
	USART_SendData(USART1,data);
}

uint8_t rx_buffer[10];
uint16_t num_str[10];
uint16_t toString(uint8_t a[]) {
  uint16_t c, n;

  n = 0;
  for (c = 0; a[c] != ','; c++) {
    n = n * 10 + a[c] - '0';
  }

  return n;
}
uint8_t WaitForPkt(uint8_t count, uint8_t numstr)
{
	uint32_t timeout;
	timeout = Sys_GetTick()+5000;
	uint8_t i,n;
	uint8_t tmpchar;
	i=0;
	if(!numstr)
	{
		while(i<count)
		{
			while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET){
				if(timeout<Sys_GetTick())
					return 0;
			}
			rx_buffer[i] = USART_ReceiveData(USART1);
			timeout = Sys_GetTick()+5000;
			i++;
		}
		return 1;
	}else
	{


		while(i<count)
		{
			tmpchar = 0;
			n = 0;
			while(tmpchar != ',')
			{
				while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET){
					if(timeout<Sys_GetTick())
						return 0;
				}
				tmpchar = USART_ReceiveData(USART1);
				rx_buffer[n++] = tmpchar;
				timeout = Sys_GetTick()+5000;
			}
			num_str[i] = toString(rx_buffer);
			i++;
		}
		return 1;
	}
}

uint8_t srcipaddr[4] = {0,0,0,0};
uint8_t srvipaddr[4] = {0,0,0,0};
uint16_t srcportaddr,dstportaddr,bacidaddr,bdevaddr,srvportaddr = 0;
uint8_t cfgdone;

void GetCfg(void)
{
	printf("\n Waiting of Config - 5 secs timeout\n");

	//enum cfgstate{sync,srcip,srcport,dstport,bacid,instnum,srvip,srvport,bdevid} CFG;
	enum cfgstate{sync,srcip,srcport,dstport,bacid,srvip,srvport} CFG;
	CFG = sync;
	cfgdone = 0;
	while(!cfgdone)
	{
		switch(CFG){
		case sync : 	if(WaitForPkt(4,0))
						{
//							printf("Received 4 chars\n");
//							USART1_Send(rx_buffer[0]);
//							USART1_Send(rx_buffer[1]);
//							USART1_Send(rx_buffer[2]);
							USART1_Send(rx_buffer[3]);
							if( (rx_buffer[0] == 'C') &&
								(rx_buffer[1] == 'F') &&
								(rx_buffer[2] == 'G') &&
								(rx_buffer[3] == ':')){
								CFG = srcip;
							}
						}else{
							return;
						}
						break;
		case srcip : 	if(WaitForPkt(4,1)){
							//printf("received num %d %d %d %d",num_str[0],num_str[1],num_str[2],num_str[3]);
							srcipaddr[0] = num_str[0];
							srcipaddr[1] = num_str[1];
							srcipaddr[2] = num_str[2];
							srcipaddr[3] = num_str[3];
							CFG = srcport;
						}else{
							return;
						}
						break;
		case srcport : 	if(WaitForPkt(1,1)){
							//printf("received num %d %d %d %d",num_str[0],num_str[1],num_str[2],num_str[3]);
							srcportaddr = num_str[0];
							CFG = dstport;
						}else{
							return;
						}
						break;
		case dstport : 	if(WaitForPkt(1,1)){
							//printf("received num %d %d %d %d",num_str[0],num_str[1],num_str[2],num_str[3]);
							dstportaddr = num_str[0];
							CFG = bacid;
						}else{
							return;
						}
						break;
		case bacid : 	if(WaitForPkt(1,1)){
							//printf("received num %d %d %d %d",num_str[0],num_str[1],num_str[2],num_str[3]);
							bacidaddr = num_str[0];
							CFG = srvip;
						}else{
							return;
						}
						break;
		case srvip : 	if(WaitForPkt(4,1)){
							//printf("received num %d %d %d %d",num_str[0],num_str[1],num_str[2],num_str[3]);
							srvipaddr[0] = num_str[0];
							srvipaddr[1] = num_str[1];
							srvipaddr[2] = num_str[2];
							srvipaddr[3] = num_str[3];
							CFG = srvport;

						}else{
							return;
						}
						break;
		case srvport : 	if(WaitForPkt(1,1)){
							//printf("received num %d %d %d %d",num_str[0],num_str[1],num_str[2],num_str[3]);
							srvportaddr = num_str[0];
							cfgdone =1;
						}else{
							return;
						}
						break;
		default :	    break;
		}
	}
	printf("Config Complete SRC IP - %d.%d.%d.%d \nServer IP - %d.%d.%d.%d \nSRC - Port%d DST Port - %d \nBacnet ID - %d \nServer Port - %d" ,srcipaddr[0]
														,srcipaddr[1]
														,srcipaddr[2]
														,srcipaddr[3]
													    ,srvipaddr[0]
														,srvipaddr[1]
														,srvipaddr[2]
														,srvipaddr[3]
														,srcportaddr
														,dstportaddr
														,bacidaddr
														,srvportaddr);

}

uint8_t SetCfg(uint16_t *bacsrcport, uint16_t *bacdstport, uint16_t *bacid)
{
	if(cfgdone)
	{
		*bacsrcport = srcportaddr;
		*bacdstport = dstportaddr;
		*bacid = bacidaddr;
		return 1;
	}else{
		return 0;
	}
}
uint8_t SetCfgSrv(uint8_t *srvaddr, uint16_t *srvport)
{
	if(cfgdone)
	{
		srvaddr[0] = srvipaddr[0];
		srvaddr[1] = srvipaddr[1];
		srvaddr[2] = srvipaddr[2];
		srvaddr[3] = srvipaddr[3];
		*srvport = srvportaddr;
		return 1;
	}else{
		return 0;
	}
}
