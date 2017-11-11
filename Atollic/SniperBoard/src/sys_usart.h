/*
 * sys_usart.h
 *
 *  Created on: Mar 20, 2017
 *      Author: Aravindan
 */

#ifndef SYS_USART_H_
#define SYS_USART_H_

#include "stm32f10x.h"

void myUSART3_Init(void);
void USART3_Send(uint8_t data);
void myUSART1_Init(void);
void USART1_Send(uint8_t data);
void GetCfg(void);
uint8_t SetCfg(uint16_t *bacsrcport, uint16_t *bacdstport, uint16_t *bacid);
uint8_t SetCfgSrv(uint8_t *srvaddr, uint16_t *srvport);
#endif /* SYS_USART_H_ */
