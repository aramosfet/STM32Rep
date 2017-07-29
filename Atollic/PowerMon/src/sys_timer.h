/*
 * sys_timer.h
 *
 *  Created on: Jan 22, 2017
 *      Author: Aravindan
 */

#ifndef SYS_TIMER_H_
#define SYS_TIMER_H_

#include "stm32f10x.h"


#ifdef __cplusplus
extern "C" {
#endif
void sys_timer_init(void);
void Sys_Delay(uint32_t);
void delayMicroseconds(uint16_t Delay);
uint32_t Sys_GetTick();
void TIM2_IRQHandler(void);
void RTC_Configuration(void);
void Time_Show(void);
void RTC_Init(void);
void Time_Adjust(void);
void timer3_init(void);
uint16_t get_adc(void);
#ifdef __cplusplus
}
#endif

#define HAL_Delay Sys_Delay
#define HAL_GetTick Sys_GetTick

#endif /* SYS_TIMER_H_ */
