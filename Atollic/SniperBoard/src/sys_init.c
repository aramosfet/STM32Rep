/*
 * sys_init.c
 *
 *  Created on: Apr 22, 2017
 *      Author: Aravindan
 */
#include <stdio.h>
#include "sys_timer.h"
#include "sys_spi.h"
#include "sys_usart.h"
void myGPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_13;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOC,&GPIO_InitStruct);

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_6;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOB,&GPIO_InitStruct);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_4;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_2;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);

	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_IPD;
	GPIO_InitStruct.GPIO_Pin = GPIO_Pin_3;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_Init(GPIOA,&GPIO_InitStruct);
}

//uint8_t led =0;
///**
//  * @brief  Sets System clock frequency to 72MHz and configure HCLK, PCLK2
//  *         and PCLK1 prescalers.
//  * @note   This function should be used only after reset.
//  * @param  None
//  * @retval None
//  */
//static void SetClock(void)
//{
//  __IO uint32_t StartUpCounter = 0, HSEStatus = 0;
//
//  /* SYSCLK, HCLK, PCLK2 and PCLK1 configuration ---------------------------*/
//  /* Enable HSE */
//  RCC->CR |= ((uint32_t)RCC_CR_HSEON);
//
//  /* Wait till HSE is ready and if Time out is reached exit */
//  do
//  {
//    HSEStatus = RCC->CR & RCC_CR_HSERDY;
//    StartUpCounter++;
//    led = !led;
//    GPIO_WriteBit(GPIOC,GPIO_Pin_13,led);
//  } while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));
//
//  if ((RCC->CR & RCC_CR_HSERDY) != RESET)
//  {
//    HSEStatus = (uint32_t)0x01;
//  }
//  else
//  {
//    HSEStatus = (uint32_t)0x00;
//  }
//
//  if (HSEStatus == (uint32_t)0x01)
//  {
////    /* Enable Prefetch Buffer */
////    FLASH->ACR |= FLASH_ACR_PRFTBE;
////
////    /* Flash 2 wait state */
////    FLASH->ACR &= (uint32_t)((uint32_t)~FLASH_ACR_LATENCY);
////    FLASH->ACR |= (uint32_t)FLASH_ACR_LATENCY_2;
//
//
//    /* HCLK = SYSCLK */
//    RCC->CFGR |= (uint32_t)RCC_CFGR_HPRE_DIV1;
//
//    /* PCLK2 = HCLK */
//    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE2_DIV1;
//
//    /* PCLK1 = HCLK */
//    RCC->CFGR |= (uint32_t)RCC_CFGR_PPRE1_DIV2;
//
//#ifdef STM32F10X_CL
//    /* Configure PLLs ------------------------------------------------------*/
//    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
//    /* PREDIV1 configuration: PREDIV1CLK = PLL2 / 5 = 8 MHz */
//
//    RCC->CFGR2 &= (uint32_t)~(RCC_CFGR2_PREDIV2 | RCC_CFGR2_PLL2MUL |
//                              RCC_CFGR2_PREDIV1 | RCC_CFGR2_PREDIV1SRC);
//    RCC->CFGR2 |= (uint32_t)(RCC_CFGR2_PREDIV2_DIV5 | RCC_CFGR2_PLL2MUL8 |
//                             RCC_CFGR2_PREDIV1SRC_PLL2 | RCC_CFGR2_PREDIV1_DIV5);
//
//    /* Enable PLL2 */
//    RCC->CR |= RCC_CR_PLL2ON;
//    /* Wait till PLL2 is ready */
//    while((RCC->CR & RCC_CR_PLL2RDY) == 0)
//    {
//    }
//
//
//    /* PLL configuration: PLLCLK = PREDIV1 * 9 = 72 MHz */
//    RCC->CFGR &= (uint32_t)~(RCC_CFGR_PLLXTPRE | RCC_CFGR_PLLSRC | RCC_CFGR_PLLMULL);
//    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLXTPRE_PREDIV1 | RCC_CFGR_PLLSRC_PREDIV1 |
//                            RCC_CFGR_PLLMULL9);
//#else
//    /*  PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
//    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_PLLSRC | RCC_CFGR_PLLXTPRE |
//                                        RCC_CFGR_PLLMULL));
//    RCC->CFGR |= (uint32_t)(RCC_CFGR_PLLSRC_HSE | RCC_CFGR_PLLMULL9);
//#endif /* STM32F10X_CL */
//
//    /* Enable PLL */
//    RCC->CR |= RCC_CR_PLLON;
//
//    /* Wait till PLL is ready */
//    while((RCC->CR & RCC_CR_PLLRDY) == 0)
//    {
//
//    }
//
//    /* Select PLL as system clock source */
//    RCC->CFGR &= (uint32_t)((uint32_t)~(RCC_CFGR_SW));
//    RCC->CFGR |= (uint32_t)RCC_CFGR_SW_PLL;
//
//    /* Wait till PLL is used as system clock source */
//    while ((RCC->CFGR & (uint32_t)RCC_CFGR_SWS) != (uint32_t)0x08)
//    {
//
//    }
//  }
//  else
//  { /* If HSE fails to start-up, the application will have wrong clock
//         configuration. User can add here some code to deal with this error */
//  }
//}

void sys_init(void)
{

	//SetClock();
	myGPIO_Init();
	sys_timer_init();
	myUSART3_Init();
	myUSART1_Init();
	mySPI2_Init();
	mySPI1_Init();
	//Sys_Delay(2000);
	RTC_Init();
	GetCfg();

}
