/*
 * sys_timer.c
 *
 *  Created on: Jan 22, 2017
 *      Author: Aravindan
 */

#include "sys_timer.h"
#include <stdio.h>
#include <stddef.h>
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "calendar.h"
#include "sys_usart.h"
volatile uint32_t systick = 0;
volatile uint8_t led1 = 0;
void sys_timer_init(void) {
	TIM_TimeBaseInitTypeDef TIM_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM2EN, ENABLE);

	TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	//TIM_InitStruct.TIM_Period = 4000;
	TIM_InitStruct.TIM_Period = 12000;
	TIM_InitStruct.TIM_Prescaler = 5;
	//TIM_InitStruct.TIM_RepetitionCounter = 2;
	TIM_TimeBaseInit(TIM2, &TIM_InitStruct);

	NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn; /*TIM3 interrupt*/
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; /*Preemptive priority level 0*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; /*From the priority level 3*/
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; /*The IRQ channel is enabled*/
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM2, TIM_IT_Update, ENABLE);

	TIM_Cmd(TIM2, ENABLE);
	NVIC_EnableIRQ(TIM2_IRQn);

}

void Sys_Delay(uint32_t Delay) {
	uint32_t tickstart = 0;
	tickstart = Sys_GetTick();
	while ((Sys_GetTick() - tickstart) < Delay) {
	}
}
void delayMicroseconds(uint16_t Delay)
{
	uint16_t tickstart = 0;
	tickstart = Sys_GetTick();
	while ((Sys_GetTick() - tickstart) < Delay) {
	}
}
uint32_t Sys_GetTick() {
	return systick;
}

void TIM2_IRQHandler(void) {
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) != RESET) /*Check the TIM3 update interrupt occurs or not*/
	{
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update); /*Remove TIMx update interrupt flag */
        led1 = !led1;
        GPIO_WriteBit(GPIOB,GPIO_Pin_7,led1);
		systick++;
	}
}
void RTC_Init(void)
{
	  if (BKP_ReadBackupRegister(BKP_DR1) != 0xA5A5)
	  {
	    /* Backup data register value is not correct or not yet programmed (when
	       the first time the program is executed) */

	    printf("\r\n\n RTC not yet configured....");

	    /* RTC Configuration */
	    RTC_Configuration();

	    printf("\r\n RTC configured....");

	    /* Adjust time by values entered by the user on the hyperterminal */
	    Time_Adjust();

	    BKP_WriteBackupRegister(BKP_DR1, 0xA5A5);
	  }
	  else
	  {
	    /* Check if the Power On Reset flag is set */
	    if (RCC_GetFlagStatus(RCC_FLAG_PORRST) != RESET)
	    {
	      printf("\r\n\n Power On Reset occurred....");
	    }
	    /* Check if the Pin Reset flag is set */
	    else if (RCC_GetFlagStatus(RCC_FLAG_PINRST) != RESET)
	    {
	      printf("\r\n\n External Reset occurred....");
	    }

	    printf("\r\n No need to configure RTC....");
	    /* Wait for RTC registers synchronization */
	    RTC_WaitForSynchro();

	    /* Enable the RTC Second */
	    RTC_ITConfig(RTC_IT_SEC, ENABLE);
	    /* Wait until last write operation on RTC registers has finished */
	    RTC_WaitForLastTask();
	  }

	#ifdef RTCClockOutput_Enable
	  /* Enable PWR and BKP clocks */
	  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

	  /* Allow access to BKP Domain */
	  PWR_BackupAccessCmd(ENABLE);

	  /* Disable the Tamper Pin */
	  BKP_TamperPinCmd(DISABLE); /* To output RTCCLK/64 on Tamper pin, the tamper
	                                 functionality must be disabled */

	  /* Enable RTC Clock Output on Tamper Pin */
	  BKP_RTCOutputConfig(BKP_RTCOutputSource_CalibClock);
	#endif

	  /* Clear reset flags */
	  RCC_ClearFlag();

	  /* Display time in infinite loop */
	  //Time_Show();
}
void RTC_Configuration(void)
{
  /* Enable PWR and BKP clocks */
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);

  /* Allow access to BKP Domain */
  PWR_BackupAccessCmd(ENABLE);

  /* Reset Backup Domain */
  BKP_DeInit();

  /* Enable LSE */
  RCC_LSEConfig(RCC_LSE_ON);
  /* Wait till LSE is ready */
  while (RCC_GetFlagStatus(RCC_FLAG_LSERDY) == RESET)
  {}

  /* Select LSE as RTC Clock Source */
  RCC_RTCCLKConfig(RCC_RTCCLKSource_LSE);

  /* Enable RTC Clock */
  RCC_RTCCLKCmd(ENABLE);

  /* Wait for RTC registers synchronization */
  RTC_WaitForSynchro();

  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();

  /* Enable the RTC Second */
  RTC_ITConfig(RTC_IT_SEC, ENABLE);

  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();

  /* Set RTC prescaler: set RTC period to 1sec */
  //RTC_SetPrescaler(32767); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
  RTC_SetPrescaler(32773); /* RTC period = RTCCLK/RTC_PR = (32.768 KHz)/(32767+1) */
  /* Wait until last write operation on RTC registers has finished */
  RTC_WaitForLastTask();
}

uint16_t USART_Scanf(uint16_t value,uint8_t len)
{
	uint8_t index = 0;
	uint8_t tmp[4] = {0, 0, 0, 0};
	uint16_t ret_val = 0;

	while (index < len)
	{
		/* Loop until RXNE = 1 */
		while (USART_GetFlagStatus(USART1, USART_FLAG_RXNE) == RESET)
		{}
		tmp[index] = (USART_ReceiveData(USART1));
		USART1_Send(tmp[index++]);
		if ((tmp[index - 1] < 0x30) || (tmp[index - 1] > 0x39))
		{
			printf("\n\rPlease enter valid number between 0 and 9");
			index--;
		}
	}
	/* Calculate the Corresponding value */
	if(len == 4){
		ret_val = (tmp[3] - 0x30) + ((tmp[2] - 0x30) * 10) + ((tmp[1] - 0x30) * 100) + ((tmp[0] - 0x30) * 1000);
	}else if(len == 2){
		ret_val = (tmp[1] - 0x30) + ((tmp[0] - 0x30) * 10);
	}
	/* Checks */
	if (ret_val > value)
	{
		printf("\n\rPlease enter valid number between 0 and %d ", value);
		return 0xFF;
	}
	return ret_val;
}

/**
  * @brief  Returns the time entered by user, using Hyperterminal.
  * @param  None
  * @retval Current time RTC counter value
  */
uint32_t Time_Regulate(void)
{
	uint16_t Tmp_YY = 0xFFFF;
	uint8_t Tmp_MO = 0xFF, Tmp_DD = 0xFF, Tmp_HH = 0xFF, Tmp_MM = 0xFF, Tmp_SS = 0xFF;

	printf("\r\n==============Time Settings=====================================");
	printf("\r\n  Please Set Year:");
	while (Tmp_YY == 0xFFFF)
	{
	 Tmp_YY = USART_Scanf(2099,4);
	}
	printf("\r\n  Please Set Month:");
	while (Tmp_MO == 0xFF)
	{
	 Tmp_MO = USART_Scanf(12,2);
	}
	printf("\r\n  Please Set Date:");
	while (Tmp_DD == 0xFF)
	{
	 Tmp_DD = USART_Scanf(31,2);
	}
	printf("\r\n  Please Set Hours:");

	while (Tmp_HH == 0xFF)
	{
	Tmp_HH = USART_Scanf(23,2);
	}
	//printf(":  %d", Tmp_HH);
	printf("\r\n  Please Set Minutes:");
	while (Tmp_MM == 0xFF)
	{
	Tmp_MM = USART_Scanf(59,2);
	}
	//printf(":  %d", Tmp_MM);
	printf("\r\n  Please Set Seconds:");
	while (Tmp_SS == 0xFF)
	{
	Tmp_SS = USART_Scanf(59,2);
	}
	//printf(":  %d", Tmp_SS);


	return(set_RTC(Tmp_YY,  Tmp_MO,  Tmp_DD,  Tmp_HH,  Tmp_MM,  Tmp_SS));
	/* Return the value to store in RTC counter register */
	//return((Tmp_HH*3600 + Tmp_MM*60 + Tmp_SS));
}

/**
  * @brief  Adjusts time.
  * @param  None
  * @retval None
  */
void Time_Adjust(void)
{
	uint32_t rtc_time;
	rtc_time = Time_Regulate();
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
	/* Change the current time */
	//Time_Regulate();
	RTC_SetCounter(rtc_time);
	/* Wait until last write operation on RTC registers has finished */
	RTC_WaitForLastTask();
}

/**
  * @brief  Displays the current time.
  * @param  TimeVar: RTC counter value.
  * @retval None
  */
void Time_Display(uint32_t TimeVar)
{
  uint32_t THH = 0, TMM = 0, TSS = 0;

  /* Reset RTC Counter when Time is 23:59:59 */
  if (RTC_GetCounter() == 0x0001517F)
  {
     RTC_SetCounter(0x0);
     /* Wait until last write operation on RTC registers has finished */
     RTC_WaitForLastTask();
  }

  /* Compute  hours */
  THH = TimeVar / 3600;
  /* Compute minutes */
  TMM = (TimeVar % 3600) / 60;
  /* Compute seconds */
  TSS = (TimeVar % 3600) % 60;

  printf("Time: %d:%d:%d\r", (int)THH, (int)TMM, (int)TSS);
}

/**
  * @brief  Shows the current time (HH:MM:SS) on the Hyperterminal.
  * @param  None
  * @retval None
  */
void Time_Show(void)
{
 // printf("\n");

  /* Display current time */
  Time_Display(RTC_GetCounter());


}

