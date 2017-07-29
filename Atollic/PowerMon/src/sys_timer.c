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
#include "stm32f10x_spi.h"
#include "calendar.h"
#include "sys_usart.h"
#include "math.h"
volatile uint32_t systick = 0;
volatile uint8_t led1 = 0;




volatile uint16_t adcval[256];
volatile uint8_t vphaseval[256];
volatile uint8_t samplecnt = 0;
volatile uint8_t sampledone = 0;
volatile uint8_t buf_offset = 0;
volatile uint8_t vphase = 0;

const int varray[128] = {	0,11,22,33,44,55,65,76,86,96,106,116,125,134,143,151,159,167,
							174,181,187,193,198,203,208,212,215,218,221,223,224,225,225,
							225,224,223,221,218,215,212,208,203,198,193,187,181,174,167,
							159,151,143,134,125,116,106,96,86,76,65,55,44,33,22,11,0,-11,
							-22,-33,-44,-55,-65,-76,-86,-96,-106,-116,-125,-134,-143,-151,
							-159,-167,-174,-181,-187,-193,-198,-203,-208,-212,-215,-218,
							-221,-223,-224,-225,-225,-225,-224,-223,-221,-218,-215,-212,
							-208,-203,-198,-193,-187,-181,-174,-167,-159,-151,-143,-134,
							-125,-116,-106,-96,-86,-76,-65,-55,-44,-33,-22,-11,0};
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

void timer3_init(void) {
	TIM_TimeBaseInitTypeDef TIM_InitStruct;
	NVIC_InitTypeDef NVIC_InitStructure;
	RCC_APB1PeriphClockCmd(RCC_APB1ENR_TIM3EN, ENABLE);

	TIM_InitStruct.TIM_ClockDivision = TIM_CKD_DIV4;
	TIM_InitStruct.TIM_CounterMode = TIM_CounterMode_Up;
	//TIM_InitStruct.TIM_Period = 4000;
	TIM_InitStruct.TIM_Period = 1874;
	TIM_InitStruct.TIM_Prescaler = 5;
	//TIM_InitStruct.TIM_RepetitionCounter = 2;
	TIM_TimeBaseInit(TIM3, &TIM_InitStruct);

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn; /*TIM3 interrupt*/
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0; /*Preemptive priority level 0*/
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3; /*From the priority level 3*/
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; /*The IRQ channel is enabled*/
	NVIC_Init(&NVIC_InitStructure);

	TIM_ITConfig(TIM3, TIM_IT_Update, ENABLE);

	TIM_Cmd(TIM3, ENABLE);
	NVIC_EnableIRQ(TIM3_IRQn);

}
void TIM3_IRQHandler(void) {
	uint16_t temp;
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) /*Check the TIM3 update interrupt occurs or not*/
	{
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update); /*Remove TIMx update interrupt flag */
        led1 = !led1;

        //GPIO_WriteBit(GPIOC,GPIO_Pin_13,led1);
        	GPIO_WriteBit(GPIOB,GPIO_Pin_12,0);
        	SPI2->DR = 0x01;
        	while (!(SPI2->SR & SPI_I2S_FLAG_TXE));
        	while (!(SPI2->SR & SPI_I2S_FLAG_RXNE));
        	while (SPI2->SR & SPI_I2S_FLAG_BSY);
        	temp = SPI2->DR;
        	SPI2->DR = 0x80;
        	while (!(SPI2->SR & SPI_I2S_FLAG_TXE));
        	while (!(SPI2->SR & SPI_I2S_FLAG_RXNE));
        	while (SPI2->SR & SPI_I2S_FLAG_BSY);
        	temp = SPI2->DR;
        	temp = temp & 0x000F;
        	temp = temp << 8;
        	//temp = temp & 0xFF00;
        	SPI2->DR = 0x00;
        	while (!(SPI2->SR & SPI_I2S_FLAG_TXE));
        	while (!(SPI2->SR & SPI_I2S_FLAG_RXNE));
        	while (SPI2->SR & SPI_I2S_FLAG_BSY);
        	adcval[samplecnt] = (SPI2->DR & 0x00FF) + temp;
        	vphaseval[samplecnt] = vphase;
        	GPIO_WriteBit(GPIOB,GPIO_Pin_12,1);

            if(samplecnt == 255)
            	samplecnt = 0;
            else
            	samplecnt++;

            if(samplecnt == 0){
            	buf_offset = 128;
            	sampledone = 1;
            }else if(samplecnt == 128){
            	buf_offset = 0;
            	sampledone = 1;
            }

	}
}

uint8_t led = 0;
/* Set interrupt handlers */
void EXTI15_10_IRQHandler(void) {
	/* Make sure that interrupt flag is set */
	if (EXTI_GetITStatus(EXTI_Line10) != RESET) {
		/* Do your stuff when PD0 is changed */
		GPIO_WriteBit(GPIOC,GPIO_Pin_13,led);
		led = !led;
		if(vphase == 127)
			vphase = 0;
		else
			vphase++;
		/* Clear interrupt flag */
		EXTI_ClearITPendingBit(EXTI_Line10);
	}
}

int midpoint = 2055;
int maxpoint;
int minpoint;
uint16_t get_adc(void)
{
	uint8_t k;
	float calc;
	float temp;
	float Irms;
	calc = 0;
	temp = 0;
	while(!sampledone);
	//GPIO_WriteBit(GPIOC,GPIO_Pin_13,1);
	for(k=0;k<128;k++){
		//calc += (adcval[k + buf_offset]-midpoint) * (adcval[k + buf_offset]-midpoint);
		temp = adcval[k + buf_offset]-midpoint;
		temp = (temp * 5)/(4096 * 0.12);
		calc += temp*temp;
	}
	calc = calc/128;
	calc = sqrt(calc);

	//Irms = (calc * 5)/(4096 * 0.12);
	Irms = calc *1000;
	printf("RMS %d   |",(uint16_t)Irms);
	calc = 0;
	for(k=0;k<128;k++){
		temp = adcval[k + buf_offset]-midpoint;
		temp = (temp * 5)/(4096 * 0.12);
		temp = temp*varray[vphaseval[k+buf_offset]];
		calc += temp;
	}
	Irms = calc/128;
	//Irms = (Irms * 230)/1000;
	printf("Power %d   |",(uint16_t)Irms);


	maxpoint = 0;
	minpoint = 4095;
	for(k=0;k<128;k++){
		if(adcval[k + buf_offset] > maxpoint){
			maxpoint = adcval[k + buf_offset];
		}
	}
	printf("adcmax %d  |",(int)maxpoint-midpoint);

	for(k=0;k<128;k++){
		if(adcval[k + buf_offset] < minpoint){
			minpoint = adcval[k + buf_offset];
		}
	}
	printf("adcmin %d\n",(int)minpoint-midpoint);
	//GPIO_WriteBit(GPIOC,GPIO_Pin_13,0);
	sampledone = 0;
	//midpoint = (maxpoint+minpoint)/2;
	return((uint16_t)calc);

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
		while (USART_GetFlagStatus(USART3, USART_FLAG_RXNE) == RESET)
		{}
		tmp[index] = (USART_ReceiveData(USART3));
		USART3_Send(tmp[index++]);
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

