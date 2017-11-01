/*
 * calendar.c
 *
 *  Created on: May 28, 2017
 *      Author: Aravindan
 */

#include <stdio.h>
#include <stddef.h>
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "stm32f10x_rtc.h"

const uint8_t month_table[12] = {31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

uint8_t cal_hour = 0;
uint8_t cal_date = 1;
uint8_t cal_month = 1;
uint8_t cal_minute = 0;
uint8_t cal_second = 0;

uint16_t cal_year = 1970;

struct DateTime {
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t date;
	uint8_t day;
	uint8_t month;
	uint16_t year;
};

struct DateTime current_date;

uint8_t check_for_leap_year(uint16_t year)
{
    if(year % 4 == 0)
    {
        if(year % 100 == 0)
        {
            if(year % 400 == 0)
            {
                return 1;
            }

            else
            {
                return 0;
            }
        }

        else
        {
            return 1;
        }
    }

    else
    {
        return 0;
    }
}

void get_RTC()
{
	 uint16_t temp1 = 0;
     static uint16_t day_count;

     uint32_t temp = 0;
     uint32_t counts = 0;

//     counts = RTC_CNTH;
//     counts <<= 16;
//     counts += RTC_CNTL;
     counts = RTC_GetCounter();
     counts = counts + 14400; // UTC + 4:00 Dubai time
     //printf("RTC Count %d ",(uint16_t)((counts >> 16 ) & 0x0000FFFF));
     //printf("%d\n",(uint16_t)(counts & 0x0000FFFF));
     //printf("Count:");
//     uint32_t i = counts;
//     while(i) {
//    	 printf("%d",(uint16_t)(i % 10));
//		i /= 10;
//	}
     temp = (counts / 86400);
     //printf("\ndays %d\n",(uint16_t)(temp));
     if(day_count != temp)
     {
         day_count = temp;
         temp1 = 1970;

         while(temp >= 365)
         {
             if(check_for_leap_year(temp1) == 1)
             {
                 if(temp >= 366)
                 {
                     temp -= 366;
                 }

                 else
                 {
                     break;
                 }
             }

             else
             {
                 temp -= 365;
             }

             temp1++;
         };

         cal_year = temp1;

         temp1 = 0;
         while(temp >= 28)
         {
             if((temp1 == 1) && (check_for_leap_year(cal_year) == 1))
             {
                 if(temp >= 29)
                 {
                     temp -= 29;
                 }

                 else
                 {
                     break;
                 }
             }

             else
             {
                 if(temp >= month_table[temp1])
                 {
                     temp -= ((uint32_t)month_table[temp1]);
                 }

                 else
                 {
                     break;
                 }
             }

             temp1++;
         };

         cal_month = (temp1 + 1);
         cal_date = (temp + 1);
     }

     temp = (counts % 86400);

     cal_hour = (temp / 3600);
     cal_minute = ((temp % 3600) / 60);
     cal_second = ((temp % 3600) % 60);

     current_date.seconds = cal_second;
     current_date.minutes = cal_minute;
     current_date.hours   = cal_hour;
     current_date.date    = cal_date;
     current_date.month   = cal_month;
     current_date.year    = cal_year;

}

uint32_t set_RTC(uint16_t year,  uint8_t month,  uint8_t date,  uint8_t hour,  uint8_t minute,  uint8_t second)
{
    uint16_t i = 0;
    uint32_t counts = 0;

    if(year > 2099)
    {
        year = 2099;
    }

    if(year < 1970)
    {
        year = 1970;
    }

    for(i = 1970; i < year; i++)
    {
          if(check_for_leap_year(i) == 1)
          {
              counts += 31622400;
          }

          else
          {
              counts += 31536000;
          }
    }

    month -= 1;

    for(i = 0; i < month; i++)
    {
          counts += (((uint32_t)month_table[i]) * 86400);
    }

    if(check_for_leap_year(cal_year) == 1)
    {
        counts += 86400;
    }

    counts += ((uint32_t)(date - 1) * 86400);
    counts += ((uint32_t)hour * 3600);
    counts += ((uint32_t)minute * 60);
    counts += second;
    return(counts);
//    RCC->APB1ENR |= RCC_APB1ENR_PWREN;
//    RCC->APB1ENR |= RCC_APB1ENR_BKPEN;
//
//    PWR->CR |= PWR_CR_DBP;
//    RTC_SetCounter(counts);
//
//    while(!(RTC->CRL & RTC_CRL_RTOFF));
//
//    PWR->CR &= ~PWR_CR_DBP;
}

//void display_date()
//{
//	printf("Year : %d   -   ",current_date.year);
//	printf("month : %d   -   \n",current_date.month);
//	printf("date : %d   -   ",current_date.date);
//	printf("hours : %d   -   \n",current_date.hours);
//	printf("minutes : %d   -   ",current_date.minutes);
//	printf("seconds : %d   -   \n",current_date.seconds);
//}

void display_date()
{
	printf("%d/",current_date.date);
	printf("%d/",current_date.month);
	printf("%d ",current_date.year);
	printf("%d:",current_date.hours);
	printf("%d:",current_date.minutes);
	printf("%d",current_date.seconds);
}

void set_date(void)
{
    current_date.seconds = 00;
    current_date.minutes = 58;
    current_date.hours   = 17;
    current_date.date    = 29;
    current_date.month   = 7;
    current_date.year    = 2017;
    set_RTC(current_date.year,  current_date.month,  current_date.date,  current_date.hours,  current_date.minutes,  current_date.seconds);
}
