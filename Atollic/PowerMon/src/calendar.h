/*
 * calendar.h
 *
 *  Created on: May 28, 2017
 *      Author: Aravindan
 */

#ifndef CALENDAR_H_
#define CALENDAR_H_

void get_RTC(void);
void display_date(void);
void set_date(void);
uint32_t set_RTC(uint16_t year,  uint8_t month,  uint8_t date,  uint8_t hour,  uint8_t minute,  uint8_t second);
#endif /* CALENDAR_H_ */
