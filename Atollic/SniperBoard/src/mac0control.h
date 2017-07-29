/*
 * mac0control.h
 *
 *  Created on: Apr 22, 2017
 *      Author: Aravindan
 */

#ifndef MAC0CONTROL_H_
#define MAC0CONTROL_H_



void mac0_init(void);
void mac0_tick(void);
void mac0_service(uint8_t tcp_service_enable, uint8_t reg, uint8_t alarm_id);
#endif /* MAC0CONTROL_H_ */
