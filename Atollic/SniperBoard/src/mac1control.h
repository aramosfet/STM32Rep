/*
 * mac1control.h
 *
 *  Created on: Apr 22, 2017
 *      Author: Aravindan
 */

#ifndef MAC1CONTROL_H_
#define MAC1CONTROL_H_

void mac1_tick(void);
void mac1_init(void);
void mac1_service(void);
uint16_t GetBacInstNum(uint8_t n);
uint16_t GetBacInstNuma(uint8_t n);
uint8_t retrdpcount(void);
uint8_t retrdpacount(void);
uint8_t retrdpval(uint8_t n);
float retrdpaval(uint8_t n);

#endif /* MAC1CONTROL_H_ */
