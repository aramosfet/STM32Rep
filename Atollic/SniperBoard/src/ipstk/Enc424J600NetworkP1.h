/*
 Enc424J600NetworkClass.h
 UIPEthernet network driver for Microchip ENC28J60 Ethernet Interface.

 Copyright (c) 2013 Norbert Truchsess <norbert.truchsess@t-online.de>
 All rights reserved.

 inspired by enc28j60.c file from the AVRlib library by Pascal Stang.
 For AVRlib See http://www.procyonengineering.com/

 This program is free software: you can redistribute it and/or modify
 it under the terms of the GNU General Public License as published by
 the Free Software Foundation, either version 3 of the License, or
 (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef Enc424J600NetworkP1_H_
#define Enc424J600NetworkP1_H_
#include <stddef.h>
#include "stm32f10x.h"
#include <stdbool.h>


//#define UIP_RECEIVEBUFFERHANDLE 0xff

//#define ENC28J60DEBUG

/*
 * Empfangen von ip-header, arp etc...
 * wenn tcp/udp -> tcp/udp-callback -> assign new packet to connection
 */




//  /* basic functions */
//  static void enc_SBI(uint8_t instruction, bool keepEnabled ); // single byte instruction
//  static void enc_writeOp(uint8_t op, uint8_t address, uint8_t* data, uint8_t len); // perform write operation
//  static void enc_readOp(uint8_t op, uint8_t address, uint8_t* data, uint8_t len); // perform read operation

//
//  static void writeControlRegister(uint8_t address, uint8_t data); // select bank and write control register
//  static void writeControlRegister16(uint8_t address, uint16_t data); // select bank and write 2 bytes to control register
//  static uint8_t readControlRegister(uint8_t address); // select bank and read control register
//  static uint16_t readControlRegister16(uint8_t address); // select bank and read control register
//  static void writeBitField(uint8_t address, uint8_t data); // select bank and write control register bit
//  static void writePointer(uint8_t instruction, uint16_t address, bool keepEnabled ); // select bank and write 2 bytes to a pointer
  
  void enc424j600PacketSendP1(unsigned int len, unsigned char* packet);
  unsigned int enc424j600PacketReceiveP1(unsigned int maxlen, unsigned char* packet);

//  //void enc_writeOp(uint8_t op, uint8_t address, uint8_t data);
//  static void enc_writeOp(uint8_t op, uint8_t address, uint8_t* data, uint8_t len);
//  static void setERXRDPT();

//
//
//  static void phyWrite(uint8_t address, uint16_t data);
//  static uint16_t phyRead(uint8_t address);

//  static bool linkStatus();

  void Enc424J600Network_initP1(uint8_t* macaddr);
  uint8_t Enc424J600Network_linkStatus();

#endif /* Enc424J600NetworkClass_H_ */
