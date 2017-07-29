/*
 Enc424J600NetworkClass.h
 UIPEthernet network driver for Microchip ENC28J60 Ethernet Interface.

 Copyright (c) 2013 Norbert Truchsess <norbert.truchsess@t-online.de>
 All rights reserved.

 based on enc28j60.c file from the AVRlib library by Pascal Stang.
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
#include "uip/Enc424J600NetworkP0.h"
#include <stddef.h>
#include <stdio.h>
//#include "Arduino.h"
#include "uip/uip.h"
#include "enc424j600.h"
#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_spi.h"
#include "sys_timer.h"
//#include "uip/mempool.h"
#define PORT0
#ifdef PORT0
//------------------------------------------------------------------------------------------------------
// set CS to 0 = active
#define CSACTIVE GPIO_ResetBits(GPIOA,GPIO_Pin_8);
// set CS to 1 = passive
#define CSPASSIVE GPIO_SetBits(GPIOA,GPIO_Pin_8);
//
//#define waitspi() while (SPI2->SR & SPI_I2S_FLAG_BSY);
//static void waitspi(void)
//{
//	while (!(SPI2->SR & SPI_I2S_FLAG_TXE));
//	//
//	//    Wait for any data on MISO pin to be received.
//	//
//	while (!(SPI2->SR & SPI_I2S_FLAG_RXNE));
//	//
//	//    All data transmitted/received but SPI may be busy so wait until done.
//	//
//	while (SPI2->SR & SPI_I2S_FLAG_BSY);
//}

//uint16_t nextPacketPtr;
//uint8_t bank=0xff;

//struct memblock receivePkt;

//static bool isActive = false;

static uint8_t SPISend(uint8_t data)
{
	//
	//    Setting the Data Register (DR) transmits the byte of data on MOSI.
	//
	SPI2->DR = data;
	//
	//    Wait until the data has been transmitted.
	//
	while (!(SPI2->SR & SPI_I2S_FLAG_TXE));
	//
	//    Wait for any data on MISO pin to be received.
	//
	while (!(SPI2->SR & SPI_I2S_FLAG_RXNE));
	//
	//    All data transmitted/received but SPI may be busy so wait until done.
	//
	while (SPI2->SR & SPI_I2S_FLAG_BSY);
	//
	//    Return the data received on MISO pin.
	//
	//Sys_Delay(1);
	return(SPI2->DR);

}
//------------------------------------------------------------------------------------------------------
#else
// set CS to 0 = active
#define CSACTIVE GPIO_ResetBits(GPIOB,GPIO_Pin_0);
// set CS to 1 = passive
#define CSPASSIVE GPIO_SetBits(GPIOB,GPIO_Pin_0);
//
//#define waitspi() while (SPI2->SR & SPI_I2S_FLAG_BSY);
void waitspi(void)
{
	while (!(SPI1->SR & SPI_I2S_FLAG_TXE));
	//
	//    Wait for any data on MISO pin to be received.
	//
	while (!(SPI1->SR & SPI_I2S_FLAG_RXNE));
	//
	//    All data transmitted/received but SPI may be busy so wait until done.
	//
	while (SPI1->SR & SPI_I2S_FLAG_BSY);
}

//uint16_t nextPacketPtr;
//uint8_t bank=0xff;

//struct memblock receivePkt;

bool isActive = false;

uint8_t SPISend(uint8_t data)
{
	//
	//    Setting the Data Register (DR) transmits the byte of data on MOSI.
	//
	SPI1->DR = data;
	//
	//    Wait until the data has been transmitted.
	//
	while (!(SPI1->SR & SPI_I2S_FLAG_TXE));
	//
	//    Wait for any data on MISO pin to be received.
	//
	while (!(SPI1->SR & SPI_I2S_FLAG_RXNE));
	//
	//    All data transmitted/received but SPI may be busy so wait until done.
	//
	while (SPI1->SR & SPI_I2S_FLAG_BSY);
	//
	//    Return the data received on MISO pin.
	//
	Sys_Delay(1);
	return(SPI1->DR);

}
//------------------------------------------------------------------------------------------------------
#endif
static uint16_t nextPacketPtr;
static uint8_t bank;
static void enc_setBank(uint8_t address, bool keepEnabled ); // select the memory bank
static void readBuffer(uint16_t len, uint8_t* data);
  static void writeBuffer(uint16_t len, uint8_t* data);
/* issue an single byte instruction */
static void enc_SBI(uint8_t instruction, bool keepEnabled)
{

	CSACTIVE;

	// issue the instruction
	SPISend(instruction);

	if (!keepEnabled)
	    CSPASSIVE;
}


static void enc_writeOp(uint8_t op, uint8_t address, uint8_t* data, uint8_t len)
{
	CSACTIVE;

	// issue write command
	SPISend( op | (address & ADDR_MASK));
	
	for ( int i = 0; i<len; i++)
	  SPISend(  *data++);

	CSPASSIVE;
}


static void enc_readOp(uint8_t op, uint8_t address, uint8_t* data, uint8_t len)
{

	CSACTIVE;

	// issue write command
	SPISend( op | (address & ADDR_MASK));
	
	for (int i = 0; i< len; i++)
	*data++ = SPISend(  0x00);


	CSPASSIVE;

}

/* select bank and write control register
will always put CS high to end operation
*/
static void writeControlRegister(uint8_t address, uint8_t data)
{
  // set the bank
  enc_setBank(address,true);
  // do the write
  enc_writeOp(ENC624J600_WRITE_CONTROL_REGISTER, address, &data, 1);
}

static void writeControlRegister16(uint8_t address, uint16_t data)
{
  uint8_t dat;
  // set the bank
  enc_setBank(address,true);
  // do the write
  
  dat = data & 0x00FF;
  enc_writeOp(ENC624J600_WRITE_CONTROL_REGISTER, address&0x1F,(uint8_t*) &dat , 1);
  dat = (data & 0xFF00) >> 8;
  enc_writeOp(ENC624J600_WRITE_CONTROL_REGISTER, ((address+1) & 0x1F),(uint8_t*) &dat , 1);

}



static void writePointer(uint8_t instruction, uint16_t address, bool keepEnabled)
{
	CSACTIVE;
	
	SPISend(instruction);
	SPISend( address&0x00FF);
	SPISend( address>>8);

  if (!keepEnabled)
	  CSPASSIVE;

}

/* select bank and write control register
will always put CS high to end operation
*/
static uint8_t readControlRegister(uint8_t address)
{
  uint16_t retval;
  // set the bank
  enc_setBank(address,true);
  // do the write
  enc_readOp(ENC624J600_READ_CONTROL_REGISTER, address&0x1F,(uint8_t*) &retval ,1);
  
  return retval;
}

/* select bank and read control register
will always put CS high to end operation
*/
static uint16_t readControlRegister16(uint8_t address)
{
  uint16_t retval;
  // set the bank
  enc_setBank(address,true);
  // do the write
  enc_readOp(ENC624J600_READ_CONTROL_REGISTER, address&0x1F,(uint8_t*) &retval ,2);
  
  return retval;
}


/* select bank and write control register bit
will always put CS high to end operation
*/
static void writeBitField(uint8_t address, uint8_t data) {
  // set the bank
  enc_setBank(address,true);
  // do the write
  enc_writeOp(ENC624J600_BIT_FIELD_SET, address, &data,1);
}


static void enc_setBank(uint8_t address, bool keepEnabled)
{
  // set the bank (if needed)
  if (((address & BANK_MASK) != bank) && ((address & BANK_MASK) != 0xE0))
  {
    // set the bank

    bank = (address & BANK_MASK);
    
    switch((bank)>>5){
			case 0 :
				enc_SBI(ENC624J600_BANK0_SELECT, keepEnabled);
				break;
			case 1 :
				enc_SBI(ENC624J600_BANK1_SELECT, keepEnabled);
				break;
			case 2 :
				enc_SBI(ENC624J600_BANK2_SELECT, keepEnabled);
				break;
			case 3 :
				enc_SBI(ENC624J600_BANK3_SELECT, keepEnabled);
				break;
		}
  }
}







void Enc424J600Network_initP0(uint8_t* macaddr)
{
  //MemoryPool_init(); // 1 byte in between RX_STOP_INIT and pool to allow prepending of controlbyte
  //printf("memory pool init\n");
#ifdef ENC28J60DEBUG
  SerialUSB.println("ENC624J600Init");
#endif
  /* enable SPI */
//	pinMode(ENC28J60_CONTROL_CS, OUTPUT);
//	digitalWrite(ENC28J60_CONTROL_CS, HIGH);
//  SPI.begin();
  
	//8.1 RESET
	//STEP ONE
	writeControlRegister16(EUDASTL,0x1234);

	//STEP TWO
	while(readControlRegister16(EUDASTL)!=0x1234)
	{
		writeControlRegister16(EUDASTL,0x1234);
	}
  
	//STEP THREE
	while(readControlRegister(ESTATH) & ESTAT_CLKRDY);

	//STEP FOUR
	// reset command
	enc_SBI(ENC624J600_ETH_RESET,0);

	//STEP FIVE
	delayMicroseconds(25);
	//STEP SIX
	if (readControlRegister16(EUDASTL)==0x0000)
	{
		delayMicroseconds(265);		
		//8.2 CLKOUT Frequency
		// Arduino : 16MHz =>  COCON=0100 
		// We do not use the clkout
		//writeBitField( ECON2H,ECON2_COCON2>>8);
		//8.3 reception
		nextPacketPtr = RXSTART_INIT;
		writeControlRegister16(ERXSTL, RXSTART_INIT);

		
		writeControlRegister16(ERXTAILL, RXSTOP_INIT);
			
 		// USER buffer : EUDAST Pointer at a higher memory address relative to the end address.
 		writeControlRegister16(EUDASTL, 0x5FFF);
 		writeControlRegister16(EUDANDL, 0x5FFF);
			/*
#ifndef IPV6
 		// fill user-defined area with arpResponse
		// only for IPv4
 		unsigned char arpResponse[ARP_RESPONSE_PACKET_SIZE-MAC_ADDR_SIZE] = { 
			0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
			0x08, 0x06,
			0x00, 0x01,
			0x08, 0x00,
			0x06,
			0x04,
			0x00, 0x02,
			ENC624J600_MAC0, ENC624J600_MAC1, ENC624J600_MAC2, ENC624J600_MAC3, ENC624J600_MAC4, ENC624J600_MAC5,
			IP_ADDR_0, IP_ADDR_1, IP_ADDR_2, IP_ADDR_3,
			0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
			0x00, 0x00, 0x00, 0x00
		};

 		writeRegGPBuffer(USER_START_INIT,arpResponse,ARP_PACKET_SIZE-MAC_ADDR_SIZE);
#endif*/

		//8.4 RAF
		//8.5 RECEIVE FILTER TODO!!!
#ifdef IPV6
		// We need multicast for neighbor sollicitation
	d	writeControlRegister(ERXFCONL,ERXFCON_CRCEN|ERXFCON_RUNTEN|ERXFCON_UCEN|ERXFCON_MCEN);
#else
		// crc ERROR FILTER => disabled
		// frames shorter than 64 bits => disabled
		// CRC error rejection => enabled
		// Unicast collection filter => enabled
		// Not me unicast filter => disabled
		// Multicast collection filter 
		writeControlRegister(ERXFCONL,ERXFCON_CRCEN|ERXFCON_RUNTEN|ERXFCON_BCEN|ERXFCON_UCEN);//ERXFCON_CRCEN|ERXFCON_RUNTEN|ERXFCON_UCEN);
		// brodcast collection filter => enabled
		// Hash table collection filter.. c
		// Magic packet => disabled TODO
		// PAttern
#endif
			
#ifdef CHECKSUM_PATTERN
		// Checksum only for IPv4
		//window 
		writeControlRegister16(EPMOL,0x0000);
			
		//pattern
		writeControlRegister(EPMM1L, WINDOW_PATTERN_0);
		writeControlRegister(EPMM1H, WINDOW_PATTERN_1);
		writeControlRegister(EPMM2L, WINDOW_PATTERN_2);
		writeControlRegister(EPMM2H, WINDOW_PATTERN_3);
		writeControlRegister(EPMM3L, WINDOW_PATTERN_4);
		writeControlRegister(EPMM3H, WINDOW_PATTERN_5);
		writeControlRegister(EPMM4L, WINDOW_PATTERN_6);
		writeControlRegister(EPMM4H, WINDOW_PATTERN_7);
		//CheckSum
		writeControlRegister(EPMCSL,CHECKSUM_PATTERN&0xFF);
		writeControlRegister(EPMCSH,CHECKSUM_PATTERN>>8);
#endif
		//exact pattern
		//writeControlRegister(ERXFCONH,0x01);
					      
		// 8.6 MAC initialization ...
		//flow control ???
		writeBitField( MACON2L, MACON2_TXCRCEN|MACON2_PADCFG0|MACON2_PADCFG1|MACON2_PADCFG2);

		writeControlRegister16(MAMXFLL, MAX_FRAMELEN);



		writeControlRegister(MAADR1L, macaddr[0]);
		writeControlRegister(MAADR1H, macaddr[1]);
		writeControlRegister(MAADR2L, macaddr[2]);
		writeControlRegister(MAADR2H, macaddr[3]);
		writeControlRegister(MAADR3L, macaddr[4]);
		writeControlRegister(MAADR3H, macaddr[5]);
		
		///writeControlRegister(ECON2H, 0xa0);
		//writeControlRegister(ECON2H, 224);
		
		//8.7 PHY initialization 
		// auto-negotiation ?
		//	ENC624J600PhyWrite(PHANA,0x05E1);
		// 8.8 OTHER considerations
		//half-duplex mode
			//writeBitField( MACON2H,MACON2_DEFER|MACON2_BPEN|MACON2_NOBKOFF);$
			
		// enable interuption
		//writeControlRegister(EIEL,0x40);
		//writeControlRegister(EIEH,0x80);
		// configuration LED
		//		ENC624J600WCRU(EIDLEDH, 0x06);

			 //	ENC624J600PhyWrite(PHCON1,PHCON1_PFULDPX);
		// enable reception
		enc_SBI(ENC624J600_ENABLE_RX,0);

	}
//	else
//	{
//	  SerialUSB.println ("Error in initialization!");
//		// Oops something went wrong
//	}
#ifdef ENC28J60DEBUG
	SerialUSB.println ("ENC624J600Init complete");
#endif
}


//uint16_t
//Enc424J600Network_receivePacket(uint8_t* buffer, uint16_t len)
//{
//  // uint8_t rxstat;
//  uint16_t len;
//  // check if a packet has been received and buffered
//  // if( !(readControlRegister(EIR) & EIR_PKTIF) ){
//  // The above does not work. See Rev. B4 Silicon Errata point 6.
//
//  // SerialUSB.println(readControlRegister(ESTATL));
//
//  if( (readControlRegister(EIRL) & EIR_PKTIF) ){
//
//      uint16_t readPtr = nextPacketPtr+8 > RXSTOP_INIT ? nextPacketPtr+8-RXSTOP_INIT+RXSTART_INIT : nextPacketPtr+8 ;
//      // Set the read pointer to the start of the received packet
//     // writeControlRegister16(ENC624J600_WRITE_ERXRDPT, nextPacketPtr);
//      writePointer(ENC624J600_WRITE_ERXRDPT, nextPacketPtr,true);
//
//      SPISend(ENC624J600_READ_ERXDATA);
//      // read the next packet pointer
//      nextPacketPtr = SPISend( 0x00);
//      nextPacketPtr |= SPISend( 0x00) << 8;
//
//      // read the packet length (see datasheet page 43)
//      len = SPISend( 0x00);
//      len |= SPISend( 0x00) << 8;
//      len -= 4; //remove the CRC count
//      // read the receive status (see datasheet page 43)
//      // rxstat = enc_readOp(ENC624J600_READ_ERXDATA, 0);
//     // rxstat |= enc_readOp(ENC624J600_READ_ERXDATA, 0) << 8;
//      CSPASSIVE;
//
//
//      // decrement the packet counter indicate we are done with this packet
//
//      // check CRC and symbol errors (see datasheet page 44, table 7-3):
//      // The ERXFCON.CRCEN is set by default. Normally we should not
//      // need to check this.
//		setERXRDPT();
//
//		writeControlRegister16(ERXTAILL, readPtr-1);
//		enc_SBI(ENC624J600_SETPKTDEC,0);
//		printf("Receive packet!");
//		if (len > 500) {
//			printf("Packet discarded!");
//
//			return (0);
//		}
//		readBuffer(len, buffer);
//
//
//		return len;
//   // Move the RX read pointer to the start of the next received packet
//      // This frees the memory we just read out
//
//    }
//  return (0);
//}

unsigned int enc424j600PacketReceiveP0(unsigned int maxlen, unsigned char* packet)
{
    unsigned char rxdata[6];
    unsigned int len;
    uint16_t newRXTail;
    if (!(readControlRegister(EIRL) & EIR_PKTIF))
    {
        return (0);
    }
    //printf("Packets 0 %d\n",readControlRegister(ESTATL));
    // Set the RX Read Pointer to the beginning of the next unprocessed packet
    writePointer(ENC624J600_WRITE_ERXRDPT,nextPacketPtr,0);
    readBuffer(2, rxdata);
    nextPacketPtr  =  rxdata[1];
    nextPacketPtr  =  nextPacketPtr<<8;
    nextPacketPtr  |=  rxdata[0];
    readBuffer(6, rxdata);
    len  =  rxdata[1];
    len  =  len<<8;
    len  |=  rxdata[0];
    len-=4;
    if ((rxdata[2] & 0x80)==0)
    {
        len=0;
    }
    if(len>maxlen)
    	len=0;
    readBuffer(len,packet);
    newRXTail = nextPacketPtr - 2;

    if (nextPacketPtr == RXSTART_INIT)
        newRXTail = 0x5FFE - 2;
    //Packet decrement
    writeControlRegister(ECON1H, 0x01);
    //printf("Packets 1 %d\n",readControlRegister(ESTATL));
    //Write new RX tail
    writeControlRegister(ERXTAILL, newRXTail);     //
    writeControlRegister(ERXTAILH, newRXTail>>8);
    if (len>maxlen-1)
	{
		//len=maxlen-1;
		return 0;
	}
    return len;
}


void enc424j600PacketSendP0(unsigned int len, unsigned char* packet)
{
	writePointer(ENC624J600_WRITE_EGPWRPT,TXSTART_INIT,0);
	writeControlRegister(ETXSTL,(TXSTART_INIT)&0x00FF);
	writeControlRegister(ETXSTH,(TXSTART_INIT)>>8);
    writeControlRegister(ETXLENL, len&0xFF);
    writeControlRegister(ETXLENH, len>>8);
    writeBuffer(len, packet);
    //enc_writeOp(ENC624J600_BIT_FIELD_SET, ECON1L, ECON1_TXRTS,1);
    //writeBitField(ECON1L,ECON1_TXRTS);
    writeControlRegister(ECON1L,0x03);
    //printf("ECON1L Value %d ",readControlRegister(ECON1L));
}


static void
readBuffer(uint16_t len, uint8_t* data)
{
  CSACTIVE;
  // issue read command
  SPISend(  ENC624J600_READ_ERXDATA);
  
  #ifdef ENC28J60DEBUG
    SerialUSB.print("Readbuffer: ");
  #endif
  while(len)
  {
    len--;
    // read data

    *data = SPISend(0x00);
    #ifdef ENC28J60DEBUG
    SerialUSB.print(" ");
    SerialUSB.print(*data,HEX);
    #endif
    data++;
  }
  //*data='\0';
  CSPASSIVE;
  #ifdef ENC28J60DEBUG
  SerialUSB.println(" ");
  #endif
}

static void
writeBuffer(uint16_t len, uint8_t* data)
{
  CSACTIVE;
  // issue write command
	SPISend(ENC624J600_WRITE_EGPDATA);
  //SerialUSB.print("writeBuffer: ");
	while(len--)
	{
    //SerialUSB.print(*data,HEX);
    //SerialUSB.print(" ");
    SPISend( *data);
    data++;
	}
  //SerialUSB.println("");   
  CSPASSIVE;
}





//static void
//phyWrite(uint8_t address, uint16_t data)
//{
//  // set the PHY register address
//  writeControlRegister(MIREGADRL, address);
//  // write the PHY data
//  writeControlRegister16(MIWRL, data);
//  // wait until the PHY write completes
//  while(readControlRegister(MISTATL) & MISTAT_BUSY){
//    delayMicroseconds(15);
//  }
//}

//static uint16_t
//phyRead(uint8_t address)
//{
//  writeControlRegister(MIREGADRL,address);
//  writeControlRegister(MICMDL, MICMD_MIIRD);
//  // wait until the PHY read completes
//  while(readControlRegister(MISTATL) & MISTAT_BUSY){
//    delayMicroseconds(15);
//  }  //and MIRDH
//  writeControlRegister(MICMDL, 0);
//  return (readControlRegister(MIRDL) | readControlRegister(MIRDH) << 8);
//}

//static uint8_t
//Enc424J600Network_linkStatus()
//{
//  return (phyRead(PHSTAT2) & 0x0400) > 0;
//}


