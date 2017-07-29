/*
 * mac1control.c
 *
 *  Created on: Apr 22, 2017
 *      Author: Aravindan
 */

#include "ipstk/ip_config.h"
#include "ipstk/dhcp.h"
#include "ipstk/Enc424J600NetworkP1.h"
#include "ipstk/ip_arp_udp_tcp.h"
#include "ipstk/net.h"
#include "sys_timer.h"
#include "mac1control.h"



uint8_t macaddr1[6] = {0x48,0x48,0x48,0x48,0x48,0x49};
uint8_t myip[4] = {192,168,1,98};
uint8_t gwip[4] = {192,168,1,1};
uint8_t bacdev_ip[4] = {0,0,0,0};
uint8_t mynetmask[4];
uint8_t dnsip[4] = {8,8,8,8};
uint8_t dhcpsvrip[4];
uint16_t myport = 0x8080;
uint8_t dhcp_up;
uint32_t bac_timeout=0;
uint16_t plen;
uint8_t bacnet_found;
uint8_t invokeid;
uint8_t bac_inst = 1;
uint8_t bac_binval[10] = {0,0,0,0,0,0,0,0,0,0};
uint8_t dip[4];
uint16_t dport = 47808;
uint16_t sport = 47808;
uint8_t set_bacnet_bin_alarm =0;
uint16_t mac1_pkt_cntr=0;

#define BUFFER_SIZE 800
uint8_t buf1[BUFFER_SIZE+1];
#define FALSE 0
#define TRUE 1




void ES_dhcp_start(uint8_t *buf, uint8_t *macaddrin, uint8_t *ipaddrin,
     uint8_t *maskin, uint8_t *gwipin, uint8_t *dhcpsvrin, uint8_t *dnssvrin ) {
	dhcp_start(buf, macaddrin, ipaddrin, maskin, gwipin, dhcpsvrin, dnssvrin );
}
uint8_t ES_dhcp_state(void)
{
        return( dhcp_state() );
}

uint8_t ES_check_for_dhcp_answer(uint8_t *buf,uint16_t plen){
	return( check_for_dhcp_answer( buf, plen) );
}

// Utility functions
//void print_packet(uint16_t len, uint8_t* pkt)
//{
//	uint16_t i;
//	printf("\n*******PACKET******** \n");
//	for(i=0;i<len;i++){
//		printf("%d ",pkt[i]);
//	}
//	printf("\n*******PACKET******** \n");
//}
// Perform all processing to get an IP address plus other addresses returned, e.g. gw, dns, dhcp server.
// Returns 1 for successful IP address allocation, 0 otherwise
uint8_t allocateIPAddress(uint8_t *buf1, uint16_t buffer_size, uint8_t *mymac, uint16_t myport, uint8_t *myip, uint8_t *mynetmask, uint8_t *gwip, uint8_t *dnsip, uint8_t *dhcpsvrip ) {
  uint16_t dat_p;
  int plen = 0;
  long lastDhcpRequest = HAL_GetTick();
  uint8_t dhcpState = 0;
  bool gotIp = FALSE;
  uint8_t dhcpTries = 3;	// After 10 attempts fail gracefully so other action can be carried out
  //printf("0xA0");
  dhcp_start( buf1, mymac, myip, mynetmask,gwip, dnsip, dhcpsvrip );
  //printf("0xA1");
  while( !gotIp ) {
    // handle ping and wait for a tcp packet
    plen = enc424j600PacketReceiveP1(buffer_size, buf1);
    if(plen)
    {
		//printf("0xA2");
		//printf("Packet length %d\n",plen);
		//print_packet(plen,buf);
		Sys_Delay(100);
		dat_p=packetloop_icmp_tcp(buf1,plen);
		//USART3_Send(dat_p);
		if(dat_p==0) {
		  check_for_dhcp_answer( buf1, plen);
		  dhcpState = dhcp_state();
		  //USART3_Send(0xA5);
		  //USART3_Send(dhcpState);
		  // we are idle here
		  if( dhcpState != DHCP_STATE_OK ) {
			if (HAL_GetTick() > (lastDhcpRequest + 1000L) ){
			  lastDhcpRequest = HAL_GetTick();
		  if( --dhcpTries <= 0 )
			  return 0;		// Failed to allocate address
			  // send dhcp
			  dhcp_start( buf1, macaddr1, myip, mynetmask,gwip, dnsip, dhcpsvrip );
			  //USART3_Send(0xA5);
			}
		  } else {
			if( !gotIp ) {
			  gotIp = TRUE;

			  //init the ethernet/ip layer:
			  init_ip_arp_udp_tcp(macaddr1, myip, myport);

			  // Set the Router IP
			  client_set_gwip(gwip);  // e.g internal IP of dsl router

			  // Set the DNS server IP address if required, or use default
			  //dnslkup_set_dnsip( dnsip );

			}
		  }
      }
    }
  }

  return 1;

}

void process_bacnet_reply(uint8_t *buf,uint16_t len)
{
	//uint8_t bac_length;
	uint8_t bac_apdu_offset;
	//Check bacnet Source port No                        Dest Port No                               Bactnet /IP
	if(((buf[0x22] == 0xba) && (buf[0x23]==0xc0) && (buf[0x24] == 0xba) && (buf[0x25]==0xc0) && (buf[0x2A] == 0x81)) ||
	   ((buf[0x22] == 0xe4) && (buf[0x23]==0x03) && (buf[0x24] == 0xba) && (buf[0x25]==0xc0) && (buf[0x2A] == 0x81)))
	{
		//bac_length = buf[0x2D];
		bac_apdu_offset = 0;
		if(buf[0x2F] & 0x20){
			bac_apdu_offset = bac_apdu_offset+4;
		}
		if(buf[0x2F] & 0x08){
			bac_apdu_offset = bac_apdu_offset+4;
		}
		//Check Bacnet I am
		//printf(" bacnet reply Length %d\n",len);
		if((buf[0x30+bac_apdu_offset] == 0x10) && (buf[0x31+bac_apdu_offset] == 0x00) && (bacnet_found == 0)){
			bacdev_ip[0] = buf[0x1A];
			bacdev_ip[1] = buf[0x1B];
			bacdev_ip[2] = buf[0x1C];
			bacdev_ip[3] = buf[0x1D];
			printf("Bacnet device found on IP %d.%d.%d.%d \n",bacdev_ip[0],bacdev_ip[1],bacdev_ip[2],bacdev_ip[3]);
			bacnet_found = 1;
		}else if((buf[0x30] == 0x30) && (buf[0x32] == 0x0E)){
			//printf("Backnet Read property Reply\n");
			if((buf[0x37] > 0) && (buf[0x37] <= 10) && (buf[0x3D] < 2) && (buf[0x3D] >= 0)){
				if((bac_binval[buf[0x37]-1] == 0x00) && (buf[0x3D] == 0x01)){
					set_bacnet_bin_alarm = buf[0x37]-1;
				}
				bac_binval[buf[0x37]-1] = buf[0x3D];
				printf("Bacnet Binary reply %d  - ",buf[0x37]);
				printf("Bacnet Binary status %d:%d:%d:%d:%d:%d:%d:%d:%d:%d\n",bac_binval[0],bac_binval[1],bac_binval[2],bac_binval[3],bac_binval[4],bac_binval[5],bac_binval[6],bac_binval[7],bac_binval[8],bac_binval[9]);
			}else
			{
				printf("Invalid Instance No\n");
			}
		}
	}
}

void mac1_tick(void)
{
	plen = enc424j600PacketReceiveP1(BUFFER_SIZE, buf1);
	if(plen>0){

		mac1_pkt_cntr++;
		//printf("MAC1 Pkt Count:%d\n",mac1_pkt_cntr);
		if(eth_type_is_ip_and_my_ip(buf1,plen))
		{
			process_bacnet_reply(buf1,plen);
//				printf("----------------\n");
//				for(uint16_t q=0;q<plen;q++)
//					printf("%d ",buf1[q]);
//				printf("\n----------------\n");
		}
//		else if((buf1[12] == 0x08) && (buf1[13]==0x06)){
//			if(buf1[0x15] == 0x02){
//				printf("ARP Reply Rcvd");
//				printf("From : %d:%d:%d:%d:%d:%d \n",buf1[6],buf1[7],buf1[8],buf1[9],buf1[10],buf1[11]);
//			}else if(buf1[0x15] == 0x01){
//				printf("ARP Req Rcvd");
//				printf("From : %d:%d:%d:%d:%d:%d \n",buf1[6],buf1[7],buf1[8],buf1[9],buf1[10],buf1[11]);
//			}
//		}
	}
	packetloop_icmp_tcp(buf1,plen);

}
void mac1_init(void)
{
	Enc424J600Network_initP1(macaddr1);
	dhcp_up=0;
	dhcp_up = allocateIPAddress(buf1, sizeof(buf1), macaddr1, myport, myip, mynetmask, gwip, dnsip, dhcpsvrip );
	printf("\n--------------------------\n");
	printf("IP Address Port 2 -  %d.%d.%d.%d \n",myip[0],myip[1],myip[2],myip[3]);
	printf("Gateway IP Port 2 -  %d.%d.%d.%d \n",gwip[0],gwip[1],gwip[2],gwip[3]);
	printf("DNS  IP    Port 2 -  %d.%d.%d.%d \n",dnsip[0],dnsip[1],dnsip[2],dnsip[3]);
	printf("Init Complete Port 1\n");
	printf("\n--------------------------\n");
	bac_timeout = Sys_GetTick() + 2000;
	bacnet_found = 0;
	invokeid=0;
	bac_inst = 1;
}
void bacnet_whois(void)
{

	uint16_t i;
	char whois_pkt[] = {0x81,0x0B,0x00,0x0C,0x01,0x20,0xFF,0xFF,0x00,0x0E,0x10,0x08};
	dip[0] = myip[0];
	dip[1] = myip[1];
	dip[2] = myip[2];
	dip[3] = 255;
	send_udp_prepare(buf1,sport,dip,dport);
	i=0;
	while(i<sizeof(whois_pkt)){
		buf1[UDP_DATA_P+i] = whois_pkt[i];
		i++;
	}
	//Force Broadcast
	buf1[0] = 0xFF;
	buf1[1] = 0xFF;
	buf1[2] = 0xFF;
	buf1[3] = 0xFF;
	buf1[4] = 0xFF;
	buf1[5] = 0xFF;
	send_udp_transmit(buf1,sizeof(whois_pkt));
	//send_udp(buf1,whois_pkt,sizeof(whois_pkt),sport, dip, dport);
}

void bacnet_read_prop(uint8_t id, uint8_t inst)
{
	uint16_t i;
	char bacrd_pkt[] = {0x81,0x0A,0x00,0x13,0x01,0x04,0x02,0x75,0x53,0x0E,0x0C,0x00,0xC0,0x00,0x02,0x1E,0x09,0x08,0x1F};
	dip[0] = bacdev_ip[0];
	dip[1] = bacdev_ip[1];
	dip[2] = bacdev_ip[2];
	dip[3] = bacdev_ip[3];
	send_udp_prepare(buf1,sport,dip,dport);
	i=0;
	while(i<sizeof(bacrd_pkt)){
		buf1[UDP_DATA_P+i] = bacrd_pkt[i];
		i++;
	}
	//Force Broadcast
	buf1[0] = 0xFF;
	buf1[1] = 0xFF;
	buf1[2] = 0xFF;
	buf1[3] = 0xFF;
	buf1[4] = 0xFF;
	buf1[5] = 0xFF;
	buf1[UDP_DATA_P+8] = id;
	buf1[UDP_DATA_P+14] = inst;
	send_udp_transmit(buf1,sizeof(bacrd_pkt));
}

void mac1_service(void)
{
	//printf("Bacnet timeout %d:%d",bac_timeout,Sys_GetTick());
	if(bac_timeout < Sys_GetTick()){
		if(!bacnet_found)
		{
			printf("Sending Bacnet Who Is \n");
			bacnet_whois();
			bac_timeout = Sys_GetTick() + 10000;
		}else
		{
			//for(uint8_t q = 1; q < 11 ; q++){
			//	bac_inst = q;
				//printf("Reading bacnet instance %d\n",bac_inst);
				bacnet_read_prop(invokeid,bac_inst++);
				bac_timeout = Sys_GetTick() + 5000;
			//}
			if(bac_inst == 11)
				bac_inst = 1;
		}

	}

}
