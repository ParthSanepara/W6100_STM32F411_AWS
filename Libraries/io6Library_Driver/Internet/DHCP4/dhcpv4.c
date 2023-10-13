//*****************************************************************************
//
//! \file dhcpv4.c
//! \brief DHCP APIs implement file.
//! \details Processing DHCP protocol as DISCOVER, OFFER, REQUEST, ACK, NACK and DECLINE.
//! \version 1.1.1
//! \date 2019/10/08
//! \par  Revision history
//!       <2019/10/08> compare DHCP server ip address
//!       <2013/11/18> 1st Release
//!       <2012/12/20> V1.1.0
//!         1. Optimize code
//!         2. Add reg_dhcpv4_cbfunc()
//!         3. Add DHCPv4_stop()
//!         4. Integrate check_DHCPv4_state() & DHCPv4_run() to DHCPv4_run()
//!         5. Don't care system endian
//!         6. Add comments
//!       <2012/12/26> V1.1.1
//!         1. Modify variable declaration: dhcpv4_tick_1s is declared volatile for code optimization
//! \author Eric Jung & MidnightCow
//! \copyright
//!
//! Copyright (c)  2013, WIZnet Co., LTD.
//! All rights reserved.
//!
//! Redistribution and use in source and binary forms, with or without
//! modification, are permitted provided that the following conditions
//! are met:
//!
//!     * Redistributions of source code must retain the above copyright
//! notice, this list of conditions and the following disclaimer.
//!     * Redistributions in binary form must reproduce the above copyright
//! notice, this list of conditions and the following disclaimer in the
//! documentation and/or other materials provided with the distribution.
//!     * Neither the name of the <ORGANIZATION> nor the names of its
//! contributors may be used to endorse or promote products derived
//! from this software without specific prior written permission.
//!
//! THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
//! AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
//! IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
//! ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
//! LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
//! CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
//! SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
//! INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
//! CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
//! ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF
//! THE POSSIBILITY OF SUCH DAMAGE.
//
//*****************************************************************************

#include "socket.h"
#include "dhcpv4.h"

/* If you want to display debug & processing message, Define _DHCPV4_DEBUG_ in dhcp.h */

#ifdef _DHCPV4_DEBUG_
   #include <stdio.h>
#endif

/* DHCP state machine. */
#define STATE_DHCPV4_INIT          0        ///< Initialize
#define STATE_DHCPV4_DISCOVER      1        ///< send DISCOVER and wait OFFER
#define STATE_DHCPV4_REQUEST       2        ///< send REQEUST and wait ACK or NACK
#define STATE_DHCPV4_LEASED        3        ///< ReceiveD ACK and IP leased
#define STATE_DHCPV4_REREQUEST     4        ///< send REQUEST for maintaining leased IP
#define STATE_DHCPV4_RELEASE       5        ///< No use
#define STATE_DHCPV4_STOP          6        ///< Stop processing DHCP

#define DHCPV4_FLAGSBROADCAST      0x8000   ///< The broadcast value of flags in @ref RIP_MSG
#define DHCPV4_FLAGSUNICAST        0x0000   ///< The unicast   value of flags in @ref RIP_MSG

/* DHCP message OP code */
#define DHCPV4_BOOTREQUEST         1        ///< Request Message used in op of @ref RIP_MSG
#define DHCPV4_BOOTREPLY           2        ///< Reply Message used i op of @ref RIP_MSG

/* DHCP message type */
#define DHCPV4_DISCOVER            1        ///< DISCOVER message in OPT of @ref RIP_MSG
#define DHCPV4_OFFER               2        ///< OFFER message in OPT of @ref RIP_MSG
#define DHCPV4_REQUEST             3        ///< REQUEST message in OPT of @ref RIP_MSG
#define DHCPV4_DECLINE             4        ///< DECLINE message in OPT of @ref RIP_MSG
#define DHCPV4_ACK                 5        ///< ACK message in OPT of @ref RIP_MSG
#define DHCPV4_NAK                 6        ///< NACK message in OPT of @ref RIP_MSG
#define DHCPV4_RELEASE             7        ///< RELEASE message in OPT of @ref RIP_MSG. No use
#define DHCPV4_INFORM              8        ///< INFORM message in OPT of @ref RIP_MSG. No use

#define DHCPV4_HTYPE10MB           1        ///< Used in type of @ref RIP_MSG
#define DHCPV4_HTYPE100MB          2        ///< Used in type of @ref RIP_MSG

#define DHCPV4_HLENETHERNET        6        ///< Used in hlen of @ref RIP_MSG
#define DHCPV4_HOPS                0        ///< Used in hops of @ref RIP_MSG
#define DHCPV4_SECS                0        ///< Used in secs of @ref RIP_MSG

#define INFINITE_LEASETIME       0xffffffff	///< Infinite lease time

#define OPT_SIZE                 312               /// Max OPT size of @ref RIP_MSG
#define RIP_MSG_SIZE             (236+OPT_SIZE)    /// Max size of @ref RIP_MSG

/*
 * @brief DHCP option and value (cf. RFC1533)
 */
enum
{
   padOption               = 0,
   subnetMask              = 1,
   timerOffset             = 2,
   routersOnSubnet         = 3,
   timeServer              = 4,
   nameServer              = 5,
   dns                     = 6,
   logServer               = 7,
   cookieServer            = 8,
   lprServer               = 9,
   impressServer           = 10,
   resourceLocationServer	= 11,
   hostName                = 12,
   bootFileSize            = 13,
   meritDumpFile           = 14,
   domainName              = 15,
   swapServer              = 16,
   rootPath                = 17,
   extentionsPath          = 18,
   IPforwarding            = 19,
   nonLocalSourceRouting   = 20,
   policyFilter            = 21,
   maxDgramReasmSize       = 22,
   defaultIPTTL            = 23,
   pathMTUagingTimeout     = 24,
   pathMTUplateauTable     = 25,
   ifMTU                   = 26,
   allSubnetsLocal         = 27,
   broadcastAddr           = 28,
   performMaskDiscovery    = 29,
   maskSupplier            = 30,
   performRouterDiscovery  = 31,
   routerSolicitationAddr  = 32,
   staticRoute             = 33,
   trailerEncapsulation    = 34,
   arpCacheTimeout         = 35,
   ethernetEncapsulation   = 36,
   tcpDefaultTTL           = 37,
   tcpKeepaliveInterval    = 38,
   tcpKeepaliveGarbage     = 39,
   nisDomainName           = 40,
   nisServers              = 41,
   ntpServers              = 42,
   vendorSpecificInfo      = 43,
   netBIOSnameServer       = 44,
   netBIOSdgramDistServer	= 45,
   netBIOSnodeType         = 46,
   netBIOSscope            = 47,
   xFontServer             = 48,
   xDisplayManager         = 49,
   dhcpRequestedIPaddr     = 50,
   dhcpIPaddrLeaseTime     = 51,
   dhcpOptionOverload      = 52,
   dhcpMessageType         = 53,
   dhcpServerIdentifier    = 54,
   dhcpParamRequest        = 55,
   dhcpMsg                 = 56,
   dhcpMaxMsgSize          = 57,
   dhcpT1value             = 58,
   dhcpT2value             = 59,
   dhcpClassIdentifier     = 60,
   dhcpClientIdentifier    = 61,
   endOption               = 255
};

/*
 * @brief DHCP message format
 */
typedef struct {
	uint8_t  op;            ///< @ref DHCPV4_BOOTREQUEST or @ref DHCPV4_BOOTREPLY
	uint8_t  htype;         ///< @ref DHCPV4_HTYPE10MB or @ref DHCPV4_HTYPE100MB
	uint8_t  hlen;          ///< @ref DHCPV4_HLENETHERNET
	uint8_t  hops;          ///< @ref DHCPV4_HOPS
	uint32_t xid;           ///< @ref DHCPV4_XID  This increase one every DHCP transaction.
	uint16_t secs;          ///< @ref DHCPV4_SECS
	uint16_t flags;         ///< @ref DHCPV4_FLAGSBROADCAST or @ref DHCPV4_FLAGSUNICAST
	uint8_t  ciaddr[4];     ///< @ref Request IP to DHCP sever
	uint8_t  yiaddr[4];     ///< @ref Offered IP from DHCP server
	uint8_t  siaddr[4];     ///< No use
	uint8_t  giaddr[4];     ///< No use
	uint8_t  chaddr[16];    ///< DHCP client 6bytes MAC address. Others is filled to zero
	uint8_t  sname[64];     ///< No use
	uint8_t  file[128];     ///< No use
	uint8_t  OPT[OPT_SIZE]; ///< Option
} RIP_MSG;



uint8_t DHCPV4_SOCKET;                      // Socket number for DHCP

uint8_t DHCPV4_SIP[4];                      // DHCP Server IP address
uint8_t DHCPV4_REAL_SIP[4];                 // For extract my DHCP server in a few DHCP server

// Network information from DHCP Server
uint8_t OLD_allocated_ip[4]   = {0, };    // Previous IP address
uint8_t DHCPv4_allocated_ip[4]  = {0, };    // IP address from DHCP
uint8_t DHCPv4_allocated_gw[4]  = {0, };    // Gateway address from DHCP
uint8_t DHCPv4_allocated_sn[4]  = {0, };    // Subnet mask from DHCP
uint8_t DHCPv4_allocated_dns[4] = {0, };    // DNS address from DHCP


int8_t   dhcpv4_state        = STATE_DHCPV4_INIT;   // DHCP state
int8_t   dhcpv4_retry_count  = 0;

uint32_t dhcpv4_lease_time   			= INFINITE_LEASETIME;
volatile uint32_t dhcpv4_tick_1s      = 0;                 // unit 1 second
uint32_t dhcpv4_tick_next    			= DHCPV4_WAIT_TIME ;

uint32_t DHCPV4_XID;      // Any number

RIP_MSG* pDHCPv4MSG;      // Buffer pointer for DHCP processing

uint8_t HOST_NAMEv4[] = DCHPV4_HOST_NAME;

uint8_t DHCPv4_CHADDR[6]; // DHCP Client MAC address.

/* The default callback function */
void default_ipv4_assign(void);
void default_ipv4_update(void);
void default_ipv4_conflict(void);

/* Callback handler */
void (*dhcp_ipv4_assign)(void)   = default_ipv4_assign;     /* handler to be called when the IP address from DHCP server is first assigned */
void (*dhcp_ipv4_update)(void)   = default_ipv4_update;     /* handler to be called when the IP address from DHCP server is updated */
void (*dhcp_ipv4_conflict)(void) = default_ipv4_conflict;   /* handler to be called when the IP address from DHCP server is conflict */

void reg_dhcpv4_cbfunc(void(*ip_assign)(void), void(*ip_update)(void), void(*ip_conflict)(void));

char NibbleToHex(uint8_t nibble);

/* send DISCOVER message to DHCP server */
void     send_DHCPv4_DISCOVER(void);

/* send REQEUST message to DHCP server */
void     send_DHCPv4_REQUEST(void);

/* send DECLINE message to DHCP server */
void     send_DHCPv4_DECLINE(void);

/* IP conflict check by sending ARP-request to leased IP and wait ARP-response. */
int8_t   check_DHCPv4_leasedIP(void);

/* check the timeout in DHCP process */
uint8_t  check_DHCPv4_timeout(void);

/* Initialize to timeout process.  */
void     reset_DHCPv4_timeout(void);

/* Parse message as OFFER and ACK and NACK from DHCP server.*/
int8_t   parseDHCPCMSG(void);

/* The default handler of ip assign first */
void default_ipv4_assign(void)
{
   setSIPR(DHCPv4_allocated_ip);
   setSUBR(DHCPv4_allocated_sn);
   setGAR (DHCPv4_allocated_gw);
}

/* The default handler of ip changed */
void default_ipv4_update(void)
{
	/* WIZchip Software Reset */
   setSYCR0(SYCR0_RST);
   getSYCR0(); // for delay
   default_ipv4_assign();
   setSHAR(DHCPv4_CHADDR);
}

/* The default handler of ip changed */
void default_ipv4_conflict(void)
{
	// WIZchip Software Reset
	setSYCR0(SYCR0_RST);
	getSYCR0(); // for delay
	setSHAR(DHCPv4_CHADDR);
}

/* register the call back func. */
void reg_dhcpv4_cbfunc(void(*ip_assign)(void), void(*ip_update)(void), void(*ip_conflict)(void))
{
   dhcp_ipv4_assign   = default_ipv4_assign;
   dhcp_ipv4_update   = default_ipv4_update;
   dhcp_ipv4_conflict = default_ipv4_conflict;
   if(ip_assign)   dhcp_ipv4_assign = ip_assign;
   if(ip_update)   dhcp_ipv4_update = ip_update;
   if(ip_conflict) dhcp_ipv4_conflict = ip_conflict;
}

/* make the common DHCP message */
void makeDHCPV4MSG(void)
{
   uint8_t  bk_mac[6];
   uint8_t* ptmp;
   uint8_t  i;
   getSHAR(bk_mac);
	pDHCPv4MSG->op      = DHCPV4_BOOTREQUEST;
	pDHCPv4MSG->htype   = DHCPV4_HTYPE10MB;
	pDHCPv4MSG->hlen    = DHCPV4_HLENETHERNET;
	pDHCPv4MSG->hops    = DHCPV4_HOPS;
	ptmp              = (uint8_t*)(&pDHCPv4MSG->xid);
	*(ptmp+0)         = (uint8_t)((DHCPV4_XID & 0xFF000000) >> 24);
	*(ptmp+1)         = (uint8_t)((DHCPV4_XID & 0x00FF0000) >> 16);
   *(ptmp+2)         = (uint8_t)((DHCPV4_XID & 0x0000FF00) >>  8);
	*(ptmp+3)         = (uint8_t)((DHCPV4_XID & 0x000000FF) >>  0);
	pDHCPv4MSG->secs    = DHCPV4_SECS;
	ptmp              = (uint8_t*)(&pDHCPv4MSG->flags);
	*(ptmp+0)         = (uint8_t)((DHCPV4_FLAGSBROADCAST & 0xFF00) >> 8);
	*(ptmp+1)         = (uint8_t)((DHCPV4_FLAGSBROADCAST & 0x00FF) >> 0);

	pDHCPv4MSG->ciaddr[0] = 0;
	pDHCPv4MSG->ciaddr[1] = 0;
	pDHCPv4MSG->ciaddr[2] = 0;
	pDHCPv4MSG->ciaddr[3] = 0;

	pDHCPv4MSG->yiaddr[0] = 0;
	pDHCPv4MSG->yiaddr[1] = 0;
	pDHCPv4MSG->yiaddr[2] = 0;
	pDHCPv4MSG->yiaddr[3] = 0;

	pDHCPv4MSG->siaddr[0] = 0;
	pDHCPv4MSG->siaddr[1] = 0;
	pDHCPv4MSG->siaddr[2] = 0;
	pDHCPv4MSG->siaddr[3] = 0;

	pDHCPv4MSG->giaddr[0] = 0;
	pDHCPv4MSG->giaddr[1] = 0;
	pDHCPv4MSG->giaddr[2] = 0;
	pDHCPv4MSG->giaddr[3] = 0;

	pDHCPv4MSG->chaddr[0] = DHCPv4_CHADDR[0];
	pDHCPv4MSG->chaddr[1] = DHCPv4_CHADDR[1];
	pDHCPv4MSG->chaddr[2] = DHCPv4_CHADDR[2];
	pDHCPv4MSG->chaddr[3] = DHCPv4_CHADDR[3];
	pDHCPv4MSG->chaddr[4] = DHCPv4_CHADDR[4];
	pDHCPv4MSG->chaddr[5] = DHCPv4_CHADDR[5];

	for (i = 6; i < 16; i++)  pDHCPv4MSG->chaddr[i] = 0;
	for (i = 0; i < 64; i++)  pDHCPv4MSG->sname[i]  = 0;
	for (i = 0; i < 128; i++) pDHCPv4MSG->file[i]   = 0;

	// MAGIC_COOKIE
	pDHCPv4MSG->OPT[0] = (uint8_t)((MAGIC_COOKIE & 0xFF000000) >> 24);
	pDHCPv4MSG->OPT[1] = (uint8_t)((MAGIC_COOKIE & 0x00FF0000) >> 16);
	pDHCPv4MSG->OPT[2] = (uint8_t)((MAGIC_COOKIE & 0x0000FF00) >>  8);
	pDHCPv4MSG->OPT[3] = (uint8_t) (MAGIC_COOKIE & 0x000000FF) >>  0;
}

/* SEND DHCP DISCOVER */
void send_DHCPv4_DISCOVER(void)
{
	uint16_t i;
	uint8_t ip[4];
	uint16_t k = 0;

   makeDHCPV4MSG();
   DHCPV4_SIP[0]=0;
   DHCPV4_SIP[1]=0;
   DHCPV4_SIP[2]=0;
   DHCPV4_SIP[3]=0;
   DHCPV4_REAL_SIP[0]=0;
   DHCPV4_REAL_SIP[1]=0;
   DHCPV4_REAL_SIP[2]=0;
   DHCPV4_REAL_SIP[3]=0;

   k = 4;     // because MAGIC_COOKIE already made by makeDHCPV4MSG()

	// Option Request Param
	pDHCPv4MSG->OPT[k++] = dhcpMessageType;
	pDHCPv4MSG->OPT[k++] = 0x01;
	pDHCPv4MSG->OPT[k++] = DHCPV4_DISCOVER;

	// Client identifier
	pDHCPv4MSG->OPT[k++] = dhcpClientIdentifier;
	pDHCPv4MSG->OPT[k++] = 0x07;
	pDHCPv4MSG->OPT[k++] = 0x01;
	pDHCPv4MSG->OPT[k++] = DHCPv4_CHADDR[0];
	pDHCPv4MSG->OPT[k++] = DHCPv4_CHADDR[1];
	pDHCPv4MSG->OPT[k++] = DHCPv4_CHADDR[2];
	pDHCPv4MSG->OPT[k++] = DHCPv4_CHADDR[3];
	pDHCPv4MSG->OPT[k++] = DHCPv4_CHADDR[4];
	pDHCPv4MSG->OPT[k++] = DHCPv4_CHADDR[5];

	// host name
	pDHCPv4MSG->OPT[k++] = hostName;
	pDHCPv4MSG->OPT[k++] = 0;          // fill zero length of hostname
	for(i = 0 ; HOST_NAMEv4[i] != 0; i++)
   	pDHCPv4MSG->OPT[k++] = HOST_NAMEv4[i];
	pDHCPv4MSG->OPT[k++] = NibbleToHex(DHCPv4_CHADDR[3] >> 4);
	pDHCPv4MSG->OPT[k++] = NibbleToHex(DHCPv4_CHADDR[3]);
	pDHCPv4MSG->OPT[k++] = NibbleToHex(DHCPv4_CHADDR[4] >> 4);
	pDHCPv4MSG->OPT[k++] = NibbleToHex(DHCPv4_CHADDR[4]);
	pDHCPv4MSG->OPT[k++] = NibbleToHex(DHCPv4_CHADDR[5] >> 4);
	pDHCPv4MSG->OPT[k++] = NibbleToHex(DHCPv4_CHADDR[5]);
	pDHCPv4MSG->OPT[k - (i+6+1)] = i+6; // length of hostname

	pDHCPv4MSG->OPT[k++] = dhcpParamRequest;
	pDHCPv4MSG->OPT[k++] = 0x06;	// length of request
	pDHCPv4MSG->OPT[k++] = subnetMask;
	pDHCPv4MSG->OPT[k++] = routersOnSubnet;
	pDHCPv4MSG->OPT[k++] = dns;
	pDHCPv4MSG->OPT[k++] = domainName;
	pDHCPv4MSG->OPT[k++] = dhcpT1value;
	pDHCPv4MSG->OPT[k++] = dhcpT2value;
	pDHCPv4MSG->OPT[k++] = endOption;

	for (i = k; i < OPT_SIZE; i++) pDHCPv4MSG->OPT[i] = 0;

	// send broadcasting packet
	ip[0] = 255;
	ip[1] = 255;
	ip[2] = 255;
	ip[3] = 255;

#ifdef _DHCPV4_DEBUG_
	printf("> Send DHCP_DISCOVER\r\n");
#endif

	sendto(DHCPV4_SOCKET, (uint8_t *)pDHCPv4MSG, RIP_MSG_SIZE, ip, DHCPV4_SERVER_PORT, 4);
}

/* SEND DHCP REQUEST */
void send_DHCPv4_REQUEST(void)
{
	int i;
	uint8_t ip[4];
	uint16_t k = 0;

   makeDHCPV4MSG();

   if(dhcpv4_state == STATE_DHCPV4_LEASED || dhcpv4_state == STATE_DHCPV4_REREQUEST)
   {
   	*((uint8_t*)(&pDHCPv4MSG->flags))   = ((DHCPV4_FLAGSUNICAST & 0xFF00)>> 8);
   	*((uint8_t*)(&pDHCPv4MSG->flags)+1) = (DHCPV4_FLAGSUNICAST & 0x00FF);
   	pDHCPv4MSG->ciaddr[0] = DHCPv4_allocated_ip[0];
   	pDHCPv4MSG->ciaddr[1] = DHCPv4_allocated_ip[1];
   	pDHCPv4MSG->ciaddr[2] = DHCPv4_allocated_ip[2];
   	pDHCPv4MSG->ciaddr[3] = DHCPv4_allocated_ip[3];
   	ip[0] = DHCPV4_SIP[0];
   	ip[1] = DHCPV4_SIP[1];
   	ip[2] = DHCPV4_SIP[2];
   	ip[3] = DHCPV4_SIP[3];
   }
   else
   {
   	ip[0] = 255;
   	ip[1] = 255;
   	ip[2] = 255;
   	ip[3] = 255;
   }

   k = 4;      // because MAGIC_COOKIE already made by makeDHCPV4MSG()

	// Option Request Param.
	pDHCPv4MSG->OPT[k++] = dhcpMessageType;
	pDHCPv4MSG->OPT[k++] = 0x01;
	pDHCPv4MSG->OPT[k++] = DHCPV4_REQUEST;

	pDHCPv4MSG->OPT[k++] = dhcpClientIdentifier;
	pDHCPv4MSG->OPT[k++] = 0x07;
	pDHCPv4MSG->OPT[k++] = 0x01;
	pDHCPv4MSG->OPT[k++] = DHCPv4_CHADDR[0];
	pDHCPv4MSG->OPT[k++] = DHCPv4_CHADDR[1];
	pDHCPv4MSG->OPT[k++] = DHCPv4_CHADDR[2];
	pDHCPv4MSG->OPT[k++] = DHCPv4_CHADDR[3];
	pDHCPv4MSG->OPT[k++] = DHCPv4_CHADDR[4];
	pDHCPv4MSG->OPT[k++] = DHCPv4_CHADDR[5];

   if(ip[3] == 255)  // if(dchp_state == STATE_DHCPV4_LEASED || dchp_state == DHCP_REREQUEST_STATE)
   {
		pDHCPv4MSG->OPT[k++] = dhcpRequestedIPaddr;
		pDHCPv4MSG->OPT[k++] = 0x04;
		pDHCPv4MSG->OPT[k++] = DHCPv4_allocated_ip[0];
		pDHCPv4MSG->OPT[k++] = DHCPv4_allocated_ip[1];
		pDHCPv4MSG->OPT[k++] = DHCPv4_allocated_ip[2];
		pDHCPv4MSG->OPT[k++] = DHCPv4_allocated_ip[3];

		pDHCPv4MSG->OPT[k++] = dhcpServerIdentifier;
		pDHCPv4MSG->OPT[k++] = 0x04;
		pDHCPv4MSG->OPT[k++] = DHCPV4_SIP[0];
		pDHCPv4MSG->OPT[k++] = DHCPV4_SIP[1];
		pDHCPv4MSG->OPT[k++] = DHCPV4_SIP[2];
		pDHCPv4MSG->OPT[k++] = DHCPV4_SIP[3];
	}

	// host name
	pDHCPv4MSG->OPT[k++] = hostName;
	pDHCPv4MSG->OPT[k++] = 0; // length of hostname
	for(i = 0 ; HOST_NAMEv4[i] != 0; i++)
   	pDHCPv4MSG->OPT[k++] = HOST_NAMEv4[i];
	pDHCPv4MSG->OPT[k++] = NibbleToHex(DHCPv4_CHADDR[3] >> 4);
	pDHCPv4MSG->OPT[k++] = NibbleToHex(DHCPv4_CHADDR[3]);
	pDHCPv4MSG->OPT[k++] = NibbleToHex(DHCPv4_CHADDR[4] >> 4);
	pDHCPv4MSG->OPT[k++] = NibbleToHex(DHCPv4_CHADDR[4]);
	pDHCPv4MSG->OPT[k++] = NibbleToHex(DHCPv4_CHADDR[5] >> 4);
	pDHCPv4MSG->OPT[k++] = NibbleToHex(DHCPv4_CHADDR[5]);
	pDHCPv4MSG->OPT[k - (i+6+1)] = i+6; // length of hostname

	pDHCPv4MSG->OPT[k++] = dhcpParamRequest;
	pDHCPv4MSG->OPT[k++] = 0x08;
	pDHCPv4MSG->OPT[k++] = subnetMask;
	pDHCPv4MSG->OPT[k++] = routersOnSubnet;
	pDHCPv4MSG->OPT[k++] = dns;
	pDHCPv4MSG->OPT[k++] = domainName;
	pDHCPv4MSG->OPT[k++] = dhcpT1value;
	pDHCPv4MSG->OPT[k++] = dhcpT2value;
	pDHCPv4MSG->OPT[k++] = performRouterDiscovery;
	pDHCPv4MSG->OPT[k++] = staticRoute;
	pDHCPv4MSG->OPT[k++] = endOption;

	for (i = k; i < OPT_SIZE; i++) pDHCPv4MSG->OPT[i] = 0;

#ifdef _DHCPV4_DEBUG_
	printf("> Send DHCP_REQUEST\r\n");
#endif

	sendto(DHCPV4_SOCKET, (uint8_t *)pDHCPv4MSG, RIP_MSG_SIZE, ip, DHCPV4_SERVER_PORT, 4);

}

/* SEND DHCP DHCPDECLINE */
void send_DHCPv4_DECLINE(void)
{
	int i;
	uint8_t ip[4];
	uint16_t k = 0;

	makeDHCPV4MSG();

   k = 4;      // because MAGIC_COOKIE already made by makeDHCPV4MSG()

	*((uint8_t*)(&pDHCPv4MSG->flags))   = ((DHCPV4_FLAGSUNICAST & 0xFF00)>> 8);
	*((uint8_t*)(&pDHCPv4MSG->flags)+1) = (DHCPV4_FLAGSUNICAST & 0x00FF);

	// Option Request Param.
	pDHCPv4MSG->OPT[k++] = dhcpMessageType;
	pDHCPv4MSG->OPT[k++] = 0x01;
	pDHCPv4MSG->OPT[k++] = DHCPV4_DECLINE;

	pDHCPv4MSG->OPT[k++] = dhcpClientIdentifier;
	pDHCPv4MSG->OPT[k++] = 0x07;
	pDHCPv4MSG->OPT[k++] = 0x01;
	pDHCPv4MSG->OPT[k++] = DHCPv4_CHADDR[0];
	pDHCPv4MSG->OPT[k++] = DHCPv4_CHADDR[1];
	pDHCPv4MSG->OPT[k++] = DHCPv4_CHADDR[2];
	pDHCPv4MSG->OPT[k++] = DHCPv4_CHADDR[3];
	pDHCPv4MSG->OPT[k++] = DHCPv4_CHADDR[4];
	pDHCPv4MSG->OPT[k++] = DHCPv4_CHADDR[5];

	pDHCPv4MSG->OPT[k++] = dhcpRequestedIPaddr;
	pDHCPv4MSG->OPT[k++] = 0x04;
	pDHCPv4MSG->OPT[k++] = DHCPv4_allocated_ip[0];
	pDHCPv4MSG->OPT[k++] = DHCPv4_allocated_ip[1];
	pDHCPv4MSG->OPT[k++] = DHCPv4_allocated_ip[2];
	pDHCPv4MSG->OPT[k++] = DHCPv4_allocated_ip[3];

	pDHCPv4MSG->OPT[k++] = dhcpServerIdentifier;
	pDHCPv4MSG->OPT[k++] = 0x04;
	pDHCPv4MSG->OPT[k++] = DHCPV4_SIP[0];
	pDHCPv4MSG->OPT[k++] = DHCPV4_SIP[1];
	pDHCPv4MSG->OPT[k++] = DHCPV4_SIP[2];
	pDHCPv4MSG->OPT[k++] = DHCPV4_SIP[3];

	pDHCPv4MSG->OPT[k++] = endOption;

	for (i = k; i < OPT_SIZE; i++) pDHCPv4MSG->OPT[i] = 0;

	//send broadcasting packet
	ip[0] = 0xFF;
	ip[1] = 0xFF;
	ip[2] = 0xFF;
	ip[3] = 0xFF;

#ifdef _DHCPV4_DEBUG_
	printf("\r\n> Send DHCPV4_DECLINE\r\n");
#endif

	sendto(DHCPV4_SOCKET, (uint8_t *)pDHCPv4MSG, RIP_MSG_SIZE, ip, DHCPV4_SERVER_PORT, 4);
}

/* PARSE REPLY pDHCPv4MSG */
int8_t parseDHCPv4MSG(void)
{
	uint8_t svr_addr[6];
	uint16_t  svr_port;
	uint16_t len;

	uint8_t * p;
	uint8_t * e;
	uint8_t type = 0;
	uint8_t opt_len;
	uint8_t addr_len;

   if((len = getSn_RX_RSR(DHCPV4_SOCKET)) > 0)
   {
   	len = recvfrom(DHCPV4_SOCKET, (uint8_t *)pDHCPv4MSG, len, svr_addr, &svr_port, &addr_len);
   #ifdef _DHCPV4_DEBUG_
      printf("DHCP message : %d.%d.%d.%d(%d) %d received. \r\n",svr_addr[0],svr_addr[1],svr_addr[2], svr_addr[3],svr_port, len);
   #endif
   }
   else return 0;
	if (svr_port == DHCPV4_SERVER_PORT) {
      // compare mac address
		if ( (pDHCPv4MSG->chaddr[0] != DHCPv4_CHADDR[0]) || (pDHCPv4MSG->chaddr[1] != DHCPv4_CHADDR[1]) ||
		     (pDHCPv4MSG->chaddr[2] != DHCPv4_CHADDR[2]) || (pDHCPv4MSG->chaddr[3] != DHCPv4_CHADDR[3]) ||
		     (pDHCPv4MSG->chaddr[4] != DHCPv4_CHADDR[4]) || (pDHCPv4MSG->chaddr[5] != DHCPv4_CHADDR[5])   )
		{
#ifdef _DHCPV4_DEBUG_
            printf("No My DHCP Message. This message is ignored.\r\n");
#endif
         return 0;
		}
        //compare DHCP server ip address
        if((DHCPV4_SIP[0]!=0) || (DHCPV4_SIP[1]!=0) || (DHCPV4_SIP[2]!=0) || (DHCPV4_SIP[3]!=0)){
            if( ((svr_addr[0]!=DHCPV4_SIP[0])|| (svr_addr[1]!=DHCPV4_SIP[1])|| (svr_addr[2]!=DHCPV4_SIP[2])|| (svr_addr[3]!=DHCPV4_SIP[3])) &&
                ((svr_addr[0]!=DHCPV4_REAL_SIP[0])|| (svr_addr[1]!=DHCPV4_REAL_SIP[1])|| (svr_addr[2]!=DHCPV4_REAL_SIP[2])|| (svr_addr[3]!=DHCPV4_REAL_SIP[3]))  )
            {
#ifdef _DHCPV4_DEBUG_
                printf("Another DHCP sever send a response message. This is ignored.\r\n");
#endif
                return 0;
            }
        }
		p = (uint8_t *)(&pDHCPv4MSG->op);
		p = p + 240;      // 240 = sizeof(RIP_MSG) + MAGIC_COOKIE size in RIP_MSG.opt - sizeof(RIP_MSG.opt)
		e = p + (len - 240);

		while ( p < e ) {

			switch ( *p ) {

   			case endOption :
   			   p = e;   // for break while(p < e)
   				break;
            case padOption :
   				p++;
   				break;
   			case dhcpMessageType :
   				p++;
   				p++;
   				type = *p++;
   				break;
   			case subnetMask :
   				p++;
   				p++;
   				DHCPv4_allocated_sn[0] = *p++;
   				DHCPv4_allocated_sn[1] = *p++;
   				DHCPv4_allocated_sn[2] = *p++;
   				DHCPv4_allocated_sn[3] = *p++;
   				break;
   			case routersOnSubnet :
   				p++;
   				opt_len = *p++;
   				DHCPv4_allocated_gw[0] = *p++;
   				DHCPv4_allocated_gw[1] = *p++;
   				DHCPv4_allocated_gw[2] = *p++;
   				DHCPv4_allocated_gw[3] = *p++;
   				p = p + (opt_len - 4);
   				break;
   			case dns :
   				p++;
   				opt_len = *p++;
   				DHCPv4_allocated_dns[0] = *p++;
   				DHCPv4_allocated_dns[1] = *p++;
   				DHCPv4_allocated_dns[2] = *p++;
   				DHCPv4_allocated_dns[3] = *p++;
   				p = p + (opt_len - 4);
   				break;
   			case dhcpIPaddrLeaseTime :
   				p++;
   				opt_len = *p++;
   				dhcpv4_lease_time  = *p++;
   				dhcpv4_lease_time  = (dhcpv4_lease_time << 8) + *p++;
   				dhcpv4_lease_time  = (dhcpv4_lease_time << 8) + *p++;
   				dhcpv4_lease_time  = (dhcpv4_lease_time << 8) + *p++;
            #ifdef _DHCPV4_DEBUG_
               dhcpv4_lease_time = 10;
 				#endif
   				break;
   			case dhcpServerIdentifier :
   				p++;
   				opt_len = *p++;
   				DHCPV4_SIP[0] = *p++;
   				DHCPV4_SIP[1] = *p++;
   				DHCPV4_SIP[2] = *p++;
   				DHCPV4_SIP[3] = *p++;
                DHCPV4_REAL_SIP[0]=svr_addr[0];
                DHCPV4_REAL_SIP[1]=svr_addr[1];
                DHCPV4_REAL_SIP[2]=svr_addr[2];
                DHCPV4_REAL_SIP[3]=svr_addr[3];
   				break;
   			default :
   				p++;
   				opt_len = *p++;
   				p += opt_len;
   				break;
			} // switch
		} // while
	} // if
	return	type;
}

uint8_t DHCPv4_run(void)
{
	uint8_t  type;
	uint8_t  ret;

	if(dhcpv4_state == STATE_DHCPV4_STOP) return DHCPV4_STOPPED;

	if(getSn_SR(DHCPV4_SOCKET) != SOCK_UDP)
	   socket(DHCPV4_SOCKET, Sn_MR_UDP, DHCPV4_CLIENT_PORT, 0x00);

	ret = DHCPV4_RUNNING;
	type = parseDHCPv4MSG();


	switch ( dhcpv4_state ) {
	   case STATE_DHCPV4_INIT     :
         DHCPv4_allocated_ip[0] = 0;
         DHCPv4_allocated_ip[1] = 0;
         DHCPv4_allocated_ip[2] = 0;
         DHCPv4_allocated_ip[3] = 0;
   		send_DHCPv4_DISCOVER();
   		dhcpv4_state = STATE_DHCPV4_DISCOVER;
   		break;
		case STATE_DHCPV4_DISCOVER :
			if (type == DHCPV4_OFFER){
#ifdef _DHCPV4_DEBUG_
				printf("> Receive DHCP_OFFER\r\n");
#endif
            DHCPv4_allocated_ip[0] = pDHCPv4MSG->yiaddr[0];
            DHCPv4_allocated_ip[1] = pDHCPv4MSG->yiaddr[1];
            DHCPv4_allocated_ip[2] = pDHCPv4MSG->yiaddr[2];
            DHCPv4_allocated_ip[3] = pDHCPv4MSG->yiaddr[3];

				send_DHCPv4_REQUEST();
				dhcpv4_state = STATE_DHCPV4_REQUEST;
			} else ret = check_DHCPv4_timeout();
         break;

		case STATE_DHCPV4_REQUEST :
			if (type == DHCPV4_ACK) {

#ifdef _DHCPV4_DEBUG_
				printf("> Receive DHCP_ACK\r\n");
#endif
				if (check_DHCPv4_leasedIP()) {
					// Network info assignment from DHCP
					dhcp_ipv4_assign();
					reset_DHCPv4_timeout();

					dhcpv4_state = STATE_DHCPV4_LEASED;
				} else {
					// IP address conflict occurred
					reset_DHCPv4_timeout();
					dhcp_ipv4_conflict();
				    dhcpv4_state = STATE_DHCPV4_INIT;
				}
			} else if (type == DHCPV4_NAK) {

#ifdef _DHCPV4_DEBUG_
				printf("> Receive DHCP_NACK\r\n");
#endif

				reset_DHCPv4_timeout();

				dhcpv4_state = STATE_DHCPV4_DISCOVER;
			} else ret = check_DHCPv4_timeout();
		break;

		case STATE_DHCPV4_LEASED :
		   ret = DHCP_IPV4_LEASED;
			if ((dhcpv4_lease_time != INFINITE_LEASETIME) && ((dhcpv4_lease_time/2) < dhcpv4_tick_1s)) {

#ifdef _DHCPV4_DEBUG_
 				printf("> Maintains the IP address \r\n");
#endif

				type = 0;
				OLD_allocated_ip[0] = DHCPv4_allocated_ip[0];
				OLD_allocated_ip[1] = DHCPv4_allocated_ip[1];
				OLD_allocated_ip[2] = DHCPv4_allocated_ip[2];
				OLD_allocated_ip[3] = DHCPv4_allocated_ip[3];

				DHCPV4_XID++;

				send_DHCPv4_REQUEST();

				reset_DHCPv4_timeout();

				dhcpv4_state = STATE_DHCPV4_REREQUEST;
			}
		break;

		case STATE_DHCPV4_REREQUEST :
		   ret = DHCP_IPV4_LEASED;
			if (type == DHCPV4_ACK) {
				dhcpv4_retry_count = 0;
				if (OLD_allocated_ip[0] != DHCPv4_allocated_ip[0] ||
				    OLD_allocated_ip[1] != DHCPv4_allocated_ip[1] ||
				    OLD_allocated_ip[2] != DHCPv4_allocated_ip[2] ||
				    OLD_allocated_ip[3] != DHCPv4_allocated_ip[3])
				{
					ret = DHCP_IPV4_CHANGED;
					dhcp_ipv4_update();
               #ifdef _DHCPV4_DEBUG_
                  printf(">IP changed.\r\n");
               #endif

				}
         #ifdef _DHCPV4_DEBUG_
            else printf(">IP is continued.\r\n");
         #endif
				reset_DHCPv4_timeout();
				dhcpv4_state = STATE_DHCPV4_LEASED;
			} else if (type == DHCPV4_NAK) {

#ifdef _DHCPV4_DEBUG_
				printf("> Receive DHCP_NACK, Failed to maintain ip\r\n");
#endif

				reset_DHCPv4_timeout();

				dhcpv4_state = STATE_DHCPV4_DISCOVER;
			} else ret = check_DHCPv4_timeout();
	   	break;
		default :
   		break;
	}

	return ret;
}

void    DHCPv4_stop(void)
{
   close(DHCPV4_SOCKET);
   dhcpv4_state = STATE_DHCPV4_STOP;
}

uint8_t check_DHCPv4_timeout(void)
{
	uint8_t ret = DHCPV4_RUNNING;

	if (dhcpv4_retry_count < MAX_DHCPV4_RETRY) {
		if (dhcpv4_tick_next < dhcpv4_tick_1s) {

			switch ( dhcpv4_state ) {
				case STATE_DHCPV4_DISCOVER :
//					printf("<<timeout>> state : STATE_DHCPV4_DISCOVER\r\n");
					send_DHCPv4_DISCOVER();
				break;

				case STATE_DHCPV4_REQUEST :
//					printf("<<timeout>> state : STATE_DHCPV4_REQUEST\r\n");

					send_DHCPv4_REQUEST();
				break;

				case STATE_DHCPV4_REREQUEST :
//					printf("<<timeout>> state : STATE_DHCPV4_REREQUEST\r\n");

					send_DHCPv4_REQUEST();
				break;

				default :
				break;
			}

			dhcpv4_tick_1s = 0;
			dhcpv4_tick_next = dhcpv4_tick_1s + DHCPV4_WAIT_TIME;
			dhcpv4_retry_count++;
		}
	} else { // timeout occurred

		switch(dhcpv4_state) {
			case STATE_DHCPV4_DISCOVER:
				dhcpv4_state = STATE_DHCPV4_INIT;
				ret = DHCPV4_FAILED;
				break;
			case STATE_DHCPV4_REQUEST:
			case STATE_DHCPV4_REREQUEST:
				send_DHCPv4_DISCOVER();
				dhcpv4_state = STATE_DHCPV4_DISCOVER;
				break;
			default :
				break;
		}
		reset_DHCPv4_timeout();
	}
	return ret;
}

int8_t check_DHCPv4_leasedIP(void)
{
	uint8_t tmp;
	int32_t ret;

	//WIZchip RCR value changed for ARP Timeout count control
	tmp = getRCR();
	setRCR(0x03);

	// IP conflict detection : ARP request - ARP reply
	// Broadcasting ARP Request for check the IP conflict using UDP sendto() function
	ret = sendto(DHCPV4_SOCKET, (uint8_t *)"CHECK_IP_CONFLICT", 17, DHCPv4_allocated_ip, 5000, 4);

	// RCR value restore
	setRCR(tmp);

	if(ret == SOCKERR_TIMEOUT) {
		// UDP send Timeout occurred : allocated IP address is unique, DHCP Success

#ifdef _DHCPV4_DEBUG_
		printf("\r\n> Check leased IP - OK\r\n");
#endif

		return 1;
	} else {
		// Received ARP reply or etc : IP address conflict occur, DHCP Failed
		send_DHCPv4_DECLINE();

		ret = dhcpv4_tick_1s;
		while((dhcpv4_tick_1s - ret) < 2) ;   // wait for 1s over; wait to complete to send DECLINE message;

		return 0;
	}
}

void DHCPv4_init(uint8_t s, uint8_t * buf)
{
   uint8_t zeroip[4] = {0,0,0,0};
   getSHAR(DHCPv4_CHADDR);
   if((DHCPv4_CHADDR[0] | DHCPv4_CHADDR[1]  | DHCPv4_CHADDR[2] | DHCPv4_CHADDR[3] | DHCPv4_CHADDR[4] | DHCPv4_CHADDR[5]) == 0x00)
   {
      // assigning temporary mac address, you should be set SHAR before call this function.
      DHCPv4_CHADDR[0] = 0x00;
      DHCPv4_CHADDR[1] = 0x08;
      DHCPv4_CHADDR[2] = 0xdc;
      DHCPv4_CHADDR[3] = 0x00;
      DHCPv4_CHADDR[4] = 0x00;
      DHCPv4_CHADDR[5] = 0x00;
      setSHAR(DHCPv4_CHADDR);
   }

	DHCPV4_SOCKET = s; // SOCK_DHCP
	pDHCPv4MSG = (RIP_MSG*)buf;
	DHCPV4_XID = 0x12345678;

	// WIZchip Netinfo Clear
	setSIPR(zeroip);
	setGAR(zeroip);

	reset_DHCPv4_timeout();
	dhcpv4_state = STATE_DHCPV4_INIT;
}


/* Reset the DHCP timeout count and retry count. */
void reset_DHCPv4_timeout(void)
{
	dhcpv4_tick_1s = 0;
	dhcpv4_tick_next = DHCPV4_WAIT_TIME;
	dhcpv4_retry_count = 0;
}

void DHCPv4_time_handler(void)
{
	dhcpv4_tick_1s++;
}

void getIPfromDHCPv4(uint8_t* ip)
{
	ip[0] = DHCPv4_allocated_ip[0];
	ip[1] = DHCPv4_allocated_ip[1];
	ip[2] = DHCPv4_allocated_ip[2];
	ip[3] = DHCPv4_allocated_ip[3];
}

void getGWfromDHCPv4(uint8_t* ip)
{
	ip[0] =DHCPv4_allocated_gw[0];
	ip[1] =DHCPv4_allocated_gw[1];
	ip[2] =DHCPv4_allocated_gw[2];
	ip[3] =DHCPv4_allocated_gw[3];
}

void getSNfromDHCPv4(uint8_t* ip)
{
   ip[0] = DHCPv4_allocated_sn[0];
   ip[1] = DHCPv4_allocated_sn[1];
   ip[2] = DHCPv4_allocated_sn[2];
   ip[3] = DHCPv4_allocated_sn[3];
}

void getDNSfromDHCPv4(uint8_t* ip)
{
   ip[0] = DHCPv4_allocated_dns[0];
   ip[1] = DHCPv4_allocated_dns[1];
   ip[2] = DHCPv4_allocated_dns[2];
   ip[3] = DHCPv4_allocated_dns[3];
}

uint32_t getDHCPv4Leasetime(void)
{
	return dhcpv4_lease_time;
}

char NibbleToHex(uint8_t nibble)
{
  nibble &= 0x0F;
  if (nibble <= 9)
    return nibble + '0';
  else
    return nibble + ('A'-0x0A);
}
