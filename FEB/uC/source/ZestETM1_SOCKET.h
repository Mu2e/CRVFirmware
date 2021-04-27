//////////////////////////////////////////////////////////////////////
//
// File:  ZestETMI_SOCKET.h (was named Socket.h )
// Fermilab Terry Kiper 2016-2021
//
// Purpose:
//    ZestETM1 Example Programs
//    Socket layer to interface to GigExpedite device.
//
// Copyright (c) 2015 Orange Tree Technologies.
// May not be reproduced without permission.
//
//////////////////////////////////////////////////////////////////////

#ifndef SOCKET_H_
#define SOCKET_H_

// Local structure for socket state
#define NUM_SOCKETS 4           //tek was 16


// GigExpedite register map
#define CHANNEL_SPACING			32
#define LOCAL_PORT_HIGH         0
#define REMOTE_IP_ADDR_3        2       //16BIT MODE HIGH REM IP
#define REMOTE_IP_ADDR_1        4       //16BIT MODE LOW  REM IP
#define REMOTE_PORT_HIGH        6       //16BIT MODE PORT NUMBER
#define MTU                     8       //16BIT value MTU and TTL
#define TTL                     8       //16BIT value MTU and TTL

#define INTERRUPT_ENA           10     //TEK Changed  INTERRUPT_ENABLE(used by 'IAR Compiler') to INTERRUPT_ENA
#define INTERRUPT_STATUS        10
#define CONNECTION_STATE        12
#define FRAME_LENGTH            14
#define DATA_FIFO               16

#define LOCAL_IP_ADDR_3         0x200   //16BIT MODE HIGH IP
#define LOCAL_IP_ADDR_1         0x202   //16BIT MODE LOW  IP
#define LINK_STATUS             0x21f

// GigExpedite interrupt bits
#define IE_INCOMING          	0x0001
#define IE_OUTGOING_EMPTY    	0x0002
#define IE_OUTGOING_NOT_FULL 	0x0004
#define IE_STATE             	0x0008 

// Connection states (for GigExpedite CONNECTION_STATE register)
#define CLOSED       			0x0000
#define LISTEN       			0x0001
#define CONNECT      			0x0002
#define ESTABLISHED  			0x0003
#define STATE_MASK   			0x000f
#define CONN_TCP     			0x0010
#define CONN_ENABLE  			0x0020
#define CONN_PAUSE  			0x0040


//Zest Group 0-15 Registers; Network connections management (16 Bit Addr Map)
#define LocPortReg      (0x000)   //local port reg
#define zLocPortHI      (0x002)   //local port reg high16 bits
#define zLocPortLO      (0x004)   //local port reg low16 bits
#define zRemPortLO      (0x006)   //remote port
#define zPayLdSize      (0x008)   //payload size(udp) and time to live
#define zIntrEna        (0x00A)   //interrupt enable control and status
#define zConnectCSR     (0x00C)   //connection control and status
#define zFrameLen       (0x00E)   //network frame length(read), or write UDP  datagr len
#define zNetDataFIFO    (0x010)   //network fifo data read/write

//Zest Group 16 Registers: Global network settings (16 Bit Addr Map)
#define zLocIP_HI       (0x200)   //local ip high16 r/w
#define zLocIP_LO       (0x202)   //local ip low16 r/w
#define zSubNetHI       (0x204)   //local subnet mask upper16 r/w
#define zSubNetLO       (0x208)   //local subnet mask lower16 r/w
#define zDCHP_CSR       (0x20A)   //DHCP CSR
#define zGatewayHI      (0x20C)   //gateway upper16
#define zGatewayLO      (0x20E)   //gateway lower16
#define zJumboLen       (0x210)   //Jumbo fram max len bytes
#define zGigExLocPrt    (0x212)   //GigExpedite network Control local port
#define zGigExWebPrt    (0x214)   //GigExpedite network Web server local port
#define zUpDateNetCSR   (0x216)   //update network settings 
#define zLocStatusPHY   (0x21E)   //local status PHY device

//Zest Group 17 Registers: PTP/RIEEE1588 control and status registers (16 Bit Addr Map)
#define zPTP_TimeLat    (0x220)   //PTP time latch
#define zPTP_Event      (0x222)   //PTP event time
#define zPTP_PulWid     (0x224)   //PTP trigger/1pps pulse width, 100ns/cnt
#define zPTP_TrigCSR    (0x22A)   //PTP trigger/event CSR
#define zPTP_TimeB63    (0x22C)   //PTP time latch bits 63:48
#define zPTP_TimeB47    (0x22E)   //PTP time latch bits 47:32
#define zPTP_TimeB31    (0x230)   //PTP time latch bits 31:16
#define zPTP_TimeB15    (0x232)   //PTP time latch bits 15:00



// Type declarations
typedef int SOCKET;
#pragma pack(4) 
struct in_addr {
    unsigned long s_addr;
};
#pragma pack(4) 
struct sockaddr {
    unsigned short sa_family;
    char sa_data[14];
}; 
#pragma pack(4) 
struct sockaddr_in {
    short sin_family;
    unsigned short sin_port;
    struct in_addr sin_addr;
    char sin_zero[8];
};

#pragma pack(4) 
struct zSockets {
	char Used;
	int Protocol;
	struct sockaddr LocalAddr;
	unsigned short RemotePort;
	unsigned long RemoteAddr;
	int State;
	int Accept;
	int Connect;
	int UDPEstablished;
	int Closing;
	int IE;
	char *RxBuffer;
	int RxBufferLen;
	int RxBufferPtr;
    int RxFrameLen;
    int RxFrameLenLast;         //tek added
	const char *TxBuffer;
	int TxBufferPtr;
	int TxBufferLen;
};


#define LISTENQ 0

#define AF_INET 0

#define SOCK_STREAM 0
#define SOCK_DGRAM 1

#define IPPROTO_UDP 0
#define IPPROTO_TCP 1

#define INVALID_SOCKET -1
#define SOCKET_ERROR 1

#define htonl(x) (x)
#define htons(x) (x)

#define INADDR_ANY 0

// API
void    SocketInit(void);
SOCKET  socket(int af, int type, int protocol);
int     bind(SOCKET s, const struct sockaddr *name, int namelen);
int     listen(SOCKET s, int backlog);
SOCKET  accept(SOCKET s, struct sockaddr *addr, int *addrlen);
int     connect(SOCKET s, const struct sockaddr *addr, int addrlen); 

int     send(SOCKET s, const char *buf, int len, int flags);
int     recv(SOCKET s, char *buf, int len, int flags);

int     sendto(SOCKET s, const char *buf, int len, int flags, const struct sockaddr *toaddr, int tolen);
int     recvfrom(SOCKET s, char *buf, int len, int flags, const struct sockaddr *fromaddr, int *fromlen);

int     socket_close(SOCKET s);
void    SocketPause(SOCKET s, int p);
unsigned long SocketGetLocalIPAddr(void);
void    SocketISR(void);

#endif /*SOCKET_H_*/
