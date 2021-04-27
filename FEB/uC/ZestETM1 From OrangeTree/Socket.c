//////////////////////////////////////////////////////////////////////
//
// File:      Socket.c
//
// Purpose:
//    ZestETM1 Example Programs
//    Socket layer to interface to GigExpedite device.
//  
// Copyright (c) 2015 Orange Tree Technologies.
// May not be reproduced without permission.
//
//////////////////////////////////////////////////////////////////////

#include <string.h>
#include "socket.h"

//
// External functions
// These need platform specific definitions created for your system
//
void GigExInterruptDisableAll(void);
void GigExInterruptEnableAll(void);
void GigExWriteReg(unsigned long Addr, unsigned char Data);
unsigned char GigExReadReg(unsigned long Addr);
unsigned long GetTime(void);

// Local structure for socket state
#define NUM_SOCKETS 16
static volatile struct {
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
	const char *TxBuffer;
	int TxBufferPtr;
	int TxBufferLen;
} Sockets[NUM_SOCKETS];

// GigExpedite register map
#define CHANNEL_SPACING			32
#define LOCAL_PORT_HIGH         0
#define LOCAL_PORT_LOW          1
#define REMOTE_IP_ADDR_3        2
#define REMOTE_IP_ADDR_2        3
#define REMOTE_IP_ADDR_1        4
#define REMOTE_IP_ADDR_0        5
#define REMOTE_PORT_HIGH        6
#define REMOTE_PORT_LOW         7
#define MTU                     8
#define TTL                     9
#define INTERRUPT_ENABLE        (0x8000+11)
#define INTERRUPT_STATUS        11
#define CONNECTION_STATE        13
#define FRAME_LENGTH            15
#define DATA_FIFO               17

#define LOCAL_IP_ADDR_3         0x200
#define LOCAL_IP_ADDR_2         0x201
#define LOCAL_IP_ADDR_1         0x202
#define LOCAL_IP_ADDR_0         0x203
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


//////////////////////////////////////////////////////////////////////////////
// Local functions

//
// Read data from GigExpedite
//
static void Socket_ReadFrame(SOCKET s)
{
    unsigned char Data;

    do
    {
        if (Sockets[s].RxFrameLen==0)
        {
        	// Read the frame length
			Sockets[s].RxFrameLen = GigExReadReg(s*CHANNEL_SPACING+FRAME_LENGTH);
			Sockets[s].RxFrameLen <<= 8;
			Sockets[s].RxFrameLen |= GigExReadReg(s*CHANNEL_SPACING+FRAME_LENGTH);
			if (Sockets[s].Protocol==IPPROTO_UDP)
			{
				unsigned long RemoteIP;
				unsigned long RemotePort;

				RemoteIP = GigExReadReg(s*CHANNEL_SPACING+DATA_FIFO);
				RemoteIP <<= 8;
				RemoteIP |= GigExReadReg(s*CHANNEL_SPACING+DATA_FIFO);
				RemoteIP <<= 8;
				RemoteIP |= GigExReadReg(s*CHANNEL_SPACING+DATA_FIFO);
				RemoteIP <<= 8;
				RemoteIP |= GigExReadReg(s*CHANNEL_SPACING+DATA_FIFO);
				RemotePort = GigExReadReg(s*CHANNEL_SPACING+DATA_FIFO);
				RemotePort <<= 8;
				RemotePort |= GigExReadReg(s*CHANNEL_SPACING+DATA_FIFO);

				// This information is passed to the recvfrom function
				Sockets[s].RemoteAddr = RemoteIP;
				Sockets[s].RemotePort = RemotePort;
			}
        }

        if (Sockets[s].RxFrameLen!=0)
        {
			// Read data byte
			Data = GigExReadReg(s*CHANNEL_SPACING+DATA_FIFO);
			*(Sockets[s].RxBuffer + Sockets[s].RxBufferPtr) = Data;
			Sockets[s].RxBufferPtr++;
			Sockets[s].RxFrameLen--;
        }
    } while (Sockets[s].RxBufferPtr<Sockets[s].RxBufferLen && Sockets[s].RxFrameLen!=0);

    // For UDP, throw away the rest of the datagram
    if (Sockets[s].Protocol==IPPROTO_UDP)
    {
    	while (Sockets[s].RxFrameLen!=0)
    	{
            Data = GigExReadReg(s*CHANNEL_SPACING+DATA_FIFO);
            Sockets[s].RxFrameLen--;
    	}
    }
}

//
// Send data to GigExpedite
//
static void Socket_SendFrame(SOCKET s)
{
	unsigned long Count;
	unsigned char *Ptr = (unsigned char *)(Sockets[s].TxBuffer) + Sockets[s].TxBufferPtr;
	int Bytes;

	Sockets[s].State |= CONN_PAUSE;
	GigExWriteReg(s*CHANNEL_SPACING+CONNECTION_STATE, Sockets[s].State);

	if (Sockets[s].Protocol==IPPROTO_UDP)
	{
		// Send frame length
		GigExWriteReg(s*CHANNEL_SPACING+FRAME_LENGTH, Sockets[s].TxBufferLen>>8);
		GigExWriteReg(s*CHANNEL_SPACING+FRAME_LENGTH, Sockets[s].TxBufferLen&0xff);
	}

	Bytes = Sockets[s].TxBufferLen>=65536 ? 65536 : Sockets[s].TxBufferLen;
	for (Count=0; Count<Sockets[s].TxBufferLen; Count++)
	{
		GigExWriteReg(s*CHANNEL_SPACING+DATA_FIFO, *Ptr++);
	}
	Sockets[s].TxBufferPtr+=Bytes;
	Sockets[s].TxBufferLen-=Bytes;

	Sockets[s].State &= ~CONN_PAUSE;
	GigExWriteReg(s*CHANNEL_SPACING+CONNECTION_STATE, Sockets[s].State);
}

//
// Establish a UDP connection
//
static int Socket_ConnectUDP(SOCKET s, unsigned long RemoteIP, unsigned long RemotePort, unsigned short LocalPort)
{
    unsigned long State = CONN_ENABLE | ESTABLISHED;
    int Done = 0;
    int Start = GetTime();

	// Program GigExpedite registers
	GigExInterruptDisableAll();
	Sockets[s].IE |= IE_STATE;
	Sockets[s].State = State;
	Sockets[s].UDPEstablished = 0;
	GigExWriteReg(s*CHANNEL_SPACING + REMOTE_IP_ADDR_3, RemoteIP>>24);
	GigExWriteReg(s*CHANNEL_SPACING + REMOTE_IP_ADDR_2, RemoteIP>>16);
	GigExWriteReg(s*CHANNEL_SPACING + REMOTE_IP_ADDR_1, RemoteIP>>8);
	GigExWriteReg(s*CHANNEL_SPACING + REMOTE_IP_ADDR_0, RemoteIP&0xff);
	GigExWriteReg(s*CHANNEL_SPACING + REMOTE_PORT_HIGH, RemotePort>>8);
	GigExWriteReg(s*CHANNEL_SPACING + REMOTE_PORT_LOW, RemotePort&0xff);
	GigExWriteReg(s*CHANNEL_SPACING + LOCAL_PORT_HIGH, LocalPort>>8);
	GigExWriteReg(s*CHANNEL_SPACING + LOCAL_PORT_LOW, LocalPort&0xff);
	GigExWriteReg(s*CHANNEL_SPACING + MTU, 0x80);
	GigExWriteReg(s*CHANNEL_SPACING + TTL, 0x80);
	GigExWriteReg(s*CHANNEL_SPACING + INTERRUPT_ENABLE, Sockets[s].IE);
	GigExWriteReg(s*CHANNEL_SPACING + CONNECTION_STATE, Sockets[s].State);
	GigExInterruptEnableAll();

	// Busy wait loop until connection is made
	// This code be changed for a semaphore from the ISR if available
	while (!Done && (GetTime()-Start)<1000)
	{
		GigExInterruptDisableAll();
		if (Sockets[s].UDPEstablished!=0)
		{
			Sockets[s].UDPEstablished--;
			Done = 1;
		}
		GigExInterruptEnableAll();
	}
	if (!Done)
	{
		GigExInterruptDisableAll();
		Sockets[s].IE &= ~IE_STATE;
		Sockets[s].State = 0;
		Sockets[s].UDPEstablished = 0;
		GigExWriteReg(s*CHANNEL_SPACING + INTERRUPT_ENABLE, Sockets[s].IE);
		GigExWriteReg(s*CHANNEL_SPACING + CONNECTION_STATE, Sockets[s].State);
		GigExInterruptEnableAll();
		return SOCKET_ERROR;
	}

	return 0;
}

//
// Close UDP connection
//
static int Socket_CloseUDP(SOCKET s)
{
    int Start = GetTime();

	GigExInterruptDisableAll();
    Sockets[s].State &= ~STATE_MASK;
    Sockets[s].State |= CLOSED;
	Sockets[s].IE |= IE_STATE;
	GigExWriteReg(s*CHANNEL_SPACING + CONNECTION_STATE, Sockets[s].State);
	GigExWriteReg(s*CHANNEL_SPACING + INTERRUPT_ENABLE, Sockets[s].IE);
	GigExInterruptEnableAll();
	while ((Sockets[s].IE&IE_STATE) &&
           (GetTime()-Start)<1000)
	{
	}
    if (Sockets[s].IE&IE_STATE)
    {
        // Timeout
    	GigExInterruptDisableAll();
        Sockets[s].IE &= ~IE_STATE;
        GigExWriteReg(s*CHANNEL_SPACING + INTERRUPT_ENABLE, Sockets[s].IE);
        GigExInterruptEnableAll();
        return SOCKET_ERROR;
    }
    return 0;
}

//
// Receive data on a socket
//
static void Socket_RecvData(SOCKET s)
{
    int Start = GetTime();

    // Set up receive
	GigExInterruptDisableAll();
	Sockets[s].IE |= IE_INCOMING;
	GigExWriteReg(s*CHANNEL_SPACING + INTERRUPT_ENABLE, Sockets[s].IE);
	GigExInterruptEnableAll();

    // Wait for completion
    // Could use semaphore from ISR if available to avoid busy wait
	while ((Sockets[s].State&STATE_MASK)==ESTABLISHED &&
           (Sockets[s].IE&IE_INCOMING) &&
           (GetTime()-Start)<1000)
	{
	}
    if ((Sockets[s].State&STATE_MASK)==ESTABLISHED &&
        (Sockets[s].IE&IE_INCOMING))
    {
        // Timeout
    	GigExInterruptDisableAll();
        Sockets[s].IE &= ~IE_INCOMING;
        GigExWriteReg(s*CHANNEL_SPACING + INTERRUPT_ENABLE, Sockets[s].IE);
        GigExInterruptEnableAll();
    }
}

//
// Send data on a socket
//
static void Socket_SendData(SOCKET s)
{
    int Start = GetTime();

    // Set up transmit
    GigExInterruptDisableAll();
	Sockets[s].IE |= IE_OUTGOING_NOT_FULL;
	GigExWriteReg(s*CHANNEL_SPACING + INTERRUPT_ENABLE, Sockets[s].IE);
    GigExInterruptEnableAll();

    // Wait for completion
    // Could use semaphore from ISR if available to avoid busy wait
	while (((Sockets[s].IE&IE_OUTGOING_NOT_FULL) ||
		    (Sockets[s].IE&IE_OUTGOING_EMPTY)) &&
           (GetTime()-Start)<100)
	{
	}
    if ((Sockets[s].IE&IE_OUTGOING_NOT_FULL) ||
        (Sockets[s].IE&IE_OUTGOING_EMPTY))
    {
    	GigExInterruptDisableAll();
        Sockets[s].TxBufferLen = 0;
        Sockets[s].IE &= ~(IE_OUTGOING_NOT_FULL|IE_OUTGOING_EMPTY);
        GigExWriteReg(s*CHANNEL_SPACING + INTERRUPT_ENABLE, Sockets[s].IE);
    	GigExInterruptEnableAll();
    }
}

//////////////////////////////////////////////////////////////////////////////
// External API

//
// Initialise library
//
void SocketInit(void)
{
	int s;
	int Done;
	
	// Clean GigExpedite registers
 	for (s=0; s<NUM_SOCKETS; s++)
	{
		GigExWriteReg(s*CHANNEL_SPACING+INTERRUPT_ENABLE, 0);
		GigExWriteReg(s*CHANNEL_SPACING+CONNECTION_STATE, 0);
	}
	do
	{
		Done = 1;
		for (s=0; s<NUM_SOCKETS; s++)
		{
			if (GigExReadReg(s*CHANNEL_SPACING+CONNECTION_STATE)!=0)
				Done = 0;
		}
	} while (Done==0);

	// Clear leftover interrupts
	for (s=0; s<NUM_SOCKETS; s++)
	{
		GigExReadReg(s*CHANNEL_SPACING+INTERRUPT_ENABLE);
	}

	// Initialise connection state
	memset((void *)Sockets, 0, sizeof(Sockets));
}

//
// Allocate new socket
// af and type are ignored
//
SOCKET socket(int af, int type, int protocol)
{
	int s;
    static int SockStart = 0; // Used to rotate socket use
	
    GigExInterruptDisableAll();
    
	// Look for free socket
    s = SockStart;
    do
    {
		if (Sockets[s].Used==0) break;
        s = (s==NUM_SOCKETS-1) ? 0 : (s+1);
	} while (s!=SockStart);
    SockStart = (SockStart==NUM_SOCKETS-1) ? 0 : (SockStart+1);
    if (Sockets[s].Used!=0)
    {
        GigExInterruptEnableAll();
        return INVALID_SOCKET;
    }
    
	// Initialise connection state
	Sockets[s].Used = 1;
	Sockets[s].Accept = 0;
	Sockets[s].Connect = 0;
	Sockets[s].UDPEstablished = 0;
	Sockets[s].Closing = 0;
	Sockets[s].IE = 0;
	Sockets[s].State = 0;
	Sockets[s].Protocol = protocol;
    Sockets[s].RxBufferLen = 0;
    Sockets[s].RxBufferPtr = 0;
    Sockets[s].RxFrameLen = 0;
    Sockets[s].RemotePort = 0;
    Sockets[s].RemoteAddr = 0;
    Sockets[s].TxBufferLen = 0;
    GigExWriteReg(s*CHANNEL_SPACING + INTERRUPT_ENABLE, 0);
    GigExWriteReg(s*CHANNEL_SPACING + CONNECTION_STATE, 0);

    GigExInterruptEnableAll();

	return s;
}

//
// Bind socket to an address
//
int bind(SOCKET s, const struct sockaddr *name, int namelen)
{
	memcpy((void *)&Sockets[s].LocalAddr, name, namelen);
	
	return 0;
}

//
// Listen for incoming connections
// backlog is ignored
//
int listen(SOCKET s, int backlog)
{
	struct sockaddr_in *Addr = (struct sockaddr_in *)&Sockets[s].LocalAddr;
	unsigned long State = CONN_ENABLE | CONN_TCP | LISTEN;

	if (Sockets[s].Protocol!=IPPROTO_TCP)
		return -1;
	
	// Set up listening connection
	GigExInterruptDisableAll();
    Sockets[s].RemoteAddr = 0;
    Sockets[s].RemotePort = 0;
	Sockets[s].IE |= IE_STATE;
	Sockets[s].State = State;
	Sockets[s].Accept = 0;
	GigExWriteReg(s*CHANNEL_SPACING + LOCAL_PORT_HIGH, Addr->sin_port>>8);
	GigExWriteReg(s*CHANNEL_SPACING + LOCAL_PORT_LOW, Addr->sin_port&0xff);
	GigExWriteReg(s*CHANNEL_SPACING + MTU, 0x80);
	GigExWriteReg(s*CHANNEL_SPACING + TTL, 0x80);
	GigExWriteReg(s*CHANNEL_SPACING + INTERRUPT_ENABLE, Sockets[s].IE);
	GigExWriteReg(s*CHANNEL_SPACING + CONNECTION_STATE, Sockets[s].State);
	GigExInterruptEnableAll();
        
    return 0;
}

//
// Wait for connection from remote device
// addrlen is ignored
//
SOCKET accept(SOCKET s, struct sockaddr *addr, int *addrlen)
{
	int Done = 0;

	if (Sockets[s].Protocol!=IPPROTO_TCP)
		return -1;

	// Busy wait loop until connection is made
	// This could be changed for a semaphore from the ISR if available
	while (!Done)
	{
		GigExInterruptDisableAll();
		if (Sockets[s].Accept!=0)
		{
			Sockets[s].Accept--;
			Done = 1;
			if (addr!=NULL)
			{
				struct sockaddr_in *Addr = (struct sockaddr_in *)addr;
				Addr->sin_port = Sockets[s].RemotePort;
				Addr->sin_addr.s_addr = Sockets[s].RemoteAddr;
				*addrlen = sizeof(struct sockaddr_in);
			}
		}
		GigExInterruptEnableAll();
	}
	
	return s;
}

//
// Connect to remote device
//
int connect(SOCKET s, const struct sockaddr *addr, int addrlen)
{
    struct sockaddr_in *Target = (struct sockaddr_in *)addr;
    struct sockaddr_in *Addr = (struct sockaddr_in *)&Sockets[s].LocalAddr;
    unsigned long State = CONN_ENABLE | CONN_TCP | CONNECT;
    int Done = 0;
    int Start = GetTime();

	if (Sockets[s].Protocol!=IPPROTO_TCP)
		return -1;

    // Set up connection
    GigExInterruptDisableAll();
    Sockets[s].IE |= IE_STATE;
    Sockets[s].State = State;
    Sockets[s].Connect = 0;
    GigExWriteReg(s*CHANNEL_SPACING + REMOTE_IP_ADDR_3, Target->sin_addr.s_addr>>24);
    GigExWriteReg(s*CHANNEL_SPACING + REMOTE_IP_ADDR_2, Target->sin_addr.s_addr>>16);
    GigExWriteReg(s*CHANNEL_SPACING + REMOTE_IP_ADDR_1, Target->sin_addr.s_addr>>8);
    GigExWriteReg(s*CHANNEL_SPACING + REMOTE_IP_ADDR_0, Target->sin_addr.s_addr&0xff);
    GigExWriteReg(s*CHANNEL_SPACING + REMOTE_PORT_HIGH, Target->sin_port>>8);
    GigExWriteReg(s*CHANNEL_SPACING + REMOTE_PORT_LOW, Target->sin_port&0xff);
    GigExWriteReg(s*CHANNEL_SPACING + LOCAL_PORT_HIGH, Addr->sin_port>>8);
    GigExWriteReg(s*CHANNEL_SPACING + LOCAL_PORT_LOW, Addr->sin_port&0xff);
    GigExWriteReg(s*CHANNEL_SPACING + MTU, 0x80);
    GigExWriteReg(s*CHANNEL_SPACING + TTL, 0x80);
    GigExWriteReg(s*CHANNEL_SPACING + INTERRUPT_ENABLE, Sockets[s].IE);
    GigExWriteReg(s*CHANNEL_SPACING + CONNECTION_STATE, Sockets[s].State);
    GigExInterruptEnableAll();

    // Busy wait loop until connection is made
    // This code be changed for a semaphore from the ISR if available
    while (!Done && (GetTime()-Start)<1000)
    {
        GigExInterruptDisableAll();
        if (Sockets[s].Connect!=0)
        {
            Sockets[s].Connect--;
            Done = 1;
        }
        GigExInterruptEnableAll();
    }
    if (!Done)
    {
        GigExInterruptDisableAll();
        Sockets[s].IE &= ~IE_STATE;
        Sockets[s].State = 0;
        Sockets[s].Connect = 0;
        GigExWriteReg(s*CHANNEL_SPACING + INTERRUPT_ENABLE, Sockets[s].IE);
        GigExWriteReg(s*CHANNEL_SPACING + CONNECTION_STATE, Sockets[s].State);
        GigExInterruptEnableAll();
        return SOCKET_ERROR;
    }
    
    return 0;
}

//
// Send data to remote device
// flags is ignored
// NB: Max send length is 64kbytes
//
int send(SOCKET s, const char *buf, int len, int flags)
{
    if (Sockets[s].Protocol!=IPPROTO_TCP) return -1;
    
	// Initiate transfer
	Sockets[s].TxBuffer = buf;
	Sockets[s].TxBufferPtr = 0;
	Sockets[s].TxBufferLen = len;
	Socket_SendData(s);
		   
	return Sockets[s].TxBufferPtr;
}

//
// Receive data from remote device
// flags is ignored
//
int recv(SOCKET s, char *buf, int len, int flags)
{
    if (len==0) return 0;
    if (Sockets[s].Protocol!=IPPROTO_TCP) return -1;
    
    Sockets[s].RxBuffer = buf;
    Sockets[s].RxBufferLen = len;
    Sockets[s].RxBufferPtr = 0;
    
    // Check for data available from previous frame
    if (Sockets[s].Protocol==IPPROTO_TCP)
    {
		GigExInterruptDisableAll();
        if (Sockets[s].RxFrameLen!=0)
        	Socket_ReadFrame(s);
		GigExInterruptEnableAll();
		if (Sockets[s].RxBufferPtr>=Sockets[s].RxBufferLen)
		{
			// Complete
			return Sockets[s].RxBufferPtr;
		}
    }

	// Initiate receive
    Socket_RecvData(s);
    
	return Sockets[s].RxBufferPtr;
}

//
// Send data to UDP connection
//
int sendto(SOCKET s, const char *buf, int len, int flags,
           const struct sockaddr *toaddr, int tolen)
{
    struct sockaddr_in *Target = (struct sockaddr_in *)toaddr;
    struct sockaddr_in *Addr = (struct sockaddr_in *)&Sockets[s].LocalAddr;
    int Res;

    if (Sockets[s].Protocol!=IPPROTO_UDP) return -1;

    // Set up connection
    Res = Socket_ConnectUDP(s, Target->sin_addr.s_addr, Target->sin_port, Addr->sin_port);
    if (Res==SOCKET_ERROR)
    	return Res;

	// Initiate transfer
	Sockets[s].TxBuffer = buf;
	Sockets[s].TxBufferPtr = 0;
	Sockets[s].TxBufferLen = len > 65536 ? 65536 : len;
	Socket_SendData(s);

	// Close down connection
	Res = Socket_CloseUDP(s);
	if (Res==SOCKET_ERROR)
		return Res;

	return len > 65536 ? 65536 : len;
}
           
//
// Receive from UDP connection
//
int recvfrom(SOCKET s, char *buf, int len, int flags,
             const struct sockaddr *fromaddr, int *fromlen)
{
    struct sockaddr_in *Addr = (struct sockaddr_in *)&Sockets[s].LocalAddr;
    int Res;

    if (Sockets[s].Protocol!=IPPROTO_UDP) return -1;
    if (len==0) return 0;

    // Set up connection
    Res = Socket_ConnectUDP(s, 0, 0, Addr->sin_port);
    if (Res==SOCKET_ERROR)
    	return Res;

	// Initiate receive
    Sockets[s].RxBuffer = buf;
    Sockets[s].RxBufferLen = len;
    Sockets[s].RxBufferPtr = 0;
	Socket_RecvData(s);

	// Fill in 'from' address
	if (fromaddr!=NULL)
	{
		struct sockaddr_in *Addr = (struct sockaddr_in *)fromaddr;
		Addr->sin_port = Sockets[s].RemotePort;
		Addr->sin_addr.s_addr = Sockets[s].RemoteAddr;
		*fromlen = sizeof(struct sockaddr_in);
	}

	// Close down connection
	Res = Socket_CloseUDP(s);
	if (Res==SOCKET_ERROR)
		return Res;

	return Sockets[s].RxBufferPtr;
}

//
// Close and free socket
//
int socket_close(SOCKET s)
{
	GigExInterruptDisableAll();
    Sockets[s].State &= ~STATE_MASK;
    Sockets[s].State |= CLOSED;
	Sockets[s].IE |= IE_STATE;
	Sockets[s].Closing = 1;
	GigExWriteReg(s*CHANNEL_SPACING + CONNECTION_STATE, Sockets[s].State);
	GigExWriteReg(s*CHANNEL_SPACING + INTERRUPT_ENABLE, Sockets[s].IE);
	GigExInterruptEnableAll();
        
    // The socket may still be open here - the ISR will
    // clean up when GigExpedite acknowledges close
    
    return 0;
}

//
// Pause/unpause transmission on a socket
//
void SocketPause(SOCKET s, int p)
{
	GigExInterruptDisableAll();
	if (p)
		Sockets[s].State |= CONN_PAUSE;
	else
		Sockets[s].State &= ~CONN_PAUSE;
	GigExWriteReg(s*CHANNEL_SPACING + CONNECTION_STATE, Sockets[s].State);
	GigExInterruptEnableAll();
}

//
// Get local IP address from GigEx
//
unsigned long SocketGetLocalIPAddr(void)
{
    unsigned long Res = 0;
    
    Res = GigExReadReg(LOCAL_IP_ADDR_3);
    Res<<=8;
    Res |= GigExReadReg(LOCAL_IP_ADDR_2);
    Res<<=8;
    Res |= GigExReadReg(LOCAL_IP_ADDR_1);
    Res<<=8;
    Res |= GigExReadReg(LOCAL_IP_ADDR_0);
    
    return Res;
}

//
// GigExpedite interrupt service routine
//
void SocketISR(void)
{
	unsigned long State;
	unsigned long IntStatus;
	int s;

    for (s=0; s<NUM_SOCKETS; s++)
    {
        if (Sockets[s].IE==0)
            continue;

    	// Read interrupt status bits
    	// This will clear any state change interrupts
		IntStatus = GigExReadReg(s*CHANNEL_SPACING+INTERRUPT_STATUS);

        do
        {
	        // Read connection state
	        State = GigExReadReg(s*CHANNEL_SPACING+CONNECTION_STATE);

		    // Handle changes in connection state
    	    if ((Sockets[s].State&STATE_MASK)==LISTEN &&
	    	    (State&STATE_MASK)==ESTABLISHED)
    	    {
    		    // TCP connection established
    		    // Read connected address and acknowledge state change
    		    unsigned long Val;

    		    Sockets[s].Accept++;
    		    Val = GigExReadReg(s*CHANNEL_SPACING+REMOTE_PORT_HIGH);
    		    Val <<= 8;
    		    Val |= GigExReadReg(s*CHANNEL_SPACING+REMOTE_PORT_LOW);
    		    Sockets[s].RemotePort = Val;

    		    Val = GigExReadReg(REMOTE_IP_ADDR_3);
    		    Val <<= 8;
    		    Val |= GigExReadReg(REMOTE_IP_ADDR_2);
    		    Val <<= 8;
    		    Val |= GigExReadReg(REMOTE_IP_ADDR_1);
    		    Val <<= 8;
    		    Val |= GigExReadReg(REMOTE_IP_ADDR_0);
    		    Sockets[s].RemoteAddr = Val;

                Sockets[s].IE &= ~IE_STATE;
                GigExWriteReg(s*CHANNEL_SPACING+INTERRUPT_ENABLE, Sockets[s].IE);
                GigExWriteReg(s*CHANNEL_SPACING+CONNECTION_STATE, State);
			    Sockets[s].State = State;
    	    }
    	    if ((Sockets[s].State&STATE_MASK)==CONNECT &&
	    	    (State&STATE_MASK)==ESTABLISHED)
    	    {
    		    // TCP connection established
    		    // Acknowledge state change
    		    Sockets[s].Connect++;
                Sockets[s].IE &= ~IE_STATE;
                GigExWriteReg(s*CHANNEL_SPACING+INTERRUPT_ENABLE, Sockets[s].IE);
                GigExWriteReg(s*CHANNEL_SPACING+CONNECTION_STATE, State);
			    Sockets[s].State = State;
    	    }
    	    if ((Sockets[s].State&STATE_MASK)==ESTABLISHED &&
	    	    (State&STATE_MASK)==ESTABLISHED)
    	    {
    		    // UDP connection established
    		    // Acknowledge state change
    		    Sockets[s].UDPEstablished++;
                Sockets[s].IE &= ~IE_STATE;
                GigExWriteReg(s*CHANNEL_SPACING+INTERRUPT_ENABLE, Sockets[s].IE);
			    Sockets[s].State = State;
    	    }
    	    if (Sockets[s].Used==1 &&
    		    (Sockets[s].State&STATE_MASK)==CLOSED &&
	    	    (State&STATE_MASK)==CLOSED)
    	    {
    		    // Connection closed - free socket and acknowledge state change
			    Sockets[s].IE = 0;
                Sockets[s].State = 0;
                GigExWriteReg(s*CHANNEL_SPACING+INTERRUPT_ENABLE, Sockets[s].IE);
                GigExWriteReg(s*CHANNEL_SPACING+CONNECTION_STATE, Sockets[s].State);
                if (Sockets[s].Closing)
                	Sockets[s].Used = 0;
	        }
	        IntStatus &= ~IE_STATE;

		    // Service incoming data
		    if ((Sockets[s].IE&IE_INCOMING) &&
			    (IntStatus&IE_INCOMING))
		    {
			    // Data available - read a frame from GigExpedite
                Socket_ReadFrame(s);
                if (Sockets[s].Protocol==IPPROTO_UDP || Sockets[s].RxBufferPtr==Sockets[s].RxBufferLen)
                {
                	// Receive is complete
					Sockets[s].IE &= ~IE_INCOMING;
					GigExWriteReg(s*CHANNEL_SPACING+INTERRUPT_ENABLE, Sockets[s].IE);
                }
            }
		} while (IntStatus&IE_STATE);

		// Service outgoing data
		if ((Sockets[s].IE&IE_OUTGOING_EMPTY) &&
			(IntStatus&IE_OUTGOING_EMPTY))
		{
			// Outgoing connection empty - all data sent
			Sockets[s].IE &= ~IE_OUTGOING_EMPTY;
			GigExWriteReg(s*CHANNEL_SPACING+INTERRUPT_ENABLE, Sockets[s].IE);
		}

        if ((Sockets[s].IE&IE_OUTGOING_NOT_FULL) &&
			(IntStatus&IE_OUTGOING_NOT_FULL))
		{
			// Space available - send data (up to 64k at a time)
        	Socket_SendFrame(s);
        	if (Sockets[s].TxBufferLen==0)
        	{
        		Sockets[s].IE &= ~IE_OUTGOING_NOT_FULL;
        		Sockets[s].IE |= IE_OUTGOING_EMPTY;
        		GigExWriteReg(s*CHANNEL_SPACING+INTERRUPT_ENABLE, Sockets[s].IE);
        	}
		}
    }
}

