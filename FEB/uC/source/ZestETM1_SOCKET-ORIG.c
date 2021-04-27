//********************************************************
// @file ZestETM1.c 
//  Orange Tree GigaBit Ethernet Interface Module 'ZestETM1'
//  RM48 Micro Controller
//  RM48 Additions

#include "ver_io.h"
#include "sys_common.h"
#include <stdio.h>

extern  volatile struct msTimers    mStime; 
extern  uint32  g_timeMs;
//********************************************************

#define SERVER5000   5000


//////////////////////////////////////////////////////////////////////
//
// File:      Socket.c 'renamed to ZestETMI_Socket.h"'
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
#include "ZestETMI_Socket.h"

//
// External functions
// These need platform specific definitions created for your system
//
unsigned long GetTime(void);
void GigExInterruptDisableAll(void);
void GigExInterruptEnableAll(void);

void GigExWriteReg(unsigned long Addr, unsigned char Data);
unsigned char GigExReadReg(unsigned long Addr);


// Local structure for socket state
#define NUM_SOCKETS 1  //tek was 16

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
    int RxFrameLenLast;         //tek added
	const char *TxBuffer;
	int TxBufferPtr;
	int TxBufferLen;
} Sockets[NUM_SOCKETS];

// GigExpedite register map
#define CHANNEL_SPACING			32
#define LOCAL_PORT_HIGH         0
//#define LOCAL_PORT_LOW          1
#define REMOTE_IP_ADDR_3        2       //16BIT MODE HIGH REM IP
//#define REMOTE_IP_ADDR_2        3
#define REMOTE_IP_ADDR_1        4       //16BIT MODE LOW  REM IP
//#define REMOTE_IP_ADDR_0        5
#define REMOTE_PORT_HIGH        6       //16BIT MODE PORT NUMBER
//#define REMOTE_PORT_LOW         7
#define MTU                     8
#define TTL                     8//9
//#define INTERRUPT_ENA        (0x8000+11)  //(0x200+10) //
#define INTERRUPT_ENA           10//11          //TEK Changed  INTERRUPT_ENABLE(used by 'IAR Compiler') to INTERRUPT_ENA
#define INTERRUPT_STATUS        10//11
#define CONNECTION_STATE        12//13
#define FRAME_LENGTH            14//15
#define DATA_FIFO               16//17

#define LOCAL_IP_ADDR_3         0x200   //16BIT MODE HIGH IP
//#define LOCAL_IP_ADDR_2         0x201
#define LOCAL_IP_ADDR_1         0x202   //16BIT MODE LOW  IP
//#define LOCAL_IP_ADDR_0         0x203
#define LINK_STATUS             0x21f

// GigExpedite interrupt bits
#define IE_INCOMING          	0x0001
#define IE_OUTGOING_EMPTY    	0x0002
#define IE_OUTGOING_NOT_FULL 	0x0004
#define IE_STATE             	0x0008 //0x0008

// Connection states (for GigExpedite CONNECTION_STATE register)
#define CLOSED       			0x0000
#define LISTEN       			0x0001
#define CONNECT      			0x0002
#define ESTABLISHED  			0x0003
#define STATE_MASK   			0x000f
#define CONN_TCP     			0x0010
#define CONN_ENABLE  			0x0020
#define CONN_PAUSE  			0x0040


uint16_t GigExReadReg16(unsigned long Addr);
void GigExWriteReg16(unsigned long Addr, uint16_t Data);

//my register mapping for 16bit TCP mode
#define ZestETM1        0x68000000                  //ETHERNET base address (uC hardware, chip sel 4)

//Zest Group 0-15 Registers: Network connections management
#define LocPortReg      (0x000)   //local port reg
#define zLocPortHI      (0x002)   //local port reg high16 bits
#define zLocPortLO      (0x004)   //local port reg low16 bits
#define zRemPortLO      (0x006)   //remote port
#define zPayLdSize      (0x008)   //payload size(udp) and time to live
#define zIntrEna        (0x00A)   //interrupt enable control and status
#define zConnectCSR     (0x00C)   //connection control and status
#define zFrameLen       (0x00E)   //network frame length(read), or write UDP  datagr len
#define zNetDataFIFO    (0x010)   //network fifo data read/write

//Zest Group 16 Registers: Global network settings
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

//Zest Group 17 Registers: PTP/RIEEE1588 control and status registers
#define zPTP_TimeLat    (0x220)   //PTP time latch
#define zPTP_Event      (0x222)   //PTP event time
#define zPTP_PulWid     (0x224)   //PTP trigger/1pps pulse width, 100ns/cnt
#define zPTP_TrigCSR    (0x22A)   //PTP trigger/event CSR
#define zPTP_TimeB63    (0x22C)   //PTP time latch bits 63:48
#define zPTP_TimeB47    (0x22E)   //PTP time latch bits 47:32
#define zPTP_TimeB31    (0x230)   //PTP time latch bits 31:16
#define zPTP_TimeB15    (0x232)   //PTP time latch bits 15:00


//Zest Group 23 Registers: General purpose user comms control and status registers
#define zFLASH_CSR      *(sPTR)(ZestETM1+(0x2E0))   //flash control for misc register
#define zMAIL_ISR       *(sPTR)(ZestETM1+(0x2EA))   //mail box intr and status

//Zest Group 24-31 Registers: General purpose user comms
#define zGenUserRegs    *(sPTR)(ZestETM1+(0x300))   //general purpose user registers



//////////////////////////////////////////////////////////////////////////////
// Local functions

//
// Read data from GigExpedite
//
void Socket_ReadFrame(SOCKET s)
{
    uint16_t Data, *dPtr;
    dPtr= (uint16_t*)(Sockets[s].RxBuffer + Sockets[s].RxBufferPtr);

    do
        {
        if (Sockets[s].RxFrameLen==0)
            {
        	// Read the frame length
			Sockets[s].RxFrameLen = GigExReadReg16(s*CHANNEL_SPACING+FRAME_LENGTH);  //16bit mode
            
            //RxFrameLenLast    
            Sockets[s].RxFrameLenLast= Sockets[s].RxFrameLen;  //TEK ADDED

        //tek length is in bytes, from here we will readout as words
            //convert byte to word count
            //Sockets[s].RxFrameLen = ((Sockets[s].RxFrameLen +1)/2);
            
			if (Sockets[s].Protocol==IPPROTO_UDP)
                {
				unsigned long RemoteIP;
				unsigned long RemotePort;
				RemoteIP = GigExReadReg16(s*CHANNEL_SPACING+DATA_FIFO);
				RemoteIP <<= 16;
				RemoteIP |= GigExReadReg16(s*CHANNEL_SPACING+DATA_FIFO);

				RemotePort = GigExReadReg16(s*CHANNEL_SPACING+DATA_FIFO);

				// This information is passed to the recvfrom function
				Sockets[s].RemoteAddr = RemoteIP;
				Sockets[s].RemotePort = RemotePort;
                }
            }

        if (Sockets[s].RxFrameLen!=0)
            {
			// Read data byte 
			Data = GigExReadReg16(s*CHANNEL_SPACING+DATA_FIFO);
            Data= SwapWord(Data); //tek added
            *dPtr++= Data;

			//*(Sockets[s].RxBuffer + Sockets[s].RxBufferPtr) = Data;
            
  //tek  RxBufferPtr++ and RxFrameLen-- may needed doubling now in 16bit mode
			Sockets[s].RxBufferPtr++;   
			Sockets[s].RxBufferPtr++;   
			Sockets[s].RxFrameLen--;
            if (Sockets[s].RxFrameLen)
                Sockets[s].RxFrameLen--;
            }
    } while (Sockets[s].RxBufferPtr<Sockets[s].RxBufferLen && Sockets[s].RxFrameLen!=0);

    // For UDP, throw away the rest of the datagram
    if (Sockets[s].Protocol==IPPROTO_UDP)
        {
    	while (Sockets[s].RxFrameLen!=0)
            {
            Data = GigExReadReg16(s*CHANNEL_SPACING+DATA_FIFO);
            Sockets[s].RxFrameLen--;
            }
        }
}



#include "sys_dma.h"







//-------------------- DMA CODE ------------------------------

//DMA TESTING ONLY
#define D_SIZE      4*256
extern uint16 TX_dmaDATA[D_SIZE];          // transmit buffer in sys ram 
extern uint16 RX_dmaDATA[D_SIZE*2];        // receive  buffer in sys ram 
g_dmaCTRL g_dmaCTRLPKTz;                     // dma control packet configuration stack
extern  struct      blocksnd    BinSt;



/** void dmaConfigCtrlPacket(uint32 sadd,uint32 dadd,uint32 dsize)
*
*   configuring dma control packet stack
*
*       sadd  > source address
*       dadd  > destination  address
*       dsize > data size
*
*   @ note : after configuring the stack the control packet needs to be set by calling dmaSetCtrlPacket()
*
*  note  The frame/element count fields of the the  ITCOUNT register is only 13 bit wide, 
*  note  hence a max of 8191 frames/elemets. bits 15:13 are ignored - so 8192 is same as 0.
*/
void dmaConfigCtrlPacketZest(uint32 sadd,uint32 dadd,uint32 dsize)
{
  g_dmaCTRLPKTz.SADD      = sadd;			  /* source address             */
  g_dmaCTRLPKTz.DADD      = dadd;			  /* destination  address       */
  g_dmaCTRLPKTz.CHCTRL    = 0;                 /* channel control            */
  g_dmaCTRLPKTz.FRCNT	 = 1;                 /* frame count                */
  g_dmaCTRLPKTz.ELCNT     = dsize;             /* Element / Frame            */
  g_dmaCTRLPKTz.ELDOFFSET = 0;                 /* element destination offset */
  g_dmaCTRLPKTz.ELSOFFSET = 0;		          /* element destination offset */
  g_dmaCTRLPKTz.FRDOFFSET = 0;		          /* frame destination offset   */
  g_dmaCTRLPKTz.FRSOFFSET = 0;                 /* frame destination offset   */
  g_dmaCTRLPKTz.PORTASGN  = 4;                 /* assign dma #               */
  g_dmaCTRLPKTz.RDSIZE    = ACCESS_16_BIT;	  /* read size                  */
  g_dmaCTRLPKTz.WRSIZE    = ACCESS_16_BIT; 	  /* write size                 */
  g_dmaCTRLPKTz.TTYPE     = FRAME_TRANSFER ;   /* transfer type              */
  g_dmaCTRLPKTz.ADDMODERD = ADDR_INC1;        /* address mode read          */
  g_dmaCTRLPKTz.ADDMODEWR = ADDR_FIXED;        /* address mode write         */
//g_dmaCTRLPKTz.ADDMODEWR = ADDR_OFFSET;       /* address mode write         */
  g_dmaCTRLPKTz.AUTOINIT  = AUTOINIT_ON;       /* autoinit                   */
}



//
// Send data to GigExpedite
//
static void Socket_SendFrame(SOCKET s)
{
	unsigned long Count;
    unsigned short d16;
  //unsigned char *Ptr = (unsigned char *)(Sockets[s].TxBuffer) + Sockets[s].TxBufferPtr;
    
  //make sure this address mathes the 8bit value  
    u_16Bit *dPtr = (u_16Bit *)((Sockets[s].TxBuffer) + Sockets[s].TxBufferPtr);
	int Words; 

	Sockets[s].State |= CONN_PAUSE;
	GigExWriteReg16(s*CHANNEL_SPACING+CONNECTION_STATE, Sockets[s].State);

	if (Sockets[s].Protocol==IPPROTO_UDP)
        {
		// Send frame length
		GigExWriteReg16(s*CHANNEL_SPACING+FRAME_LENGTH, Sockets[s].TxBufferLen);
        }

	Words = Sockets[s].TxBufferLen>=65536 ? 65536 : Sockets[s].TxBufferLen;
#define USE_DMA     
#ifdef USE_DMA
    unsigned char* xSockFIFO = (unsigned char*) ZestETM1 + ((s*CHANNEL_SPACING+DATA_FIFO));
    dmaDisable();
                    
    //assigning dma request: channel-1 with request line - 1
    dmaReqAssign(1,1 );

    //configuring dma control packets   (srcadd, destadd, datasize)
    dmaConfigCtrlPacketZest((uint32)dPtr, (uint32)xSockFIFO, Words );
    g_dmaCTRLPKTz.RDSIZE = ACCESS_16_BIT;	  //change read size to 16bits now

    // setting dma control packets, upto 32 control packets are supported
    dmaSetCtrlPacket(DMA_CH1,g_dmaCTRLPKTz);

    //setting the dma channel to trigger on software request
    dmaSetChEnable(DMA_CH1, DMA_SW);
    dmaEnableInterrupt(DMA_CH1, FTC);
    //enabling dma module
  //BinSt.gDMA_Mem2Wiz=1;
    dmaEnable();
   //while(BinSt.gDMA_Mem2Wiz==1);   must be done outside intr routine   //DMA intr handler in 'notifications.c' change value
    while(dmaREG->DMASTAT & BIT1 != 1);            //1==CH ACTIVE

#else   
	for (Count=0; Count<Sockets[s].TxBufferLen; Count++)
        {
        d16= SwapWord(*dPtr++);
		GigExWriteReg16(s*CHANNEL_SPACING+DATA_FIFO, d16);
        }
#endif
	Sockets[s].TxBufferPtr+= Words;
	Sockets[s].TxBufferLen-= Words;

	Sockets[s].State &= ~CONN_PAUSE;
	GigExWriteReg16(s*CHANNEL_SPACING+CONNECTION_STATE, Sockets[s].State);
}


//
// Establish a UDP connection
//
static int Socket_ConnectUDP(SOCKET s, unsigned long RemoteIP, unsigned long RemotePort, unsigned short LocalPort)
{
    unsigned long State = CONN_ENABLE | ESTABLISHED;
    int Done = 0;
    int Start = GetTime();
 
//tek udp mode not update for 16bit access yet   

	// Program GigExpedite registers
	GigExInterruptDisableAll();
	Sockets[s].IE |= IE_STATE;
	Sockets[s].State = State;
	Sockets[s].UDPEstablished = 0;
	GigExWriteReg16(s*CHANNEL_SPACING + REMOTE_IP_ADDR_3, RemoteIP>>16);    //16BIT MODE USE LOWER ADDR
	GigExWriteReg16(s*CHANNEL_SPACING + REMOTE_IP_ADDR_1, RemoteIP&0xffff); //16BIT MODE USE LOWER ADDR
	GigExWriteReg16(s*CHANNEL_SPACING + REMOTE_PORT_HIGH, RemotePort);      //16BIT MODE USE LOWER ADDR
	GigExWriteReg(s*CHANNEL_SPACING + LOCAL_PORT_HIGH, LocalPort);          //16BIT MODE USE LOWER ADDR
	GigExWriteReg(s*CHANNEL_SPACING + MTU, 0x80);
	GigExWriteReg(s*CHANNEL_SPACING + TTL, 0x80);
	GigExWriteReg(s*CHANNEL_SPACING + INTERRUPT_ENA, Sockets[s].IE);
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
		GigExWriteReg(s*CHANNEL_SPACING + INTERRUPT_ENA, Sockets[s].IE);
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
	GigExWriteReg(s*CHANNEL_SPACING + INTERRUPT_ENA, Sockets[s].IE);
	GigExInterruptEnableAll();
	while ((Sockets[s].IE&IE_STATE) && ((GetTime()-Start)<1000))
        {
        }
    if (Sockets[s].IE&IE_STATE)
        {
        // Timeout
    	GigExInterruptDisableAll();
        Sockets[s].IE &= ~IE_STATE;
        GigExWriteReg(s*CHANNEL_SPACING + INTERRUPT_ENA, Sockets[s].IE);
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
	GigExWriteReg16(s*CHANNEL_SPACING + INTERRUPT_ENA, Sockets[s].IE);
	GigExInterruptEnableAll();

    // Wait for completion
    // Could use semaphore from ISR if available to avoid busy wait
	while (((Sockets[s].State&STATE_MASK)==ESTABLISHED) && (Sockets[s].IE&IE_INCOMING) && ((GetTime()-Start)<2))
        {
        }
    if ((Sockets[s].State&STATE_MASK)==ESTABLISHED && (Sockets[s].IE&IE_INCOMING))
        {
        // Timeout
    	GigExInterruptDisableAll();
        Sockets[s].IE &= ~IE_INCOMING;
        GigExWriteReg16(s*CHANNEL_SPACING + INTERRUPT_ENA, Sockets[s].IE);
        GigExInterruptEnableAll();
        }
}

int g_Socket_SendFrame;

//
// Send data on a socket
//
static void Socket_SendData(SOCKET s)
{
    int Start = GetTime();

    // Set up transmit
    GigExInterruptDisableAll();
	Sockets[s].IE |= IE_OUTGOING_NOT_FULL;
	GigExWriteReg16(s*CHANNEL_SPACING + INTERRUPT_ENA, Sockets[s].IE);
    GigExInterruptEnableAll();

    // Wait for completion
    // Could use semaphore from ISR if available to avoid busy wait
    g_Socket_SendFrame=1;               //flag Send Complete 1==busy
//while ((((Sockets[s].IE&IE_OUTGOING_NOT_FULL) || (Sockets[s].IE&IE_OUT GOING_EMPTY)) && ((GetTime()-Start)<2)) )
	while ((((Sockets[s].IE&IE_OUTGOING_NOT_FULL) || (Sockets[s].IE&IE_OUTGOING_EMPTY)) && g_Socket_SendFrame ))
        {
        }
    if ((Sockets[s].IE&IE_OUTGOING_NOT_FULL) || (Sockets[s].IE&IE_OUTGOING_EMPTY))
        {
    	GigExInterruptDisableAll();
        Sockets[s].TxBufferLen = 0;
        Sockets[s].IE &= ~(IE_OUTGOING_NOT_FULL|IE_OUTGOING_EMPTY);
        GigExWriteReg16(s*CHANNEL_SPACING + INTERRUPT_ENA, Sockets[s].IE);
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
        //tek Interrupts(bit0-3), Enable=1, Disable=0
        //tek #define zIntrCntrl      (ZestETM1+ (INTERRUPT_ENA*2))
        //tek does addr need to be doubled as above '(INTERRUPT_ENA*2))'
		GigExWriteReg16(s*CHANNEL_SPACING+INTERRUPT_ENA, 0);  //Intr Ena= 0 (0x8000+11)
		GigExWriteReg16(s*CHANNEL_SPACING+CONNECTION_STATE, 0);  //ETM1 State= 0
        //later enable interrupts 
        }
	do
        {
		Done = 1;
		for (s=0; s<NUM_SOCKETS; s++)
            {
			if (GigExReadReg16(s*CHANNEL_SPACING+CONNECTION_STATE)!=0)
				Done = 0;
            }
        } while (Done==0);

	// Clear leftover interrupts
	for (s=0; s<NUM_SOCKETS; s++)
        {
		GigExReadReg16(s*CHANNEL_SPACING+INTERRUPT_ENA);
        }

	// Initialise connection state
	memset((void *)Sockets, 0, sizeof(Sockets));        //clear socket status array
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
    GigExWriteReg16(s*CHANNEL_SPACING + INTERRUPT_ENA, 0);
    GigExWriteReg16(s*CHANNEL_SPACING + CONNECTION_STATE, 0);

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
	GigExWriteReg16(s*CHANNEL_SPACING + LOCAL_PORT_HIGH, Addr->sin_port);
    //GigExWriteReg16(s*CHANNEL_SPACING + MTU, 0x80);
	//GigExWriteReg16(s*CHANNEL_SPACING + TTL, 0x80);
	GigExWriteReg16(s*CHANNEL_SPACING + TTL, 0x2E80);
	GigExWriteReg16(s*CHANNEL_SPACING + INTERRUPT_ENA, Sockets[s].IE);
	GigExWriteReg16(s*CHANNEL_SPACING + CONNECTION_STATE, Sockets[s].State);
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
    GigExWriteReg16(s*CHANNEL_SPACING + REMOTE_IP_ADDR_3, Target->sin_addr.s_addr>>16); //16BIT MODE USE LOWER ADDR
    GigExWriteReg16(s*CHANNEL_SPACING + REMOTE_IP_ADDR_1, Target->sin_addr.s_addr&0xffff); //16BIT MODE USE LOWER ADDR
    
    GigExWriteReg16(s*CHANNEL_SPACING + REMOTE_PORT_HIGH, Target->sin_port); //16BIT MODE LOWER ADDR
    GigExWriteReg16(s*CHANNEL_SPACING + LOCAL_PORT_HIGH, Addr->sin_port);   //16BIT MODE LOWER ADDR

    GigExWriteReg16(s*CHANNEL_SPACING + MTU, 0x80);
    GigExWriteReg16(s*CHANNEL_SPACING + TTL, 0x80);
    GigExWriteReg16(s*CHANNEL_SPACING + INTERRUPT_ENA, Sockets[s].IE);
    GigExWriteReg16(s*CHANNEL_SPACING + CONNECTION_STATE, Sockets[s].State);
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
        GigExWriteReg16(s*CHANNEL_SPACING + INTERRUPT_ENA, Sockets[s].IE);
        GigExWriteReg16(s*CHANNEL_SPACING + CONNECTION_STATE, Sockets[s].State);
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
	Socket_SendData(s);         //sends data to socket via interrupt enable, ISR does rest
		   
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
 
    //enable state change tek added 2 lines
    //Sockets[s].IE = IE_STATE;  //tek added
    //GigExWriteReg16(s*CHANNEL_SPACING+INTERRUPT_ENA, Sockets[s].IE); //tek added
    
    
    
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
int sendto(SOCKET s, const char *buf, int len, int flags, const struct sockaddr *toaddr, int tolen)
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
int recvfrom(SOCKET s, char *buf, int len, int flags, const struct sockaddr *fromaddr, int *fromlen)
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
	GigExWriteReg16(s*CHANNEL_SPACING + CONNECTION_STATE, Sockets[s].State);
	GigExWriteReg16(s*CHANNEL_SPACING + INTERRUPT_ENA, Sockets[s].IE);
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
	GigExWriteReg16(s*CHANNEL_SPACING + CONNECTION_STATE, Sockets[s].State);
	GigExInterruptEnableAll();
}

//
// Get local IP address from GigEx
//
unsigned long SocketGetLocalIPAddr(void)
{
    unsigned long Res = 0;
    
    Res = GigExReadReg16(LOCAL_IP_ADDR_3);  //16BIT MODE UPPER ADDR
    Res<<=16;
    Res |= GigExReadReg16(LOCAL_IP_ADDR_1); //16BIT MODE LOWER ADDR
    
    return Res;
}

//
// GigExpedite interrupt service routine
//
void SocketISR(void)            //this function must be called from main processing loop or allow port interrepts
{
	unsigned long State;
	unsigned long IntStatus;
	int s;
 
    for (s=0; s<NUM_SOCKETS; s++)
        {
        IntStatus= Sockets[s].IE & (IE_STATE);//+IE_INCOMING);
        if (Sockets[s].IE==0)
      //if (!(Sockets[s].IE&IntStatus))
            continue;

    	// Read interrupt status bits
    	// This will clear any state change interrupts
		IntStatus = GigExReadReg16(s*CHANNEL_SPACING+INTERRUPT_STATUS);

        do
            {
	        // Read connection state
	        State = GigExReadReg16(s*CHANNEL_SPACING+CONNECTION_STATE);

		    // Handle changes in connection state
            // Server, from Clodes to Listen Connection
    	    if ((((Sockets[s].State&STATE_MASK)==LISTEN) && (State&STATE_MASK)==ESTABLISHED)) 
                {
    		    // TCP connection established
    		    // Read connected address and acknowledge state change
    		    unsigned long Val;
    		    Sockets[s].Accept++;
                
    		    Val = GigExReadReg16(s*CHANNEL_SPACING+REMOTE_PORT_HIGH); //16BIT MODE USE LOWER ADDR
    		    Sockets[s].RemotePort = Val;
    		    Val = GigExReadReg16(REMOTE_IP_ADDR_3);     //16BIT MODE USE LOWER ADDR (IP upper 16)
                Val <<= 16;
    		    Val += GigExReadReg16(REMOTE_IP_ADDR_1);     //16BIT MODE USE LOWER ADDR (IP lower 16)
    		    Sockets[s].RemoteAddr = Val;

                Sockets[s].IE &= ~IE_STATE;
                GigExWriteReg16(s*CHANNEL_SPACING+INTERRUPT_ENA, Sockets[s].IE);
                GigExWriteReg16(s*CHANNEL_SPACING+CONNECTION_STATE, State);
			    Sockets[s].State = State;
                }
            // Client, from closed to Connect connection
    	    if ((((Sockets[s].State&STATE_MASK)==CONNECT) && ((State&STATE_MASK)==ESTABLISHED))) 
                {
    		    // TCP connection established
    		    // Acknowledge state change
    		    Sockets[s].Connect++;
                Sockets[s].IE &= ~IE_STATE;
                GigExWriteReg16(s*CHANNEL_SPACING+INTERRUPT_ENA, Sockets[s].IE);
                GigExWriteReg16(s*CHANNEL_SPACING+CONNECTION_STATE, State);
			    Sockets[s].State = State;
                }
            
                //Note that the user application must respond to a connection status of ESTABLISHED by writing
                //ESTABLISHED back to the connection control register. This acknowledges receipt of the status change.
                //Similarly, the user application should respond to the state change to CLOSED by writing CLOSED back to
                //the control register. (user guide p45)

    	    if ((((Sockets[s].State&STATE_MASK)==ESTABLISHED) && ((State&STATE_MASK)==ESTABLISHED)))
                {
    		    // UDP connection established
    		    // Acknowledge state change
    		    Sockets[s].UDPEstablished++;
                Sockets[s].IE &= ~IE_STATE;
                GigExWriteReg16(s*CHANNEL_SPACING+INTERRUPT_ENA, Sockets[s].IE);
              //GigExWriteReg16(s*CHANNEL_SPACING+CONNECTION_STATE, State);  //tek added
			    Sockets[s].State = State;
                }
    	    if ((((Sockets[s].Used==1 && (Sockets[s].State&STATE_MASK))==CLOSED) && ((State&STATE_MASK)==CLOSED)) )
                {
    		    // Connection closed - free socket and acknowledge state change
			    Sockets[s].IE = 0;
                Sockets[s].State = 0;
                GigExWriteReg16(s*CHANNEL_SPACING+INTERRUPT_ENA, Sockets[s].IE);
                GigExWriteReg16(s*CHANNEL_SPACING+CONNECTION_STATE, Sockets[s].State);
                if (Sockets[s].Closing)
                	Sockets[s].Used = 0;
                }
	        IntStatus &= ~IE_STATE;

		    // Service incoming data
		    if ((((Sockets[s].IE&IE_INCOMING)) && (IntStatus&IE_INCOMING) ))
                {
			    // Data available - read a frame from GigExpedite
                Socket_ReadFrame(s);
                if (Sockets[s].Protocol==IPPROTO_UDP || Sockets[s].RxBufferPtr==Sockets[s].RxBufferLen)
                    {
                	// Receive is complete
					Sockets[s].IE &= ~IE_INCOMING;
					GigExWriteReg16(s*CHANNEL_SPACING+INTERRUPT_ENA, Sockets[s].IE);
                    }
                }
		} while (IntStatus&IE_STATE);

		// Service outgoing data
		if ((((Sockets[s].IE&IE_OUTGOING_EMPTY)) && (IntStatus&IE_OUTGOING_EMPTY) ))
            {
			// Outgoing connection empty - all data sent
			Sockets[s].IE &= ~IE_OUTGOING_EMPTY;
			GigExWriteReg16(s*CHANNEL_SPACING+INTERRUPT_ENA, Sockets[s].IE);
            }

        if ((((Sockets[s].IE&IE_OUTGOING_NOT_FULL)) && (IntStatus&IE_OUTGOING_NOT_FULL) ))
            {
			// Space available - send data (up to 64k at a time)
            //tek add dma here
        	Socket_SendFrame(s);
        	if (Sockets[s].TxBufferLen==0)
                {
        		Sockets[s].IE &= ~IE_OUTGOING_NOT_FULL;
        		Sockets[s].IE |= IE_OUTGOING_EMPTY;
        		GigExWriteReg16(s*CHANNEL_SPACING+INTERRUPT_ENA, Sockets[s].IE);
                }
            g_Socket_SendFrame=0;               //flag Send Complete
            }
        } //end for loop
}





//******************************************************************************
// External functions
// These need platform specific definitions created for your system
//
//********************    RM48L to ZestETM1 functions     **********************
//
//******************************************************************************


//ETHERNET base ADDR (uC chip sel 4)
#define ZestETM1         0x68000000 

//Interrupts(bit0-3), Enable=1, Disable=0
#define zIntrCntrl      (ZestETM1+ INTERRUPT_ENA)
 

//Read/write GigExpedite registers
//Addr is the GigExpedite register address which should be memory mapped into
//the processor address space.  In this example, the 8 bit registers are mapped
//to 32 bit locations with a spacing of 4 bytes.


//RM48L Definition for ZestETMI Bus Write
//
void GigExWriteReg(unsigned long Addr, unsigned char Data)
{
    unsigned char* Ptr8 = (unsigned char*) ZestETM1 + (Addr/2);
    *Ptr8= Data;
}
void GigExWriteReg16(unsigned long Addr, uint16_t Data)
{
    sPTR Ptr16 = (sPTR) ZestETM1 + (Addr/2);  //addr may need... +(Addr*2)
    *Ptr16= Data;
}


//RM48L Definition for ZestETMI Bus Read
//
unsigned char GigExReadReg(unsigned long Addr)
{
    unsigned char d8;
    unsigned char* Ptr8 = (unsigned char*) ZestETM1 + (Addr/2);
    d8= *Ptr8;
    return d8;
}

uint16_t GigExReadReg16(unsigned long Addr)
{
    uint16_t d16;
    sPTR Ptr16 = (sPTR) ZestETM1 + (Addr/2);
    d16= *Ptr16;
    return d16;
}



/*
RM48L Definition for ZestETMI Interrupt Enable
Bit 15-4: Reserved. Must be set to 0.
Bit 3: State change interrupt
Bit 2: To network buffer not full interrupt
Bit 1: To network buffer empty interrupt
Bit 0: From network data available interrupt
*/

unsigned short Intr16;
//Definition Interrupt On
//
void GigExInterruptEnableAll(void)
{

  _enable_interrupt_();
  return;
  sPTR Ptr16 = (sPTR) zIntrCntrl;
  for (int s=0; s<NUM_SOCKETS; s++)     //do all channels
    {
  //*Ptr16= 0xf;                        //all intr bits on
    *Ptr16= Intr16;                     //restore intr bits
     Ptr16 += CHANNEL_SPACING/2;           //do all channels
    }
}


//Definition Interrupt Off
//

////   note setup valid for only channel0 because of offset adjust
////   note setup valid for only channel0 because of offset adjust
////   note setup valid for only channel0 because of offset adjust

void GigExInterruptDisableAll(void)
{
  _disable_interrupt_();
  return;

  sPTR Ptr16 = (sPTR) zIntrCntrl;
  Intr16= *Ptr16;
  for (int s=0; s<NUM_SOCKETS; s++)     //do all channels
    {
    *Ptr16= 0;                          //all intr bits off
    Ptr16 += CHANNEL_SPACING/2;         //do all channels
    }
}


//This is used to timeout various socket functions
//You can disable the timeouts by just returning 0 from this function
//
//RM48L Definition for ZestETMI GetTime in milli-seconds
//
unsigned long  GetTime(void)
{
    return mStime.g_SockMs;
}


void tcp_server_read_ena(SOCKET s)
{
    Sockets[s].IE |= IE_INCOMING;
}





int tcp_server_rec_cnt_decrement(SOCKET s)
{
   int retval;
   retval= Sockets[s].RxFrameLenLast--;
   return retval;
}


int tcp_server_rec_cnt(SOCKET s)
{
   int retval;
   retval= Sockets[s].RxFrameLenLast;
   return retval;
}


 void initRec(SOCKET s)
 {
	struct sockaddr_in *Addr = (struct sockaddr_in *)&Sockets[s].LocalAddr;

//  	Sockets[s].Protocol= IPPROTO_TCP;
//    Sockets[s].RxBuffer= &Buf1500;
//   Sockets[s].LocalAddr.sa_family= ServPort;
//    socket(0,0,IPPROTO_TCP);   //socket(int af, int type, int protocol); 
//    Addr->sin_port= 5000;
//    listen(s, 0);               //sets 'time to live'

    //enable state change
    Sockets[s].IE = IE_STATE;
    GigExWriteReg16(s*CHANNEL_SPACING+INTERRUPT_ENA, Sockets[s].IE);
   
 }



int loopback_tcps(SOCKET g_Sock, uint16 port, uint8* eBuf, uint16 flag)    //socket, port, buffer, mode(Sn_MR for no delay)
{
    int len;
    len= recv(g_Sock, eBuf, TX_RX_MAX_BUF_SIZE, 0);             //'recv(SOCKET s, char *buf, int len, int flags)'
    len= Sockets[g_Sock].RxFrameLenLast;
    return len;
}

extern char Buf1500;

void tcp_server_init(SOCKET s, uint16_t ServPort, char* eBuf )
{
  //GigExWriteReg16(s*CHANNEL_SPACING+LocPortReg, ServPort); //set channnels server port number

    sPTR Ptr16 = (sPTR) zIntrCntrl;
	struct sockaddr_in *Addr = (struct sockaddr_in *)&Sockets[s].LocalAddr;
   
   
  	Sockets[s].Protocol= IPPROTO_TCP;
    
  //tek check if initial setting needed here  
  //Sockets[s].RxBuffer= eBuf; 
    Sockets[s].RxBuffer= &Buf1500;

    Sockets[s].LocalAddr.sa_family= ServPort;
    
    socket(s,0,IPPROTO_TCP);   //socket(int af, int type, int protocol); 
    
    Addr->sin_port= ServPort;
    listen(s, 0);               //sets 'time to live'

    
    //enable state change
  //Sockets[s].IE = IE_STATE;
  //GigExWriteReg16(s*CHANNEL_SPACING+INTERRUPT_ENA, Sockets[s].IE);
    
}





