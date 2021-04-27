//********************************************************
// @file ZestETM1.c 
//  Orange Tree GigaBit Ethernet Interface Module 'ZestETM1'
//  RM48 Micro Controller
//  RM48 Additions


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


#include "sys_common.h" 
#include "sys_core.h"       // arm system enable/disable interrupts

#include <stdio.h>
#include <string.h>
#include "ZestETM1_SOCKET.h"

#include "ver_io.h"             //uC i/o ref
#include "het.h"                //uc Test Point ref
#include "sys_dma.h"            //uC DMA ref

#include "ZestSock_TEK.h"      //mu2e orange tree zest interface

struct zSockets Sockets[NUM_SOCKETS];

//
// External functions
// These need platform specific definitions created for your system
//
extern      volatile struct msTimers    mStime; 
extern      uint32  g_timeMs;
extern      char Buf1500[];
extern      unsigned short SwapBytes(unsigned short x);
extern      void putBuf(int prt, char* sBuf,  int len);
extern      void uDelay(uint32 cnt);

extern int  g_IntrCnt;
extern      uint32 iFlag;
char        ttbuf[40];
extern struct   netinfo_s netInfo;                  
extern uint8    eRecDatBuf[NUM_SOCKETS][TXRX_BytSiz1600]; //socket(s) command line buffer size

unsigned long int badIP=0, badIP_OVR=0;

//////////////////////////////////////////////////////////////////////////////
// Local functions






//
// Read data from GigExpedite
//
void Socket_ReadFrameTEK(SOCKET s)
{
    uint16_t Data, *dPtr;
    dPtr= (uint16_t*)(Sockets[s].RxBuffer + Sockets[s].RxBufferPtr);
    do
        {        
        if (Sockets[s].RxFrameLen==0)
            {
            //Read the frame length
            Sockets[s].RxFrameLen = GigExReadReg16(s*CHANNEL_SPACING+FRAME_LENGTH);  //16bit mode
            //bad ip overflow
            if (Sockets[s].RxFrameLen> TXRX_BytSiz1600)  //badIP_OVR      
                badIP_OVR++;

            //Save Len
            Sockets[s].RxFrameLenLast= Sockets[s].RxFrameLen;  //TEK ADDED
        //tek length is in bytes, from here we will readout as words
        //convert byte to word count
        //Sockets[s].RxFrameLen = ((Sockets[s].RxFrameLen +1)/2);
            }

        if (Sockets[s].RxFrameLen!=0)
            {
            if (Sockets[s].RxFrameLen==1)
                {
                // Read data byte, for now store as word
                Data = GigExReadReg8(s*CHANNEL_SPACING+DATA_FIFO);
                if (Sockets[s].RxFrameLen < TXRX_BytSiz1600)//must get all data else 'otree socket hangs up'
                    *dPtr++= Data;          //okay to move data
                else
                    *dPtr= Data;            //no incr dest, dump extra data else buf overflow
                Sockets[s].RxBufferPtr++;   //incr one byte
                Sockets[s].RxFrameLen--;    //dec once
                }
            else
                {
                // Read data word 
                Data = GigExReadReg16(s*CHANNEL_SPACING+DATA_FIFO);
                Data= SwapBytes(Data);      //tek added
                if (Sockets[s].RxFrameLen < TXRX_BytSiz1600)//must get all data else 'otree socket hangs up'
                    *dPtr++= Data;          //okay to move data
                else
                    *dPtr= Data;            //no incr dest, dump extra data else buf overflow
                Sockets[s].RxBufferPtr++;   //incr one byte
                Sockets[s].RxBufferPtr++;   //incr one byte
                Sockets[s].RxFrameLen--;    //dec once
                Sockets[s].RxFrameLen--;
                }
                }
            } while (Sockets[s].RxBufferPtr<Sockets[s].RxBufferLen && Sockets[s].RxFrameLen!=0);    
}




//Read/write GigExpedite registers
//Addr is the GigExpedite register address which should be memory mapped into
//the processor address space.  In this example, the 16 bit registers are mapped
//to 32 bit locations with a spacing of 4 bytes.

//ZESTETM1 16 Bit Write Access Mode
//Normal 16bits access @ address offset 0x10
void GigExWriteReg16(unsigned long Addr, uint16_t Data)
{
    sPTR Ptr16 = (sPTR) ZestETM1 + (Addr/2);  //addr may need... +(Addr*2)
    *Ptr16= Data;
}


//ZESTETM1 16 Bit Read Access Mode
//Normal 16bits access @ address offset 0x10
uint16_t GigExReadReg16(unsigned long Addr)
{
    //clr address 'a0' low (even addr) for word access mode via 'fpga to ZestETM1' logic
    sPTR Ptr16 = (sPTR) ZestETM1 + (Addr/2);
    return *Ptr16;
}


//ZESTETM1 8 Bit Write Mode via (Short or Char)
//FPGA LOGIC Controls re-directs uC signal to ZESTETM1 
//Char 8bits access @ address offset 0x13 uses 8 bit Data
//Short 16bits access @ address offset 0x11 uses 8 bit Data 'upper byte'
//
//nBE[1..0] : Active low byte write enables. nBE1 should be low to
//indicate the data on DQ[15..8] is valid during a write. nBE0 should
//be low to indicate the data on DQ[7..0] is valid during a write.
//
void GigExWriteReg8(unsigned long Addr, uint16_t Data)
{
    //set address 'a0' high (odd addr) for bytes access mode via 'fpga to ZestETM1' logic
    cPTR Ptr8 = (cPTR) ZestETM1 + (Addr)+3;  
    *Ptr8= Data;
}

uint8_t GigExReadReg8(unsigned long Addr)
{
    //set address 'a0' high (odd addr) for bytes access mode via 'fpga to ZestETM1' logic
    cPTR Ptr8 = (cPTR) ZestETM1 + (Addr)+3;  
    return *Ptr8;
}



//
// Send data to GigExpedite TCP Mode Only
//
static void Socket_SendFrame(SOCKET s, int flags)
{
  //unsigned int Words;
    unsigned short d16;
    int Count=0; 
    u_16Bit *dPtr = (u_16Bit *)((Sockets[s].TxBuffer) + Sockets[s].TxBufferPtr);
    
    sPTR wPtr16 = (sPTR) ZestETM1 + (((s*CHANNEL_SPACING)+DATA_FIFO)/2);  //addr may need... +(Addr*2)
    
    //If paused, when bit is set back to 0 the GigExpedite will begin transmitting 
    //the buffered data in large packets rather than multiple small packets
	Sockets[s].State |= CONN_PAUSE;
	GigExWriteReg16(s*CHANNEL_SPACING+CONNECTION_STATE, Sockets[s].State);
  //Words = Sockets[s].TxBufferLen>=65536 ? 65536 : Sockets[s].TxBufferLen; //limit

	for (; Count<Sockets[s].TxBufferLen; Count++)
        {
        d16= SwapBytes(*dPtr++);
        *wPtr16= d16;       //GigExWriteReg16(s*CHANNEL_SPACING+DATA_FIFO, d16);

        //TEK MAY 21,2018 moved below as addition //Sockets[s].TxBufferPtr++;
        //Sockets[s].TxBufferLen--;
        }
    Sockets[s].TxBufferPtr +=Count;
    Sockets[s].TxBufferLen=0;   //just set it to zero, it always emptys buffer
    //word cnt doesnt matter, all is sent
	//Sockets[s].TxBufferPtr+= Words;
	//Sockets[s].TxBufferLen-= Words;
    if(flags) //flags==1==odd count to send
        {
        d16= *dPtr;    //use data in lower byte, if using char mode.  
        GigExWriteReg8(s*CHANNEL_SPACING +DATA_FIFO, d16);
        }
	Sockets[s].State &= ~CONN_PAUSE;
	GigExWriteReg16(s*CHANNEL_SPACING+CONNECTION_STATE, Sockets[s].State);
}




//////////////////////////////////////////////////////////////////////////////
// External API

//
// Initialise library
//
void SocketInit(void)
{
	int s, Done, d16, errbreak=0;
	// Clean GigExpedite registers
 	//while(1) 
    
    for (s=0; s<NUM_SOCKETS; s++)
        {
        GigExWriteReg16(s*CHANNEL_SPACING+CONNECTION_STATE, 0);  //disable channel(s) closed
        uDelay(100);
        GigExWriteReg16(s*CHANNEL_SPACING+CONNECTION_STATE, 0x10);  //disable channel(s) tcp
        uDelay(100);
        }
    
	do
        {
		Done = 1;
		for (s=0; s<NUM_SOCKETS; s++)
            {
			//if (GigExReadReg16(s*CHANNEL_SPACING+CONNECTION_STATE)!=0)
            d16= GigExReadReg16(s*CHANNEL_SPACING+CONNECTION_STATE);
			if (d16 != 0)
				Done = 0;
            }
        //This routine sometimes hangs until Orange Tree Board pwr restart ???
        //This routine sometimes hangs until Orange Tree Board pwr restart ???
        if (errbreak++>50000)
            {            
            iFlag |= OTREE_CONFIG;   
            break;      //ot failed
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

 //Write 0x2E80 to the ‘Payload size/TTL’ register to set the 
 //maximum payload size to 1472 bytes and time to live to 46 
 //(note the maximum payload size is ignored for TCP)  
 // GigExWriteReg16(s*CHANNEL_SPACING + TTL, 0x2E80);
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
    GigExWriteReg16(s*CHANNEL_SPACING + REMOTE_IP_ADDR_3, Target->sin_addr.s_addr>>16);    //16BIT MODE USE LOWER ADDR
    GigExWriteReg16(s*CHANNEL_SPACING + REMOTE_IP_ADDR_1, Target->sin_addr.s_addr&0xffff); //16BIT MODE USE LOWER ADDR
    
    GigExWriteReg16(s*CHANNEL_SPACING + REMOTE_PORT_HIGH, Target->sin_port); //16BIT MODE LOWER ADDR
    GigExWriteReg16(s*CHANNEL_SPACING + LOCAL_PORT_HIGH, Addr->sin_port);    //16BIT MODE LOWER ADDR

    //Write 0x2E80 to the ‘Payload size/TTL’ register to set the 
    //maximum payload size to 1472 bytes and time to live to 128 
    //(note the maximum payload size is ignored for TCP)  
	GigExWriteReg16(s*CHANNEL_SPACING + TTL, 0x2E80);
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
    //Setup should be in IPPROTO_TCP Protocol
	//Initiate transfer
	Sockets[s].TxBuffer = buf;
	Sockets[s].TxBufferPtr = 0;
	Sockets[s].TxBufferLen = len;  //word count from top level 'putBuf()'
    int IntStatus, del=0;
   // static int zStatusBad=0;
    //if (zStatusBad>100)
    //    {
     //   return 0;
     //   }

    //Not Full status means at least 64K bytes available, up to max of 3MBytes
    //assuming total 48Mbytes evenly split on 16 channels
    //now checking not full status before getting here
    
 //tek may2018 not sure if needed
    
 /*
    IntStatus = GigExReadReg16(s*CHANNEL_SPACING+INTERRUPT_STATUS);
    while ((IntStatus& IE_OUTGOING_NOT_FULL) != IE_OUTGOING_NOT_FULL) //or test IE_OUTGOING_EMPTY
      {
      if (del++>4000) //4000*.025mS==1000mS 
        break;
      uDelay(25);
      IntStatus = GigExReadReg16(s*CHANNEL_SPACING+INTERRUPT_STATUS);
      }
    //this should never happen
    if (del>1000)
        {
        sprintf(ttbuf,"SendDelay=%duSec  ISTAT=%4X\r\n", (del/25), IntStatus);
        putBuf(s, ttbuf,0);
        putBuf(tty, ttbuf,0);
       // zStatusBad++;
       // if (zStatusBad>10)
        //    {
       //     sprintf(ttbuf,"ZEST_ETM1 status bad, disable xmits until board reset\r\n");
        //    putBuf(s, ttbuf,0);
       //     }
        }
 */
    
    Socket_SendFrame(s, flags);
	return Sockets[s].TxBufferPtr;
}


//
// Receive data from remote device
// flags is ignored
//
int recvTEK(SOCKET s, char *buf, int len, int flags)
{
    int IntStatus;
    if (len==0) return 0;
    if (Sockets[s].Protocol!=IPPROTO_TCP) return 0; //return len as 0
    
    //TCP SERVER MODE ONLY
    Sockets[s].RxBuffer = buf;
    Sockets[s].RxBufferLen = len;
    Sockets[s].RxBufferPtr = 0;
    
    //Check Data Available, will always process complete frame
    GigExInterruptDisableAll();
    
    if (Sockets[s].RxFrameLen!=0)
        Socket_ReadFrameTEK(s);
	GigExInterruptEnableAll();
    
	if (Sockets[s].RxBufferPtr>=Sockets[s].RxBufferLen)
		{// Complete
		return Sockets[s].RxBufferPtr;
		}

    //Initiate receive //tek added jun2018
    Socket_RecvData(s);

	return Sockets[s].RxBufferPtr;  //updated rec'd data ptr
}





//
// Receive data on a socket
// tek this temporarily turns on incoming data interrupt
void Socket_RecvData(SOCKET s)
{
    //int Start = GetTime();
    int w=0;
    // Set up receive
	GigExInterruptDisableAll();
	Sockets[s].IE |= IE_INCOMING;
	GigExWriteReg16(s*CHANNEL_SPACING + INTERRUPT_ENABLE, Sockets[s].IE);
    
	GigExInterruptEnableAll();

    // Wait for completion
    // Could use semaphore from ISR if available to avoid busy wait
	while ((Sockets[s].State&STATE_MASK)==ESTABLISHED && (Sockets[s].IE&IE_INCOMING))
        {
        uDelay(10);     //(GetTime()-Start)<1000)
        if(w++> 5)
          break;
        }
    if ((Sockets[s].State&STATE_MASK)==ESTABLISHED && (Sockets[s].IE&IE_INCOMING))
        {
        // Timeout
    	GigExInterruptDisableAll();
        Sockets[s].IE &= ~IE_INCOMING;
        GigExWriteReg16(s*CHANNEL_SPACING + INTERRUPT_ENABLE, Sockets[s].IE);        
        GigExInterruptEnableAll();
        }
}






//
// Close and free socket  (not used here, using OTree Auto-Open Settings via web-page )
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
    
    //The socket may still be open here - the ISR will
    //clean up when GigExpedite acknowledges close
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



//****************  RM48L to ZestETM1 functions   **********************
//
//**********************************************************************

// RM48L Definition for ZestETMI Interrupt Enable
// Bit 15-4: Reserved. Must be set to 0.
// Bit 3: State change interrupt
// Bit 2: To network buffer not full interrupt
// Bit 1: To network buffer empty interrupt
// Bit 0: From network data available interrupt

//Interrupt On
void GigExInterruptEnableAll(void)
{
  //Enable processor intr only
  _enable_interrupt_();
  return;
}


//Interrupt Off
void GigExInterruptDisableAll(void)
{
  _disable_interrupt_();
  return;
}


int tcp_server_rec_cnt_decrement(SOCKET s)
{
   int retval;
   retval= Sockets[s].RxFrameLenLast--;
   return retval;
}



//display ZestETM1 IP,GW, MASK Registers
void Display_ZestETM1_NET(int port,  char* Buf)
{
    int i,j;

    sprintf(Buf,"\r\nZESTETM1 LNK: Enabled\r\n");
    putBuf(port,Buf,0);          
    sPTR zAdr = (sPTR) ZestETM1+ (0x21E/2);   //get Phy Link Status
    i= *zAdr;
    if (i&0x10)                         //check if link connected
        {
        sprintf(Buf,    "ZESTETM1 LNK: Ethernet Connected\r\n");
        putBuf(port,Buf,0);   
        if(i&1) 
            sprintf(Buf,"ZESTETM1 LNK: Speed 10 Mbps\r\n");
        else if(i&2)
            sprintf(Buf,"ZESTETM1 LNK: Speed 100 Mbps\r\n");
        else 
            sprintf(Buf,"ZESTETM1 LNK: Speed 1000 Mbps\r\n");
        putBuf(port,Buf,0);  

    zAdr = (sPTR) ZestETM1;                 
    i= *(zAdr+(0x200/2));
    j= *(zAdr+(0x202/2));
    sprintf(Buf,"IPv4 Address: %u.%u.%u.%u\r\n", i>>8&0xff,i&0xff,j>>8&0xff,j&0xff);
    putBuf(port, Buf,0);                    
    i= *(zAdr+(0x20C/2));
    j= *(zAdr+(0x20E/2));
    sprintf(Buf,"GATEWAY Addr: %u.%u.%u.%u\r\n", i>>8&0xff,i&0xff,j>>8&0xff,j&0xff);
    putBuf(port, Buf,0);                    
    i= *(zAdr+(0x204/2));
    j= *(zAdr+(0x206/2));
    sprintf(Buf,"SUBNET Mask : %u.%u.%u.%u\r\n\n", i>>8&0xff,i&0xff,j>>8&0xff,j&0xff);
    putBuf(port, Buf,0);                    
        }
    else 
        {
        sprintf(Buf,"ZESTETM1 LNK: Ethernet Not Connected\r\n\n");
        putBuf(port,Buf,0);   
        }
}

 
//When GigExpedite closes a connection it will not reset the data channel. 
//This means that any data in the incoming or outgoing buffers will be left 
//for when the next connection is made. The external device can flush the data 
//buffers by writing a 0 to the appropriate connection control register.
void tcp_server_init(SOCKET s, uint16_t ServPort, uint8* eBuf )
{
    //tek April 2018 let orangeTree use auto-open tcp mode 
	struct sockaddr_in *Addr = (struct sockaddr_in *)&Sockets[s].LocalAddr;
    socket(s,0,IPPROTO_TCP);    //socket(int af, int type, int protocol); 
    Addr->sin_port= ServPort;
    listen(s, 0);               //sets 'time to live'
}


//loopback_tcps called for main, check for new data in network rec buffer
//socket, port, buffer, mode(Sn_MR for no delay)
int loopback_tcps(SOCKET g_Sock, uint16 port, char* eBuf, uint16 flag)    
{
    int len, state, sReg;
    uint16_t Data;
    Sockets[g_Sock].RxFrameLenLast= 0;
	state = GigExReadReg16(g_Sock*CHANNEL_SPACING+CONNECTION_STATE);
    if (((state& 0xF)==CLOSED) || (Sockets[g_Sock].Protocol!=IPPROTO_TCP)) 
        {
         u_16Bit *uPtr= &netInfo.sTelnet0 + g_Sock; 
         tcp_server_init(g_Sock, *uPtr, eRecDatBuf[g_Sock] );//(sock,port)  
         return 0;
       }

     sReg= REG16(ZestETM1+ (g_Sock*CHANNEL_SPACING)+ zConnectCSR); //CHANNEL_SPACING==16words
     sReg= sReg & 0xf;
     if (sReg == LISTEN)
       return 0;
     
    //tek May24,2018
    //trap ip that locks up all ports
    //Name:     fsf-galileo.fnal.gov
    //Address:  131.225.12.216
     
    if(Sockets[g_Sock].RemoteAddr== 0x83e10cd8) 
        {
        badIP++;
        //tek added may2018
        while ((Sockets[g_Sock].RxFrameLen!=0))  //replaced above, TEK ADDED
            {
            // Read data word 
            Data= GigExReadReg16(g_Sock*CHANNEL_SPACING+DATA_FIFO);
            Sockets[g_Sock].RxBufferPtr++;   //incr one byte
            Sockets[g_Sock].RxBufferPtr++;   //incr one byte
            Sockets[g_Sock].RxFrameLen--;    //dec once
            if (Sockets[g_Sock].RxFrameLen)  //if dummy byte read, Rec'd Byte Cnt may have been odd
                Sockets[g_Sock].RxFrameLen--;
            }
        len=0;
        }
     
    //now using intr routine to call 'SocketISRTEK()'
    //'SOCKET s, char *buf, int len, int flags'
    len= recvTEK(g_Sock, eBuf, TXRX_BytSiz1600, 0); //check of any data
    len= Sockets[g_Sock].RxFrameLenLast;            //data len if any
    return len;
}



//GigExpedite interrupt service routine
//Handles sockets states, does not handle data input or output
//added timer check 'notifications.c' to come here every 10 seconds in case intr missed (hangUp)
void SocketISRTEK(SOCKET s)            
{
	unsigned long State;
	unsigned long IntStatus;
    
    g_IntrCnt++;                                    //interrupt entry counter
    for (s=0; s<NUM_SOCKETS; s++)
    {
        if (Sockets[s].IE==0)
            continue;
    
  	// Read interrupt status bits
   	// This will clear any state change interrupts
	IntStatus = GigExReadReg16(s*CHANNEL_SPACING+INTERRUPT_STATUS);
    
 //tek may24,2018 uncommented out
 //if ((IntStatus & IE_STATE) != IE_STATE)
   //return;

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
                
    	    Val = GigExReadReg16(s*CHANNEL_SPACING+REMOTE_PORT_HIGH);   //16BIT MODE USE LOWER ADDR (port)
    	    Sockets[s].RemotePort = Val;
    	    Val = GigExReadReg16(s*CHANNEL_SPACING+REMOTE_IP_ADDR_3);   //16BIT MODE USE LOWER ADDR (IP upper 16)
            Val <<= 16;
    	    Val += GigExReadReg16(s*CHANNEL_SPACING+REMOTE_IP_ADDR_1);  //16BIT MODE USE LOWER ADDR (IP lower 16)
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
         // GigExWriteReg16(s*CHANNEL_SPACING+CONNECTION_STATE, State);  //tek added
		    Sockets[s].State = State;
            }
        if ((((Sockets[s].Used==1 && (Sockets[s].State&STATE_MASK))==CLOSED) && ((State&STATE_MASK)==CLOSED)) )
            {
            //Connection closed - free socket and acknowledge state change
            //tek aug4, this is done in cmd 'QUIT' along with tcp_server_init() that calls listen()
		    Sockets[s].IE = 0;
            Sockets[s].State = 0;
            GigExWriteReg16(s*CHANNEL_SPACING+INTERRUPT_ENA, Sockets[s].IE);
            GigExWriteReg16(s*CHANNEL_SPACING+CONNECTION_STATE, Sockets[s].State);
            if (Sockets[s].Closing)
             	Sockets[s].Used = 0;
            }

		    // Service incoming data
		if ((Sockets[s].IE&IE_INCOMING) && (IntStatus&IE_INCOMING))
		    {
			// Data available - read a frame from GigExpedite
            Socket_ReadFrameTEK(s);
            if (Sockets[s].Protocol==IPPROTO_UDP || Sockets[s].RxBufferPtr==Sockets[s].RxBufferLen)
                {
              	// Receive is complete
				Sockets[s].IE &= ~IE_INCOMING;
				GigExWriteReg16(s*CHANNEL_SPACING+INTERRUPT_ENABLE, Sockets[s].IE);
                }

            }
        
        //tek may24,2018 comment out
        //u_16Bit *uPtr= &netInfo.sTelnet0 +s; 
        //tcp_server_init(s, *uPtr, eRecDatBuf[s] );//(sock,port)                     
	    IntStatus &= ~IE_STATE;
        //Service incoming data, no longer done here
            
	} while (IntStatus&IE_STATE);
    }
    //Service outgoing data, no longer done here
}


