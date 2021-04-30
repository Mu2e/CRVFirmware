/*
 * W5300 example applications
 * Revision History :
 * ----------  -------  -----------  ----------------------------
 * Date        Version  Author       Description
 * ----------  -------  -----------  ----------------------------
 * 24/03/2008  1.0.0    MidnightCow  Release with W5300 launching
 * ----------  -------  -----------  ----------------------------
 * 01/05/2008  1.0.1    MidnightCow  Modify 'w5300.c'. Refer M_01052008.
 * ----------  -------  -----------  ----------------------------
 * 15/05/2008  1.1.0    MidnightCow  Refer M_15052008.
 *                                   Modify 'w5300.c', 'w5300.h' and 'socket.c'.
 * ----------  -------  -----------  ----------------------------
 */

//********************************************************
//  @file Wizmain.c
//  RM48 Code
//  RM48 Hercules device supports little-endian [LE] format
//
//  Parallel 16Bit Read/Writes on Wiznet W5300 Ethernet Interface Chip
//  The byte ordering of W5300 registers is big-endian,
//      low address byte is used as the most significant byte.
//  tek 2018
//********************************************************
   
   

#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include "socket.h"
#include "w5300.h"
#include "ver_io.h"
#include "sys_mu2e.h"
#include "sys_mu2e_misc.h"

//extern structures
extern struct   msTimers mStime;
extern struct   udpHeader   udpHdr;
extern struct   blocksnd BinSt;
extern struct   netinfo_s netInfo;                   //active setup data here
extern struct   udpMessage udpMsg;

//extern functions
extern void     __iar_program_start(void);

//extern buffers
extern char    tBuf[400];
extern char    Buf1500[1500];           //stores boot up messages
extern uint8   check_sendok_flag[];     //The flag to check if first send or not.
extern unsigned int genFlag;


//functions
void    GW_processing(int, int);
int     wizMain(void);
int     loopback_tcps(SOCKET s, uint16 port, uint8* buf,uint16 mode);
void    loopback_tcpc(SOCKET s, uint8* addr, uint16 port, uint8* buf,uint16 mode);
int     loopback_udp(SOCKET s, uint16 port, uint8* buf, uint16 mode);
uint32  sendto_mod(SOCKET s, uint8 * buf, uint32 len, uint8 * addr, uint16 port, sPTR rData);
uint32  send_mod(SOCKET s, uint8 * buf, uint32 len); //DMA 'ftp' send function
//prototype modified functions
uint32   wiz_write_buf_dmaW(SOCKET s,uint8* buf,uint32 len);


//buffers
char    eRecDatBuf[4][CHAR_BUF_SZ_1600]; //socket(s) command line buffer size
int     recDatSz[8];                     //store last rec'd packet data size (diagnostics)
int     sockConnectReq[4];               //socket connect request timer

//W5300 internally contains 16 memory blocks of 8Kbyte
//One socket for ARP 
//setup for 8 Sockets   0  1  2  3 4 5 6 7      //0,1,2 Socks are Telnet,(Sock4 for ARP)
uint8 tx_mem_conf[8] = {16,16,16,8,8,0,0,0};    //for setting TMSR regsiter
uint8 rx_mem_conf[8] = {16,16,16,8,8,0,0,0};    //for setting RMSR regsiter

uint8 ip[4] = {131,225,53,84};                  //for setting SIP register HLSWH5
uint8 gw[4] = {131, 225, 55, 200};              //for setting GAR register
uint8 sn[4] = {255,255,255,0};                  //for setting SUBR register
uint8 mac[6]= {0x00,0x80,0x55,0xBD,0x00,0x15};  //for setting SHAR register



int wizMain()
{
   setMR(0x80); 		                                  // soft reset
   uDelay(100);

   /*initiate W5300*/
   iinchip_init();
   mDelay(11);

   /* allocate internal TX/RX Memory of W5300 */
   if(!sysinit(tx_mem_conf,rx_mem_conf))
      {
      sprintf(tBuf, "MEMORY CONFIG ERR!!, Wiznet is disabled\r\n");
      putBuf(tty, tBuf,0);
      return 1; 
      }

   // The byte ordering of W5300 registers is big-endian – low address byte is used as the most significant byte.
   //'MR_FS' FIFO swap bit of MR, change byte order of tx and rx data fifos.
   setMR(getMR() | MR_FS);                      //If Little-endian, set MR_FS
   getMR();

    //tek Aug2019, mod to file socket.c   function recv(), see below
   /*now send a packet acknowledge after receiving a packet
   #if 1  //was 0 tek aug2019
   if(getSn_RX_RSR(s) == 0)                     // check if the window size is full or not
   { // Sn_RX_RSR can be compared with another value instead of ¡®0¡¯,
     // according to the host performance of receiving data
      setSn_TX_WRSR(s,1);                       // size : 1 byte dummy size
      IINCHIP_WRITE(Sn_TX_FIFOR(s),0x0000);     // write dummy data into tx memory
      setSn_CR(s,Sn_CR_SEND);                   // send
      while(!(getSn_IR(s) & Sn_IR_SENDOK));     // wait SEND command completion
      setSn_IR(s,Sn_IR_SENDOK);                 // clear Sn_IR_SENDOK bit
   }
   #endif
      */   
   
   
   
   //NOTE: loopback_tcps(..,FLAG) will set reg 'Sn_MR' at connection time
   //TCP data is composed of PACKET-INFO and DATA packet in case of Sn_MR(ALIGN)='0'.
   //In case of Sn_MR(ALIGN) = '1', TCP data has only DATA packet by removing PACKETINFO.
   //setMR(getMR()|Sn_MR_ALIGN);                      //remove 'PACKETINFO'

   //eRecSiz defined as 1024
   setSn_MSSR(0, eRecSz1024 );                      //1460 max
   setSn_MSSR(1, eRecSz1024 );                      //1460 max 
   setSn_MSSR(2, eRecSz1024 );                      //1460 max
   setSn_MSSR(3, eRecSz1024 );                      //if udp use 1472 max
   setSn_MSSR(4, 1460 );                            //1472 max
   setSHAR((uint8*)&netInfo.macAddr);               //Set source hardware address

   /* configure network information */
   setSIPR((uint8*)&netInfo.ipAddr);                // set source IP address
   setGAR((uint8*)&netInfo.gateWay);                // set gateway IP address
   setSUBR((uint8*)&netInfo.netMask);               // set subnet mask address

   /* verify network information */
   getSHAR(mac);                                    // get source hardware address
   getGAR(gw);                                      // get gateway IP address
   getSUBR(sn);                                     // get subnet mask address
   getSIPR(ip);                                     // get source IP address
   sprintf(tBuf,"IP      : %d.%d.%d.%d\r\n",ip[0],ip[1],ip[2],ip[3]);
   putBuf(tty, tBuf,0);
   sprintf(tBuf,"GATE    : %d.%d.%d.%d\r\n",gw[0],gw[1],gw[2],gw[3]);
   putBuf(tty, tBuf,0);
   sprintf(tBuf,"MASK    : %d.%d.%d.%d\r\n",sn[0],sn[1],sn[2],sn[3]);
   putBuf(tty, tBuf,0);
   sprintf(tBuf,"MAC     : %02x:%02x:%02x:%02x:%02x:%02x\r\n\n",mac[0],mac[1],mac[2],mac[3],mac[4],mac[5]);
   putBuf(tty, tBuf,0);
   return 0;
}



/*
 * "TCP SERVER" loopback program.
 */
int loopback_tcps(SOCKET s, uint16 port, uint8* buf, uint16 mode)
{
   uint32 len=0;
   switch(getSn_SSR(s))                 // check SOCKET status
   {                                    // ------------
      case SOCK_ESTABLISHED:            // ESTABLISHED?
        if(getSn_IR(s) & Sn_IR_CON)     // check Sn_IR_CON bit,   Interrupt Status
            {
            setSn_IR(s,Sn_IR_CON);      //clear Sn_IR_CON
            }
        if((len=getSn_RX_RSR(s)) > 0)   //check the size of received data
            {
            //note that packet size is limited by 'setSn_MSSR(s,eRecSz1024)'
            //getSn_RX_RSR(s) may hold multiple packets waiting to be processed
            //make sure receive buffer is 'CHAR_BUF_SZ_1600' >= 'setSn_MSSR(s,eRecSz1024)'
            //note: len returns char cnt of 1st packet, buf may hold multiple packets
            len = recv(s,buf,len);      //recv
            recDatSz[s]=len;            //store for sock diagnostic menu display
            }
        break;
                                        // ---------------
   case SOCK_CLOSE_WAIT:                // PASSIVE CLOSED
        if (s==BinSt.gSndPrt)
            {
            BinSt.gSndBytCnt=0;
            }
        disconnect(s);                  // disconnect
        sprintf(tBuf,"%d : SockCloseWait(%d) Close Socket.\r\n",s,port);
        putBuf(tty, tBuf,0);
        break;
                                        // --------------
   case SOCK_CLOSED:                    // CLOSED
        //Phenomenon
        //W5300 ERRATA SHEET Aug 19, 2008
        //In TCP Mode, Sn_SSR(Socket status register)value does not change from "0x10" or "0x11" during the TCP connect process.
        //fix by sending UPD msg then close socket
        //this fix already in code by Wiznet
        //socket closing, stop any active 'rdb data block sends'
        if (s==BinSt.gSndPrt)
            {
            BinSt.gSndBytCnt=0;
            }
        close(s);                       // close the SOCKET
        socket(s,Sn_MR_TCP,port,mode);  // open the SOCKET
        break;
                                       // ------------------------------
   case SOCK_INIT:                     // The SOCKET opened with TCP mode
        //set socket keep alive time to auto mode
        listen(s);                      // listen to any connection request from "TCP CLIENT"
        sprintf(tBuf,"%d : SOCK_TCPS(%d) Started\r\n",s,port);
        putBuf(tty, tBuf,0);
        setSn_KPALVTR(s, 12);            // SOCKET s Keep Alive Time, 5Sec/cnt (set 60 secs)
        break;
        //note on '0x11' must allow time for initial socket connection will showing this status
        //see 'SOCK_CLOSED' comment above
   case 0x11:                           // Hung UP ???
        if (sockConnectReq[s]++ > 10000) // 1 second timeout, approx.
            {
            sockConnectReq[s]= 0;
            //socket closing, stop any active 'rdb data block sends'
            if (s==BinSt.gSndPrt)
                {
                BinSt.gSndBytCnt=0;
                }
            close(s);                       // close the SOCKET
            //socket(s,Sn_MR_TCP,port,mode);  // open the SOCKET
            sprintf(tBuf,"%d : SOCK_CONNECT_TIMEOUT PORT(%d).\r\n",s,port);
            putBuf(tty, tBuf,0);
            }
        break;
   default:
        break;
   }
   return len;
}



//   "TCP CLIENT" loopback function  
//
void     loopback_tcpc(SOCKET s, uint8* addr, uint16 port, uint8* buf, uint16 mode)
{
   uint32 len;
   static uint16 any_port = 1000;

   switch(getSn_SSR(s))                   // check SOCKET status
   {                                      // ------------
      case SOCK_ESTABLISHED:              // ESTABLISHED?
         if(getSn_IR(s) & Sn_IR_CON)      // check Sn_IR_CON bit
         {
            sprintf(tBuf,"%d : Connect OK\r\n",s);
            putBuf(tty, tBuf,0);
            setSn_IR(s,Sn_IR_CON);        // clear Sn_IR_CON
         }
         if((len=getSn_RX_RSR(s)) > 0)    // check the size of received data
         {
            len = recv(s,buf,len);        // recv
            if(len !=send(s,buf,len))     // send
                {
                sprintf(tBuf,"%d : Send Fail.len=%d\r\n",s,len);
                putBuf(tty, tBuf,0);
                }
         }
         break;
                                          // ---------------
   case SOCK_CLOSE_WAIT:                  // PASSIVE CLOSED
         disconnect(s);                   // disconnect
         break;
                                          // --------------
   case SOCK_CLOSED:                      // CLOSED
      close(s);                           // close the SOCKET
      socket(s,Sn_MR_TCP,any_port++,mode);// open the SOCKET with TCP mode and any source port number
      break;
                                          // ------------------------------
   case SOCK_INIT:                        // The SOCKET opened with TCP mode
      connect(s, addr, port);             // Try to connect to "TCP SERVER"
      sprintf(tBuf,"%d : LOOPBACK_TCPC(%d.%d.%d.%d:%d) Started.\r\n",s,addr[0],addr[1],addr[2],addr[3],port);
      putBuf(tty, tBuf,0);
      break;
   default:
      break;
   }
}



//   UDP loopback function
//
int loopback_udp(SOCKET s, uint16 port, uint8* buf, uint16 mode)
{
   uint32 len,lenRet=0;
   switch(getSn_SSR(s))
   {
    case SOCK_UDP:                                      
         if((len=getSn_RX_RSR(s)) > 0)                  // check the size of received data
            {
            if (len > CHAR_BUF_SZ_1600)
                {
                len = CHAR_BUF_SZ_1600;                 //the data size to read is MAX_BUF_SIZE.
                close(s);                               // close the SOCKET
                sprintf(tBuf,"\rWiz_Diag_Msg: BufOverRun Sock=%d Len=%d\r\n" ,s,len);
                putBuf(tty, tBuf,0);
                lenRet= 0;
                }
            else{
                //return Succeed: received data size, Failed:  -1
                //note buffer may have multiple udps, only len of 1st is returned
                //using setMR(getMR() & ~MR_FS) in 'socket.c' to toggle between big/little endian
                lenRet = recvfrom(s, (uint8*)eRecDatBuf[s], len,(u_char*)&udpHdr.destip, (uint16*)&udpHdr.destport);
                }
            len=getSn_RX_RSR(s);
            }
         recDatSz[s]=lenRet;                            //store for sock diagnostic menu display
         break;
                                                        // -----------------
    case SOCK_CLOSED:                                   // CLOSED
         close(s);                                      // close the SOCKET
         break;

    default:
         break;
   }
return lenRet;
}



//modified sendto() from function socket.c
//If Little-endian mode for Stellaris LM3S5B91
//after udp header, data will be sent as words in big_endian format
uint32   sendto_mod(SOCKET s, uint8 * buf, uint32 len, uint8 * addr, uint16 port, sPTR rData)
{
   static uint8 status=0;
   static uint8 isr=0;
   uint32 ret= len;
   uint32 idx=0;
   uint32 freesize=0;

   isr = getSn_IR(s);
   if ( isr = getSn_IR(s) )                     // clear any timeouts
       if (isr & Sn_IR_TIMEOUT )
           setSn_IR(s,Sn_IR_TIMEOUT);

   // NOTE : CODE BLOCK START
  do
   {
   freesize = getSn_TX_FSR(s);
   } while (freesize < len);
  // NOTE : CODE BLOCK END


   // set destination IP address
   IINCHIP_WRITE(Sn_DIPR(s),(((uint16)addr[0])<<8) + (uint16) addr[1]);
   IINCHIP_WRITE(Sn_DIPR2(s),(((uint16)addr[2])<<8) + (uint16) addr[3]);

   // set destination port number
   IINCHIP_WRITE(Sn_DPORTR(s),port);

   //tek,10-26-11 for udp data16, turn off byte swap
   //If Little-endian, set MR_FS. (Stellaris LM3S5B9)
   setMR(getMR() & ~MR_FS);

   //load udp header, (always 4 words)
   for (idx = 0; idx < 4; idx++, buf=buf+2)
        IINCHIP_WRITE(Sn_TX_FIFOR(s),*(uint16*)buf );

   //If Little-endian, set MR_FS. (Stellaris LM3S5B9)
   setMR(getMR() | MR_FS);

   //use DMA to xfer data form uC to Wiznet (rData used data port option on fpga for little endian format)
   len -=8;
   wiz_write_buf_dmaW(s, (void*)rData, len);            // copy data, len=byte to word cnt

   //load data raw data (word count)
   //len /=2;
   //for (; idx < len; idx++)
   //     IINCHIP_WRITE(Sn_TX_FIFOR(s),*rData);

   //set data send size
   setSn_TX_WRSR(s,ret);
   //send it
   setSn_CR(s, Sn_CR_SEND);

   //TP21_ON                                        //Test Point 21 (PJ7)
   idx=0;
   //moved send wait to complete code
   while (!((isr = getSn_IR(s)) & Sn_IR_SENDOK))    //wait SEND command completion
       {
          status = getSn_SSR(s);
          //-------------- warning -------------------------
          // Sn_IR_TIMEOUT causes the decrement of Sn_TX_FSR
          // -----------------------------------------------
          if ((status == SOCK_CLOSED) || (isr & Sn_IR_TIMEOUT))
          {
             udpMsg.SockErr++;
             //send fail status
             setSn_IR(s,Sn_IR_TIMEOUT);
             return 0;
          }
          uDelay(5);
          if (idx++ > 1000000)
              {
              //reset board at this point, cant wait forever
              sprintf(tBuf,"\a\a\a : System Fatal Error on UDP_SendTo_Mod()\r\n");
              putBuf(tty, tBuf,0);
              //allow time for msg to print
              mDelay(10);
              __iar_program_start();                //reset uC, should never happen?
              //old code below left but we dont get there
              udpMsg.SockErr++;
              close(s);                             //close the SOCKET
              return 0xffff;
              }
           }
       setSn_IR(s, Sn_IR_SENDOK);                  //Clear Sn_IR_SENDOK
   //TP21_OFF                                      //Test Point 21 (PJ7)
   return ret;
}




//modified send() from function socket.c
uint32 send_mod(SOCKET s, uint8 * buf, uint32 len)  //len is byte count
{
   uint32 ret= len;
   static int ii=0;

   //below code waits for free size to equal or greater than my send 'len'
   while((getSn_TX_FSR(s)) < len);          //get buffer freesize

   //use DMA to xfer data form uC to Wiznet
   wiz_write_buf_dmaW(s, buf, len);          // copy data, len=byte to word cnt
   ii=0;
   if(!check_sendok_flag[s])                // if first send, skip.
    {
    while (!(getSn_IR(s) & Sn_IR_SENDOK))   // wait previous SEND command completion.
        {
        ii++;
        uDelay(50);
        if (getSn_SSR(s) == SOCK_CLOSED)    // check timeout or abnormal closed.
            {
            return 0;
            }
        }
   setSn_IR(s, Sn_IR_SENDOK);               // clear Sn_IR_SENDOK
   }
   else check_sendok_flag[s] = 0;

   if (ii>20)
       {
      //printf("wiz_in:uS_Delay %d\r\n", ii*50);
        udpMsg.udpSendDly++;
       }

   // send
   setSn_TX_WRSR(s,len);
   setSn_CR(s,Sn_CR_SEND);
   return ret;
}




//Update gw Addr Resolution Protocol (ARP cache)
//On Wiznet socket connect(), ARP 0x0806 will first be sent
//do this so network gateway will ack new 'MAC' on network 
//AutoIP is a method to negotiate a IPv4 address reserved 169.254.1.0 to 169.254.254.255 
//

int  retLen=0, any_port= 0x3000;
uint8 dest_IP[4]= {255,255,255,255};    //dest broadcast addr 
uint8 GW_MAC[6];                        //gateway mac address

void GW_processing(int mSecs, int port) //mSesonds delay, port# for printf strings
{
    uint8 j=0, sock=sockARP;      
    uint16 dat = 0x1;
    
    memcpy(&dest_IP, &netInfo.ipAddr, sizeof(dest_IP));//dest, src, size      
    do
        {        
        socket(sock,Sn_MR_UDP,any_port,0);             //cleanup socket error
        sendto(sock,(uint8*)&dat,1, dest_IP, any_port);//sends 1 chr
        mDelay(mSecs);

        //any data back
        retLen= loopback_udp(sock, any_port, (uint8*)Buf1500, 0);            
        sprintf(Buf1500,"GW_MAC   : RTN_LEN %d\r\n", retLen);
        putBuf(tty, Buf1500,0);
        close(sock);                                   //close the SOCKET
        } while ((retLen==0) && j++<2);               
}


//sock4 used for keep alive ARP packet
//time for ARP request, keeps network access alive by anoying GateWays
//ARP_REQ flag set on a 5 minute interval in function 'SysTick_Handler
//Telnet Sockets 0,1,2 
//Socket 3 not used on wiznet chip, but in software sock3 refers to ePhy RJ45 I/O 
//Socket 4 is for ARP only
//
void arpKeepAlive()
{
    static u_16Bit PassOne=0;
    uint16 dat = 0x11;
    if ((genFlag & ARP_REQ) || (PassOne==0)) //loopback_tcps
        {
    //Clear nError latch, do a cycle to clk flip/flop,
    CLR_ERR_HI
    CLR_ERR_LO
          
        if (PassOne==0)         
            {
            PassOne++;
            GW_processing(200, tty);
            }
        else if ( (getSn_SSR(sock0)!=SOCK_ESTABLISHED) ||  (getSn_SSR(sock1)!=SOCK_ESTABLISHED) 
                ||  (getSn_SSR(sock2)!=SOCK_ESTABLISHED) )
            {  
            socket(sockARP,Sn_MR_UDP, any_port, 0);             //cleanup socket error
            sendto(sockARP,(uint8*)&dat, 1, dest_IP, any_port); //sendto sends data as 16bit words
            close(sockARP);                                     //close the SOCKET            
            }
        genFlag &= ~ARP_REQ;
        }
}


