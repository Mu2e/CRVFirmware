//*******************************************************************************
// ZestSock_TEK.c functions
// 'mu2e_Zest_IO.h'
//*******************************************************************************

#ifndef _mu2e_Zest_IO
#define _mu2e_Zest_IO

//internal fuctions
unsigned long GetTime(void);
void        GigExInterruptDisableAll(void);
void        GigExInterruptEnableAll(void);
void        tcp_server_init(SOCKET s, uint16_t ServPort, uint8* eBuf );

void 		Display_ZestETM1_NET(int port, char*);
void        SocketISRTEK(SOCKET);
void        Socket_RecvData(SOCKET s);
void        connectStatus(SOCKET s);

uint16_t    GigExReadReg16(unsigned long Addr);
void        GigExWriteReg16(unsigned long Addr, uint16_t Data);
void        GigExWriteReg8(unsigned long Addr, uint16_t Data);
uint8_t     GigExReadReg8(unsigned long Addr);

void        mDelay(uint32 cnt);
void        uDelay(uint32 cnt);

#endif