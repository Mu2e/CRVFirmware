//*******************************************************************************
// Mu2e_Ctrl.c functions
// 'Mu2e_Ctrl.h'
//*******************************************************************************

#ifndef _mu2e_cntrl
#define _mu2e_cntrl

#include "esm.h"
#include "het.h"
#include "rti.h"
#include "spi.h"
#include "adc.h"
#include "sys_dma.h"
#include "gio.h"

#include "sys_core.h"           //intr enable/disable
#include "reg_system.h"
#include "i2c.h"
#include "sci.h"

#define CMD_PREFETCH 0xA1         // Prefetch command code                   
#define CMD_DATAREQUEST 0xA2      // Data request command code               

typedef struct {                                                              
    uint8_t cmdType;           // Command type (e.g., prefetch, data request) 
    uint16_t eventWindowTag;   // Unique identifier for the event window      
    char payload[609]; // Packet data - buffer was 600?
} DataPacket; 


//internal fuctions
int     putchar(int  c);
void    putBuf(int prt, char* sBuf,  int len);
char    *mytok(char *, char **);            //token test search in cmd line
long    arg_hex(char **, long);             //get hex token from buffer
long    arg_dec(char **, long);             //get dec token from buffer
long    arg_hex(char **, long);             //get hex token from buffer
int     isdigit(int);
int     isxdigit(int);
int     toupper(int);
char    *mytok(char *, char **);            //token test search in cmd line
char    myupper(char);
void    newLinePrompt(int);
char*   itoa(int val, int base);


void    mDelay(uint32 cnt);
void    uDelay(uint32 cnt);
int     getBufBin();
int     SockKeyWait(int wait, uint16 sock, uint16 *key);
int     KeyBoardWait(int wait);
int     ClkDrvInit();

int     LDFILE(int prt);
int     LDFLASH(int prt);
int     SEND_2_FEB(int prt, int xports);
int     link_check(int prt);
int     LvdsDataAvail(int* d16);
int     PoolDataReq(int);
int     loadFLASH_SOCK(int, char*, int);

int     SockKeyWait(int wait, uint16 sock, uint16 *key);

int SET_SDADDR_RDx(int chipOffset, int HI, int LO);
int SET_SDADDR_WRx(int chipOffset, int HI, int LO);


//Reverse bits of parameter in the register R0, return value is passed back
//to its caller in register R0.
u_8Bit revBits8(u_8Bit c);                       

//byte swap
unsigned short SwapBytes(unsigned short x);

//Load-Multiple memory copy
//The previous example is modified to use LDM and STM instructions, 
//transferring 8 words per iteration. Due to the extra registers used, 
//these must be stored to the stack and later restored.

//The .n suffix is used in ARM UAL Thumb-2 assembly where you have a 
//choice of instruction encodings: 16- or 32-bit. It forces a 16-bit 
//encoding to be used.
unsigned short movStrBlk32(unsigned short *dst, unsigned short *src, int cnt);

//32bit moves, cnt is in lngword count
unsigned short movStr32(unsigned short *src, unsigned short *dst, short int cnt);

//16bit moves, cnt is in word count
unsigned short movStr16(unsigned short *src, unsigned short *dst, short int cnt);


void    USB_Rec_Init();

// DMA STUFF
void    dmaConfigCtrlPacket(uint32 sadd,uint32 dadd,uint32 dsize);
void    loadDataPattern(uint32 psize, uint16* pptr);


//command line processing
int     isdigit(int);
int     isxdigit(int);
int     toupper(int);
char    myupper(char);
char*   inet_ntoa(unsigned long addr);

int     sdRamTest(int,int,int);
int     trigDMA1_mem2wiznet(uint16_t, uint16_t);
int     trigDMA0_fpga2mem(uint16_t, uint16_t , uint16_t *);
void    dataRecall(int mode, u_32Bit fpga);

void    dispNet(int);
int     sendBin(int prt);
int     fpga_Check(int dev);
float   readTemperature(void);
float   readTemperatureC(void);
void    hDelayuS(uint32 del, uint32 wait );
char*   getline(char *, int, int *);
int     process(int, char *);
int     FLSD(int prt);
int     LDF(int, int, char*); 
float   poePower(int poeprt);
int     link_Init(int prt);
void    InitFPGA_REGISTERS();
int     linkCmdFunc(int, char*);

int     pfmget(int);
int     stab_dPool(int, int);
int     HappyBusCheck();
int     param2FramAddr(int reg);

//SI5338 CLOCK GEN
//void    sdRamRead16(int chip, uint32 addr32, uint16 count, uint16 prt);
//void    sdRamWrite16(int chip, uint32 addr32, uint16 data);

uint16  movStr16_NOICSRC(unsigned short *src, unsigned short *dst, long int cnt);
uint16  movStr16_NOICDEST(unsigned short *src, unsigned short *dst, long int cnt);
uint16  movStr16_NOINC(unsigned short *src, unsigned short *dst, long int cnt);

uint16  movStrBlk32(unsigned short *dst, unsigned short *src, int cnt);
uint16  movStr16(unsigned short *src, unsigned short *dst, short int cnt);
uint16  movStr32(unsigned short *src, unsigned short *dst, short int cnt);
void    MemCopy32(unsigned long *dst, unsigned long *src, int bytes);
void    sciUartSendText(sciBASE_t *sci, uint8 *text, uint32 length);
void    HexDump(char *addr, int len, int port);
void    HexDump16(char *addr, int len);
void    header1(int);
void    appendTerm(int prt);


static int find_free_prefetch_slot();
static int find_prefetch_slot(uint16_t event_lo, uint16_t event_hi);
static void clear_prefetch_slot(int idx);
  
#define putchar     __putchar
#endif
