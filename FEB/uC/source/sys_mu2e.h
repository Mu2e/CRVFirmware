// @file sys_mu2e.h
// Fermilab Terry Kiper 2016-2019

#ifndef _sys_mu2e
#define _sys_mu2e

int         putchar(int  c);
void        putBuf(int prt, char* sBuf,  int len);
char        *mytok(char *, char **);            //token test search in cmd line
long        arg_hex(char **, long);             //get hex token from buffer
long        arg_dec(char **, long);             //get dec token from buffer
long        arg_hex(char **, long);             //get hex token from buffer
int         isdigit(int);
int         isxdigit(int);
int         toupper(int);
char        *mytok(char *, char **);            //token test search in cmd line
char        myupper(char);
void        mDelay(uint32 cnt);
void        uDelay(uint32 cnt);
int         getBufBin();
int         SockKeyWait(int wait, int sock, int *key);
int         KeyBoardWait(int wait);
void        USB_Rec_Init();
void        LVDS_Intr_Time(uint32 cnt);

uint32      send_full(uint8 s, uint32 len);


//DMA 
void        dmaConfigCtrlPacket(uint32 sadd,uint32 dadd,uint32 dsize);
void        loadDataPattern(uint32 psize, uint16* pptr);
void        DMA_Initialization(void);
int         sendBinNonDAQ(int prt);
extern int  sendRDX();


//Misc
void        dispNet();
int         isdigit(int);
int         isxdigit(int);
int         toupper(int);
char        myupper(char);
uSHT        revB(short int c);
u_8Bit      revB_byte(u_8Bit);          //reverse 8 bits
uSHT        SwapByte(unsigned short);


int         sdRamTest(int,int,int);
int         trigDMA1_mem2wiznet(uint16_t, uint16_t);
int         trigDMA0_fpga2mem (uint16_t, uint16_t *);
int         trigDMA0_fpga2mem1(uint16_t, uint16_t *);
void        dataRecall(int mode, u_32Bit fpga);
void        stab(int fpga);

int         sendBin(int prt);
int         fpga_Check();
float       readTemperature(void);

int         lfdsFMsendNow();
void        HexDump(char *addr, int len, int port);
void        header(int, int);
void        header1(int);
void        header1TTY(int);
void        appendTerm(int prt);
void        headerHTML(int prt);
void        handleW3();
void        adcRefresh();
void        nError();

int         eCMD_DAQ_DY2_Handler(uSHT*, int, int);
int         eCMD_DAQ_DY2_Handler_TST(uSHT*, int, int);

//SI5338 CLOCK GEN
int         si5338_init(void);
uint8_t     i2c_write_5338(uint8_t reg, uint8_t data);
uint8_t     i2c_read_5338(uint8_t reg);
uint8_t     i2c_RdModWr_5338(uint8_t WRreg, uint8_t RDreg, uint8_t mask);

int         i2cSendData(uint8_t slvAddr, uint8_t slvReg, uint16_t cnt, uint8_t *dBlk );
int         i2cRecvData(uint8_t slvAddr, uint8_t slvReg, uint16_t rdcnt, uint8_t *dBlk );
int         pga280(uint16 gain, uint16 reset);
int         pga280read(uint16 gain);
float       ADC_ads1259(int);
int         EQW_GS3140(uint16 *data, int cnt);
int         EQR_GS3140(uint16 *addr, uint16 *Destdata, int cnt);

char*       getline(char *, int, int *);
int         process(int, char *);
int         pacsnd(long param1,long param2);        //handle cmd

void        hDelayuS(uint32 del);

//DAQ
void        create_packet(int size2DataCnt);
int         ubGetData(int,sPTR);
int         ubGetDataNew(int,sPTR);

extern      void headerType(int prt);
extern      void headerStatus(int prt, int clr);
extern      void fill_daq_packet(int size2DataCnt, u_8Bit *dataPtr); //mac phy packet loader 'sys_mu2e_daq.c'
extern      void arpKeepAlive();  //see 'wizmain.c'
extern      int phy_Pac_check(void); //mac phy command packet check 'sys_mu2e_daq.c' 
int         lfdsFMsendBUF4096(char* sBuf, int lenWrds);
//int         poeInitRequest(void);
#endif		/* _sys_mu2e */
