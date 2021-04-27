//*******************************************************************************
// Mu2e_Ctrl_i2c.c functions
// '_Mu2e_Ctrl_i2c.h'
//*******************************************************************************

#ifndef _Mu2eCtrl_i2c
#define _Mu2eCtrl_i2c

//internal fuctions
uint8_t     i2c_RdModWr(uint8_t port, uint8_t WRreg, uint8_t RDreg, uint8_t mask);
uint8_t     i2c_write_port (uint8_t port, uint8_t reg, uint8_t data);
uint8_t     i2c_read_port (uint8_t port, uint8_t reg);
uint8_t     poePortWr(uint8_t dev, uint8_t addr, uint8_t cnt, uint8_t* data);

uint8_t     i2cSendData(uint8_t slvAddr, uint8_t slvReg, uint16_t cnt, uint8_t *dBlk );
uint8_t     i2cRecvData(uint8_t slvAddr, uint8_t slvReg, uint16_t rdcnt, uint8_t *dBlk, int mode );
int      	i2c_rec_intr_mode();
int         PEO_CurrentRead(void);
int         RG45LEDS(int leds);
int         POE_Volts_Check_One(void);

#endif