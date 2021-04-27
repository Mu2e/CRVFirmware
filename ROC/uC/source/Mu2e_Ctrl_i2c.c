//********************************************************
// @file Mu2e_Ctrl_i2c.c 
//  Fermilab Terry Kiper 2016-2021
//
//  mu2e CRV Readout Controller 
//  RM48 Micro Controller
//********************************************************

//I2C CLOCK SETUP
//1 MHz Clock max for LTC4266

/* Include Files */
#include "sys_common.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "esm.h"
#include "het.h"
#include "rti.h"
#include "spi.h"
#include "emif.h"
#include "sci.h"
#include "adc.h"
#include "i2c.h"
#include "sys_dma.h"
#include "gio.h"
#include "emac.h"
#include "hw_reg_access.h"
#include "sys_core.h"
#include "reg_system.h"

#include "nHetCap.h"
#include "reg_het.h"
#include "hal_stdtypes.h"

#include "ver_io.h"
#include "mu2e_Ctrl_i2c.h"       //mu2e i2c link functions
#include "Mu2e_Ctrl.h"

uint16  g_i2cErrors;

//extern data
extern uint16 g_i2cErr;

//i2c send/rec command/data temp array
//extern uint8_t i2Data[10];
extern uint32  genFlag;

//interal data
uint16_t    fpLEDs_STORE[POECHsALL];        //fnt pnl led state store
uint16_t    POE_VLT_ALL[POECHsALL];
uint16_t    POE_CUR_ALL[POECHsALL];
                                
uint16      Timeouts_i2c=0;

//ltc4266 port map to rg45 header block (seq as top_left 1, bom_left 2 ...)
const   int     I2VltPortRegMap[]= {0x3A,0x3B, 0x3E,0x3F, 0x32,0x33, 0x36,0x37};
const   int     I2CurPortRegMap[]= {0x38,0x39, 0x3C,0x3D, 0x30,0x31, 0x34,0x35};
const   int     PoeAddr[]= {0x42, 0x44, 0x46, 0x48, 0x4A, 0x4C};

//                         {chip, regH, regL, fntPnlLed, chip1of6}
const int PoeVltsSlaveAdr[][5]= {0x42,0x3A,0x3B,4,0, 0x42,0x3E,0x3F,0,0, 0x42,0x32,0x33,12,0, 0x42,0x36,0x37,8,0,
                             0x44,0x3A,0x3B,4,1, 0x44,0x3E,0x3F,0,1, 0x44,0x32,0x33,12,1, 0x44,0x36,0x37,8,1,
                             0x46,0x3A,0x3B,4,2, 0x46,0x3E,0x3F,0,2, 0x46,0x32,0x33,12,2, 0x46,0x36,0x37,8,2,
                             0x48,0x3A,0x3B,4,3, 0x48,0x3E,0x3F,0,3, 0x48,0x32,0x33,12,3, 0x48,0x36,0x37,8,3,
                             0x4A,0x3A,0x3B,4,4, 0x4A,0x3E,0x3F,0,4, 0x4A,0x32,0x33,12,4, 0x4A,0x36,0x37,8,4,
                             0x4C,0x3A,0x3B,4,5, 0x4C,0x3E,0x3F,0,5, 0x4C,0x32,0x33,12,5, 0x4C,0x36,0x37,8,5};

const int PoeCursChipAddr[][3]= {0x42,0x38,0x39, 0x42,0x3C,0x3D, 0x42,0x30,0x31, 0x42,0x34,0x35,
                             0x44,0x38,0x39, 0x44,0x3C,0x3D, 0x44,0x30,0x31, 0x44,0x34,0x35,
                             0x46,0x38,0x39, 0x46,0x3C,0x3D, 0x46,0x30,0x31, 0x46,0x34,0x35,
                             0x48,0x38,0x39, 0x48,0x3C,0x3D, 0x48,0x30,0x31, 0x48,0x34,0x35,
                             0x4A,0x38,0x39, 0x4A,0x3C,0x3D, 0x4A,0x30,0x31, 0x4A,0x34,0x35,
                             0x4C,0x38,0x39, 0x4C,0x3C,0x3D, 0x4C,0x30,0x31, 0x4C,0x34,0x35};


extern struct g_i2cTransfer
{
    uint32  mode;
    uint32  length;
    uint8   * data;
} g_i2cTransfer_t;


/*
//send i2c data byte to one of many chip, registers
//
uint8_t i2c_write_port (uint8_t DevPort, uint8_t reg, uint8_t data)
{
    uint8_t err;
    err= i2cSendData(DevPort, reg, 1, &data );      //cnt, &data, rtns data in i2Data[]
    if (err)
        {
        g_i2cErrors++;
        putBuf(tty, "i2c Err(1)\n\r",0);
        return 1;
        }
    return 0;
}


//read i2c data byte to one of many chip, registers
//
uint8_t i2c_read_port(uint8_t DevPort, uint8_t reg)
{
    g_i2cErr= i2cRecvData(DevPort, reg, 1, i2Data, 0);
    if (g_i2cErr)
        {
        g_i2cErrors++;
        putBuf(tty, "i2c Err(2)\n\r",0);
        return 1;
        }
    return i2Data[0];
}



//Modify i2c chip Reg   ...Read ... Modify .... Write
uint8_t i2c_RdModWr(uint8_t port, uint8_t wrReg, uint8_t rdReg, uint8_t mask)
{
    //do a read-modify-write    (1st...read)
    g_i2cErr= i2cRecvData(port, rdReg, 1, i2Data, 0);
    if (g_i2cErr)
        {
        g_i2cErrors++;
        putBuf(tty, "i2c Err(3)\n\r",0);
        return 1;
        }
      
    i2Data[0] &= mask;                  //reg mask to modify
    //do a read-modify-write    (2nd...write)
    g_i2cErr += i2cSendData(port, wrReg, 1, i2Data );  //cnt, &data, rtns data in i2Data[]
    if (g_i2cErr)
        {
        g_i2cErrors++;
        putBuf(tty, "i2c Err(3)\n\r",0);
        return 1;
        }
    return 0;           //return 0 for no error
}
 */



//............i2c sequence.............
//Setup to write
//Trigger Start condition
//  Start and Address are on the bus
//Wait for TX or ARDY

//Write first data byte into DXR
//Wait for TX or ARDY

//Check for NACK
//  If NACK than send stop, clear flags and exit
//  if ACK than first data byte was on the bus

//Write second data byte into DXR
//Wait for TX or ARDY

//Check for NACK
//  If NACK than send stop, clear flags and exit
//  if ACK than second data byte was on the bus
//...
//However, checking for a NACK after the start bit and address, as I did in my code, shouldn't hurt.



//Send block of data using RM48L seq logic
// in:  data count
//      data ptr
// ret: 0=fail
//ack  --- active low level
//nack --- active high level
uint8_t i2cSendData(uint8_t slvAddr, uint8_t slvReg, uint16_t cnt, uint8_t *dBlk )
{
    int i=0, iTimout=0;
    //Set the Destination Slave address 
    slvAddr>>=1;
  //slvReg>>=1;
    i2cSetSlaveAdd(i2cREG1, slvAddr);   

    //Check for Bus Busy
    while ((i2cREG1->STR & I2C_BUSBUSY) )   
     //i2c->MDR |= I2C_STOP_COND;   //Send stop so SCL isn't held low
       i2cREG1->MDR = 0;		    //reset I2C so SCL isn't held low
    
    //Disable I2C during configuration
    i2cREG1->MDR= 0;
    uDelay(10);
    //Set the I2C controller to write len bytes
    i2cREG1->CNT=cnt+1;               //used when I2C_REPEATMODE bit is set
    
    //reset change this later

    i= (I2C_RESET_OUT | I2C_MASTER | I2C_TRANSMITTER | I2C_START_COND); //I2C_REPEATMODE
    i2cREG1->MDR= i;
    uDelay(20);

    //Transmit Slv Reg Select byte
    // Wait for "XRDY" flag to transmit data or "ARDY" if we get NACKed
    while ( !(i2cREG1->STR & (I2C_TX|I2C_ARDY)) )
            {
            uDelay(5);
            if (iTimout++> 500)
                {
                Timeouts_i2c++;
                return 1;
                }
            }
    
    
    // If a NACK occurred then SCL is held low and STP bit cleared
    if ( i2cREG1->STR & I2C_NACK )
        {
        i2cREG1->MDR = 0;		    //reset I2C so SCL isn't held low
        Timeouts_i2c++;
        return 1;                   //1==fail
        }
    i2cREG1->DXR= slvReg;           //slave chips register 

    uDelay(10);

    //Transmit data 
    for (i=0; i< cnt; i++)
        {
        // Wait for "XRDY" flag to transmit data or "ARDY" if we get NACKed
        iTimout= 0;
        while ( !(i2cREG1->STR & (I2C_TX|I2C_ARDY)) )
            {
            uDelay(5);
            if (iTimout++> 500)
                {
                Timeouts_i2c++;
                return 1;
                }
            }
        
        // If a NACK occurred then SCL is held low and STP bit cleared
        if ( i2cREG1->STR & I2C_NACK )
            {
            i2cREG1->MDR = 0;		    //reset I2C so SCL isn't held low
            Timeouts_i2c++;
            return 1;                   //1==fail
            }
        i2cREG1->DXR= dBlk[i];          //slave chip data to receive
        }
    //i2cREG1->MDR |= I2C_STOP_COND;      // Generate STOP
    //uDelay(5);
	//while(i2cREG1->STR & I2C_STOP_COND);

    //allow sequence finish, else future code may interfer
    //uDelay(10);                        //uS Delay    
    return 0;
}
 

//will hang here if optimized beyond 'LOW' 
//#pragma optimize=low

//receive block of data using RM48L seq logic
// in:  reg to read
//      read count
//      data ptr
// ret: 0=fail
//ack       --- active low level
//nack      --- active high level
//intrMode  --- intr mode flag for code early exit
//get data via interrupts
//
uint8_t i2cRecvData(uint8_t slvAddr, uint8_t slvReg, uint16_t rdcnt, uint8_t *dBlk, int intrMode)
{
    int iTimout=0;
    //Set the Destination Slave address 
    slvAddr >>=1;
    i2cSetSlaveAdd(i2cREG1, slvAddr);   
    
    //tek mar2018 use intr on ready, uses 'i2c->IMR' 
    //i2cEnableNotification(i2cREG1, I2C_ARDY_INT|I2C_RX_INT); //wait for access ready
    
    
    //Check for Bus Busy
    while ((i2cREG1->STR & I2C_BUSBUSY) )
       i2cREG1->MDR = 0;		        //reset I2C so SCL isn't held low
    
    //Disable I2C during configuration
    i2cREG1->MDR= 0;
    //Set the I2C controller to write len bytes
    i2cREG1->CNT=1;                     //used when I2C_REPEATMODE bit is set  
    //i2c Master mode Transmitter
    i2cREG1->MDR= (I2C_RESET_OUT| I2C_MASTER| I2C_TRANSMITTER| I2C_FREE_RUN| I2C_START_COND);

    //Transmit data 
    //Wait for "XRDY" flag to transmit data or "ARDY" if we get NACKed
    while ( !(i2cREG1->STR & (I2C_TX|I2C_ARDY)) )
        if (iTimout++> 500000)
            {
            Timeouts_i2c++;
            return 1;
            }
        
    //If a NACK occurred then SCL is held low and STP bit cleared
    if ( i2cREG1->STR & I2C_NACK )
        {
        i2cREG1->MDR = 0;		        //reset I2C so SCL isn't held low
        Timeouts_i2c++;
        return 1;                       //1==fail
        }
    i2cREG1->DXR= slvReg;  
    
    i2cEnableNotification(i2cREG1, I2C_ARDY_INT|I2C_RX_INT); //wait for access ready    
    return 1;
}



//poe port writes
uint8_t poePortWr(uint8_t dev, uint8_t addr, uint8_t cnt, uint8_t* data)
{
    int retVal=0;
    //i2cSendData (dev,reg,cnt,data[])
    retVal= i2cSendData(dev, addr, 1, (uint8_t*) data ); 
    return retVal;  //0=fail
}



//read one I2C register per pass, data updated via intr.
//callback every 10mS, to read one of 
//function time == 50 uSec
int i2c_rec_intr_mode()
{
    static int Port=0, Type=0;
    unsigned short *LEDPtr= &csLED1;
    static uint8_t* PtrVlt= (uint8_t *)POE_VLT_ALL;;
    static uint8_t* PtrCur = (uint8_t *)POE_CUR_ALL;
    static uint8 i2cData[4]; 
     
    //reset flag
    genFlag &= ~POE_CHECK_DUE;
    
    if(Port > (POECHsALL-1))
        {
        Port=0;
        PtrVlt = (uint8_t *)POE_VLT_ALL;
        PtrCur = (uint8_t *)POE_CUR_ALL;
        //update front panel LEDs
        //format 4 bits a single 16bit for each controlling frnt panel led driver '74LVC595APW'(6)
        for (int i=0,k,p=0; i<LED_DRIVERS; i++)
            {
            k=   fpLEDs_STORE[p++];
            k+=  fpLEDs_STORE[p++];
            k+=  fpLEDs_STORE[p++];
            k+=  fpLEDs_STORE[p++];
            *LEDPtr++= k;                   //set rj45 gang connector leds
          //fpLEDs_STORE[i]=k;              //store for later restore during spill gate
            }
        }
    
    //intr func updata structure 'g_i2cTransfer_t.length' and 'g_i2cTransfer_t.dataPtr' , .length set to zero
    g_i2cTransfer_t.length = 1;

    i2cData[0]= 0;
    if(Type==0)
        {
        //READ Voltage  READ LSB
        g_i2cTransfer_t.data= PtrVlt++;     //load data prt into .data for intr roution to use
        i2cRecvData(PoeVltsSlaveAdr[Port][0], PoeVltsSlaveAdr[Port][1], 1, &i2cData[0], 1);//use intr mode
        }
    else if(Type==1)
        {
        //READ Voltage  READ MSB
        g_i2cTransfer_t.data= PtrVlt++;     //load data prt into .data for intr roution to use
        i2cRecvData(PoeVltsSlaveAdr[Port][0], PoeVltsSlaveAdr[Port][2], 1, &i2cData[0], 1);//use intr mode;  
        }
    else if(Type==2)
        {
        //READ Current READ LSB
        g_i2cTransfer_t.data= PtrCur++;     //load data prt into .data for intr roution to use
        i2cRecvData(PoeCursChipAddr[Port][0], PoeCursChipAddr[Port][1], 1, &i2cData[0], 1 ); 
        
        //flag for rg45 LED on/off
        
        if (POE_VLT_ALL[Port] > 100)
      //if (*g_i2cTransfer_t.data > 10)
          fpLEDs_STORE[Port]= 1<<PoeVltsSlaveAdr[Port][3];  
        else
          fpLEDs_STORE[Port]= 0<<PoeVltsSlaveAdr[Port][3];
        }
    else 
        {
        //READ Current READ MSB
        g_i2cTransfer_t.data= PtrCur++;     //load data prt into .data for intr roution to use
        i2cRecvData(PoeCursChipAddr[Port][0], PoeCursChipAddr[Port][2], 1, &i2cData[0], 1 );
        }
 
    //reset change this later (I2C_REPEATMODE)
    //i2cREG1->MDR= (I2C_MASTER| I2C_FREE_RUN);
    /* Set the I2C controller to read len bytes */
    //i2cREG1->CNT=1;                 //used when I2C_REPEATMODE bit is set
    
 //tek Mar2018
 //i2cEnableNotification(i2cREG1, I2C_RX_INT);

    //'g_i2cTransfer_t.length'  == 0 when interrupt complete
    //'g_i2cTransfer_t.dataPtr' == stored data location
    if (++Type>3)
        {
        Type=0;
        Port++;
        }
    return 0;
}




//using fpga address mapped to led driver 74VC595AWP
int RG45LEDS(int leds)
{
    csLED1=leds;
    csLED2=leds;
    csLED3=leds;
    csLED4=leds;
    csLED5=leds;
    csLED6=leds;
    return 0;
}

