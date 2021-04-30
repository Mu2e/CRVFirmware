
//tek Jan2015 save this file of passed used code for possible future reference

/*
                else if (!strcmp(tok, "T3"))                //test emif port
                   {
                    int j=2;
                    *(unsigned short *)  0x60000000= j;     //16 BIT
                    j= *(unsigned short*)0x60000000;        //16 BIT
                    j= *(unsigned short*)0x64000000;        //16 BIT
                    j= *(unsigned short*)0x68000000;        //16 BIT
                    break;
                    }
                else if (!strcmp(tok, "TPI"))               //FRAM test via spi3 cs3
                   {
                    uint16 txTestData[16] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10 };
                    uint16 rxTestData[16] = { 0 };
                                       
                    //This function transmits blocksize number of data from source buffer using polling method or intr mode.
                    sciEnableNotification(UART ,SCI_RX_INT | SCI_TX_INT);   
                    //uint32 spiTransmitData(spiBASE_t *spi, spiDAT1_t *dataconfig_t, uint32 blocksize, uint16 * srcbuff)
                    spiTransmitData(spiREG1, &dataconfig_FRAM, 2, txTestData);

                    //Initiate SPI1 Transmit and Receive through Polling Mode
                    spiTransmitAndReceiveData(spiREG1, &dataconfig_FRAM, 16, txTestData, rxTestData);
                    break;
                    }

                else if (!strcmp(tok, "T3"))               //test i2c loopback 
                   {
                    uint8 txTestData[16] = { 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F, 0x10 };
                    uint8 rxTestData[16] = { 0 };
                    uint8  *t_buff = &txTestData[0];
                    uint8  *r_buff = &rxTestData[0];
                    uint32 buf_size = bsize;
                    uint32 g_recWait=0;
                   
                    i2cSetMode(i2cREG1, I2C_MASTER);
                    // set i2c own address        
                    i2cSetOwnAdd(i2cREG1,own_add_w);  
                    // enable Loopback mode for self test.   
                    i2cEnableLoopback(i2cREG1);     
                    // Initiate Start condition for Transmission  
                    i2cSetStart(i2cREG1);

                    // send data packets          
                    while(buf_size--)
                        {
                       i2cSendByte(i2cREG1,*t_buff++);       

                       //Checks to see if the Rx buffer full flag is set, returns
                       //0 if flags not set otherwise will return the Rx flag itself.
                       while (i2cIsRxReady(i2cREG1)==0 )   //I2C_RX_INT==Flag
                         g_recWait++;
                       // receive data packets    
                       *r_buff++ = i2cReceiveByte(i2cREG1);  
                        }
                    break;
                    }
//_____________________________________________________________________________________________________
                else if (!strcmp(tok, "T5"))               //test I2C logic
                    {
                    uint8_t dBlk[4]= {0xaa, 0x55, 0xaa, 0x55};
                    uint8_t reg= 0x2;
                    sprintf (tBuf,"SET LEDs GRP(1of4) ON, ADDR 20-23\n\r");
                    putBuf(prt, tBuf,0);
                    
                    param1=arg_hex(&paramPtr,0);            //chip addr to read
                    if (param1 == 0) param1=0x20;
                    
                    param2=arg_hex(&paramPtr,2);            //reg ti read
                    
                   
                    i2cSendData(param1, reg, param2, &dBlk[0] );         //addr, register, cnt, data[]
                    break;
                    }
                    

//_____________________________________________________________________________________________________
                else if (!strcmp(tok, "T6"))                //test I2C logic 
                    {                                       //turn off rj45 leds, 'pre SI-5338 CHIP testing'
                    uint8_t dBlk[4]= {0, 0, 0, 0};
                    uint8_t reg= 0x2;
                    uint8_t slvAddr= 0x20, cnt=4;
                   
                    i2cSendData(slvAddr++, reg, cnt, &dBlk[0] );  //addr, reg, cnt, data[]
                    i2cSendData(slvAddr++, reg, cnt, &dBlk[0] );  //addr, reg, cnt, data[]
                    i2cSendData(slvAddr++, reg, cnt, &dBlk[0] );  //addr, reg, cnt, data[]
                    i2cSendData(slvAddr++, reg, cnt, &dBlk[0] );  //addr, reg, cnt, data[]
                    sprintf (tBuf,"SET All LEDs OFF\n\r");
                    putBuf(prt, tBuf,0);
                    break;
                    }
//_____________________________________________________________________________________________________
                else if (!strcmp(tok, "T7"))               //test I2C logic , READ RJ45 LED DRV CHIP TST CODE
                    {
                    static uint8_t dBlk[20];
                    int8_t slaveAddr, slaveReg= 2;
                    slaveAddr=arg_hex(&paramPtr,0x20);      //addr to reads at
                    param2=arg_hex(&paramPtr,2);            //numb of read to do
                    if(param2>20) param2=20;                //limit to array size

                    sprintf (Buf1500,"READ BLOCK OF DATA, 'T7 REG CNT'\n\r");
                    for(int j=0; j<4; j++,slaveAddr++)
                        {
                        i2cRecvData(slaveAddr, slaveReg, param2, &dBlk[0] );    //cnt, data[]
                        for (int i=0;i<param2;i++)
                            {
                            sprintf (tBuf,"i2c_addr 0x%X   D%1d  0x%X\n\r", slaveAddr, i,dBlk[i]);
                            strcat(Buf1500, tBuf);
                            }
                        }
                    putBuf(prt, Buf1500,0);
                    break;
                    }
                else if (!strcmp(tok, "T8"))               //timer testing, uDelay uses hardware timer
                    {
                    si5338_init();
                    break;
                    
                    int nPeriod;
                    nPeriod=arg_dec(&paramPtr,100);         //default 100
                    hetREG1->DSET=  BIT0;
                    uDelay(nPeriod);
                    hetREG1->DCLR=  BIT0;
                    nPeriod= PII_STAT;
                    gioToggleBit(spiPORT5, 19);                    
                    //gioToggleBit(spiPORT5, 27);   

                    spiPORT5->DSET=(uint32)1U << 27;
                    spiPORT5->DCLR=(uint32)1U << 27;
                    break;
                    }      
                else if (!strcmp(tok, "T9"))                //timer testing, uDelay uses hardware timer
                    {
                    int ret=1;
                    ret= i2c_write_5338(0xE6, 0x10);        //OEB_ALL = 1 (0xE6=230dec)
                    break;
                    }      
                else if (!strcmp(tok, "T10"))               //timer testing, uDelay uses hardware timer
                    {
                    int ret=1, counter;
                    for(int i=0; i<NUM_REGS_MAX ;i++)
                        {
                        si5338 = Reg_Store[counter++];
                        if(si5338.Reg_Mask == 0x00)                     //mask zero, skip
                            continue;
                          
                        ret= i2c_read_5338(i);   
                        sprintf (tBuf,"%2X ", ret);
                        putBuf(prt, tBuf,0);
                        if (i%16==0) 
                            {
                            sprintf(tBuf,"\r\nAddr %4d ",i);
                            putBuf(prt, tBuf, i);
                            }
                        }
                    break;
                    }
*/
