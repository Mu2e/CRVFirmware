//////////////////////////////////////////////////////////////////////
//
// File:      Example2.c
//
// Purpose:
//    ZestETM1 Example Programs
//    Top level function and platform specific functions
//
// Copyright (c) 2013 Orange Tree Technologies.
// May not be reproduced without permission.
//
//////////////////////////////////////////////////////////////////////

//#include <xparameters.h>
//#include <xintc.h>
#include "ZestETMI_SOCKET.h"
//#include "HTTP.h"

//////////////////////////////////////////////////////////////////////////////
// Platform specific function definitions
// These must be re-written for your platform
//

// Interrupt handling
// Need to call SocketISR when a GigExpedite interrupt occurs
// Also require enable and disable routines for the GigEx interrupt
static XIntc IntController;
void InterruptDisable(void)
{
	// Disable interrupts from the GigExpedite device
    XIntc_Disable(&IntController, XPAR_XPS_INTC_0_GIGEXPEDITE8_0_ETHERNETIRQ_INTR);
}
void InterruptEnable(void)
{
	// Enable interrupts from the GigExpedite device
    XIntc_Enable(&IntController, XPAR_XPS_INTC_0_GIGEXPEDITE8_0_ETHERNETIRQ_INTR);
}
void InterruptService(void *Arg)
{
	// Function called when interrupt generated by GigExpedite
	SocketISR();
	XIntc_Acknowledge(&IntController, XPAR_XPS_INTC_0_GIGEXPEDITE8_0_ETHERNETIRQ_INTR);
}
void InterruptConnect(void)
{
	// Connect ISR to interrupt from GigExpedite and enable interrupts on CPU
	XIntc_Initialize(&IntController, XPAR_XPS_INTC_0_DEVICE_ID);
	XIntc_Connect(&IntController, XPAR_XPS_INTC_0_GIGEXPEDITE8_0_ETHERNETIRQ_INTR,
				  InterruptService, NULL);
	XIntc_Start(&IntController, XIN_REAL_MODE);
	XIntc_Acknowledge(&IntController, XPAR_XPS_INTC_0_GIGEXPEDITE8_0_ETHERNETIRQ_INTR);
	microblaze_enable_interrupts();
	InterruptEnable();
}

// Read/write GigExpedite registers
// Addr is the GigExpedite register address which should be memory mapped into
// the processor address space.  In this example, the 8 bit registers are mapped
// to 32 bit locations with a spacing of 4 bytes.
void GigExWriteReg(unsigned long Addr, unsigned char Data)
{
	*((volatile unsigned long *)(XPAR_GIGEXPEDITE8_0_MEM0_BASEADDR+Addr*4)) = Data;
}
unsigned char GigExReadReg(unsigned long Addr)
{
	unsigned long Val = *((volatile unsigned long *)(XPAR_GIGEXPEDITE8_0_MEM0_BASEADDR+Addr*4));
	return Val;
}

// Return current time in ms
unsigned long GetTime(void)
{
	// This is used to timeout various socket functions
	// You can disable the timeouts by just returning 0 from this function
	return 0;
}


//////////////////////////////////////////////////////////////////////////////
// Entry point
//
int main(int argc, char **argv)
{
	// Initialise socket library
	SocketInit();

	// Initialise interrupts
	InterruptConnect();

	// Enter main control loop
    HTTPInit();

    while(1);

	return 0;
}


