#ifndef __demo1_het_h
#define __demo1_het_h

#define HET_v2 1
#define AID1_7

#include "std_nhet.h"

#define HET_L00_0	(e_HETPROGRAM0_UN.Program0_ST.L00_0)
#define pHET_L00_0  	0

#define HET_L01_0	(e_HETPROGRAM0_UN.Program0_ST.L01_0)
#define pHET_L01_0  	1



typedef union 
{ 
 	HET_MEMORY	Memory0_PST[2];
	struct
	{
		PCNT_INSTRUCTION L00_0;
		PCNT_INSTRUCTION L01_0;
	} Program0_ST; 

} HETPROGRAM0_UN;

extern volatile HETPROGRAM0_UN e_HETPROGRAM0_UN;

extern const HET_MEMORY HET_INIT0_PST[2];

#endif

