#ifndef __nHetCap_h
#define __nHetCap_h

#define HET_v2 1
#define AID1_7

#include "std_nhet.h"

#define HET_START_0	(e_HETPROGRAM0_UN.Program0_ST.START_0)
#define pHET_START_0  	0

#define HET_hetlabel_2_0	(e_HETPROGRAM0_UN.Program0_ST.hetlabel_2_0)
#define pHET_hetlabel_2_0  	1

#define HET_DISCOUNTER_0	(e_HETPROGRAM0_UN.Program0_ST.DISCOUNTER_0)
#define pHET_DISCOUNTER_0  	2

#define HET_OFFSTATE_0	(e_HETPROGRAM0_UN.Program0_ST.OFFSTATE_0)
#define pHET_OFFSTATE_0  	3

#define HET_TEMPL_0	(e_HETPROGRAM0_UN.Program0_ST.TEMPL_0)
#define pHET_TEMPL_0  	4

#define HET_TEMPH_0	(e_HETPROGRAM0_UN.Program0_ST.TEMPH_0)
#define pHET_TEMPH_0  	5

#define HET_L00_0	(e_HETPROGRAM0_UN.Program0_ST.L00_0)
#define pHET_L00_0  	6

#define HET_L01_0	(e_HETPROGRAM0_UN.Program0_ST.L01_0)
#define pHET_L01_0  	7

#define HET_OFFSTATE1_0	(e_HETPROGRAM0_UN.Program0_ST.OFFSTATE1_0)
#define pHET_OFFSTATE1_0  	8

#define HET_ENCOUNTER_0	(e_HETPROGRAM0_UN.Program0_ST.ENCOUNTER_0)
#define pHET_ENCOUNTER_0  	9

#define HET_hetlabel_14_0	(e_HETPROGRAM0_UN.Program0_ST.hetlabel_14_0)
#define pHET_hetlabel_14_0  	10

#define HET_TIMERCOMP_0	(e_HETPROGRAM0_UN.Program0_ST.TIMERCOMP_0)
#define pHET_TIMERCOMP_0  	11



typedef union 
{ 
 	HET_MEMORY	Memory0_PST[12];
	struct
	{
		MOV32_INSTRUCTION START_0;
		ECMP_INSTRUCTION hetlabel_2_0;
		MOV32_INSTRUCTION DISCOUNTER_0;
		BR_INSTRUCTION OFFSTATE_0;
		PCNT_INSTRUCTION TEMPL_0;
		PCNT_INSTRUCTION TEMPH_0;
		PWCNT_INSTRUCTION L00_0;
		BR_INSTRUCTION L01_0;
		BR_INSTRUCTION OFFSTATE1_0;
		CNT_INSTRUCTION ENCOUNTER_0;
		MOV32_INSTRUCTION hetlabel_14_0;
		ECMP_INSTRUCTION TIMERCOMP_0;
	} Program0_ST; 

} HETPROGRAM0_UN;

extern volatile HETPROGRAM0_UN e_HETPROGRAM0_UN;

extern const HET_MEMORY HET_INIT0_PST[12];

#endif

