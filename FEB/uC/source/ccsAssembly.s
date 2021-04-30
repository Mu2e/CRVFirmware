;
;  @file ccsAssembly.s
;  Fermilab Terry Kiper 2016-2019
;  assembler routines for cc8 GNU compiler

.text .align 4
	.thumb

	.global MystrLen
	
; reg def
;	Input
;		R0 --addr of strign
;	Output
;		R0 --computed len of string
;	Working Regs
;		R5 --current string byte
;		R6 --length counter

MystrLen:
	PUSH	{R4-R11}
	MOV 	R6, #0
COUNT
	LDRB	R5, [R0,R6]
	CMP 	R5, #0
	BEQ 	EXIT
	ADD 	R6, #1
	B		COUNT
EXIT:
	MOV 	R0, R6
	POP 	{R4-R11}
	BX 		LR



		.global movStr16_NOICSRC
;	Input
;		R0--R4 first 4 passed params, no reg save req
;	Output
;		R0 --computed len of string
movStr16_NOICSRC:
NOICSRC
    LDRH 	R0,[R3]         ;r3=src
    STRH 	R0,[R1],#2    	;r1=dst, yes incr dst only

    SUBS 	R2, R2, #1    	;r2=cnt
    BGT.N	NOICSRC
	BX 		LR



		.global movStr16_NOICDEST
;	Input
;		R0--R4 first 4 passed params, no reg save req
;	Output
;		R0 --computed len of string
movStr16_NOICDEST:
NOICDEST
    LDRH 	R3,[R0],#2    	;r0=src, yes incr src only
    STRH 	R3,[R1]    		;r1=dst
    SUBS 	R2, R2, #1    	;r2=cnt
    BGT.N 	NOICDEST
	BX 		LR



		.global movStr16
;	Input
;		R0--R4 first 4 passed params, no reg save req
;	Output
;		R0 --computed len of string
movStr16:
mov16
    LDRH 	R3,[R0],#2
    STRH 	R3,[R1],#2

    SUBS 	R2, R2, #1
    BGT.N 	mov16
	BX 		LR



		.global movStr16_NOINC
;	Input
;		R0--R4 first 4 passed params, no reg save req
;	Output
;		R0 --computed len of string
movStr16_NOINC:
NOINC
    LDRH 	R0,[R3]         ;r3=src
    STRH 	R0,[R1]    		;r1=dst

    SUBS 	R2, R2, #1    	;r2=cnt
    BGT.N	NOINC
	BX 		LR




;Reverse bits order in register R0, return byte value is passed back
;to its caller in register R0.
		.global revB_byte
;	Input
;		R0--R4 first 4 passed params
;	Output
;		R0 --
revB_byte:
	RBIT 	R0, R0				;Asm lng rev bits in 32 register
	MOV 	R0, R0, LSR #24		;shift and return 24 bits in R0
	BX 		LR





		.global rev_Bits
;	Input
;		R0--R4 first 4 passed params
;	Output
;		R0 --
rev_Bits:
	RBIT 	R0, R0				;Asm lng rev bits in 32 register
	MOV 	R0, R0, LSR #16		;shift and return 16 bits in R0
	BX 		LR





		.global SwapByte
;	Input
;		R0--R4 first 4 passed params
;	Output
;		R0 --
SwapByte:
	rev16   r0, r0			;reverse R0 bytes
	BX 		LR





