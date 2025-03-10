;-------------------------------------------------------------------------------
; dabort.asm
;
; Copyright (C) 2009-2016 Texas Instruments Incorporated - www.ti.com 
; 
; 
;  Redistribution and use in source and binary forms, with or without 
;  modification, are permitted provided that the following conditions 
;  are met:
;
;    Redistributions of source code must retain the above copyright 
;    notice, this list of conditions and the following disclaimer.
;
;    Redistributions in binary form must reproduce the above copyright
;    notice, this list of conditions and the following disclaimer in the 
;    documentation and/or other materials provided with the   
;    distribution.
;
;    Neither the name of Texas Instruments Incorporated nor the names of
;    its contributors may be used to endorse or promote products derived
;    from this software without specific prior written permission.
;
;  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
;  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
;  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
;  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
;  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
;  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
;  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
;  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
;  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
;  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
;  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
;
;
;

    section .text:CODE
    arm


;-------------------------------------------------------------------------------
; Run Memory Test

    import	custom_dabort
    public	_dabort
    

_dabort
		stmfd	r13!, {r0 - r12, lr}; push registers and link register on to stack

        ldr		r12, esmsr3			; ESM Group3 status register
        ldr		r0,  [r12]
        tst		r0,  #0x8			; check if bit 3 is set, this indicates uncorrectable ECC error on B0TCM
        bne		ramErrorFound
        tst		r0, #0x20			; check if bit 5 is set, this indicates uncorrectable ECC error on B1TCM
        bne		ramErrorFound2

noRAMerror
		tst		r0, #0x80			; check if bit 7 is set, this indicates uncorrectable ECC error on ATCM
		bne		flashErrorFound

		bl		custom_dabort		; custom data abort handler required
									; If this custom handler is written in assembly, all registers used in the routine
									; and the link register must be saved on to the stack upon entry, and restored before
									; return from the routine.

		ldmfd	r13!, {r0 - r12, lr}; pop registers and link register from stack
		subs	pc, lr, #8			; restore state of CPU when abort occurred, and branch back to instruction that was aborted

ramErrorFound
		ldr		r1, ramctrl			; RAM control register for B0TCM TCRAMW
		ldr		r2, [r1]
		tst		r2, #0x100			; check if bit 8 is set in RAMCTRL, this indicates ECC memory write is enabled
		beq		ramErrorReal
		mov		r2, #0x20
		str		r2, [r1, #0x10]		; clear RAM error status register

		mov		r2, #0x08
		str		r2, [r12]			; clear ESM group3 channel3 flag for uncorrectable RAM ECC errors
		mov		r2, #5
		str		r2, [r12, #0x18]	; The nERROR pin will become inactive once the LTC counter expires

		ldmfd	r13!, {r0 - r12, lr}
		subs	pc, lr, #4			; branch to instruction after the one that caused the abort
									; this is the case because the data abort was caused intentionally
									; and we do not want to cause the same data abort again.

ramErrorFound2
		ldr		r1, ram2ctrl		; RAM control register for B1TCM TCRAMW
		ldr		r2, [r1]
		tst		r2, #0x100			; check if bit 8 is set in RAMCTRL, this indicates ECC memory write is enabled
		beq		ramErrorReal
		mov		r2, #0x20
		str		r2, [r1, #0x10]		; clear RAM error status register

		mov		r2, #0x20
		str		r2, [r12]			; clear ESM group3 flags channel5 flag for uncorrectable RAM ECC errors
		mov		r2, #5
		str		r2, [r12, #0x18]	; The nERROR pin will become inactive once the LTC counter expires

		ldmfd	r13!, {r0 - r12, lr}
		subs	pc, lr, #4			; branch to instruction after the one that caused the abort
									; this is the case because the data abort was caused intentionally
									; and we do not want to cause the same data abort again.


ramErrorReal
		b		ramErrorReal		; branch here forever as continuing operation is not recommended

flashErrorFound
		ldr		r1, flashbase
		ldr		r2, [r1, #0x6C]		; read FDIAGCTRL register

		mov     r2, r2, lsr #16
		tst		r2, #5				; check if bits 19:16 are 5, this indicates diagnostic mode is enabled
		beq		flashErrorReal
		mov		r2, #1
		mov     r2, r2, lsl #8		
		
		str		r2, [r1, #0x1C]		; clear FEDACSTATUS error flag

		mov		r2, #0x80
		str		r2, [r12]			; clear ESM group3 flag for uncorrectable flash ECC error
		mov		r2, #5
		str		r2, [r12, #0x18]	; The nERROR pin will become inactive once the LTC counter expires

		ldmfd	r13!, {r0 - r12, lr}
		subs	pc, lr, #4			; branch to instruction after the one that caused the abort
									; this is the case because the data abort was caused intentionally
									; and we do not want to cause the same data abort again.


flashErrorReal
		b		flashErrorReal		; branch here forever as continuing operation is not recommended
		
esmsr3		dcd	0xFFFFF520
ramctrl		dcd	0xFFFFF800
ram2ctrl	dcd	0xFFFFF900
ram1errstat	dcd	0xFFFFF810
ram2errstat	dcd	0xFFFFF910
flashbase	dcd	0xFFF87000

    
	
	end
