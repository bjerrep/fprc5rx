;
; ========================================================================
; COMMON.INC
; Matching at least the processors 16F84 16C73 16F873
; Version history
; 001	Initial merge of old include files.
; ========================================================================
;
	nolist

FALSE				equ	0
TRUE				equ	1

false				equ	FALSE
true				equ	TRUE

BIT0      			equ     0
BIT1      			equ     1
BIT2      			equ     2
BIT3      			equ     3
BIT4    	  		equ     4
BIT5      			equ     5
BIT6      			equ     6
BIT7      			equ     7


DEBUG_BUILD     	equ     0
RELEASE_BUILD       equ     1


        ifdef BUILD_TARGET
          if BUILD_TARGET == DEBUG_BUILD
            messg "DEBUG BUILD"
          endif
          if BUILD_TARGET == RELEASE_BUILD
            messg "Release build"
          endif
        endif

		ifdef __16F877
            messg "Processor target is 16f877"
		endif
		ifdef __16F873
            messg "Processor target is 16f873"
		endif


CURRENT_BANK 	set	0
__bank 			set	0

CURRENT_PAGE 	set	0
__page			set	0


	; Unconditional bank setting macros.

SetBank0	macro
			bcf	STATUS, RP0
			bcf	STATUS, RP1
__bank		set	0
			endm


SetBank1	macro
			bsf	STATUS, RP0
			bcf	STATUS, RP1
__bank		set	1
			endm


SetBank2	macro
			bcf	STATUS, RP0
			bsf	STATUS, RP1
__bank		set	2
			endm

SetBank3	macro
			bsf	STATUS, RP0
			bsf	STATUS, RP1
__bank		set	3
			endm



		; -------------------------------------------------------------------
		; SetBankXAuto
		; -------------------------------------------------------------
		; Conditional bank setting macros based on current "__bank"
		; Use with care...

SetBank0Auto	macro
		if FILE_BANKS > 1
		if __bank == 1
			bcf	STATUS, RP0
		else
		   if __bank == 2
				bcf	STATUS, RP1
			else
			if __bank == 3
				bcf	STATUS, RP0
				bcf	STATUS, RP1
			endif
			endif
		endif
__bank		set	0
		endif
			endm


SetBank1Auto	macro				; Target 01
		if FILE_BANKS > 1
		if __bank == 0
			bsf	STATUS, RP0	; 00 -> 01
		else
			if __bank == 2		; 10 -> 01
				bsf	STATUS, RP0
				bcf	STATUS, RP1
			else
			if __bank == 3
				bcf	STATUS, RP1	; 11 -> 01
			endif
			endif
		endif
__bank		set	1
		endif
			endm


SetBank2Auto	macro				; Target 10
		if FILE_BANKS > 1
		if __bank == 0
			bsf	STATUS, RP1	; 00 -> 10
		else
			if __bank == 1		; 01 -> 10
				bcf	STATUS, RP0
				bsf	STATUS, RP1
			else
			if __bank == 3
				bcf	STATUS, RP0	; 11 -> 10
			endif
			endif
		endif
__bank		set	2
		endif
			endm


SetBank3Auto	macro				; Target 11
		if FILE_BANKS > 1
		if __bank == 0
			bsf	STATUS, RP0	; 00 -> 11
			bsf	STATUS, RP1
		else
		if __bank == 1		; 01 -> 11
			bsf	STATUS, RP1
		else
			if __bank == 2
				bsf	STATUS, RP0	; 10 -> 11
			endif
		endif
		endif
__bank		set	3
		endif
			endm



SetBnkA	        macro	REGISTER
CURRENT_BANK	set	(REGISTER & 0x180) / 0x80
		if CURRENT_BANK == 0 && __bank != 0
			SetBank0Auto
		endif
		if CURRENT_BANK == 1 && __bank != 1
			SetBank1Auto
		endif
		if CURRENT_BANK == 2 && __bank != 2
			SetBank2Auto
		endif
		if CURRENT_BANK == 3 && __bank != 3
			SetBank3Auto
		endif
			endm


SetBnkF	        macro	REGISTER
CURRENT_BANK	set	(REGISTER & 0x180) / 0x80
		if CURRENT_BANK == 0
			SetBank0
		endif
		if CURRENT_BANK == 1
			SetBank1
		endif
		if CURRENT_BANK == 2
			SetBank2
		endif
		if CURRENT_BANK == 3
			SetBank3
		endif
			endm



; -------------------------------------------------------------------
;	Program paging
; 	-------------------------------------------------------------
;	Remember interupt vector when using multiple pages !
;

		ifdef NOF_PROGRAM_PAGES

__PAGE__	equ	0x7FF


		if NOF_PROGRAM_PAGES == 2

SetPage0	macro
			bcf	PCLATH, BIT3
			bcf	PCLATH, BIT4
__page		set	0
			endm

SetPage1	macro
			bsf	PCLATH, BIT3
			bcf	PCLATH, BIT4
__page		set	1
			endm



SetPgeF		macro	Adress
__page		set	Adress / 0x800
		if __page == 0
			bcf	PCLATH, BIT3
			bcf	PCLATH, BIT4
		endif
		if __page == 1
			bsf	PCLATH, BIT3
			bcf	PCLATH, BIT4
		endif
			endm


SetPage0Auto	macro
		if __page != 0
		    bcf	PCLATH, BIT3
			bcf	PCLATH, BIT4
		endif
__page		set	0
			endm


		endif ; NOF_PROGRAM_PAGES
		endif


FORCE_7_BIT_ADR	equ	b'01111111'	; AND mask for register files in upper
								; banks, to silence assembler Message[302]




;	Interupt enable & disable
;	---------------------------------------------------
;	INTCON and STATUS exists in all pages

ei_clear	macro
			bcf	INTCON, T0IF
			bsf	INTCON, GIE
			endm

ei			macro
			bsf	INTCON, GIE
			endm


di			macro
			local	__di_fail__
__di_fail__	bcf	INTCON, GIE
			btfsc	INTCON, GIE
			goto	__di_fail__
			endm


di_fast		macro
			bcf	INTCON, GIE
			endm


		 ; Update the z flag from the register contents
		 ; Destroys W register
updatez		macro	register
			clrw
			addwf	register, W
			endm


jumpz		macro	if_z
			btfsc	STATUS, Z
			goto	if_z
			endm


		; setoption (OPTION REGISTER)
		; ---------------------------------------------------
		; b'76543210'
		;   7          	!RBPU Port B pull ups (1=disable)
		;    6		Ext. intr edge
		;     5         TMR0 source. 0=Internal
		;      4        Ext tmr edge
		;       3       Prescaler. 0=TMR0, 1=WDT
		;        210	Prescaler rate


setoption	macro
			SetBnkF	OPTION_REG
			movwf	OPTION_REG & FORCE_7_BIT_ADR
			SetBank0Auto
			endm

settris		macro	port
			SetBnkF	port
			movwf	port & FORCE_7_BIT_ADR
			SetBank0Auto
			endm

settrisbit	macro	trisport, bit
			SetBnkF	trisport
			bsf	trisport & FORCE_7_BIT_ADR, bit
			SetBank0Auto
			endm

cleartrisbit	macro	trisport, bit
			SetBnkF	trisport
			bcf	trisport & FORCE_7_BIT_ADR, bit
			SetBank0Auto
			endm



		; ZeroAllFileRegs
		; ---------------------------------------------------
		; Set all file cells in the current bank to zero.
		;
MZeroFileRegs	macro
			local	next_cell
			 ;
			movlw	RAM_BANK0_START
			movwf	FSR
			 ;
			movlw   RAM_BANK0_SIZE-1
			movwf	RAM_BANK0_LAST	; Adress of last cell
		 ;
next_cell	clrf	INDF
			incf	FSR, F
			decfsz	RAM_BANK0_LAST, F
			goto	next_cell
			;
			clrf	RAM_BANK0_LAST
			;
		ifdef ICE
			if ICE
			nop
			nop
			endif
		endif
			endm



		; ForcePageAlign
		; ---------------------------------------------------
		; Force the PC to start on next even block with an org statement.
		; Call and goto works in 2k blocks (11 bit) (0x0000-0x07FF, ...)
		; Computed goto in 256 bytes blocks (8 bit)

ForcePageAlign	macro	block_length
		if $/0x100 != (block_length-1+$)/0x100
			org	((block_length-1+$)/0x100 ) * 0x100
		endif
			endm




		; InlineAsciiHeader
		; ---------------------------------------------------
InlineAsciiHeader 	macro

		ifdef RELEASE
			if RELEASE
				data	" R e l e a s e  "
	      	else
				data	" D e b u g      "
	      	endif
		endif

		ifdef __16F84
			data	" 1 6 F 8 4      "
		endif

		ifdef __16C73A
			data	" 1 6 C 7 3      "
		endif

		ifdef __16F873
			data	" 1 6 F 8 7 3    "
		endif

		ifdef __16F877
			data	" 1 6 F 8 7 7    "
		endif

		ifdef XTAL
			data    " X t a l"
xtal_hi		equ	XTAL / .10000000
			data    '0' + xtal_hi
xtal_mill	equ     (XTAL - xtal_hi * .10000000) / .1000000
			data    '0' + xtal_mill
			data    '.'
			data    '0' + (XTAL - xtal_hi * .10000000 - xtal_mill * .1000000) / .100000
		endif


		ifdef VERSION
			data    " V e r  "
			data    '0' + VERSION / .100
decade		equ     (VERSION - .100*(VERSION / .100)) / .10
			data    '0' + decade
			data    '0' + (VERSION - .100*(VERSION/.100) - decade*.10)
			data	"  "
		endif


		ifdef SERIAL_NUMBER
			data    " S e r  "
			data    SERIAL_NUMBER + '0'
			data	"      "
		endif

		ifdef ICE
	      	if ICE == true
				data	" i c e b r k e r"
	      	endif
		endif

			endm



; -------------------------------------------------------------------
; 	FatalError
; 	-------------------------------------------------------------
;	Emulator : It is possible to force a return by clearing "FatalErrorFlags"
;       Dependencies : msDelay, LedOn, LedOff
;

FatalHalt	macro


; These can be located anywhere, they are only active once the system has entered infinite error loop.
FatalErrorMaster	equ	IntrContextS
FatalErrorCnt		equ	IntrContextS + .1
FatalErrorFlags		equ	IntrContextS + .2



FatalError

fatal_loop
			di
			movwf	FatalErrorMaster

			movlw	1
			movwf	FatalErrorFlags

fatal_forever

			movf	FatalErrorMaster, W
			movwf	FatalErrorCnt

anotherblink	LedOn
			movlw	.100
			call	msDelay

			LedOff
			movlw	.250
			call	msDelay

			decfsz	FatalErrorCnt, F
			goto	anotherblink

			movlw	.250
			call	msDelay
			movlw	.250
			call	msDelay
			movlw	.250
			call	msDelay
			movlw	.250
			call	msDelay
			movlw	.250
			call	msDelay

			if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
			endif	; -------------------------------------

			btfss	FatalErrorFlags, BIT0
			return

			goto	fatal_forever
			endm



		 ; PrescalerToWDT
		 ; ---------------------------------------------------
		 ; Does not modify prescaler value.
		 ; Prescaler MUST NOT BE 1:1 WHEN CALLING THIS !
		 ; From 33023a.pdf
		 ;
PrescalerToWDT	macro
			clrf	TMR0

			SetBank	OPTION_REG
			bsf	OPTION_REG & FORCE_7_BIT_ADR, BIT3
			clrwdt
			SetBank0Auto
			endm


		 ; PrescalerToTMR0
		 ; ---------------------------------------------------
		 ; Does not modify prescaler value.
		 ; From 33023a.pdf
		 ;
PrescalerToTMR0	macro
			clrwdt
			SetBank	OPTION_REG
			bcf	OPTION_REG & FORCE_7_BIT_ADR, BIT3
			SetBank0Auto
			endm


		list

