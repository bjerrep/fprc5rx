;
; ========================================================================
; R C 5 . A S M			               	   Claus Bjerre 2002-05-28
;
; Statemachine for decoding of Phillips RC5, Sony Sircs & Panasonic REC-80
; infrared remote signals.
; Originally a strict sampling RC5 decoder only, Sony and Panasonic have been
; added later leaving the implementation in a mess. Not suitable for futher
; development.
;
; Version history
; 001	Initial.
; ========================================================================
; ToDo : Sony code must be reversed
;

	nolist

	; -----------------------------------------------------------
	; Variables
	; -----------------------------------------------------------

IR_Variables		macro
					list

; External equates
; RC5_TICKS_PER_BIT
; RC5_TIMEOUT_PRESET
; RC5PORT, RC5PIN

; External defines
; IR_MIN_NOF_CODES



IRJumpVector		equ	next_ram + .0
RC5Counter			equ	next_ram + .1
RC5Low				equ	next_ram + .2   ; RC data recieved - lo
RC5High				equ	next_ram + .3   ; RC data recieved - hi
RC5Presc			equ	next_ram + .4


IRPrivateFlags		equ	next_ram + .5	; Private flags. Read-only access !
FIRST_BIPHASE_HALF	equ	 BIT0		; A First sample level
RC5_TIMEOUTTEST		equ	 BIT1		; B Test for timeout (in idle state)
									;   Set high after sampling complete
									;   Clear on timeout or next RC5 start
	; Used to detect whether a timeout or a abort should signal a "timeout"
	; in order to let the listener clear its outputs. This would be that the
	; last signal was "RC5_READY"
IR_DATA_SENT		equ	 BIT2		; F Set high when a sample is flagged
									;   Cleared on timeout or abort.
SONY_TEST			equ	 BIT3
IR_READY_PENDING	equ	 BIT4
;
RC5_TOGGLE_BIT		equ	 BIT6
RC5_PREV_TOGGLE_BIT	equ	 BIT7


IRPublicFlags		equ	next_ram + .6	; Public flags. Not used for state controls.
IP_P_FORMAT_L       equ	 BIT0
IP_P_FORMAT_H       equ	 BIT1
IR_TIMEOUT			equ	 BIT2		; C Set on timeout
IR_BUSY		        equ	 BIT3		; D Set when leaving idle.
									;   Cleared when IR code is sampled
IR_READY			equ	 BIT4		; E New data available
;
SECOND_LAST			equ	 BIT6


IRPrivateFlags_2	equ	next_ram + .7
IR_FORMAT_LO   		equ	 BIT0           ; IR_FORMAT must be rightadjusted (or code breaks)
IR_FORMAT_HIGH		equ	 BIT1           ; - 00 RC5 01 Sony 10 Panasonic 11 Chrashed..
;					equ	 BIT2
IR_NEW_CODE			equ	 BIT3


IRTimeoutHigh		equ	next_ram + .8
IRTimeoutLow		equ	next_ram + .9

IR_PrevLow			equ	next_ram + .10
IR_PrevHigh			equ	next_ram + .11

IR_PanaCount		equ	next_ram + .12

IR_PanaAdjVal		equ	next_ram + .13

IR_HitCounter		equ	next_ram + .14
 ;
 ; Remember to change next line if adding new entries !!!!!!!
 ;
RC5_RAM_USAGE		equ	IR_HitCounter - IRJumpVector + 1

			endm




SkipRC5Active	macro
		if IR_RECIEVER_ACTIVE_HIGH
			btfss	RC5PORT, RC5PIN
		else
			btfsc	RC5PORT, RC5PIN
		endif
			endm

SkipRC5Quiet	macro
		if IR_RECIEVER_ACTIVE_HIGH
			btfsc	RC5PORT, RC5PIN
		else
			btfss	RC5PORT, RC5PIN
		endif
			endm


; State flag events, letters refer to equates above. Not to scale
;
;     RC5 cod1        RC5 cod1         RC5 cod1        RC5 cod2       RC5 cod2       No code=Timeout
; ____xxxxxxxx________xxxxxxxx_________xxxxxxxx________xxxxxxxx_______xxxxxxxx_____________
;
;                     B=0     B=1      B=0     B=1     B=0     B=1    B=0     B=1     B=0	Private
;                     D=1     D=0      D=1     D=0     D=1     D=0    D=1     	                Private
;
;                                                              C=1                    C=1       Public
;                             E=1                                             E=1               Public
;
;
;                   |-|                     Silence. Active bits aborts the reception
;                     |--|                  DeadTime. Nothing happens at all here.
;                        |----------------| Timeout counter
;
;
; [Ref20]
;
;


; RC5 vs Sony

;                 0                 1               2                 3            RC5 bits from start

;                 0  agc   0.89   agc       2.67             4.44             6.22
;         |----------------|----------------|----------------|----------------|--  RC5 extended
;                 ^   ^         ^   ' ^         ^   '   ^        ^    '   ^
;                 hhhhhhhhh hhhhhhhh

;
;                 0  agc   0.89   agc       2.67             4.44             6.22
;         |----------------|----------------|----------------|----------------|--  RC5
;                 ^   ^         ^   ' ^         ^   '   ^        ^    '   ^
;                 hhhhhhhhh         hhhhhhhh
;
;                                   #A#B
;                                            #C
;
;                 0      "1"              2.4"0"3.0
;                 |-----------------------|-----|-----------------|-----------------     Sony
;                 ^   !         !    (^)     ^    ^     ^     ^     ^     ^
;
;                  hhhhhhhhhhhhhhhhhhhhhhh       hhhhhhhhhhhh      hhhhh
;
;
;                                                            #D
;
;                 0      "1"                        3.5 3.95
;                 |----------------------------------|---|------------------|---------|     Panasonic
;                 ^
;                             bias init                    a "1" = 1.8ms     a low = 0.9 ms
;                  ----------------------------------__________________-----_____-----
;                                                            ^   ^   ^   ^    ^    ^


IR_Timing	macro

		list

		list

; Statemachine starts with resolution of one RC5 tick = 1/8 bit = 0.221 msec.

; #A 1.77 msec. The first RC5_HALF pass here.
;       Will find a SONY_TEST == T and advance the timing by 1/8 RC5 bitperiod and sets SECOND_LAST = T
;
; #B 1.99 msec. Will find SECOND_LAST = T.
;       If valid RC5 then the missing 1/8 bitperiod from #A is added.
;       If invalid RC5 then init Sony sampling. Install a delay of #C - #B = 0.7 msec
;
;
; #C 2.7 msec. First sample in Sony mode.
;       If it fails (carrier) then goto Panasonic mode.
;
; #D 4.2 msec - Panasonic mode.



; Sampling Sony
SONY_SAMPLES_PER_BIT            equ	2
SONY_TO_SONY_RTCC_COUNTS		set	(.300000 / RTCC_PERIOD_ns) - 1

; Re #B. Used to get from RC5 (#B) to Sony (#C)
SONY_ALIGN_SAMPLES_PER_BIT      equ	2
SONY_ALIGN_RTCC_COUNTS	        set	(.350000 / RTCC_PERIOD_ns) - 1

        if SONY_ALIGN_RTCC_COUNTS >= 0x100 || SONY_TO_SONY_RTCC_COUNTS >= 0x100
         error "Sony overflow #0202"
        endif


; Sampling Panasonic (0x8B)
PANA_SAMPLES_PER_BIT            equ	2
PANA_TO_PANA_RTCC_COUNTS		set	((.435000/PANA_SAMPLES_PER_BIT) / RTCC_PERIOD_ns) - 1 + 2

; Re #D. Used to get from Sony (#C) to Panasonic (#D)
PANA_ALIGN_SAMPLES_PER_BIT      equ	4
PANA_ALIGN_RTCC_COUNTS	        set	((.1400000/PANA_ALIGN_SAMPLES_PER_BIT) / RTCC_PERIOD_ns) - 1

		if PANA_ALIGN_RTCC_COUNTS >= 0x100 || PANA_TO_PANA_RTCC_COUNTS >= 0x100
			error "Panasonic overflow #0211"
        endif





ZERO_TIME_ms		equ	.5
DEAD_TIME_ms		equ	.10
RC5_TIMEOUT_ms		equ	.113 - ZERO_TIME_ms - DEAD_TIME_ms
SONY_TIMEOUT_ms		equ	.65 - ZERO_TIME_ms - DEAD_TIME_ms
PANA_TIMEOUT_ms		equ	.120 - ZERO_TIME_ms - DEAD_TIME_ms



ZERO_PRESC			equ	4
ZERO_PASSES			equ	(.1000 * .8 * ZERO_TIME_ms) / (RC5_BIT_TIME_us * ZERO_PRESC)


DEAD_PASSES			equ	(.1000 * .8 * DEAD_TIME_ms) / RC5_BIT_TIME_us
DEAD_PRESC			equ	DEAD_PASSES

RC5_TIMEOUT_PASSES	equ	(.1000 * .8 * RC5_TIMEOUT_ms) / RC5_BIT_TIME_us		; Presc = 1
SONY_TIMEOUT_PASSES	equ	(.1000 * .8 * SONY_TIMEOUT_ms) / RC5_BIT_TIME_us	; Presc = 1
PANA_TIMEOUT_PASSES	equ	(.1000 * .8 * PANA_TIMEOUT_ms) / RC5_BIT_TIME_us	; Presc = 1
IDLE_PRESC			equ	1

ABORT_PRESC			equ	(.1000 * .8 * .15) / RC5_BIT_TIME_us

		;nolist


	if IR_DEBUG_SAMPLE_TIME
		messg "INFO: Remote decoder sample clock enabled on RC5DEBUGPORT"
	endif

	ifdef REMOTE_DECODER_TRAPS_ON
		messg "INFO: Remote trap code included"
	endif

		endm



	; -----------------------------------------------------------
	; Code
	; -----------------------------------------------------------


IR_IsrCode	macro


			list


			decfsz	RC5Presc, F
			goto	RC5_Ends

IR_mark		movlw	IR_mark / 0x100
			movwf	PCLATH
			 ;
			movf	IRJumpVector, W
			addwf	PCL, F
		 ;
IR_Zero

IR_PRE_IDLE		equ	$ - IR_Zero
				goto	IRPreIdle
IR_IDLE	        equ	$ - IR_Zero
				goto	IR_Idle
RC5_FIRST		equ	$ - IR_Zero
				goto	RC5_First
RC5_HALF		equ	$ - IR_Zero
				goto	RC5_Half
RC5_LAST		equ	$ - IR_Zero
				goto	RC5_Last
RC5_CHKSILENCE	equ	$ - IR_Zero
				goto	RC5_CheckSilence
RC5_DEADTIME	equ	$ - IR_Zero
				goto	RC5_DeadTime
SONY_INIT		equ	$ - IR_Zero
				goto	SonyInit
SONY_1			equ	$ - IR_Zero
				goto	Sony_1
SONY_2			equ	$ - IR_Zero
				goto	Sony_2
SONY_3			equ	$ - IR_Zero
				goto	Sony_3
PANA_INIT       equ     $ - IR_Zero
                goto    Pana_Init
PANA_1          equ     $ - IR_Zero
                goto    Pana1
PANA_ADJ        equ     $ - IR_Zero
                goto    Pana_Adj



                 ; IRPreIdle
                 ; --------------------------------------------------
                 ; Enables interupt on PORTB change, and then installs "IR_Idle"
                 ;
IRPreIdle
		if IR_mark / 0x0100 != IRPreIdle / 0x0100
			error "Page fault in RC5 #0294"
		endif


		 ; Install interrupt on port B change
		 ; Successor to dead time

			SkipRC5Quiet
			goto	IR_Ends

		 ; port B is now registered with an inactive level on IR input

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			 ; clear a bogus interupt flag and enable port b interrupt on change
			bcf		INTCON, RBIF
			bsf		INTCON, RBIE
			 ;
			movlw	IR_IDLE
			movwf	IRJumpVector

			bcf		IRPublicFlags, SECOND_LAST

		 ; IR_Idle
         ; --------------------------------------------------
         ;


IR_Idle

			btfss	INTCON, RBIF
			goto	no_b

			 ; We are here because of interupt on port b change

			bcf		INTCON, RBIF
			bcf		INTCON, RBIE

			movlw	RTCC_SUB_LOAD - RTCC_RELOAD / 2

		if RTCC_SUB_LOAD - RTCC_RELOAD / 2 > RTCC_SUB_LOAD
			error "RC5.ASM #0390. Sign error"
		endif

			movwf	TMR0
			bcf		INTCON, T0IF

no_b

			SkipRC5Active
			goto	test_timeout


		 ; Idle -> Start reception
		 ; ------------------------------
                 ;
		if IR_DEBUG_SAMPLE_TIME
			bsf		RC5DEBUGPORT, RC5DEBUGPIN       ; Starting reception
		endif

		 ;
			clrf	RC5Low	; ends as high byte after shifting

			bsf		IRPublicFlags, IR_BUSY
			bcf		IRPrivateFlags, RC5_TIMEOUTTEST
			bsf		IRPrivateFlags, SONY_TEST

			 ; 14 bits in RC5
			movlw	.14
			movwf	RC5Counter

			 ; The first biphase level in first start bit is per definition low.
			 ;
			bcf		IRPrivateFlags, FIRST_BIPHASE_HALF

			movlw   RC5_LAST
			movwf   IRJumpVector

            movlw   .3
            goto    IR_EndsSetTicks


		 ; Idle -> Test for timeout
		 ; ------------------------------
		 ;
test_timeout
			incf	RC5Presc, F	; Setting RC5Presc = 1

			 ; Break if not in timeout phase

			btfss	IRPrivateFlags, RC5_TIMEOUTTEST
			goto	RC5_Ends


			 ; If IR_READY_PENDING is true, then fire a RC5_READY
			 ; when client clears IR_TIMEOUT

			btfss	IRPrivateFlags, IR_READY_PENDING
			goto	no_pending

			btfsc	IRPublicFlags, IR_TIMEOUT
			goto	no_pending

		ifdef REMOTE_DECODER_TRAPS_ON
	        btfsc	IRPublicFlags, IR_READY
	        call    IR_TrapErrors
		endif

		ifdef REMOTE_DECODER_TRAPS_ON
	        btfsc	IRPrivateFlags, IR_DATA_SENT
	        call    IR_TrapErrors
		endif


			bsf		IRPrivateFlags, IR_DATA_SENT
			bsf		IRPublicFlags, IR_READY		        ; IR_READY = 1
			bcf		IRPrivateFlags, IR_READY_PENDING	; Clr:IR_READY_PENDING
no_pending

			 ; Timeout counter

			decfsz	IRTimeoutLow, F
			goto	RC5_Ends

			decfsz	IRTimeoutHigh, F
		 	goto	RC5_Ends

			 ; Timeout
			 ; -----------------

			bcf		IRPrivateFlags, RC5_TIMEOUTTEST

		if IR_DEBUG_SAMPLE_TIME
			nop
			bsf		RC5DEBUGPORT, RC5DEBUGPIN               ; Timeout
		endif


			 ; Send a timeout to client if data is "on"
			btfsc	IRPrivateFlags, IR_DATA_SENT
			bsf		IRPublicFlags, IR_TIMEOUT
			bcf		IRPrivateFlags, IR_DATA_SENT


			 ; Invalidate RC5 history
			 ;
			movlw	0xFF
		 	movwf	IR_PrevHigh

			goto	RC5_Ends




		; RC5_First
RC5_First	; --------------------------------------------------

		if IR_DEBUG_SAMPLE_TIME
			bsf		RC5DEBUGPORT, RC5DEBUGPIN
		endif


			 ;
			bcf		IRPrivateFlags, FIRST_BIPHASE_HALF
			SkipRC5Quiet
			bsf		IRPrivateFlags, FIRST_BIPHASE_HALF

			incf	IRJumpVector, F

			goto	IR_EndsSetTicksQuart




                 ; The time advanced S2 second biphase sampling jumps to Sony here
InstallSony
			btfss	IRPrivateFlags, SONY_TEST               ; ?
			goto	SonyAbort

			bcf		IRPrivateFlags, SONY_TEST
			 ;
			movlw	SONY_INIT
			movwf	IRJumpVector


            movlw   SONY_ALIGN_RTCC_COUNTS
            movwf   RtccReloadValue

            movlw   SONY_ALIGN_SAMPLES_PER_BIT
			goto	IR_EndsSetTicks



		; RC5_Half
RC5_Half	; --------------------------------------------------

			incf	IRJumpVector, F

			 ; If SONY_TEST==true then advamce timing for next sample
			 ;
			btfss	IRPrivateFlags, SONY_TEST
			goto	IR_EndsSetTicksQuart

			bsf		IRPublicFlags, SECOND_LAST
			movlw	1
			goto	IR_EndsSetTicks


		; RC5_Last
RC5_Last	; --------------------------------------------------
		; Verify that we have a biphase bit, and save value
		;

		if IR_DEBUG_SAMPLE_TIME
			bsf	RC5DEBUGPORT, RC5DEBUGPIN
		endif

			SkipRC5Quiet
			goto	first_on

			 ; Second biphase sample is off
			 ; 10 ?
			btfss	IRPrivateFlags, FIRST_BIPHASE_HALF
			goto	RC5_Abort
			 ; RC5 bit = 0
			clrc
			goto	level_ok


		 ; Second biphase sample is on
first_on
			 ; 01 ?
			btfsc	IRPrivateFlags, FIRST_BIPHASE_HALF
			goto	InstallSony		; It was a '11' -> try Sony
			 ; RC5 bit = 1
			setc

		 ; Ok, opposite phases
level_ok
			rlf		RC5Low, F
			rlf		RC5High, F

			 ; More bits ?

			decfsz	RC5Counter, F
			goto	rc5_next_bit


			 ; RC5 complete, all 14 bits home
			 ; ------------------------------
			 ;
			;bsf	IRPrivateFlags, RC5_FORMAT
			movlw   b'11111100'
			andwf   IRPrivateFlags_2, F

			;bsf	IRPublicFlags, IR_RC5_PROTOCOL

			 ; Save previous toggle bit in flags from old one
			 ;
			bcf		IRPrivateFlags, RC5_PREV_TOGGLE_BIT
			btfsc	IRPrivateFlags, RC5_TOGGLE_BIT
			bsf		IRPrivateFlags, RC5_PREV_TOGGLE_BIT

			 ; Save new toggle bit in flags
			 ;
			bcf		IRPrivateFlags, RC5_TOGGLE_BIT
			btfsc	RC5High, BIT4
			bsf		IRPrivateFlags, RC5_TOGGLE_BIT

			 ; Pack to 12 bit (move second start bit)
			 ;
			bcf		RC5High, BIT3
			btfsc	RC5High, BIT4
			bsf		RC5High, BIT3

			movlw	0x0F
			andwf	RC5High, F

			goto    InstallSilence



rc5_next_bit
			movlw	RC5_FIRST
			movwf	IRJumpVector
			 ;
			btfsc	IRPublicFlags, SECOND_LAST
			bcf		IRPrivateFlags, SONY_TEST

			movlw	SAMPLES_PER_BIT / 2
			btfsc	IRPublicFlags, SECOND_LAST
			addlw	1				; Fix timing, see #3. Next sample will be toggle bit
			bcf		IRPublicFlags, SECOND_LAST

			goto	IR_EndsSetTicks






InstallSilence

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			movlw	RC5_CHKSILENCE
			movwf	IRJumpVector

			movlw	ZERO_PASSES
			movwf	RC5Counter

			movlw	ZERO_PRESC
			goto	IR_EndsSetTicks



		; RC5_CheckSilence
		; --------------------------------------------------

RC5_CheckSilence

			SkipRC5Quiet
			goto	SilenceAbort

level_is_low
			decfsz	RC5Counter, F
			goto	rc5_next_silence


	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------




		 ; Silence okay. New IR code is accepted
		 ; -----------------------
	 	 ;

            movlw   b'11111100'
            andwf   IRPublicFlags, F

			movlw   b'00000011'
			andwf   IRPrivateFlags_2, W

			iorwf   IRPublicFlags, F




	 	 ; Figure out if this is a new code or a duplicate to the previous recieved code
	 	 ; ToDo : Include a test for different standards as well....
	 	 ;
            bsf     IRPrivateFlags_2, IR_NEW_CODE   ; take a wild guess - its new...

		 	movf	RC5Low, W
		 	subwf	IR_PrevLow, W
		 	skpz
		 	goto    code_is_new

		 	movf	RC5High, W
		 	subwf	IR_PrevHigh, W
		 	skpnz
            bcf     IRPrivateFlags_2, IR_NEW_CODE   ; no, it was not new...
code_is_new
             ; New or old code processing based on 'IR_NEW_CODE' flag
             ;
            movlw   IR_MIN_NOF_CODES
            btfsc   IRPrivateFlags_2, IR_NEW_CODE
            movwf   IR_HitCounter

             ; Wait for enough hits
			 ; -----------------------
             ;
            decfsz  IR_HitCounter, F
            goto    not_enough

			btfss	IRPrivateFlags, IR_DATA_SENT
			goto	JustStarted

			; This is a new code in a otherwise continous IR stream (there were no timeout)
			;
			bsf		IRPublicFlags, IR_TIMEOUT			; IR_TIMEOUT = 1
			bsf		IRPrivateFlags, IR_READY_PENDING	; IR_READY_PENDING = 1
			bcf		IRPrivateFlags, IR_DATA_SENT		; IR_DATA_SENT = 0
		             ;
			goto	SaveNewData

			 ; This is a plain new code (previous state was timeout)
JustStarted

			btfss	IRPrivateFlags, IR_DATA_SENT
			bsf		IRPublicFlags, IR_READY				; IR_READY = 1

		 	bsf		IRPrivateFlags, IR_DATA_SENT		; IR_DATA_SENT = 1
			bcf		IRPublicFlags, IR_BUSY				; IR_BUSY = 0

not_enough
			; IR_MIN_NOF_CODES is not received yet so no official new code is
			; flagged yet.

SaveNewData
			 ; Save new IR code in 'Prev' variables
			 ; ---------------------------------------------------
			 ; - New and repeat code branches join here.

		 	movf	RC5Low, W
		 	movwf	IR_PrevLow

		 	movf	RC5High, W
		 	movwf	IR_PrevHigh


			 ; Set timers for deadtime and timeout detection

PresetTimers
			bsf	IRPrivateFlags, RC5_TIMEOUTTEST

			 ; Preset dead time timer
			 ;
		 	movlw	RC5_DEADTIME
		 	movwf	IRJumpVector

			movlw	DEAD_PRESC
			goto	IR_EndsSetTicks


			 ; Prepare for next silence sample and exit
			 ;
rc5_next_silence
			movlw	ZERO_PRESC
			goto	IR_EndsSetTicks




		; RC5_DeadTime
		; --------------------------------------------------
RC5_DeadTime

		 ; Dead time complete - enter idle

            movlw   b'00000011'
            andwf   IRPrivateFlags_2, W

            skpnz
            goto    define_rc5

            sublw   2
            skpz
            goto    define_sony
            goto    define_pana


		;btfss	IRPrivateFlags, RC5_FORMAT
		;goto	define_sony


define_rc5
			movlw	(RC5_TIMEOUT_PASSES/0x100) + 1
			movwf	IRTimeoutHigh
			movlw	RC5_TIMEOUT_PASSES & 0xFF
			movwf	IRTimeoutLow
			goto	install_idle

define_sony
			movlw	(SONY_TIMEOUT_PASSES/0x100) + 1
			movwf	IRTimeoutHigh
			movlw	SONY_TIMEOUT_PASSES & 0xFF
			movwf	IRTimeoutLow
			goto	install_idle


define_pana
			movlw	(PANA_TIMEOUT_PASSES/0x100) + 1
			movwf	IRTimeoutHigh
			movlw	PANA_TIMEOUT_PASSES & 0xFF
			movwf	IRTimeoutLow



install_idle
			movlw	IR_PRE_IDLE
			movwf	IRJumpVector
			movlw	IDLE_PRESC
			goto	IR_EndsSetTicks




		 ; Exit points
		 ; -----------------------
		 ; A reception is terminated.

PanaAbort
			if ICE	; -------------------------------------
			movlw   1
	        nop
	        nop
			goto	IR_Abort
	       	endif	; -------------------------------------


RC5_Abort
			if ICE	; -------------------------------------
			movlw   2
			nop
			nop
			goto	IR_Abort
			endif	; -------------------------------------

SilenceAbort
			if ICE	; -------------------------------------
			movlw   3
	        nop
	        nop
			goto	IR_Abort
			endif	; -------------------------------------


FromSonyAbort
			if ICE	; -------------------------------------
			movlw   4
	        nop
	        nop
			goto	IR_Abort
			endif	; -------------------------------------




IR_Abort
			bcf		IRPublicFlags, IR_BUSY
			 ;
			movlw	IR_PRE_IDLE
			movwf	IRJumpVector

			 ; Send a timeout to client if data is "on"
			 ;
			btfsc	IRPrivateFlags, IR_DATA_SENT
			bsf		IRPublicFlags, IR_TIMEOUT			; IR_TIMEOUT = 1
			bcf		IRPrivateFlags, IR_DATA_SENT		; IR_DATA_SENT = 0

			 ; Invalidate history
			 ;
			movlw	0xFF
		 	movwf	IR_PrevHigh

			 ;
			 ; Skip RC5 part for half a RC5 period
			 ;
			movlw	ABORT_PRESC
			goto	IR_EndsSetTicks

SonyAbort
		 ;
			movlw	RTCC_SUB_LOAD
			movwf	RtccReloadValue

			goto	FromSonyAbort



                 ;;; -----------------------------------------------
                 ;;;
                 ;;; S O N Y
                 ;;;
                 ;;; -----------------------------------------------



		 	; Sony starts
SonyInit	; Sampling off level from 2.4 to 3.0 msec

		if IR_DEBUG_SAMPLE_TIME
			bsf	RC5DEBUGPORT, RC5DEBUGPIN
		endif

			SkipRC5Quiet
			goto	InitPanasonic   ; SonyAbort

			movlw	SONY_TO_SONY_RTCC_COUNTS
			movwf	RtccReloadValue

			movlw	SONY_1
			movwf	IRJumpVector

			 ; Sony will be 12 or 15.
			movlw	.15
			movwf	RC5Counter

			clrf	RC5Low	; ends as high byte after shifting

			goto	Sony_Ends



Sony_1
		if IR_DEBUG_SAMPLE_TIME
			bsf	RC5DEBUGPORT, RC5DEBUGPIN
		endif

			 ; This is the special test as an invalid level can be that this was
			 ; an 12 bit transmission (and not 15)

			SkipRC5Quiet
			goto	SonyStartOK

			 ; Failed to have active start bit.
			 ; Check if data should be accepted as an 12 bit code
			 ;
			 ;
			movlw	.3		; .15 - .12
			subwf	RC5Counter, W
			 ;
			skpnz
			goto	SonyRxComplete
			 ;
			 ;
			goto	SonyAbort	; Not 12 bits -> abort


SonyStartOK
			movlw	SONY_2
			movwf	IRJumpVector
			goto	Sony_Ends



Sony_2
		if IR_DEBUG_SAMPLE_TIME
			bsf	RC5DEBUGPORT, RC5DEBUGPIN
		endif

			SkipRC5Quiet
			goto	SonyHigh

			 ; "0"

			movlw	SONY_1
			movwf	IRJumpVector

			clrc
			rlf		RC5Low, F
			rlf		RC5High, F

			decfsz	RC5Counter, F
			goto	Sony_Ends
			 ;
			goto	SonyRxComplete

SonyHigh
			 ; Sony format
			;bcf	IRPrivateFlags, RC5_FORMAT
			bsf		IRPrivateFlags_2, IR_FORMAT_LO
			bcf		IRPrivateFlags_2, IR_FORMAT_HIGH

			;bcf	IRPublicFlags, IR_RC5_PROTOCOL

			movlw	SONY_3
			movwf	IRJumpVector

			goto	Sony_Ends





Sony_3
		if IR_DEBUG_SAMPLE_TIME
			bsf	RC5DEBUGPORT, RC5DEBUGPIN
		endif


			SkipRC5Quiet
			goto	SonyAbort

			 ; "1"

			movlw	SONY_1
			movwf	IRJumpVector

			setc
			rlf	RC5Low, F
			rlf	RC5High, F

			decfsz	RC5Counter, F
			goto	Sony_Ends
                 ;
SonyRxComplete   ;
                 ;
			movlw	RTCC_SUB_LOAD
			movwf	RtccReloadValue
			goto	InstallSilence



                 ;;; -----------------------------------------------
                 ;;;
                 ;;; P A N A S O N I C
                 ;;;
                 ;;; -----------------------------------------------

InitPanasonic

			movlw	PANA_ALIGN_RTCC_COUNTS
			movwf	RtccReloadValue

			movlw	PANA_INIT
			movwf	IRJumpVector

			movlw	.49
			movwf	RC5Counter

			clrf	RC5Low	; ends as high byte after shifting

			;bcf	IRPrivateFlags, RC5_FORMAT

			clrf    IR_PanaCount

			movlw   PANA_ALIGN_SAMPLES_PER_BIT
			goto    IR_EndsSetTicks


Pana_Init

            clrf    IR_PanaAdjVal

			movlw	PANA_1
			movwf	IRJumpVector

			movlw	PANA_TO_PANA_RTCC_COUNTS
			movwf	RtccReloadValue

                 ; Fall through


                 ; "Pana1"
                 ; --------------------
Pana1
		if IR_DEBUG_SAMPLE_TIME
			bsf	RC5DEBUGPORT, RC5DEBUGPIN
		endif


            negf    IR_PanaAdjVal, W
            addwf   RtccReloadValue, F      ;
            clrf    IR_PanaAdjVal


			SkipRC5Quiet
            goto    PanaCarrier

            incf    IR_PanaCount, F

            goto    PanaEnds


PanaCarrier
            movlw   1
            subwf   IR_PanaCount, W
            skpz
            goto    no_double

            setc
            goto    load_pana

no_double
            movlw   .3
            subwf   IR_PanaCount, W
            skpz
            goto    PanaAbortTest

            clrc

load_pana

            decfsz  RC5Counter, F
            goto    PanaInitAdj

			if ICE	; -------------------------------------
			nop		; Breakpoint : Panasonic, all bits loaded
			nop
			endif	; -------------------------------------

                 ; Define Panasonic
                 ;
			bcf		IRPrivateFlags_2, IR_FORMAT_LO
			bsf		IRPrivateFlags_2, IR_FORMAT_HIGH
	                 ;
			movlw	RTCC_SUB_LOAD
			movwf	RtccReloadValue
			 ;
			goto	InstallSilence


PanaAbortTest
            nop
            nop
            goto    PanaAbort



                 ; "PanaInitAdj"
                 ; --------------------
PanaInitAdj

			rlf	RC5Low, F
			rlf	RC5High, F

            clrf    IR_PanaCount

			movlw	PANA_ADJ
			movwf	IRJumpVector

			movlw   1
            goto    IR_EndsSetTicks



                 ; "Pana_Adj"
                 ; --------------------
                 ; Samples the IR where it is supposed to go from "high" to "low" and adjusts the
                 ; sampling period accordingly. (note that there is no 'good enough' detection !)
                 ; If signal is high then the period should be increased -> Rtcc count increased
Pana_Adj

            movlw   -15
			SkipRC5Quiet
			movlw   .15

            movwf   IR_PanaAdjVal

            addwf   RtccReloadValue, F      ; Signal "high" -> Slow down

			movlw	PANA_1
			movwf	IRJumpVector

			movlw   1
            goto    IR_EndsSetTicks





		ifdef REMOTE_DECODER_TRAPS_ON
IR_TrapErrors
			nop
			nop
			return

			movlw   FATAL_TRAP
			goto    FatalStop
		endif



IR_Ends
			movlw	1
			movwf	RC5Presc
			goto	RC5_Ends



IR_EndsSetTicksQuart
			movlw	SAMPLES_PER_BIT / 4

IR_EndsSetTicks
			movwf	RC5Presc
			goto	RC5_Ends


PanaEnds
	        movlw   PANA_SAMPLES_PER_BIT
			movwf	RC5Presc
			goto    RC5_Ends

Sony_Ends
			movlw	SONY_SAMPLES_PER_BIT
			movwf	RC5Presc


RC5_Ends

		endm





		LIST















