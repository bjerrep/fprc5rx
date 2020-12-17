;
; ========================================================================
; P I C D E L A Y . I N C
; ------------------------------------------------------------------------
;
; Blocking delay macro where the timing are adjusted for xtal frequency.
;
; A XTAL definition must exist for processor frequency in MHz in order to
; make the timing calculations. The watchdog timer is kept cleared.
; The macro(s) are not intended for inlining, but should be made as subrutines.
; (i.e with a label before and a return statement after the macro)
; NOTE : If rom space is an issue then this macro won't do it better....
; ToDo : Not especially well checked for accuracy, but at least it do tend
; to make things slower.
;
; Version history
; 001  2004-08-19  Initial version.
; 002
; ========================================================================
;
	nolist


; Tstate = 4 * 1/fosc
; Fosc MHz 	    20	15	13	12	10	8	4
; Tstate ns 	200	267	308	333	400	500 1000


    ifndef XTAL
        error "Crystal frequency must be defined ('XTAL')"
    endif

STATE_TIME_ns   equ     .1000000 / (XTAL / .4000)
STATES_PER_4us  set     .4000 / STATE_TIME_ns

        if STATES_PER_4us <= 3
			error "Processor speed out of range"
        endif


    ; -------------------------------------------------------------------
    ; MDelay_4us_sequence
    ; -------------------------------------------------------------------
    ; Adapts to xtal frequency by inserting 'goto $+1' statements as needed.

MDelay_4us_sequence  macro

            clrwdt
STATES_PER_4us  set     STATES_PER_4us - 4   	; one for clrwdt, three for decfsz


            WHILE STATES_PER_4us >= 2
            goto $+1
STATES_PER_4us  set     STATES_PER_4us - 2
            ENDW

            if STATES_PER_4us == 1
            clrwdt
STATES_PER_4us  set     STATES_PER_4us - 1
            endif

            if STATES_PER_4us != 0
                error "Delay did not compute"
            endif

            endm



    ; -------------------------------------------------------------------
    ; MDelay_Xms
    ; -------------------------------------------------------------------
    ; Args	  : The resolution in millisecs (1-255) and three file registers
    ; Input   : W = Number of passes, the delay will be 'W*resolution_ms' millisecs.
	;
MDelay_Xms	MACRO   resolution_ms, va1, va2, va3

		    local   lop1, lop2, lop3

		    movwf   va1

lop3		movlw	resolution_ms
		    movwf	va3

lop1        ; Loop the next 4 us loop 250 times = 1 msec
	    	movlw	.250
	    	movwf   va2

lop2		MDelay_4us_sequence
		    decfsz  va2, F
		    goto    lop2

		    decfsz	va3, F
		    goto	lop1

		    decfsz  va1, F
		    goto    lop3

   			ifdef ICE	; -------------------------------------
			nop			; In Circuit Emulator Breakpoint
			nop			;
			endif		; -------------------------------------

		    ENDM

	list
