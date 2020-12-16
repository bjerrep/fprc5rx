;
; ========================================================================
; A U T O C A L C . I N C	               	   Claus Bjerre 2002-05-28
;
; Version history
; 001	Initial.
; ========================================================================
;


;;; Requires these constants to be defined in advance :

; XTAL                          Typ. fixed by design
; REQUESTED_ISR_FREQ_Hz         Typ, fixed by design
; MAX_ISR_TOLL_PROMILLE
; TMR0_PRESC_VAL                Adjust this to get a pass build (2,4,... if presc on TMR0)


; Warning suppressors
; NO_WARN_TMR0_PRESC_VAL        Quiet on non-fatal values


; The time per instruction
INSTRUCTION_FREQ_Hz		equ	XTAL / .4
INSTRUCTION_PERIOD_ns	equ	.1000000000 / INSTRUCTION_FREQ_Hz


        ifdef TMR0_PRESC_VAL


; The RTCC counter frequency in Hz
RTCC_FREQ_Hz			equ	XTAL / (TMR0_PRESC_VAL * .4)

RTCC_PERIOD_ns			equ	.1000000000 / RTCC_FREQ_Hz

; Free running ISR frequency (without RTCC reload in ISR part)
ISR_FREERUN_FREQ_Hz		equ	RTCC_FREQ_Hz / .256

		; Catch a too large TMR0 prescaler as it can make
		; it impossible to reach the requested ISR frequency
		if ISR_FREERUN_FREQ_Hz > REQUESTED_ISR_FREQ_Hz
			error "ERROR: RTCC counter too fast. Increase TMR0_PRESC_VAL"
		endif

; Calculate RTCC reload value to reach the "ISR_FREQ_Hz"
; Note that the RTCC is an upcounter
RTCC_RELOAD_256			equ	.256 * 0x100 - (.256 * 0x100 * ISR_FREERUN_FREQ_Hz) / REQUESTED_ISR_FREQ_Hz


RTCC_RELOAD				equ	RTCC_RELOAD_256 / 0x100

		if RTCC_RELOAD > 0x100
			error "ERROR: TMR0 reload overflow (Decrease TMR0_PRESC_VAL)"
        else
			if RTCC_RELOAD > .128 && TMR0_PRESC_VAL > .2
				ifndef NO_WARN_TMR0_PRESC_VAL
			    	messg "TIP: Decrease TMR0_PRESC_VAL"
			    endif
			endif
		endif


RTCC_SUB_LOAD			equ	(0x100 - RTCC_RELOAD)

; This is the actual ISR frequency. It should ideally equal the requested "ISR_FREQ_Hz".
; Used for an rudimentary test too detect if something went heywire above or if specs
; are not met.
ISR_FREQ_Hz				equ	RTCC_FREQ_Hz / (.256 - RTCC_RELOAD)


; The frequency of a freerunning 8 bit prescaler inside ISR.
ISR_FREQ_MOD_256_Hz		equ	ISR_FREQ_Hz / 0x100


ACTUAL_ISR_ERR_PROMILLE	equ	(.1000 * (ISR_FREQ_Hz - REQUESTED_ISR_FREQ_Hz)) / REQUESTED_ISR_FREQ_Hz

		; The multiplication is just to ensure positive figures
		ifdef MAX_ISR_TOLL_PROMILLE
			if ACTUAL_ISR_ERR_PROMILLE*ACTUAL_ISR_ERR_PROMILLE > MAX_ISR_TOLL_PROMILLE * MAX_ISR_TOLL_PROMILLE
  	    		messg "ERROR : ISR frequency out of tolerence"
	  		endif
		endif



ISR_FREQ_HIGH_Hz		equ	RTCC_FREQ_Hz / (.255 - RTCC_RELOAD)

ISR_FREQ_LOW_Hz			equ	RTCC_FREQ_Hz / (.257 - RTCC_RELOAD)

; Time between interupts.
ISR_PERIOD_ns			equ	(.1000000000 / ISR_FREQ_Hz )

ISR_PERIOD_MOD_256_us	equ	(ISR_PERIOD_ns * 0x100) / .1000

ISR_REQ_PERIOD_ns		equ	REQUESTED_ISR_FREQ_Hz / .1000000000

ISR_PERIOD_ERROR_ns		equ	ISR_REQ_PERIOD_ns - ISR_PERIOD_ns

; The number of instructions that is executed between interupts. (Assuming one instruction per clock)
; A basic problem is when the ISR code part eats up all available instructions.
INSTRUCTIONS_BETW_INTR	equ	ISR_PERIOD_ns / INSTRUCTION_PERIOD_ns

		; Try to detect a starved system.
		if  INSTRUCTIONS_BETW_INTR < .40
			messg "WARNING: System might be heavily loaded"
		endif




	nolist




;
;
;        ; ISR Example
;
;
;	; Context saving
;	; W register and status are saved in reversed order...
;	; ------------------------------------------------
;		movwf	IntrContextW		; Save W reg
;		swapf	STATUS, W
;		clrf	STATUS
;		movwf	IntrContextS		; Save swapped STATUS
;		movf	PCLATH, W
;		movwf	IntrContextP		; Save PCLATH
;		clrf	PCLATH
;
;		; By clearing timer flag here a next interupt can be
;		; registered while in the ISR pass here
;		bcf	INTCON, T0IF		; INTCON is in all banks
;
;		SetBank0
;
;		clrwdt
;
;		movlw   RTCC_SUB_LOAD
;		subwf	TMR0, F
;
;		skpnc
;		bsf	INTCON, T0IF    ; Don't miss the interupt..
;
;
;
;
;	; Interrupt exit
;	; -----------------------------------------------------------
;
;IntrExit
;
;		movf	IntrContextP, W
;		movwf	PCLATH				; Restore PCLATH
;		swapf	IntrContextS, W
;		movwf	STATUS				; Restore swapped STATUS
;		swapf	IntrContextW, F
;		swapf	IntrContextW, W		; Restore W (leaving STATUS)
;		retfie
;
;








;;;; ---------------------------------------------------------------------------
;;;; AUTOCALC.INC
;;;; INLINE CALCULATION OF RTCC RELOAD VALUE TO GET A GIVEN ISR FREQUENCY
;;;; ---------------------------------------------------------------------------
;;;;
;;;; 			---- MANUAL DEFINES ----
;
;; Select the wanted interupt frequency in Hz
;REQUESTED_ISR_FREQ_Hz	equ	.1000000000 / SAMPLE_PERIOD_ns
;			if REQUESTED_ISR_FREQ_Hz < 0
;			  messg "FATAL: REQUESTED_ISR_FREQ_Hz is negative"
;			endif
;
;ISR_REQ_PERIOD_ns	equ	SAMPLE_PERIOD_ns
;
;; Acceptable plus/minus tolerence in promille (1 percent = 10 !)
;MAX_ISR_TOLL_PROMILLE	equ	.20
;
;; Define the selected TMR0 prescaler value.
;; TMR0_PRESC_VAL = (2^(presc#+1) [presc#=0,1,2,3...] = 2,4,8,...
;TMR0_PRESC_VAL		equ	.8
;
;;;; 			---- CALCULATIONS ----
;
;		include "..\\utility\\autocalc.inc"



	list


        endif ;; TMR0_PRESC_VAL