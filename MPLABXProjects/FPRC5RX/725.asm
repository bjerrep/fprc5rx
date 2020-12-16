;-----------------------------------------------------------------------
; 725.asm MkII
; 16 Channel Field Programmable RC5 Reciever
;-----------------------------------------------------------------------
; 001 06-06-02 00:31    Initial version
; 034 29-07-03 22:50    BiToggle and radio mode bugfixes. www release.
;
; Bugs :
; #001 02/12/04	Status : OPEN
;				If power is removed while a monostable is high it will be
;				recovered at power up.
; #002          OPEN
;				Check that all i/o changes get a delayd save request
;


true		        		equ	1
false		        		equ	0

        ;;; Main build constants
        ;;; -------------------------------------------------------------
VERSION		        		equ	.036		; Version id
XTAL		        		equ	.20000000	; in Hz

RELEASE		        		equ	1
STD_ICEBREAKER				equ	0
SINGLE_PIN_ICEBREAKER		equ	0

IR_RECIEVER_ACTIVE_HIGH		equ	0
MODE_TABLE_CONF				equ	.1

DELAYED_SAVE				equ	1			; FIXIT : Make permanent
DELAYED_COUNT				equ	.100

DELAYED_SETS				equ	1

	if RELEASE == true
			messg "INFO: Release build"
	ifdef __16F877
        list p = 16f877
	endif
	ifdef __16F873
        list p = 16f873
	endif
			
ICE							set	false
RC5_NO_ERROR_CHECK			equ	false
DEBUG_ISR_ALIVE				equ	false
IR_DEBUG_SAMPLE_TIME		equ	false
IR_MIN_NOF_CODES        	equ .2
FLASH_TABLE					equ	1
STANDARD_PINS				equ	1
ALTERNATIVE_PINS			equ	0
SINGLE_PIN					equ	0
	endif


	if STD_ICEBREAKER == true
        	messg "INFO: Debug ICE build"
			list p = 16f877
ICE							set	true
SERCOM_DEBUG_ENABLED		set	false
RC5_NO_ERROR_CHECK			equ	false
DEBUG_ISR_ALIVE				equ	false		; Toggles led at each pass -> led should run at 50:50
IR_DEBUG_SAMPLE_TIME		equ	false		; Pulses at IR sample times for scope (I think)
IR_MIN_NOF_CODES        	equ .2
FLASH_TABLE					equ	1
ALTERNATIVE_PINS			equ	1			; Completely in the wild
#define 					REMOTE_DECODER_TRAPS_ON
	endif

	; SINGLE_PIN_ICEBREAKER
	; ---------------------------------------
	if SINGLE_PIN_ICEBREAKER == true
        	messg "INFO: Debug ICE build"
			list p = 16f873
ICE							set	true
SERCOM_DEBUG_ENABLED		set	false
RC5_NO_ERROR_CHECK			equ	false
DEBUG_ISR_ALIVE				equ	false		; Toggles led at each pass -> led should run at 50:50
IR_DEBUG_SAMPLE_TIME		equ	false		; Pulses at IR sample times for scope (I think)
IR_MIN_NOF_CODES        	equ .2
FLASH_TABLE					equ	1
STANDARD_PINS				equ	0
ALTERNATIVE_PINS			equ	0			; Completely in the wild
SINGLE_PIN					equ	1			; Completely in the wild
#define 					REMOTE_DECODER_TRAPS_ON
	endif



		ifdef __16F873
			include "P16F873.inc"
		endif
		ifdef __16F877
			include "P16F877.inc"
		endif


         ; Custom include files
         ;
			include	"PICDELAY.inc"
NOF_PROGRAM_PAGES	equ	2
			include	"COMMON.inc"
			include "16F873.inc"


         	; PIC configuration settings

			__CONFIG	_HS_OSC & _WDT_ON & _PWRTE_ON & _WRT_ENABLE_ON & _DEBUG_OFF & _LVP_OFF & _BODEN_OFF


			; Custom macro libraries
			; ----------------------
			;

			include	"RC5.asm"

			; i2c
			; ----------------------
			;
			include "I2C_DEFS.inc"
#define I2C_MASTER_ASYNC


;;;-----------------------------------------------------------------------
;;; M E M O R Y  M A P   -   R A M
;;;-----------------------------------------------------------------------
;;; Words are low byte at low adress


ram						set	RAM_BANK0_START
t1						equ	ram + .0
t2						equ	ram + .1
t3						equ	ram + .2

RealtimeTickCounter    	equ	ram + .3

Flags					equ	ram + .4
IR_WAITING_FOR_TIMEOUT	equ	 BIT0		; RT. High whenever a pin goes high to disable
										; scanning of a user programming request
PROGRAM_MODE_ON    		equ  BIT1		; Can only be cleared with a reset.
WAITING_FOR_DEBOUNCE	equ	 BIT2
EXTENDED_PIN_SET		equ	 BIT3
LedFlags				equ	ram + .4
LED_ON					equ	 BIT4
LED_CLOSING				equ	 BIT5

Counter					equ	ram + .5

	; OutputsMask32
	; The source for the actual state of the output pins. Diffences between
	; this register and the physical outputs are regarded as manual user inputs.
	; This register is logically organized as the pins 0-31 from lsb to msb.
	; Restored at boot time with 'ReadTableOutputLevels()'
OutputsMask32			equ	ram + .6	; Quad 6..9. 4 bytes, std. pins 1-16 and extended 17-32

LedPreset				equ	ram + .10
LedPrescaler			equ	ram + .11
ISRSlowPrescalerCnt		equ	ram + .12

Read_Low				equ	ram + .13	; Used by rutines in "Rc5Util.asm"
Read_High				equ	ram + .14

TempIndex				equ	ram + .15	; Realtime scope
Pin						equ	ram + .16

NewIrLowByte			equ	ram + .17
NewIrHighByte			equ	ram + .18

ShiftTempQuad			equ	ram + .19	; Quad. 19..22

RadioPinActivated		equ	ram + .23	; Storage for the pin worked on in activate (for deactivate)
RadioPrescaler			equ	ram + .24

TablePointer			equ	ram + .25
Write_Low				equ	ram + .26
Write_High				equ	ram + .27

Write_Conf_Mode			equ	ram + .28
Read_Conf_Mode			equ	ram + .29
Write_Conf_Pin			equ	ram + .30
Read_Conf_Pin			equ	ram + .31

CurrentConf				equ	ram + .32
MasterCounter			equ	ram + .33

Counter2				equ	ram + .34
TempIndex2				equ	ram + .35

RtccReloadValue			equ	ram + .36
SleepSemaphore			set	ram + .37

LocalTemp				equ	ram + .38

SubCallParameter        equ ram + .39

I2C_Public              equ ram + .40
I2C_TxJumpVector        equ ram + .41
I2C_TxIndex             equ ram + .42
I2C_TxNofBytes          equ ram + .43
I2C_TxAddress           equ ram + .44
I2C_TxCommand           equ ram + .45
I2C_TxData              equ ram + .46

	; SerOutputsMaskCopy32
	; This is continously compared to 'OutputsMask32' and diffences are
	; transmitted over the I2C bus and the SerOutputsMaskCopy32 is updated accordingly.
	; This register is logically organized as the pins 0-31 from lsb to msb.
SerOutputsMaskCopy32	equ	ram + .47	; Quad. 47..50

	; Contains a high bit at the pin currently beeing checked in 'SerOutputsMaskCopy32'
SerBitMask32            equ	ram + .51	; Quad. 51..54

SER_Index               equ	ram + .55

	cblock	SER_Index+1
	endc
						include "PICFLASH.inc"
						PicFlashVariables8bit

Temporary				equ	ram + .59	; Realtime.

TempStack               equ ram + .60

CurrentPin				equ	ram + .61

SerialCopyTemp			equ	ram + .62
OutputMaskTemp			equ	ram + .63
SerialMaskTemp			equ	ram + .64

; BiToggleRegister[3:0] is bi-toggle active on pin 11, 10, 9 and 8 respectively
BiToggleRegister    	equ ram + .65
BiToggleTemp        	equ ram + .66

	if DELAYED_SAVE
DelayedSaveTimer		equ	ram + .67
	endif

KeyReleaseCnt			equ	ram + .68

	; A bit set in BiToggleRegBootDelay[3:0] means that the pin is in its boot inhibit time.
	; This register will be cleared when the inhibit time expires and the information is obsolete.
	; A bit set in BiToggleRegBootDelay also have the same bit set in 'BiToggleRegister'
	; (which is permanent).
BiToggleRegBootDelay    equ ram + .69

BiToggleCntBootDelay    equ ram + .70

	ifdef CONF_RADIO_NEXT
RadioModeNextPin		equ	ram + .71
RadioModePrevPin		equ ram + .72
RadioModeNextPrevFlags	equ ram + .73
PRESCALER_PRESCALER		equ	 BIT0
	endif

DelayedSetCount			equ	ram + .74

next_ram				set	ram + .75

			; Include IR variables

			IR_Variables

next_ram				set	next_ram + RC5_RAM_USAGE

IntrContextS			equ	RAM_BANK0_LAST - .3	; ISR context save - Must be in both banks !!
IntrContextW			equ	RAM_BANK0_LAST - .2	; ISR context save
IntrContextP			equ	RAM_BANK0_LAST - .1	; ISR context save
ReservedIceBreaker		equ	RAM_BANK0_LAST		; Reserved for ICE

	if next_ram >= IntrContextS
		error "FATAL : Ram overwrite"
	endif


;;;-----------------------------------------------------------------------
;;; M E M O R Y  M A P   -   E E P R O M   &   F L A S H
;;;-----------------------------------------------------------------------
; 16F873 has 128 bytes eeprom

		cblock	0
	EepFirstRunTest				:1
	EepOutputPinsQuad			:4
		endc


	; Assignments table layout in flash
	; --------------------------
	; [0] IR code low byte
	; [1] IR code high byte
	; [2] Conf byte		Conf<5:4> IR standard  Conf<3:0> Mode
	; [3] Pin number	Pin<4:0> Pin 0-15 & extended pins 16-31
TABLE_NOF_CELLS					equ	.32
TABLE_CELL_BYTES				equ	.4

FLASH_CODE_PAGE					equ	0x900
#define RELATIVE_FLASH_ADDRESS_CODE

FAR_CODE_PAGE					equ 0x800


;;;-----------------------------------------------------------------------
;;; M O D E   C O N S T A N T S
;;;-----------------------------------------------------------------------
;;;
; CONF_BI_TOGGLE_MODE_BOOT_DELAY
; As CONF_BI_TOGGLE_MODE except that it remains low a few seconds after
; startup. This is for a delayed 'play' or '(inverted) mute' relay.

; CONF_RADIO_NEXT, CONF_RADIO_PREV
; Will be inputs (ext pullup) selecting either next or previous radio channel.
; There are no changes on the pins while the next/prev is active, (i.e. has not
; experienced a extended time timeout). First after the extended timeout will
; the radio group shift to the newly selected by a make-before-break.

; Default values
NOF_CONF_MODES		        	set	.0

        ; Standard table
		if MODE_TABLE_CONF == 1
		messg "MODE_TABLE_CONF == 1"
NOF_CONF_MODES		        	set	.5
CONF_NORMAL_MODE	        	equ	.0      ;Standard
CONF_TOGGLE_MODE	        	equ	.1      ;Toggle
CONF_RADIO_MODE		       	 	equ	.2      ;Radio
CONF_BI_TOGGLE_MODE	    		equ	.3      ;BiToggle
CONF_BI_TOGGLE_MODE_BOOT_DELAY  equ	.4      ;BiToggleBootDelay
		endif

        ; Test table. Do not expect this one to work.
		if MODE_TABLE_CONF == .100
		messg "DEVELOPMENT : MODE_TABLE_CONF == 100"
NOF_CONF_MODES		        	set	.7
CONF_NORMAL_MODE	        	equ	.0      ;Standard
CONF_TOGGLE_MODE	        	equ	.1      ;Toggle
CONF_RADIO_MODE		       	 	equ	.2      ;Radio
CONF_BI_TOGGLE_MODE	    		equ	.3      ;BiToggle
CONF_BI_TOGGLE_MODE_BOOT_DELAY  equ	.4      ;BiToggleBootDelay
CONF_RADIO_NEXT					equ	.5		;NextRadio (hardcoded to PIN0)
CONF_RADIO_PREV					equ	.6		;PrevRadio (hardcoded to PIN1)
		endif


;;;-----------------------------------------------------------------------
;;; B I T O G G L E   C O N S T A N T S
;;;-----------------------------------------------------------------------
;;;

; These will be boolean which are allways defined :
VALUE_CONF_BI_TOGGLE_MODE               set     0
VALUE_CONF_BI_TOGGLE_MODE_BOOT_DELAY    set     0

; Next part defines 'BI_TOGGLE_DEFINED' if one or both bitoggle modes exists.

		ifdef CONF_BI_TOGGLE_MODE
#define BI_TOGGLE_DEFINED
VALUE_CONF_BI_TOGGLE_MODE               set     CONF_BI_TOGGLE_MODE
		endif

		ifdef CONF_BI_TOGGLE_MODE_BOOT_DELAY
       	ifndef BI_TOGGLE_DEFINED
#define BI_TOGGLE_DEFINED
       	endif
VALUE_CONF_BI_TOGGLE_MODE_BOOT_DELAY    set     CONF_BI_TOGGLE_MODE_BOOT_DELAY
   		endif

        if ICE
BITOGGLE_BOOT_DELAY_sec                 equ     4
        else
BITOGGLE_BOOT_DELAY_sec                 equ     3
        endif

;;;-----------------------------------------------------------------------
;;; C O N S T A N T S
;;;-----------------------------------------------------------------------
;;;
MAKE_BEFORE_BREAK_TICKS 		equ	.4      ; Time overlap between radio mode pins
MAKE_BEFORE_BREAK_RADIO_NEXT	equ	.15

FATAL_ALLREADY_EXISTS			equ	.3
FATAL_NO_MORE_ROOM				equ	.4
FATAL_ILLEGAL					equ	.5
FATAL_TRAP               		equ	.6
FATAL_REALTIME_STARVATION		equ	.7



;;;-----------------------------------------------------------------------
;;; R C 5  &  S O N Y   C O N S T A N T S
;;;-----------------------------------------------------------------------


; RC5 standard. This is the fixed RC5 base clock.
RC5_DATA_CLOCK_Hz		equ	.36000

; RC5 standard. Equals 1.778 msec
RC5_BIT_TIME_us			equ	(.1000000 * .64 ) / RC5_DATA_CLOCK_Hz

; Time to tranmit a code.
; Corrected for the invinsible low level during first sync start bit.
RC5_AIR_TIME_us			equ	(RC5_BIT_TIME_us * .27) / 2

; 113.778ms
RC5_REPEAT_INTERVAL_us	equ	RC5_BIT_TIME_us * .64

SAMPLES_PER_BIT			equ	8

SAMPLE_PERIOD_ns		equ	(.1000*RC5_BIT_TIME_us) / SAMPLES_PER_BIT



;;; ---------------------------------------------------------------------------
;;; AUTOCALC.INC
;;; INLINE CALCULATION OF RTCC RELOAD VALUE TO GET A GIVEN ISR FREQUENCY
;;; ---------------------------------------------------------------------------
;;;
;;; 			---- MANUAL DEFINES ----

; Select the wanted interupt frequency in Hz
REQUESTED_ISR_FREQ_Hz	equ	.1000000000 / SAMPLE_PERIOD_ns
		if REQUESTED_ISR_FREQ_Hz < 0
			messg "FATAL: REQUESTED_ISR_FREQ_Hz is negative"
		endif

; Acceptable plus/minus tolerence in promille (1 percent = 10 !)
MAX_ISR_TOLL_PROMILLE	equ	.20

; Define the selected TMR0 prescaler value.
; TMR0_PRESC_VAL = (2^(presc#+1) [presc#=0,1,2,3...] = 2,4,8,...
		if XTAL == .4000000
TMR0_PRESC_VAL			equ	.2      ; 4 MHz
		else
TMR0_PRESC_VAL			equ	.8      ; 20 MHz
		endif

;;; 			---- CALCULATIONS ----

		include "AUTOCALC.inc"



		; SLOW TIMER
		; ----------------------------------------------------
		; Used for LED blinking, manual inputs reading, radio mode delays etc.
		; The timer code is located in main loop and the timer is ticked by ISR.

ISR_SLOW_TIMER_Hz		equ	.30

ISR_SLOW_PRESCALER		equ	ISR_FREQ_Hz / ISR_SLOW_TIMER_Hz

KEY_DEBOUNCE_TIME		equ	ISR_SLOW_TIMER_Hz / .6

	if ISR_SLOW_PRESCALER > 0xFF
		error "LED_ISR_PRESCALER overflow"
	endif



			IR_Timing


		; Sleep timing
		; ----------------------------------------------------

SLEEP_RX_STABLE_us		equ	.500
SLEEP_TIMEOUT_us		equ	.130000

SLEEP_ISR_PRESC			equ	.4

SLEEP_ISR_PERIOD_us		equ	(ISR_PERIOD_ns * SLEEP_ISR_PRESC) / .1000

SLEEP_RX_STABLE_CNT		equ	SLEEP_RX_STABLE_us / SLEEP_ISR_PERIOD_us
SLEEP_TIMEOUT_CNT		equ	SLEEP_TIMEOUT_us / SLEEP_ISR_PERIOD_us



;;;-----------------------------------------------------------------------
;;; P O R T S
;;;-----------------------------------------------------------------------
;
; There are 2 different pin layouts, ICE and not ICE, i.e. release. The
; difference is that the ICE is a 16F877 and some pins on the normal 16F873
; are reserved when running ICE. The fix throughout is to remove the pins
; from the project when in ICE mode. (as opposed to just remapping them)
; A subclass to ICE mode is ALTERNATIVE_PINS. This is just a temporary
; development playground.
;

	if ICE

	if ALTERNATIVE_PINS
	messg "INFO : ICE ALTERNATIVE_PINS"
TrisA			equ	b'00000000'
TrisB			equ	b'01010000'
TrisC			equ	b'00011000'     ; Includes I2C pins at 3 & 4

LED_PORT		equ	PORTC
LED_PIN			equ	 BIT1
LED_INV_PORT	equ	PORTA
LED_INV_PIN		equ	 BIT4
RC5PORT			equ	PORTB
RC5PIN			equ	 BIT4
RC5DEBUGPORT	equ	PORTC
RC5DEBUGPIN		equ	 BIT2
	endif


	if STANDARD_PINS
	messg "INFO : ICE - STANDARD_PINS"
TrisA			equ	b'00000000'
TrisB			equ	b'00100000'
TrisC			equ	b'00011000'

LED_PORT		equ	PORTA
LED_PIN			equ	 BIT0

RC5PORT			equ	PORTB
RC5PIN			equ	 BIT5		; IR state machine relies on int-on-pb<7:4> change function.
RC5DEBUGPORT	equ	PORTA
RC5DEBUGPIN		equ	 BIT2
	endif

	if SINGLE_PIN
	messg "INFO : ICE - SINGLE_PIN"
TrisA			equ	b'00000000'
TrisB			equ	b'00111000'
TrisC			equ	b'00011000'

LED_PORT		equ	PORTC
LED_PIN			equ	 BIT0

RC5PORT			equ	PORTB		; IR state machine relies on int-on-pb<7:4> change function.
RC5PIN			equ	 BIT4
;RC5DEBUGPORT	equ	PORTA
;RC5DEBUGPIN		equ	 BIT2
	endif

	else ; not ICE

	  messg "INFO : Release"
TrisA			equ	b'00000000'
TrisB			equ	b'01000000'
TrisC			equ	b'00011000'     ; Includes I2C pins at 3 & 4

LED_PORT		equ	PORTB
LED_PIN			equ	 BIT7
LED_INV_PORT	equ	PORTA
LED_INV_PIN		equ	 BIT0
RC5PORT			equ	PORTB		; IR state machine relies on int-on-pb<7:4> change function.
RC5PIN			equ	 BIT6
	endif ; if ICE


	if SINGLE_PIN
PIN0_PORT		equ	PORTB
PIN0			equ	 BIT2
;PIN1_PORT		equ	PORTC
;PIN1			equ	 BIT4
	endif


	if ALTERNATIVE_PINS
PIN0_PORT		equ	PORTB
PIN0			equ	 BIT0
PIN1_PORT		equ	PORTB
PIN1			equ	 BIT1
PIN2_PORT		equ	PORTB
PIN2			equ	 BIT2
PIN3_PORT		equ	PORTB
PIN3			equ  BIT3
	endif


	if STANDARD_PINS		; Ch Pin  16F873       16F877
PIN0_PORT		equ	PORTC
PIN0			equ	 BIT2	;  0 RC2  Pin13[873]   Pin17[877]
PIN1_PORT		equ	PORTC
PIN1			equ	 BIT5	;  1 RC5
PIN2_PORT		equ	PORTC
PIN2			equ	 BIT1	;  2 RC1
PIN3_PORT		equ	PORTC
PIN3			equ  BIT6   ;  3 RC6

PIN4_PORT		equ	PORTC
PIN4			equ	 BIT0	;  5 RC0
PIN5_PORT		equ	PORTC
PIN5			equ  BIT7   ;  6 RC7  Pin18[873]   Pin26[877]
PIN6_PORT		equ	PORTB
PIN6			equ	 BIT0	;  6 RB0  Pin21[873]   Pin33[877]
PIN7_PORT		equ	PORTB
PIN7			equ	 BIT1	;  7 RB1  Pin22[873]   Pin34[877]

PIN8_PORT		equ	PORTA
PIN8			equ	 BIT5	;  8 RA5  Pin7[873]    Pin7[877]
PIN9_PORT		equ	PORTB
PIN9			equ	 BIT2	;  9 RB2  Pin??[873]   Pin35[877]
PIN10_PORT		equ	PORTB
PIN10			equ	 BIT3	; 10 RB3  Pin??[873]   Pin36[877]
PIN11_PORT		equ	PORTA
PIN11			equ	 BIT3	; 11 A3

PIN12_PORT		equ	PORTB
PIN12			equ	 BIT4	; 12 RB4  Pin25[873]   Pin37[877]
 	if ICE == false
PIN13_PORT		equ	PORTA
PIN13			equ	 BIT2	; 13 A2		This one is IR debug on ICE !
PIN14_PORT		equ	PORTB
PIN14			equ	 BIT5	; 14 B5		This one is IR input on ICE !
PIN15_PORT		equ	PORTA
PIN15			equ	 BIT1	; 15 A1		This one is IR power on ICE !
 	endif
	endif





;;;-----------------------------------------------------------------------
;;; M A C R O S
;;;-----------------------------------------------------------------------

LedOn		macro
			nop
			bsf		LED_PORT, LED_PIN
		ifdef LED_INV_PORT
			nop
			bcf		LED_INV_PORT, LED_INV_PIN
		endif
			endm

LedOff		macro
			nop
			bcf		LED_PORT, LED_PIN
		ifdef LED_INV_PORT
			nop
			bsf		LED_INV_PORT, LED_INV_PIN
		endif
			endm


LedToggle	macro
			local	led_on, led_ends
			btfss	LED_PORT, LED_PIN
			goto	led_on
			LedOff
			goto	led_ends
led_on		LedOn
led_ends
		endm



;;;-----------------------------------------------------------------------
;;;-----------------------------------------------------------------------
;;;
;;;   		C O D E  S T A R T S
;;;
;;;-----------------------------------------------------------------------
;;;-----------------------------------------------------------------------

ColdStart	org	RESET
			nop
			nop
			goto	Cont

			org	INTERUPT
			; Context saving
			; W register and status are saved in reversed order...
			; ------------------------------------------------
			movwf	IntrContextW		; Save W reg
			swapf	STATUS, W
			clrf	STATUS
			movwf	IntrContextS		; Save swapped STATUS
			movf	PCLATH, W
			movwf	IntrContextP		; Save PCLATH
			clrf	PCLATH
			; Don't want to clutter up the code by dumping the ISR body here.
			goto	IntrRutine

	;;;-----------------------------------------------------
	;;; BOOT CODE
	;;;-----------------------------------------------------

			; Becomes readable header in programmer ascii view
			data	"             ( C ) c b p   2 0 0 3"
			data	" F p r c 5 r x  "

			InlineAsciiHeader		; Version, processor etc. as ascii in hexfile

		if IR_RECIEVER_ACTIVE_HIGH
			data	" A c t . H i g h"
			else
			data	" A c t . L o w  "
		endif

 		if MODE_TABLE_CONF == 1
			data	        " M o d e T a b 1"
		else
			if MODE_TABLE_CONF == 2
				data        " M o d e T a b 2"
			else
				data	" M o d e T a b ?"
			endif
		endif

			; Reset vector continued
			; -------------------------------------------------------
Cont
			MZeroFileRegs			; Macro to clear ram files

			clrf	INTCON			; A stray "ei" in init part will have no effect...

			 ; No A/D channels
			 ;
			movlw	0x06
			SetBnkF	ADCON1
			movwf	ADCON1 & FORCE_7_BIT_ADR
			SetBank0Auto

			 ; Define i/o pins
			 ;
			call    InitializeAllOutputs
			bcf		LED_PORT, LED_PIN

			 ; Check if user requests a pin delete

			call	CheckForDeleteRequest


			 ; This shouldn't be nescessary? Inserted to get port C
			 ; working running ICE.
			SetBnkF	T1CON
			clrf	T1CON
			SetBank0Auto

            ; Delay for delayed BiToggle mode pins

            movlw   BITOGGLE_BOOT_DELAY_sec * ISR_SLOW_TIMER_Hz
            movwf   BiToggleCntBootDelay

			 ; Ensure a valid eeprom if this is first power on ever
			 ;
			call	FirstRunSetups

			 ; Restore output pins
			 ;
			call	RestoreOutputLevelMask

             ; Update BiToggle information
             ;
            call    LocateBiTogglePins

	       	if ICE	; -------------------------------------
			nop		; Breakpoint : RadioNext / Prev
			nop		;
	    	endif	; -------------------------------------

			call	LocateRadioNextPrevPins

             ; Use the bitoggle register to set the slave inputs high in OutputsMask32+1

            swapf   BiToggleRegister, W
            andlw   0xF0
            iorwf   OutputsMask32+1, F

            call    FilterHighBiToggleBootDelayPins

			call	SetOutputsFromMasks

			 ; HelloWorld - Flash LED
			 ;
			LedOn
			movlw	.150
			call	msDelay
			LedOff

             ;
             ; Initialize i2c serial
             ;
            call    I2C_MasterAsyncInit

             ; Force a complete serial update from start
             ;
            comf    OutputsMask32+0, W
            movwf   SerOutputsMaskCopy32+0
            comf    OutputsMask32+1, W
            movwf   SerOutputsMaskCopy32+1
            comf    OutputsMask32+2, W
            movwf   SerOutputsMaskCopy32+2
            comf    OutputsMask32+3, W
            movwf   SerOutputsMaskCopy32+3

             ; Initialize I2C serial monitor
             ;
            call	ResetSerialTestMask


			; IR initialize
	        ;
			movlw	IR_PRE_IDLE
			movwf	IRJumpVector

			movlw	0xFF
			movwf	IR_PrevHigh



RunTime     ; Initialize first sync period

			movlw	RTCC_SUB_LOAD
			movwf	RtccReloadValue

			clrf	TMR0

			 ; Includes "TMR0_PRESC_VAL"
		if XTAL == .4000000
			movlw   b'10000000'     ; 4 MHz
		else
			movlw   b'10000010'     ; 20 MHz
		endif

		if ALTERNATIVE_PINS
			movlw   b'00000010'     ; 20 MHz and int. pullups
		endif
			setoption

			movlw   b'10100000'		; Timer 0 intr. enabled
			movwf	INTCON



;;;-----------------------------------------------------------------------
;;;-----------------------------------------------------------------------
;;;
;;; 	Realtime loop
;;;
;;;-----------------------------------------------------------------------
;;;-----------------------------------------------------------------------



MainLoop

			 ; New IR code recieved ?
			 ; --------------------------------------------------
			 ; Set corresponding bit(s) high if the code is found in table
			 ; Assumes a cleared mask at entry

			btfss	IRPublicFlags, IR_READY
			goto	RT_RC5_Ends

			btfsc	Flags, IR_WAITING_FOR_TIMEOUT	; wtf
			goto	RT_RC5_Ends

			 ; Yes, New IR code flagged.
			 ; Save IR code. The copies are used later at the timeout as well
			 ;
			movf	RC5Low, W
			movwf	NewIrLowByte
			movf	RC5High, W
			movwf	NewIrHighByte

			 ; ToDo save IR flags and enable IR here
			 ; Loop through entire table to find any matches and effectuate them

			call	GetFirstPinNumberFromIRCode	; Pin number in W

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			skpc
			goto	RT_RC5_Clr


			; IR code were in table
			; ---------------------------------
			; FIXIT - merge with "radio_bingo"
			;
			bsf		Flags, IR_WAITING_FOR_TIMEOUT

add_pin		movwf	CurrentPin

			movf	Read_Conf_Mode, W
			movwf	CurrentConf

			 ; Program output pin
			 ;
			call	ActivateOutputMask
			 ;
	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			call	GetNextPinNumberFromIRCode

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			skpnc
			goto	add_pin

			 ; Start LED blinker
			movlw	0x22
			call	LedEnable

		if !DELAYED_SETS
			call	SetOutputsFromMasks
		else
			movlw	.35
			movwf	DelayedSetCount
		endif

			 ; Dont save output state if radio button, that is done in radio release part
			 ; (two radio buttons here in make-before-break)
		ifdef CONF_RADIO_MODE
			movf	CurrentConf, W
			call	ModeConf2Int
			sublw	CONF_RADIO_MODE
			skpnz
			goto	no_save_at_start
		endif

		ifdef CONF_RADIO_NEXT
 			movf	CurrentConf, W
			call	ModeConf2Int
			sublw	CONF_RADIO_NEXT
			skpnz
			goto	no_save_at_start
		endif

		ifdef CONF_RADIO_PREV
			movf	CurrentConf, W
			call	ModeConf2Int
			sublw	CONF_RADIO_PREV
			skpnz
			goto	no_save_at_start
		endif

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			 ; Save
		if !DELAYED_SAVE
			call	SaveCurrentOutputLevels
		endif

no_save_at_start

RT_RC5_Clr
			bcf		IRPublicFlags, IR_READY

RT_RC5_Ends





		 ; IR timeout flagged ?
		 ; --------------------------------------------------
		 ; All outputs low
RT_TimeoutTest
			btfss	IRPublicFlags, IR_TIMEOUT
			goto	RT_no_timeout

			btfss	Flags, IR_WAITING_FOR_TIMEOUT
			goto	AcknTimout

		if DELAYED_SETS
			clrf	DelayedSetCount
		endif


			; IR timeout on a previously accepted code

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
	    	endif	; -------------------------------------

			call	LedDisable

			call	GetFirstPinNumberFromIRCode	; Pin number in W
			skpc
			goto	T_RC5_Ends

ir_dis_loop
			movwf	CurrentPin
			movf	Read_Conf_Mode, W
			movwf	CurrentConf

			; Figure out which output type this one have
			;
			call	DeactivateOutputMask
			call	GetNextPinNumberFromIRCode
			skpnc
			goto	ir_dis_loop


T_RC5_Ends

			call	SetOutputsFromMasks

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
	    	endif	; -------------------------------------
			bcf		Flags, IR_WAITING_FOR_TIMEOUT

		if DELAYED_SAVE
			movlw	DELAYED_COUNT
			movwf	DelayedSaveTimer
		endif
			 ; Dont save output state if radio button, that is done in radio release part
			 ; (two radio buttons here in make-before-break)
	         ; ToDo : Double code
		ifdef CONF_RADIO_MODE
			movf	CurrentConf, W
			call	ModeConf2Int
			sublw	CONF_RADIO_MODE
			skpnz
			goto	save_at_exit
		endif

		ifdef CONF_RADIO_NEXT
			movf	CurrentConf, W
			call	ModeConf2Int
			sublw	CONF_RADIO_NEXT
			skpnz
			goto	save_at_exit
        endif

		ifdef CONF_RADIO_PREV
			movf	CurrentConf, W
			call	ModeConf2Int
			sublw	CONF_RADIO_PREV
			skpnz
			goto	save_at_exit
        endif

			goto	no_save_at_exit

       		; Save.
save_at_exit
	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

		if !DELAYED_SAVE
			call	SaveCurrentOutputLevels
		endif
no_save_at_exit

AcknTimout

			bcf		IRPublicFlags, IR_TIMEOUT

RT_no_timeout




		; I2C engine call
		; --------------------------------------------------
		;
			call    I2C_MasterAsyncEngine



		; I2C Serial Monitor
		; --------------------------------------------------
		; Transmit information about level changes over I2C bus

	        btfsc   I2C_Public, I2C_TX_START
	        goto	I2C_UpdateEnds
	         ;

	         ; Loop for all 16 outputs
	         ;
	        clrc
	        rrf     SerBitMask32+3, F
	        rrf     SerBitMask32+2, F
	        rrf     SerBitMask32+1, F
	        rrf     SerBitMask32+0, F

			decf    SER_Index, F
			btfss   SER_Index, BIT7
			goto    no_wrap_yet

             ; "SER_Index" became -1, install channel 31
             ;
            call	ResetSerialTestMask

no_wrap_yet

			 ; Get bytes to work on

			rlf	SER_Index, W
			movwf	TempIndex
			swapf	TempIndex, F
			movlw	b'00000011'
			andwf	TempIndex, F

			movlw	OutputsMask32
			addwf	TempIndex, W
			movwf	FSR
			movf	INDF, W
			movwf	OutputMaskTemp		; The current state of PIC outputs (16phys+16virt)

			movlw	SerBitMask32
			addwf	TempIndex, W
			movwf	FSR
			movf	INDF, W
			movwf	SerialMaskTemp		; The current bit mask (1 high in 32 bit register)

			movlw	SerOutputsMaskCopy32
			addwf	TempIndex, W
			movwf	FSR
			movf	INDF, W
			movwf	SerialCopyTemp		; The current state of serial outputs (32)

             ; Test byte
             ; ------------------------------
             ;
            movf    SerialCopyTemp, W
            xorwf   OutputMaskTemp, W
            andwf   SerialMaskTemp, W

             ; W=0 if SER_OutputsCopyLow[SER_Index] == OutputsMaskLow[SER_Index]
             ; then no action
             ;
            skpnz
            goto    I2C_UpdateEnds

             ; Level has changed, invert "SER_OutputsCopyLow[SER_Index]"
             ;
            movf    SerialCopyTemp, W
            andwf   SerialMaskTemp, W
            skpz
            goto    SerLowIsHigh

             ; SER_OutputsCopyLow[SER_Index] == 0, now set it high
             ;
            movf    SerialMaskTemp, W
            iorwf   SerialCopyTemp, F
            movlw	I2C_725_CMD_LEVEL_HIGH
            movwf	I2C_TxCommand
            goto    SaveNewSerialState

SerLowIsHigh
             ; SER_OutputsCopyLow[SER_Index] == 1, now clear it
             ;
            comf    SerialMaskTemp, W
            andwf   SerialCopyTemp, F
            movlw	I2C_725_CMD_LEVEL_LOW
            movwf	I2C_TxCommand
            goto    SaveNewSerialState



SaveNewSerialState
			; Save temporary back again
			; --------------------
			movf	SerialCopyTemp, W
			movwf	INDF


			; Install transmission
			; --------------------
			;
			movf    SER_Index, W
			movwf   I2C_TxData

	        movlw   I2C_725_BROADCAST_ADDRESS * 2
	        movwf   I2C_TxAddress
	         ;
	        movlw   I2C_725_LEVEL_CHANGED_CMD_LEN
	        movwf   I2C_TxNofBytes
	         ;
	        bsf     I2C_Public, I2C_TX_START


I2C_UpdateEnds




			;
			;
			; --------------------------------------------------
			; Timed code - "ISR Slow Timer"
			; --------------------------------------------------
			; Ticked by isr counter "RealtimeTickCounter"
			;
			;
			updatez	RealtimeTickCounter
			skpnz
			goto	TimedCodeEnds


		if DELAYED_SETS
			updatez	DelayedSetCount
			skpnz
			goto	dsasda
			decfsz	DelayedSetCount, F
			goto	dsasda

			nop
			call	SetOutputsFromMasks
dsasda
		endif


		if DELAYED_SAVE
			; Check if the delayed save counter is active
			updatez	DelayedSaveTimer
			skpnz
			goto	no_delayed

			; Counter is active. Reset it here if the IR is active
			movlw	DELAYED_COUNT
			btfsc	IRPublicFlags, IR_BUSY
			movwf	DelayedSaveTimer

			decfsz	DelayedSaveTimer, F
			goto	no_delayed

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------
			call	SaveCurrentOutputLevels
no_delayed
		endif

			 ;
			decf	RealtimeTickCounter, F

		if !RELEASE
			 btfss	RealtimeTickCounter, BIT4
			 goto	no_tick_ovf

			 movlw	FATAL_REALTIME_STARVATION
			 call	FatalStop

no_tick_ovf
		endif




			; Maintain BiToggleRegBootDelay - delayed on
			; ------------------------------
			;
            updatez BiToggleRegBootDelay
            skpnz
            goto    BT_done
             ;
            decfsz  BiToggleCntBootDelay, F
            goto    BT_done
			;
			; Set the delayed outputs high and clear delay register
			; See 'FilterHighBiToggleBootDelayPins()'
			;
            movf    BiToggleRegBootDelay, W
            iorwf   OutputsMask32+1, F
            clrf    BiToggleRegBootDelay
             ;
            call    SetOutputsFromMasks
BT_done




			; Check for programming mode request
			; ------------------------------

			btfss   Flags, PROGRAM_MODE_ON
			goto	CheckForPgmMode

			; ----------------------
			; Programming mode enabled
			; ----------------------

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			; If LED pin is high again when allready in programming
			; mode then use extended pin numbers 16-31

			btfss	LED_PORT, LED_PIN
			goto	NoExtended
			btfsc	LedFlags, LED_ON
			goto	NoExtended

		if EXTENDED_PIN_SET == BIT3
			movlw	b'00001000'
			xorwf	Flags, F
		else
			error "Fix bitmask @1259"
		endif
			goto	SignalProgrammingMode

NoExtended


			btfsc	Flags, IR_WAITING_FOR_TIMEOUT
			goto	UserPgmEnds

			call    InitializeAllOutputs

			 ; A pin forced to wrong level ?
			call	LookForShortedPins

			skpnc
			call	ProgrammingRequest
			goto	UserPgmEnds

			; ----------------------
  			; Programming mode test
			; ----------------------
CheckForPgmMode
			; Check if power led is pulled to unexpectedly to vcc.
			; If not then proceed to the normal run mode part below.
			;
			btfss	LED_PORT, LED_PIN
			goto	RunMode

			btfsc	LedFlags, LED_ON
			goto	UserPgmEnds


       		; Low Led output forced high -> enter programming mode
			bsf     Flags, PROGRAM_MODE_ON

SignalProgrammingMode
     		; Do a 2 sec very fast blinking
  			;
			movlw   .15
			btfsc	Flags, EXTENDED_PIN_SET
			movlw   .30

			call    LedFlasher

			clrf    RealtimeTickCounter
			goto	UserPgmEnds



			; ----------------------
			; Normal run mode
			; ----------------------
			; Intercept user shorts on the i/o pins (typ. radio or bitoggle related)
RunMode

			btfsc	Flags, WAITING_FOR_DEBOUNCE
			goto	WaitingForSwitchDebounce

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
	    	endif	; -------------------------------------

			call	LookForShortedPins

			skpc
			goto	UserPgmEnds

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop
	    	endif	; -------------------------------------

             ; User has shorted a pin. Figure out if this is part of a radiogroup ?
             ;
            call    GetFirstCodeAndConf
run_again   skpc
            goto    RunModeEnds

			; Got a assignment on this pin .. check for a 'complicated' mode
			;
   			movwf	LocalTemp       ; save pin number

		ifdef CONF_RADIO_MODE
			call    ModeConf2Int
			sublw   CONF_RADIO_MODE
			skpnz
			goto    radio_pin
		endif

		ifdef CONF_RADIO_PREV
       		movf	LocalTemp, W
			call    ModeConf2Int
			sublw   CONF_RADIO_PREV
			skpnz
			goto    radio_prev_pin
		endif

		ifdef CONF_RADIO_NEXT
       		movf	LocalTemp, W
			call    ModeConf2Int
			sublw   CONF_RADIO_NEXT
			skpnz
			goto    radio_next_pin
		endif

     	ifdef CONF_BI_TOGGLE_MODE
       		movf	LocalTemp, W
      		call    ModeConf2Int
      		sublw   CONF_BI_TOGGLE_MODE
         	skpnz
      		goto    bitoggle_pin
    	endif

		ifdef CONF_BI_TOGGLE_MODE_BOOT_DELAY
            movf	LocalTemp, W
            call    ModeConf2Int
            sublw   CONF_BI_TOGGLE_MODE_BOOT_DELAY
            skpnz
            goto    bitoggle_boot_delay_pin
		endif

                 ;
			call    GetNextCodeAndConf
			goto    run_again




		; Shorted pin belongs to radio or bitoggle group
		; ------------------------------
		; First set the new pin high here.
		; FIXIT - merge with new IR code
		;
radio_pin
bitoggle_pin
bitoggle_boot_delay_pin
radio_prev_pin
radio_next_pin


   			movf    Read_Conf_Pin, W
 			movwf	CurrentPin
            	;
			movf	Read_Conf_Mode, W
			movwf	CurrentConf
			 ;
			call	ActivateOutputMask

			call	SetOutputsFromMasks

			bsf		Flags, WAITING_FOR_DEBOUNCE
reload_wait	movlw	KEY_DEBOUNCE_TIME
			movwf	KeyReleaseCnt

			goto	RunModeEnds



WaitingForSwitchDebounce

			call	LookForShortedPins
			skpnc
			goto	reload_wait

			decfsz	KeyReleaseCnt, F
			goto	RunModeEnds

			 ; Waiting is over
			bcf		Flags, WAITING_FOR_DEBOUNCE


RunModeEnds
UserPgmEnds


			 ; Signal LED
			 ; --------------------------------------------------
			call	Led


			 ; Radio button - release of unselected pins
			 ; --------------------------------------------------

			updatez	RadioPrescaler
			skpnz
			goto	no_radio

			decfsz	RadioPrescaler, F
			goto	no_radio

			 ; RadioPrescaler has reached zero
	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
	    	endif	; -------------------------------------

		ifdef CONF_RADIO_NEXT
			btfss	RadioModeNextPrevFlags, PRESCALER_PRESCALER
			goto	prepreend

			bcf		RadioModeNextPrevFlags, PRESCALER_PRESCALER
			movf	RadioPinActivated, W
			call	SetBitInOutputMask
			call	SetOutputsFromMasks
			movlw	MAKE_BEFORE_BREAK_TICKS
			movwf	RadioPrescaler
			goto	no_radio
prepreend
		endif

			call	ClearRadioChannels
			call	SetOutputsFromMasks

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

		if DELAYED_SAVE
			movlw	DELAYED_COUNT
			movwf	DelayedSaveTimer
		else
			call	SaveCurrentOutputLevels
		endif
no_radio


TimedCodeEnds

			goto	MainLoop



;;;-----------------------------------------------------------------------
;;;-----------------------------------------------------------------------
;;;
;;; 	Realtime ends
;;;
;;;-----------------------------------------------------------------------
;;;-----------------------------------------------------------------------


WaitForRelease

still_selected
			call	SetOutputsFromMasks

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			call	LookForShortedPins
			skpnc
			goto	still_selected

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			return

WaitForRequest

no_selection
			call	SetOutputsFromMasks
			call	LookForShortedPins
			skpc
			goto	no_selection
			return


		; -------------------------------------------------------------------
		; ProgrammingRequest
		; -------------------------------------------------------------
		;
		;
ProgrammingRequest

			movwf	Pin

			bcf		IRPublicFlags, IR_READY

			clrf	Write_Conf_Mode
			clrf	Write_Conf_Pin
			 ;
			LedOn

			call	WaitForRelease

			movlw	.250
			call	msDelay
			movlw	.250
			call	msDelay

		 ; Get number of blinks plus one in counter
		 ;
NewModeBlink
			movf	Write_Conf_Mode, W
			call	ModeConf2Int
			addlw	1
			movwf	LocalTemp

led_pulse_again

			movlw	.250
			call	msDelay
			LedOff
			movlw	.150
			call	msDelay

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			call	WaitForRelease
			LedOn
			decfsz	LocalTemp, F
			goto	led_pulse_again
			 ;
			 ; Loop until a IR code is flagged or a mode change request
			 ;
code_req_wait
	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			btfsc	IRPublicFlags, IR_READY
			goto	got_a_code
			 ;
			 ; No code yet, check for a new programming pulse (change pin mode)
			 ;
			call	SetOutputsFromMasks

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			call	LookForShortedPins
			skpc
			goto	code_req_wait

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			 ; Next mode selected
			 ;
			movf	Write_Conf_Mode, W
			call	ModeConf2Int
			addlw	1
			 ;
			 ; Restrict the upper mode number
			 ;
			sublw	NOF_CONF_MODES
			skpz
			sublw	NOF_CONF_MODES
			call	ModeInt2Conf
			movwf	Write_Conf_Mode

			goto	NewModeBlink




got_a_code
			LedOff

			 ; User has pressed a key - save IR code
			 ; ----------------------------------------
			 ;
	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------


			movf	RC5Low, W
			movwf	Write_Low
			movf	RC5High, W
			movwf	Write_High


			 ; Define pin
			 ;
			movf	Pin, W
			andlw	0x0F
			iorwf	Write_Conf_Pin, F
			 ;
			btfsc	Flags, EXTENDED_PIN_SET
			bsf		Write_Conf_Pin, BIT4


			 ; Extract mode from IR flag register
			 ;
            swapf   IRPublicFlags, W
            andlw   b'00110000'
            iorwf   Write_Conf_Mode, F

			 ; If exists allready then call fatal
			 ;
			call	FindCodeAndConfAndPinInTable
			skpnc
			call	FatalExists


	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			call	AddPinDataToTable
			skpc
			call	FatalNoRoomLeft

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			 ; Define the output state to exit programming with
			 ;
			call	ClearTableOutputLevels
			call	RestoreOutputLevelMask
			call	SetOutputsFromMasks

		if !RELEASE
			clrf	RealtimeTickCounter
		endif

			return



FatalNoRoomLeft
			movlw	FATAL_NO_MORE_ROOM
			call	FatalStop
			return

FatalExists
			movlw	FATAL_ALLREADY_EXISTS
			call	FatalStop
			return




LedFlasher	movwf   TempIndex
flash_loop	LedOn
			movlw	.50
			call	msDelay
			LedOff
			movlw	.50
			call	msDelay
            decfsz  TempIndex, F
            goto    flash_loop
            return



		; -------------------------------------------------------------------
		; 	CheckForDeleteRequest
		; 	-------------------------------------------------------------
		;	A output pin forced to high level while cold-starting means
		;	that all assignments on this pin is deleted, and a high driven
		;	led pin erases everything...
		;
CheckForDeleteRequest

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			btfss	LED_PORT, LED_PIN
 			return

        	; User pulled LED pin high, enter delete mode
       		; This part will newer return
			di
      		movlw   .50
    		call    LedFlasher
    		; The short should have been removed by now ! (and LED low)

DeleteModeHome
			clrwdt

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

   			call    ClearAllOutputs
			btfss	LED_PORT, LED_PIN
   			goto	CheckPinDelete

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

        	; Delete all and enter a blink forever
          	;
			call	ClearSystem
			;
forev		movlw   .200
			call    LedFlasher
        	goto    forev


CheckPinDelete

			call	LookForShortedPins
			skpc
			goto    DeleteModeHome

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			call	ClearPinFromTable
	  		movlw   .20
	  		call    LedFlasher

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			goto    DeleteModeHome





ResetSerialTestMask

            movlw   0x80
            movwf   SerBitMask32+3
            clrf    SerBitMask32+2
            clrf    SerBitMask32+1
            clrf    SerBitMask32+0
            movlw   .31
            movwf   SER_Index
			return




;;;-----------------------------------------------------------------------
;;; S U B R U T I N E S
;;;-----------------------------------------------------------------------


	; Signal LED blinker
	; ----------------------------------------------------------
	; The purpose is to get a LED blinker that is somewhat decoupled from
	; the calling code regarding call order, and to get one that do not have
	; abrubt pulses.
	;
	; LedEnable. Rate in W reg, Led high time in W[7..4] (1..15) and led low time in W[3..0] (1..15)
	; LedDisable
	; Led. Called unconditionally with a fixed frequency.
	;

LedEnable


			movwf	LedPreset
			incf	SleepSemaphore, F

			movlw	1
			subwf	SleepSemaphore, W

			skpz
			return

	   		bcf		LedFlags, LED_CLOSING
			bsf		LedFlags, LED_ON
			goto	led_high


LedDisable

			decfsz	SleepSemaphore, F
			return

			bsf		LedFlags, LED_CLOSING
			return



			 ; The main body. This code should be called with a
			 ; frequency of about 30 Hz.

Led			btfss	LedFlags, LED_ON
			return

			decfsz	LedPrescaler, F
			return

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			btfss	LED_PORT, LED_PIN
			goto	led_high

			 ; Led is high, turn it off
			 ; ------------------------------
			 ;
			LedOff
			 ;
			 ; Check if there are a close request pending
			 ;
			btfsc	LedFlags, LED_CLOSING
			goto	led_close
			 ;
			 ;
			movlw	0x0F
			andwf	LedPreset, W
			movwf	LedPrescaler
			return

			 ; Led is low, turn it on
			 ; ------------------------------
			 ;
			 ; Check if there are a close request pending
			 ;
			btfsc	LedFlags, LED_CLOSING
			goto	led_close

led_high
			LedOn
			swapf	LedPreset, W
			andlw	0x0F
			movwf	LedPrescaler
			return

			 ; The blink sequence is at a time where we can stop

led_close
			bcf	LedFlags, LED_ON
			bcf	LedFlags, LED_CLOSING
			return





		; -------------------------------------------------------------
		; LocateBiTogglePins
		; -------------------------------------------------------------
		; Traverse assignmentstable and find any running in bitoggle mode.
		; Initialises 'BiToggleRegister' and 'BiToggleRegBootDelay'
		;
		;
LocateBiTogglePins

		ifdef BI_TOGGLE_DEFINED

             ; First the looping where any bitoggle pins get a bit set in
             ; the "BiToggleRegister"
             ; Loop goes from pin 11 -> 8

            movlw   .4
            movwf   BiToggleTemp            ; Loop counter

            clrf    BiToggleRegister

LoopAllFour
            movlw   .7
            addwf   BiToggleTemp, W
            call    GetFirstCodeAndConf     ; Param : Pin in W

BiToggleLoop
			skpc
			goto    BiToggleSearchEnds

             ; Got a assignment ...
             ;
            call    ModeConf2Int
            sublw   VALUE_CONF_BI_TOGGLE_MODE
            skpz
            goto    TestForBootDelay


             ; Found a bitoggle pin.
             ; ------------------------------------------------------------
             ; Set "BiToggleRegister" bit.
             ; Actions are the slave pin as input pins, and the output mask
             ; should be set high on slave to catch user shorts to gnd.
             ;
BiToggleJump
			movlw	BiToggleJump / 0x100
			movwf	PCLATH
			 ;
			decf	BiToggleTemp, W         ; BiToggleTemp runs from 4 to 1
			addwf	PCL, F

            goto    BiTogglePin9
            goto    BiTogglePin10
            goto    BiTogglePin11
            goto    BiTogglePin12

		if BiToggleJump / 0x0100 != $ / 0x0100
		 	error "Page fault in 725.asm #1724"
		endif

BiTogglePin12
		ifdef PIN15_PORT
			SetBnkF	TRISA                           ; Note : Absolute reference !
			bsf		PIN15_PORT, PIN15
			SetBank0Auto
	    	bsf     BiToggleRegister, BIT3
		endif
 			goto    BiToggleSearchEnds

BiTogglePin11
 		ifdef PIN14_PORT
			SetBnkF	TRISA                           ; Note : Absolute reference !
			bsf		PIN14_PORT, PIN14
			SetBank0Auto
	  		bsf     BiToggleRegister, BIT2
		endif
       		goto    BiToggleSearchEnds

BiTogglePin10
		ifdef PIN13_PORT
			SetBnkF	TRISA                           ; Note : Absolute reference !
			bsf		PIN13_PORT, PIN13
			SetBank0Auto
			bsf     BiToggleRegister, BIT1
		endif
   			goto    BiToggleSearchEnds

BiTogglePin9
 		ifdef PIN12_PORT
			SetBnkF	TRISA                           ; Note : Absolute reference !
			bsf		PIN12_PORT, PIN12
			SetBank0Auto
            bsf     BiToggleRegister, BIT0
		endif
			goto    BiToggleSearchEnds


    		; W = mode - CONF_BI_TOGGLE_MODE
TestForBootDelay
            addlw   VALUE_CONF_BI_TOGGLE_MODE_BOOT_DELAY - VALUE_CONF_BI_TOGGLE_MODE
            skpz
            goto    BiToggleNextPass


			; Found a bitoggle boot delay pin.
			; ------------------------------------------------------------
			; Set "BiToggleRegBootDelay" bit.
			; Reuses code from bitoggle above
			;
BiToggleJump2
			movlw	BiToggleJump2 / 0x100
			movwf	PCLATH
			 ;
			decf	BiToggleTemp, W         ; BiToggleTemp runs from 4 to 1
			addwf	PCL, F

            goto    BiToggleBootDelayPin9
            goto    BiToggleBootDelayPin10
            goto    BiToggleBootDelayPin11
            goto    BiToggleBootDelayPin12

BiToggleJumpEnds2
		if BiToggleJump2 / 0x0100 != BiToggleJumpEnds2 / 0x0100
		  	error "Page fault in 725.asm #1797"
		endif

BiToggleBootDelayPin12
            bsf     BiToggleRegBootDelay, BIT3
            goto    BiTogglePin12

BiToggleBootDelayPin11
            bsf     BiToggleRegBootDelay, BIT2
            goto    BiTogglePin11

BiToggleBootDelayPin10
            bsf     BiToggleRegBootDelay, BIT1
            goto    BiTogglePin10

BiToggleBootDelayPin9
            bsf     BiToggleRegBootDelay, BIT0
            goto    BiTogglePin9



BiToggleNextPass
            call    GetNextCodeAndConf
            goto    BiToggleLoop


BiToggleSearchEnds

            decfsz  BiToggleTemp, F
            goto    LoopAllFour

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

		endif ; ifdef CONF_BI_TOGGLE_MODE

			return





		; -------------------------------------------------------------------
		; FilterHighBiToggleBootDelayPins
		; -------------------------------------------------------------
		; Remove any high levels on delayed bitoggle pins in output mask,
		; modify 'BiToggleRegBootDelay' to contain queued bits only.
		;
		; Dependency : LocateBiTogglePins() must have defined

FilterHighBiToggleBootDelayPins

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

             ; Find boot delay pins which should be delayed high level
             ;
            movf    OutputsMask32+1, W
            andwf   BiToggleRegBootDelay, F

             ; Now clear these in current output mask

            comf    BiToggleRegBootDelay, W
            andwf   OutputsMask32+1, F

             ; After timer expires the BiToggleRegBootDelay now should be
             ; or'd into OutputsMask32+1

            return



		; -------------------------------------------------------------
		; LocateRadioNextPrevPins
		; -------------------------------------------------------------
		; Traverse assignmentstable and find any RadioNext or RadioPrev
		; pins.
		;
LocateRadioNextPrevPins
	ifdef CONF_RADIO_NEXT
	asd
			movlw	CONF_RADIO_NEXT
			call	FindModePinInTable
			skpc
			goto	lrnpp_check_prev
			; Found a radio next pin
			movwf	RadioModeNextPin
			bsf		RadioModeNextPin, BIT7

			SetBnkF	TRISC						; Just the tris bank in general
			bsf		PIN0_PORT, PIN0
			SetBank0Auto
			bsf		OutputsMask32+.0, BIT0

lrnpp_check_prev

			movlw	CONF_RADIO_PREV
			call	FindModePinInTable
			skpc
			goto	lrnpp_check_exit
			; Found a radio prev pin
			movwf	RadioModePrevPin
			bsf		RadioModePrevPin, BIT7

			SetBnkF	TRISC						; Just the tris bank in general
			bsf		PIN1_PORT, PIN1
			SetBank0Auto
			bsf		OutputsMask32+.0, BIT1

lrnpp_check_exit
	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

	    endif
			return


		; -------------------------------------------------------------------
		; InitializeAllOutputs
		; ClearAllOutputs
		; -------------------------------------------------------------
		; This rutine is needed both at boot but also when entering
		; programming mode. All the 16 decoder pins are set to low-level outputs.
		; Note that this must be succeded with "LocateBiTogglePins"
		; if bitoggle mode is enabled.
		;
InitializeAllOutputs

			; Port A and B programmed
			movlw	TrisA
			settris	TRISA
			movlw	TrisB
			settris	TRISB
			movlw	TrisC
			settris	TRISC

ClearAllOutputs
			; Low output pins
			;
			clrf    OutputsMask32+0
			clrf    OutputsMask32+1
			clrf    OutputsMask32+2
			clrf    OutputsMask32+3

          	call    SetOutputsFromMasks

	        return

;;;-----------------------------------------------------------------------
;;; D E B U G
;;;-----------------------------------------------------------------------


FirstRunSetups

			di

			; Check for first run
			movlw	EepFirstRunTest
			call	EepromReadW
			sublw	0xAA

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			skpnz
			return

			 ; First run
			 ;
			movlw	EepFirstRunTest
			call	EepromSetAddress
			movlw	0xAA
			call	EepromWriteW

ClearSystem

			 ; Reset Table and EepTable
			 ;
	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			call	ResetTable

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			call	ClearTableOutputLevels

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			call    SetOutputsFromMasks

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			return



;;;-----------------------------------------------------------------------
;;; M A C R O    S U B R U T I N E S
;;;-----------------------------------------------------------------------


msDelay		MDelay_Xms	1, t1, t2, t3
			return



		; FatalStop
		; -------------------------------------------------------------
		; FatalHalt from 'Common.inc'

FatalStop
			FatalHalt

			include "I2C.inc"

	        include "PICEEPROM.inc"


;;;-----------------------------------------------------------------------
;;;-----------------------------------------------------------------------
;;;
;;; 	Interrupt service
;;;
;;;-----------------------------------------------------------------------
;;;-----------------------------------------------------------------------
;
;
IntrRutine
			; Context save is done at vector entry point

		if DEBUG_ISR_ALIVE
			btfss	RC5DEBUGPORT, RC5DEBUGPIN
			goto	ghi
			bcf		RC5DEBUGPORT, RC5DEBUGPIN
			goto	gex
ghi
			bsf		RC5DEBUGPORT, RC5DEBUGPIN
gex
		endif

LoopedISR
			; By clearing timer flag here a next interupt can be
			; registered while in the ISR pass here
			bcf		INTCON, T0IF		; INTCON is in all banks
			SetBank0
			clrwdt

			 ; Led Tick generator
			 ; ----------------------------------------
			 ;
			decfsz	ISRSlowPrescalerCnt, F
			goto	led_signal_ends
			 ;
			 ; Signal a tick
			 ;
			incf	RealtimeTickCounter, F
			movlw	ISR_SLOW_PRESCALER
			movwf	ISRSlowPrescalerCnt
led_signal_ends


		 ; RC5 reception state machine
		 ; ----------------------------------------

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			IR_IsrCode


		; RTCC reload
		; ----------------------------------------
		; If C = 1 after the sub then the RTCC didn't get below 0 and an
		; interrupt will be missed...

			movf	RtccReloadValue, W
			subwf	TMR0, F

			skpnc
			bsf		INTCON, T0IF    ; Don't miss the interupt..


			 ; Let the ISR loop internally if needed to minimize switching overhead
			 ; ----------------------------------------

			btfss	INTCON, T0IF		; INTCON is in all banks
			goto	IntrExit

	       	if ICE	; -------------------------------------
			nop		; Breakpoint : Isr looped
			nop		;
	    	endif	; -------------------------------------

			goto	LoopedISR




		; Interrupt exit
		; -----------------------------------------------------------

IntrExit
			movf	IntrContextP, W
			movwf	PCLATH			; Restore PCLATH
			swapf	IntrContextS, W
			movwf	STATUS			; Restore swapped STATUS
			swapf	IntrContextW, F
			swapf	IntrContextW, W	; Restore W (leaving STATUS)
			retfie




			include "RC5UTIL.asm"


			org	FLASH_CODE_PAGE

			PicFlashCode


	; FLASH_STORE_ADRESS
	; -------------------------------------------------------------------

			data	" f l a s h s t a r t :"

FLASH_TABLE_SIZE		equ	TABLE_NOF_CELLS*TABLE_CELL_BYTES

FLASH_STORE_ADRESS

FLASH_STORE_ADRESS_END	equ	FLASH_STORE_ADRESS + FLASH_TABLE_SIZE

			END

