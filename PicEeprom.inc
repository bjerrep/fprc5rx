;
; ========================================================================
; PICEEPROM.INC
; ------------------------------------------------------------------------
; Common read/writes and utility macros to pic internal eeprom data.
; Default subrutine implementations are inlined unless actively disabled.
; RAM : None.
;
; Verified on : 16F873
; Requires common.inc
;
; NOTENOTENOTE : MANUALLY DISABLE INTERUPTS WHEN PERFORMING EEPROM WRITES !!!!
;
; 001 - 2002-05-07      : Versioning started
; ========================================================================
;
	nolist


;;; --------------------------------------------------------------
;;;   E E P R O M   M A C R O S
;;; --------------------------------------------------------------
;;;

	; WritePicEeprom
	; -------------------------------------------------------------------
	; Input   : Data in EEDATA and adress in EEADR
	; ****** NOTE : Interrupts must be disabled ! *******
	; A write cycle take typ. 10 ms - See WritePicWait
	;
WritePicEeprom	macro

			SetBnkF	EECON1
		ifdef EEPGD
			bcf		EECON1 & FORCE_7_BIT_ADR, EEPGD
		endif
			bsf		EECON1 & FORCE_7_BIT_ADR, WREN
			movlw	0x55
			movwf	EECON2 & FORCE_7_BIT_ADR
			movlw	0xAA
			movwf   EECON2 & FORCE_7_BIT_ADR
			bsf		EECON1 & FORCE_7_BIT_ADR, WR

			 ; disable eep write enable
			 ;
			bcf		EECON1 & FORCE_7_BIT_ADR, WREN
			endm


	; WritePicWait
	; -------------------------------------------------------------------
	; Returns when eeprom write is completed
	; NOTE : Interrupts must be disabled !
	;
WritePicWait	macro
			local	eep_not_ready
			SetBnkF	EECON1
		 	;
eep_not_ready
			btfsc	EECON1 & FORCE_7_BIT_ADR, WR
			goto	eep_not_ready
			endm


	; EepromIdle
	; -------------------------------------------------------------------
	; Returns C=1 if eeprom is idle
	;
EepromIdle      macro
			SetBnkF	EECON1
			setc
			btfsc	EECON1 & FORCE_7_BIT_ADR, WR
			clrc
 			SetBank0Auto
			endm


	; ReadPicEeprom
	; -------------------------------------------------------------------
	; Input   : Address in W
	; Output  : Data in W (and EEDATA)
	;
ReadPicEeprom	macro

			SetBnkF	EECON1
		ifdef EEPGD
			bcf		EECON1 & FORCE_7_BIT_ADR, EEPGD
		endif
			bsf		EECON1 & FORCE_7_BIT_ADR, RD
			;
			SetBnkA	EEDATA
			movf	EEDATA & FORCE_7_BIT_ADR, W

			endm


			list


;;; --------------------------------------------------------------
;;;   E E P R O M   S U B R U T I N E   I M P L E M E N T A T I O N S
;;; --------------------------------------------------------------
;;;
;;; Expands to inline code unless 'EEPROM_NO_IMPLEMENTATION' is defined.
;;; Returns with ram bank 0
;;; See PIC_EEPROM_TRAPS for debug trap code


	ifndef EEPROM_NO_IMPLEMENTATION

	; -------------------------------------------------------------
	; EepromWriteW
	; -------------------------------------------------------------
	; Blocking write to internal EEPROM - waits for write cycle to complete.
	; Input   : Data to write in W. See 'EepromSetAddress' for setting the address.
	; NOTE NOTE NOTE - Interupts must be disabled during eeprom writes !!!
	;
EepromWriteW

		ifdef PIC_EEPROM_TRAPS
			nop
			btfsc 	INTCON, GIE
			goto	$-2
		endif
			SetBnkF	EEDATA
			movwf	EEDATA & FORCE_7_BIT_ADR
			WritePicEeprom
			WritePicWait
			SetBank0Auto
			return

	; -------------------------------------------------------------
	; EepromReadW
	; -------------------------------------------------------------
	; Reads internal EEPROM
	; Input   : Adress to be read in W.
	;
EepromReadW	SetBnkF	EEADR
			movwf	EEADR & FORCE_7_BIT_ADR
			ReadPicEeprom
			SetBank0Auto
			return


	; -------------------------------------------------------------
	; EepromSetAddress
	; -------------------------------------------------------------
	; Sets the address for the next eeprom write.
	; Input   : Adress in W.
	;
EepromSetAddress
			SetBnkF	EEADR
			movwf	EEADR & FORCE_7_BIT_ADR
			SetBank0Auto
			return


	; -------------------------------------------------------------
	; EepromAdressAdvance
	; -------------------------------------------------------------
	; Sets the address for the next eeprom write.
	; Input   : Adress in W.
	;
        ifdef EnableEepromAdressAdvance
EepromAdressAdvance
			SetBnkF	EEADR
			incf	EEADR & FORCE_7_BIT_ADR, F
			SetBank0Auto
			return
        endif


	endif ; ifndef EEPROM_NO_IMPLEMENTATION

; ========================================================================
; End of PicEeprom.Inc
; ========================================================================

