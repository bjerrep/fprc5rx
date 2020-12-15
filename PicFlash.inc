;
; ========================================================================
; P I C F L A S H . I N C
; ------------------------------------------------------------------------
; Supports 8 and 14 bit read/writes to pic program flash memory.
; Verified on : 16F873. Misses the 4 byte block mode for 'A' devices.
; 001 - 2002-05-07 : Initial version
; 002
; ========================================================================
;
	nolist
; General layout is a variable declaration macro and a code macro
; Variable definition macros : (one of only)
;   PicFlashVariables8bit	Supports one byte per address
;   PicFlashVariables14bit  Supports one word (14bit) per address
;
; Code definitions macro :
;   PicFlashZeroPage
;       FlashAdrSetZeroPage
;   PicFlashUtility
;
; Subrutines
;  FlashAddressNext
;  FlashAddressPrev
;  FlashRead
;  FlashWriteByteW
;  FlashWrite



; PicFlashVariables8bit
; -------------------------------------------------------------------
;
PicFlashVariables8bit	macro

PicFlashDataSize		equ	.3

	cblock
	FLASH_FILE:PicFlashDataSize
	endc

FlashAdrLow				equ	FLASH_FILE + .0
FlashAdrHigh			equ	FLASH_FILE + .1
FlashDataLow			equ	FLASH_FILE + .2

	endm

; PicFlashVariables14bit
; -------------------------------------------------------------------
;
PicFlashVariables14bit	macro

PicFlashDataSize		equ	.4

	cblock
	FLASH_FILE:PicFlashDataSize
	endc

FlashAdrLow				equ	FLASH_FILE + .0
FlashAdrHigh			equ	FLASH_FILE + .1
FlashDataLow			equ	FLASH_FILE + .2
FlashDataHigh			equ	FLASH_FILE + .3

	endm




PicFlashCode	macro

CODE_SIZE_FLASH_START

FlashSetAdrZeroPage
			movwf	FlashAdrLow
			movlw	FLASH_STORE_ADRESS % 0x100
			addwf	FlashAdrLow, F

			clrf	FlashAdrHigh
			skpnc
			incf	FlashAdrHigh, F
			movlw	FLASH_STORE_ADRESS / 0x100
			addwf	FlashAdrHigh, F
			return



		ifndef CODE_SIZE_FLASH_START
CODE_SIZE_FLASH_START

		endif


FlashAddressNext
			incf    FlashAdrLow, F
			skpz
			return
			incf    FlashAdrHigh, F
			return


FlashAddressPrev
			movlw   1
			subwf   FlashAdrLow, F
			skpnc
			return
			decf    FlashAdrHigh, F
			return






;	FlashRead
; -------------------------------------------------------------------
;	In   : Adress in FlashAdrLow/FlashAdrHigh
;	Out  : 14 bit data in FlashDataLow/FlashDataHigh
;            ; W = FlashDataLow (8bit)
;	NOTE : Treats flash as a 8 bit wide store if FlashDataHigh is undefined

FlashRead

			SetBnkF FlashAdrLow
			movf    FlashAdrLow, W
			SetBnkA EEADR
			movwf   EEADR & FORCE_7_BIT_ADR
			SetBnkA FlashAdrHigh
			movf    FlashAdrHigh, W
			SetBnkA EEADRH
			movwf   EEADRH & FORCE_7_BIT_ADR

			SetBnkA EECON1
			bsf     EECON1 & FORCE_7_BIT_ADR, EEPGD ;Point to Program memory
			bsf     EECON1 & FORCE_7_BIT_ADR, RD ;Start read operation
			nop
			nop

			SetBnkA EEDATA
			movf    EEDATA & FORCE_7_BIT_ADR, W ;DATAL = EEDATA
			SetBnkA FlashDataLow
			movwf   FlashDataLow

		ifdef FlashDataHigh
			SetBnkA EEDATH
			movf    EEDATH & FORCE_7_BIT_ADR, W ;DATAL = EEDATA
			SetBnkA FlashDataHigh
			movwf   FlashDataHigh
		endif

			SetBnkA EECON1
			bcf     EECON1 & FORCE_7_BIT_ADR, EEPGD ;Point to Program memory
			SetBank0Auto

			movf	FlashDataLow, W

			return




;	FlashWrite
; -------------------------------------------------------------------
;	In   : Data in FlashAdrXXX and FlashDataXXX
;	Out  : Garbage
;	NOTE : Leaves with interupts disabled !


FlashWriteByteW

			movwf	FlashDataLow

FlashWrite

 			SetBnkF FlashAdrLow
			movf    FlashAdrLow, W
			SetBnkA EEADR
			movwf   EEADR & FORCE_7_BIT_ADR
			SetBnkA FlashAdrHigh
			movf    FlashAdrHigh, W
			SetBnkA EEADRH
			movwf   EEADRH & FORCE_7_BIT_ADR

			SetBnkA FlashDataLow
			movf    FlashDataLow, W
			SetBnkA EEDATA
			movwf   EEDATA & FORCE_7_BIT_ADR

		ifdef FlashDataHigh
			SetBnkA FlashDataHigh
 			movf    FlashDataHigh, W
 		else
			clrw
		endif
			SetBnkA EEDATH
			movwf   EEDATH & FORCE_7_BIT_ADR

    		SetBnkA EECON1
			bsf     EECON1 & FORCE_7_BIT_ADR, EEPGD ; Point to Program memory
			bsf     EECON1 & FORCE_7_BIT_ADR, WREN

    		SetBnkA INTCON
     		bcf     INTCON, GIE 					; if already enabled,

       		SetBnkA EECON2
  			movlw   0x55							; First magic number write
       		movwf   EECON2 & FORCE_7_BIT_ADR
     		movlw   0xAA 							; Second magic number write
			movwf   EECON2 & FORCE_7_BIT_ADR

        	SetBnkA EECON1
 			bsf     EECON1 & FORCE_7_BIT_ADR, WR 	; Start write operation
     		nop
    		nop
			bcf     EECON1 & FORCE_7_BIT_ADR, WREN 	; Disable writes

			SetBank0Auto

			return


		if CODE_SIZE_FLASH_START / 0x800 != $ / 0x800
			error("PicFlash.inc : Page crossing")
		endif


CODE_SIZE_FLASH		equ	$ - CODE_SIZE_FLASH_START


	endm	; PicFlashCode

; ========================================================================
;	E N D    O F    F L A S H . I N C
; ========================================================================


	list



