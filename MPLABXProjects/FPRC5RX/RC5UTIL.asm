;
; ========================================================================
; RC5UTIL.ASM
;
; Version history
; 001	Initial.
; ========================================================================
;

;
; LookForShortedPins			Returns first pin that user has shorted
; SetBitInOutputMask
; ClearBitInOutputMask
; ReverseBitInOutputMask
; ClearBitInOutputMask	  		Clears the specified bit in outputs mask
; CheckForRadioAssignement		Check if the specified pin has a radio assignment
; AddPinDataToTable
; FindCodeAndConfAndPinInTable
; FindModePinInTable			Returns the first pin used for the assignment with specified mode.
; ResetTable
; DefineWriteNullCell
; ClearPinFromTable
; TestIfCellValid
; ClearRadioChannels
; NextRadioPin
; PrevRadioPin
; GetFirstPinNumberFromIRCode
; ReadModeConf
; ModeConf2Int					Use ReadModeConf instead
; ModeInt2Conf
; ActivateOutputMask
; DeactivateOutputMask
; SaveCurrentOutputLevels		The state of the output pins are saved in eeprom if changed
; RestoreOutputLevelMask
; GetFirstCodeAndConf			Finds the IR code and conf in table from pin number
; GetNextCodeAndConf
; CheckForRadioAssignment
; SetOutputsFromMasks			Sets the 16 output pin levels as given by "OutputsMaskXxx"
;


		; TestOutputPin - Macro
		; ---------------------------------------------------
		;
		;
TestOutputPin	macro	port, bit, pin

			btfss	mask, bit
			goto	$ + 4
			btfss	port, pin
			goto	FoundUnexpectedPin	; they differ
			goto	$ + 4				; both high

			btfss	port, pin
			goto	$ + 2				; both low
			goto	FoundUnexpectedPin	; they differ

			addlw	1
			endm





		; LookForShortedPins
		; ---------------------------------------------------
		; Returns first pin that has unexpected level from a user short
		; to either gnd or vcc.
		; Scans output pins sequentially from 0 to 15.
		;
		; Output : W = pin number (0-15)
		;          C = 1 if pin where found, else C = 0
		;

LookForShortedPins

mask 		set  OutputsMask32+0

			clrw

			TestOutputPin	PIN0_PORT, BIT0, PIN0
		ifdef PIN1_PORT
			TestOutputPin	PIN1_PORT, BIT1, PIN1
		endif
		ifdef PIN2_PORT
			TestOutputPin	PIN2_PORT, BIT2, PIN2
			TestOutputPin	PIN3_PORT, BIT3, PIN3
		endif
		ifdef PIN4_PORT
			TestOutputPin	PIN4_PORT, BIT4, PIN4
			TestOutputPin	PIN5_PORT, BIT5, PIN5
			TestOutputPin	PIN6_PORT, BIT6, PIN6
			TestOutputPin	PIN7_PORT, BIT7, PIN7

mask 		set  OutputsMask32+1

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			TestOutputPin	PIN8_PORT, BIT0, PIN8
			TestOutputPin	PIN9_PORT, BIT1, PIN9
			TestOutputPin	PIN10_PORT, BIT2, PIN10
			TestOutputPin	PIN11_PORT, BIT3, PIN11

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			TestOutputPin	PIN12_PORT, BIT4, PIN12
		if ICE == false
		 	messg "Info: 16 pins available"
			TestOutputPin	PIN13_PORT, BIT5, PIN13
			TestOutputPin	PIN14_PORT, BIT6, PIN14
			TestOutputPin	PIN15_PORT, BIT7, PIN15
		endif
		endif

			; No differences
			clrc
			goto    LookForShortedPinsExit



FoundUnexpectedPin

		ifdef BI_TOGGLE_DEFINED

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			 ; If bitoggle slave pin and running in normal mode
			 ; then remap to master pin.

			btfsc   Flags, PROGRAM_MODE_ON
			goto    BitoggleEnds

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

            movwf   TempIndex

            movlw   .12
            subwf   TempIndex, W
            skpz
            goto    not@12
            btfss   BiToggleRegister, BIT0
            goto    exit_bitoggle
            goto    remap_bitoggle
not@12

            movlw   .13
            subwf   TempIndex, W
            skpz
            goto    not@13
            btfss   BiToggleRegister, BIT1
            goto    exit_bitoggle
            goto    remap_bitoggle
not@13

            movlw   .14
            subwf   TempIndex, W
            skpz
            goto    not@14
            btfss   BiToggleRegister, BIT2
            goto    exit_bitoggle
            goto    remap_bitoggle
not@14

            movlw   .15
            subwf   TempIndex, W
            skpz
            goto    not@15
            btfss   BiToggleRegister, BIT3
            goto    exit_bitoggle
            goto    remap_bitoggle
not@15
            goto    exit_bitoggle

remap_bitoggle
            movlw   .4
            subwf   TempIndex, F

exit_bitoggle
            movf    TempIndex, W

BitoggleEnds

		endif ; BI_TOGGLE_DEFINED

			setc

LookForShortedPinsExit

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------
			return



		; SetBitInOutputMask
		; ---------------------------------------------------
		; Input  : W = Bit number to set (0-15)
		; Output : "OutputsMaskxxx" (a word) has the corresponding bit set.
		;
SetBitInOutputMask

			call	CheckIfPinIsInputOnly
			skpnc
			return

			call	BitToMask

			movf	ShiftTempQuad+0, W
			iorwf	OutputsMask32+0, F

			movf	ShiftTempQuad+1, W
			iorwf	OutputsMask32+1, F

			movf	ShiftTempQuad+2, W
			iorwf	OutputsMask32+2, F

			movf	ShiftTempQuad+3, W
			iorwf	OutputsMask32+3, F

			return



		; ClearBitInOutputMask
		; ---------------------------------------------------
		; Clears the specified bit in outputs mask
		; Input  : W = Bit number to clear (0-15)
		; Output : "OutputsMaskxxx" (a word) has the corresponding bit cleared
		;
ClearBitInOutputMask

			call	CheckIfPinIsInputOnly
			skpnc
			return

			call	BitToMask

			comf	ShiftTempQuad+0, W
			andwf	OutputsMask32+0, F

			comf	ShiftTempQuad+1, W
			andwf	OutputsMask32+1, F

			comf	ShiftTempQuad+2, W
			andwf	OutputsMask32+2, F

			comf	ShiftTempQuad+3, W
			andwf	OutputsMask32+3, F

			return




		; ReverseBitInOutputMask
		; ---------------------------------------------------
		; Input  : W = Bit number to reverse (0-15)
		; Output : "OutputsMaskxxx" (a word) has the corresponding bit cleared
		;
ReverseBitInOutputMask

			call	CheckIfPinIsInputOnly
			skpnc
			return


			call	BitToMask

			movf	ShiftTempQuad+0, W
			xorwf	OutputsMask32+0, F

			movf	ShiftTempQuad+1, W
			xorwf	OutputsMask32+1, F

			movf	ShiftTempQuad+2, W
			xorwf	OutputsMask32+2, F

			movf	ShiftTempQuad+3, W
			xorwf	OutputsMask32+3, F

			return



		; -------------------------------------------------------------------
		; CheckIfPinIsInputOnly
		; -------------------------------------------------------------
		; Checks if the specified pin is a BiToggle slave input.
		;
		; Input  : Pin in W
		; Output : C=1 pin is a bitoggle slave
		;          C=0 pin is not a bitoggle slave
		;		   Pin in W (NOTE!)
		; Modify : TempIndex
		;

CheckIfPinIsInputOnly

			movwf	TempStack		; Push W

		ifdef CONF_RADIO_NEXT
            movlw   .0
            subwf   TempStack, W
            skpz
            goto    _not@0
			btfss	RadioModeNextPin, BIT7
            goto    pin_is_output
            goto    pin_is_input
_not@0
		endif

		ifdef CONF_RADIO_PREV
            movlw   .1
            subwf   TempStack, W
            skpz
            goto    _not@1
			btfss	RadioModePrevPin, BIT7
            goto    pin_is_output
            goto    pin_is_input
_not@1
		endif

            movlw   .12
            subwf   TempStack, W
            skpz
            goto    _not@12
            btfss   BiToggleRegister, BIT0
            goto    pin_is_output
            goto    pin_is_input
_not@12

            movlw   .13
            subwf   TempStack, W
            skpz
            goto    _not@13
            btfss   BiToggleRegister, BIT1
            goto    pin_is_output
            goto    pin_is_input
_not@13

            movlw   .14
            subwf   TempStack, W
            skpz
            goto    _not@14
            btfss   BiToggleRegister, BIT2
            goto    pin_is_output
            goto    pin_is_input
_not@14

            movlw   .15
            subwf   TempStack, W
            skpz
            goto    _not@15
            btfss   BiToggleRegister, BIT3
            goto    pin_is_output
            goto    pin_is_input
_not@15

            goto    pin_is_output

pin_is_input
			setc
			goto	CheckIfPinExit

pin_is_output
			clrc

CheckIfPinExit

			movf	TempStack, W
			return

			; CheckIfPinIsInputOnly



		; BitToMask
		; ---------------------------------------------------
		; Set bit at pos given by W. If W > 15 then all zeroes are
		; output.
		; Input  : W = Bit number to set.
		; Output : ShiftTemp (16bit) has requested bit set to high

BitToMask

			addlw	1
			 ;
			clrf	ShiftTempQuad+0
			clrf	ShiftTempQuad+1
			clrf	ShiftTempQuad+2
			clrf	ShiftTempQuad+3


			setc
			 ;
shift_again	rlf		ShiftTempQuad+0, F
			rlf		ShiftTempQuad+1, F
			rlf		ShiftTempQuad+2, F
			rlf		ShiftTempQuad+3, F
			 ;
			addlw	0xFF		; Subtract one from W....
			clrc
			skpz
			goto	shift_again

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
	    	endif	; -------------------------------------

			return





		; -------------------------------------------------------------------
		; SetTablePointer
		; -------------------------------------------------------------
		;
		; Input  : Cell index in W
		; Output : Relative offset of first cell byte in table
		;

SetTablePointer
			movwf	TablePointer
			clrc
			rlf		TablePointer, F
			rlf		TablePointer, F

		if FLASH_TABLE == 0
			movlw	AssignmentsTable
			addwf	TablePointer, F
		endif

			return






		; -------------------------------------------------------------------
		; PokeTableCell
		; -------------------------------------------------------------
		;
		; Input  :
		; Output :
		;

PokeTableCell

	if FLASH_TABLE

			movf	TablePointer, W

			; ToDo : looped code to save space....

			di
			SetPgeF	FLASH_CODE_PAGE
			call	FlashSetAdrZeroPage & __PAGE__
			SetPage0Auto

			movf	Write_Low, W

			SetPgeF	FLASH_CODE_PAGE
			call	FlashWriteByteW & __PAGE__
			call	FlashAddressNext & __PAGE__
			SetPage0Auto

			movf	Write_High, W

			SetPgeF	FLASH_CODE_PAGE
			call	FlashWriteByteW & __PAGE__
			call	FlashAddressNext & __PAGE__
			SetPage0Auto

			movf	Write_Conf_Mode, W

			SetPgeF	FLASH_CODE_PAGE
			call	FlashWriteByteW & __PAGE__
			call	FlashAddressNext & __PAGE__
			SetPage0Auto

			movf	Write_Conf_Pin, W

			SetPgeF	FLASH_CODE_PAGE
			call	FlashWriteByteW & __PAGE__
			SetPage0Auto
			ei

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			return


		else

			zsdsadad

			movf	TablePointer, W
			call	SetEepAdr
			SetBank0
			movf	Write_Low, W
			call	EepWriteW
			SetBank0

			movf	TablePointer, W
			addlw	1
			call	SetEepAdr
			SetBank0
			movf	Write_High, W
			call	EepWriteW
			SetBank0

			movf	TablePointer, W
			addlw	2
			call	SetEepAdr
			SetBank0
			movf	Write_Conf, W
			call	EepWriteW
			SetBank0

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			return
		endif



		; -------------------------------------------------------------------
		; PeekTableCell
		; PeekTableCellPin
		; PeekTableCellIRLow
		; -------------------------------------------------------------
		;
		; Input  : 'TablePointer'
		; Output : Read_XXX byte(s) are set
		;

PeekTableCellPin

			movf	TablePointer, W
			addlw	3
			di
			SetPgeF	FLASH_CODE_PAGE
			call	FlashSetAdrZeroPage & __PAGE__
			call	FlashRead & __PAGE__
			SetPage0Auto
			ei

			movwf	Read_Conf_Pin

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			return


PeekTableCellIRLow


			movf	TablePointer, W

			di
			SetPgeF	FLASH_CODE_PAGE
			call	FlashSetAdrZeroPage & __PAGE__
			call	FlashRead & __PAGE__
			SetPage0Auto
			ei

			movwf	Read_Low

			return


PeekTableCell

	if FLASH_TABLE

			movf	TablePointer, W

			di
			SetPgeF	FLASH_CODE_PAGE
			call	FlashSetAdrZeroPage & __PAGE__
			call	FlashRead & __PAGE__
			SetPage0Auto
			ei

			movwf	Read_Low

			di
			SetPgeF	FLASH_CODE_PAGE
			call	FlashAddressNext & __PAGE__
			call	FlashRead & __PAGE__
			SetPage0Auto

			movwf	Read_High

			SetPgeF	FLASH_CODE_PAGE
			call	FlashAddressNext & __PAGE__
			call	FlashRead & __PAGE__
			SetPage0Auto
			ei

			movwf	Read_Conf_Mode

			di
			SetPgeF	FLASH_CODE_PAGE
			call	FlashAddressNext & __PAGE__
			call	FlashRead & __PAGE__
			SetPage0Auto
			ei

			movwf	Read_Conf_Pin

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			return
	else

			movf	TablePointer, W
			call	EepReadW
			SetBank0
			movwf	Read_Low

			movf	TablePointer, W
			addlw	1
			call	EepReadW
			SetBank0
			movwf	Read_High

			movf	TablePointer, W
			addlw	2
			call	EepReadW
			SetBank0
			movwf	Read_Conf

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			return

		endif




		; -------------------------------------------------------------------
		; AddPinDataToTable
		; -------------------------------------------------------------
		;
		; Input  : Data in "Write_xxx" including pin number
		; Output : C=1 succes, C=0 no room
		;
AddPinDataToTable
			movlw	TABLE_NOF_CELLS
			movwf	Counter
			 ;
			clrf	TempIndex


etst_next_word
			movf	TempIndex, W
			call	SetTablePointer
			call	PeekTableCell

			call	TestIfCellValid

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			skpnc
			goto	not_null_cell

			 ; Found a clear spot

			call	PokeTableCell

			 ; Succes return
			setc
			return

not_null_cell

			incf	TempIndex, F

			decfsz	Counter, F
			goto	etst_next_word

			 ; Error return, table full
			clrc
			return




		; -------------------------------------------------------------------
		; FindCodeAndConfAndPinInTable
		; -------------------------------------------------------------
		;
		; Input  : Data to find in "Write_Low/_High/_Conf/_Pin"
		; Output : C=1 found, C=0 not found
		;

FindCodeAndConfAndPinInTable

			movlw	TABLE_NOF_CELLS
			movwf	Counter
			 ;
			clrf	TempIndex


etst_next_wod
			movf	TempIndex, W
			call	SetTablePointer
			call	PeekTableCell

			movf	Read_Low, W
			subwf	Write_Low, W
			skpz
			goto	fcc_not_this

			movf	Read_High, W
			subwf	Write_High, W
			skpz
			goto	fcc_not_this

			movf	Read_Conf_Mode, W
			subwf	Write_Conf_Mode, W
			skpz
			goto	fcc_not_this

			movf	Read_Conf_Pin, W
			subwf	Write_Conf_Pin, W
			skpz
			goto	fcc_not_this

			 ; Found the code, return with carry set
			 ;
			setc
			return


fcc_not_this
			incf	TempIndex, F

			decfsz	Counter, F
			goto	etst_next_wod

			 ; Not found, return with carry clear
			 ;
			clrc
			return


		; -------------------------------------------------------------------
		; FindModePinInTable
		; -------------------------------------------------------------------
		; Returns the first pin used for the assignment with specified mode.
		; Made for finding RadioNext/Prev pins.
		; Input   : The mode to find in W
		; Output  : Pin in W (0..31) and C=1 if found, C=0 if not found
		; Destroy : Write_Conf_Mode
		;
FindModePinInTable

			movwf	Write_Conf_Mode

			movlw	TABLE_NOF_CELLS
			movwf	Counter
			clrf	TempIndex

fmpt_next
			movf	TempIndex, W
			call	SetTablePointer
			call	PeekTableCell

			call	ReadModeConf
			subwf	Write_Conf_Mode, W
			skpz
			goto	fmpt_not_this

			 ; Found the code, return with carry set
			 ;
			movf	Read_Conf_Pin, W
			setc
			goto	fmpt_exit


fmpt_not_this
			incf	TempIndex, F

			decfsz	Counter, F
			goto	fmpt_next

			 ; Not found, return with carry clear
			 ;
			clrc
fmpt_exit
	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------
			return




		; -------------------------------------------------------------------
		; ResetTable
		; -------------------------------------------------------------
		; Resets the assignments table (in program flash memory)
		;
		; Input  :
		; Output :
		;

ResetTable	movlw	TABLE_NOF_CELLS
			movwf	Counter

			clrf	TempIndex

			call	DefineWriteNullCell

etst_next_wor
			movf	TempIndex, W
			call	SetTablePointer

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			call	PokeTableCell

			incf	TempIndex, F

			decfsz	Counter, F
			goto	etst_next_wor

			return



		; -------------------------------------------------------------------
		; DefineWriteNullCell
		; -------------------------------------------------------------
		;
		; Input  :
		; Output :
		;

DefineWriteNullCell

			movlw	0xFF			; ToDo -> make subrutine
			movwf	Write_Low
			movwf	Write_High
			clrf	Write_Conf_Mode
			clrf	Write_Conf_Pin
			return



		; -------------------------------------------------------------------
		; ClearPinFromTable
		; ------------------------------------------------------------
		; Removes all instances of this pin in table by writing a NULL entry
		;
		; Input  : W = pin number (0-31)
		; Output :
		;


ClearPinFromTable

			movwf	Pin

			movlw	TABLE_NOF_CELLS
			movwf	Counter

			call	DefineWriteNullCell

rclr_next_word
			 ; From highest down to lowest index in table
			decf	Counter, W
			call	SetTablePointer
			call	PeekTableCell

			movlw	0x1F
			andwf	Read_Conf_Pin, W
			xorwf	Pin, W

			skpz
			goto	not_thisr

			 ; Found pin - erase

			call	PokeTableCell


not_thisr
			decfsz	Counter, F
			goto	rclr_next_word
			return
			; ClearPinFromTable





		; -------------------------------------------------------------
		; TestIfCellValid
		; -------------------------------------------------------------
		; Legal to call after a table read. The test will fail on a 0xFFFF remote....
		; See DefineWriteNullCell()
		; Input  :
		; Output : C=1 valid, C=0 not valid cell i.e. empty
		;

TestIfCellValid

			movf	Read_Low, W
			sublw	0xFF
			skpz
			goto	has_data

			movf	Read_High, W
			sublw	0xFF
			skpz
			goto	has_data

			clrc
			return
has_data
			setc
			return







		; -------------------------------------------------------------------
		; ClearRadioChannels
		; -------------------------------------------------------------
		; Clears all radio pins except for the current active given by
		; "RadioPinActivated"
		;
		; Input  : "RadioPinActivated"
		; Output :
		;

ClearRadioChannels

		ifdef CONF_RADIO_MODE

			 ; Loop initialization
			 ;
			movlw	TABLE_NOF_CELLS
			movwf	Counter
			clrf	TempIndex

next_radio
			 ; Read table[TempIndex]
			movf	TempIndex, W
			call	SetTablePointer
			call	PeekTableCell	; Sets "Read_xxx"

			 ; Is it valid ?

			call	TestIfCellValid
			skpc
			goto	crc_next	; No, its a null entry

			 ; Is this one a radio mode thing ?
			 ;
			movf	Read_Conf_Mode, W
			call	ModeConf2Int

			sublw	CONF_RADIO_MODE
			skpz
			goto	crc_next	; No, not radio assignment

			 ; Is it another than the newly selected ?
			 ;
			movf	Read_Conf_Pin, W
			subwf	RadioPinActivated, W
			skpnz
			goto	crc_active	; Its the active pin

			 ; This is another radio button
			 ;
			movf	Read_Conf_Pin, W
			call	ClearBitInOutputMask
			goto	crc_next

crc_active
			movf	Read_Conf_Pin, W
			call	SetBitInOutputMask

crc_next
			incf	TempIndex, F
			 ;
			decfsz	Counter, F
			goto	next_radio

	        endif ; CONF_RADIO_MODE

			return



		; -------------------------------------------------------------
		; NextRadioPin
		; -------------------------------------------------------------
		; Starts with "RadioPinActivated" and finds next radio pin
		;
		; Input  : "RadioPinActivated"
		; Output : "RadioPinActivated"
		;
NextRadioPin

		ifdef CONF_RADIO_MODE

			 ; Loop initialization
			 ;
			movlw	TABLE_NOF_CELLS
			movwf	MasterCounter

_next_radio_p
			incf	RadioPinActivated, F
			movlw	0x0F
			andwf	RadioPinActivated, F
			movf	RadioPinActivated, W

			call	CheckForRadioAssignment

			skpnc
			goto	_n_exit

			decfsz	MasterCounter, F
			goto	_next_radio_p

        endif ; CONF_RADIO_MODE

_n_exit
			return


		; -------------------------------------------------------------------
		; PrevRadioPin
		; -------------------------------------------------------------
		; Starts with "RadioPinActivated" and finds prev radio
		;
		; Input  : "RadioPinActivated"
		; Output : "RadioPinActivated"
		;
PrevRadioPin

		ifdef CONF_RADIO_MODE

			 ; Loop initialization
			 ;
			movlw	TABLE_NOF_CELLS
			movwf	MasterCounter

_prev_radio_p
			decf	RadioPinActivated, F
			movlw	0x0F
			andwf	RadioPinActivated, F
			movf	RadioPinActivated, W

			call	CheckForRadioAssignment

			skpnc
			goto	_p_exit

			decfsz	MasterCounter, F
			goto	_prev_radio_p

        endif ; CONF_RADIO_MODE

_p_exit
	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			return




		; -------------------------------------------------------------------
		; GetFirstPinNumberFromIRCode
		; GetNextPinNumberFromIRCode
		; -------------------------------------------------------------
		;
		; Input  : IR code in 'NewRC5***' word (See "PackTableData")
		; 			 Upper nibble (the pin number) is ignored.
		; Output : Succes, C = 1 and W = configuration byte (incl output pin no)
		;		 	 Fail, C = 0
		;
GetFirstPinNumberFromIRCode

			movlw	TABLE_NOF_CELLS
			movwf	Counter
			clrf	TempIndex


CheckNextWord
			movf	TempIndex, W
			call	SetTablePointer
			call	PeekTableCellIRLow

			 ; Check if lower bytes are equal
			 ;
			movf	Read_Low, W
			xorwf	NewIrLowByte, W
			skpz
			goto	GetNextPinNumberFromIRCode

			movf	TempIndex, W
			call	SetTablePointer
			call	PeekTableCell

			; Lower bytes equal, check upper bytes
			;
			movf	Read_High, W
			xorwf	NewIrHighByte, W
			skpz
			goto	GetNextPinNumberFromIRCode

			 ; Code match, check if it is the same IR standard
	         ;
			swapf   Read_Conf_Mode, W
			xorwf   IRPublicFlags, W
			andlw   b'00000011'

			skpz
			goto	GetNextPinNumberFromIRCode     ; no - different standard

			 ; Match ! Get pin number and exit

			movf	Read_Conf_Pin, W
			setc
			goto	outta_here


GetNextPinNumberFromIRCode

			incf	TempIndex, F
			 ;
			decfsz	Counter, F
			goto	CheckNextWord
			 ;
			clrc

outta_here

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			return ; GetFirstPinNumberFromIRCode / GetNextPinNumberFromIRCode





		; -------------------------------------------------------------------
		; ModeConf2Int
		; ReadModeConf		(use this)
		; -------------------------------------------------------------
		; Converts configuration byte to mode in decimal (0,1,..n)
		;
		; Input  : ReadModeConf:'Read_Conf_Mode', ModeConf2Int:W
		; Output : W = Mode (0..n)
		;
ReadModeConf
			movf	Read_Conf_Mode, W
ModeConf2Int

        if NOF_CONF_MODES > 7
			error "RC5UTIL.ASM @1199"
        endif

			andlw	b'00000111'
			return



		; -------------------------------------------------------------------
		;	ModeInt2Conf
		; 	-------------------------------------------------------------
		;	Converts mode in decimal (0,1,..n) to configuration byte
		;
		; 	Input  : W = Mode (0..n)
		;	Output : W = Ready for logical or'ing into Conf byte

ModeInt2Conf
        if NOF_CONF_MODES > 7
			error "RC5UTIL.ASM @1217"
        endif
			andlw	b'00000111'
			return




		; -------------------------------------------------------------------
		; ActivateOutputMask
		; -------------------------------------------------------------
		; Extracts pin number and mode from "CurrentConf" and
		; modifies the output mask accordingly.
		; NOTE : Setter of "RadioPrescaler" in radio mode (timing the break)
		;
		; Input  : "CurrentConf" and "CurrentPin"
		; Output :
		;
			ForcePageAlign	.12

ActivateOutputMask

			movlw	ActivateOutputMask / 0x100
			movwf	PCLATH
			 ;
		  	movf	CurrentConf, W
			call	ModeConf2Int
			addwf	PCL, F
gotya		set		0
		if MODE_TABLE_CONF == 1
gotya		set		gotya+1
			goto	Standard
			goto	Toggle
			goto	Radio
	        goto	BiToggle
			goto	BiToggleDelayed
		endif
		if MODE_TABLE_CONF == .100
gotya		set		gotya+1
			goto	Standard
			goto	Toggle
			goto	Radio
	        goto	BiToggle
			goto	BiToggleDelayed
			goto	RadioNext
			goto	RadioPrev
		endif

		if gotya != 1
			error "***** Missing table in Activate/DeactivateOutputMask *****"
		endif

		if ActivateOutputMask / 0x0100 != ($-1) / 0x0100
			error "Page fault in ActivateOutputMask"
		endif


Standard
			movf	CurrentPin, W
			call	SetBitInOutputMask
			goto	TypeEnds

Toggle
			movf	CurrentPin, W
			call	ReverseBitInOutputMask
			goto	TypeEnds

Radio

			movf	CurrentPin, W
			call	SetBitInOutputMask

			movf	CurrentPin, W
			movwf	RadioPinActivated

			goto	TypeEndsWithPrescaler

BiToggle
			; The bitoggle mode were either triggered by a slave pin stimuli
			; or a IR code on the master pin.

			movf	CurrentPin, W
        	call	ReverseBitInOutputMask
			goto	TypeEnds

BiToggleDelayed

			; First issue : If a change is on a delayed bitoggle pin
			; (queued to be set after the delay time) then delete it
			; from the queue.
			; This means that the user is in charge.
			;
			movf	CurrentPin, W
			call    BitToMask

			comf    ShiftTempQuad+1, W
			andwf   BiToggleRegBootDelay, F
			goto    BiToggle

RadioNext
		ifdef CONF_RADIO_NEXT
			call	NextRadioPin
			;movf	RadioPinActivated, W
			;call	SetBitInOutputMask
	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
	    	endif	; -------------------------------------
			bsf		RadioModeNextPrevFlags, PRESCALER_PRESCALER
			movlw	MAKE_BEFORE_BREAK_RADIO_NEXT
		endif
			goto	TEWPC

RadioPrev
		ifdef CONF_RADIO_PREV
			call	PrevRadioPin
			;movf	RadioPinActivated, W
			;call	SetBitInOutputMask
	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
	    	endif	; -------------------------------------
			bsf		RadioModeNextPrevFlags, PRESCALER_PRESCALER
			movlw	MAKE_BEFORE_BREAK_RADIO_NEXT
		endif
			goto	TEWPC

TypeEndsWithPrescaler
			movlw	MAKE_BEFORE_BREAK_TICKS
TEWPC		movwf	RadioPrescaler
TypeEnds
			return




		; -------------------------------------------------------------------
		; DeactivateOutputMask
		; -------------------------------------------------------------
		; Input  : "CurrentConf" and "CurrentPin"
		; Output :
		;
			ForcePageAlign	.12

DeactivateOutputMask

			movlw	DeactivateOutputMask / 0x100
			movwf	PCLATH
			 ;
		  	movf	CurrentConf, W
			call	ModeConf2Int
			addwf	PCL, F
		if MODE_TABLE_CONF == 1
			goto	DeactStandard
			goto	DeactToggle
			goto	DeactRadio
			goto	DeactBiToggle
			goto	DeactBiToggleDelayed
		endif
		if MODE_TABLE_CONF == .100
			goto	DeactStandard
			goto	DeactToggle
			goto	DeactRadio
			goto	DeactBiToggle
			goto	DeactBiToggleDelayed
			goto	DeactRadioNext
			goto	DeactRadioPrev
		endif

		if DeactivateOutputMask / 0x0100 != $ / 0x0100
			error "Page fault in DeactivateOutputMask"
		endif


DeactStandard

			movf	CurrentPin, W
			call	ClearBitInOutputMask
DeactToggle
DeactRadio
DeactBiToggle
DeactBiToggleDelayed
DeactRadioNext
DeactRadioPrev
			return



		; -------------------------------------------------------------------
		; SaveCurrentOutputLevels
		; -------------------------------------------------------------
		; The state of the output pins are saved in eeprom if changed
		;
		; Input  :
		; Output :
		;
SaveCurrentOutputLevels
			movlw	.4
			movwf	TempIndex
scol_loop
			movlw	EepOutputPinsQuad - 1
			addwf	TempIndex, W
			call	EepromReadW
			movwf	Temporary

			movlw	OutputsMask32 - 1
			addwf	TempIndex, W
			movwf	FSR
			movf	INDF, W

			subwf	Temporary, W

			skpnz
			goto	this_is_equal

			 ; they differed, save new

			movlw	EepOutputPinsQuad - 1
			addwf	TempIndex, W
			call	EepromSetAddress

			movf	INDF, W
			di
			call	EepromWriteW
			ei

this_is_equal
			decfsz	TempIndex, F
			goto	scol_loop

			return





		; ClearTableOutputLevels
		; ---------------------------------------------------
		; Clears output mask registers and writes a (cleared)
		; copy to table.
		; See    : "SetOutputsFromMasks" to set output ports
		; Input  :
		; Output :
		; Stack	 :

ClearTableOutputLevels

			clrf	OutputsMask32+0
			clrf	OutputsMask32+1
			clrf	OutputsMask32+2
			clrf	OutputsMask32+3
			call	SaveCurrentOutputLevels
			return


		; RestoreOutputLevelMask
		; ---------------------------------------------------
		; The 2*16 bit cells "OutputsMask32" are loaded from eeprom
		; Typically called at cold start.
		;
		; Input  : EepOutputPinsQuad
		; Output : OutputsMask32
		; Stack	 : -
		; Intrs  : -


RestoreOutputLevelMask

			movlw	.4
			movwf	TempIndex

			movlw	OutputsMask32 - 1
			addwf	TempIndex, W
			movwf	FSR

rtol_load_next
			movlw	EepOutputPinsQuad - 1
			addwf	TempIndex, W
			call	EepromReadW

			movwf	INDF
			decf	FSR, F

			decfsz	TempIndex, F
			goto	rtol_load_next

			return





		; -------------------------------------------------------------------
		; GetFirstCodeAndConf
		; GetNextCodeAndConf
		; -------------------------------------------------------------
		; Find assignments for the specified i/o pin
		; Input  : Pin number in W
		; Output : Succes, C = 1 and W = configuration byte (incl pin)
		;          Read_Low & Read_High will contain IR code
		;	       Fail, C = 0
		;
GetFirstCodeAndConf

	        movwf   SubCallParameter		; Save pin number

			movlw	TABLE_NOF_CELLS
			movwf	Counter
			clrf	TempIndex

GFCC_NextWord
			movf	TempIndex, W
			call	SetTablePointer
			call	PeekTableCellPin

			 ; Check if pin number matches
			 ;
			movf	Read_Conf_Pin, W
			subwf   SubCallParameter, W
			skpz
			goto	GetNextCodeAndConf     ; no

			 ; Match ! Return configuration if its a valid cell
			 ;
			movf	TempIndex, W
			call	SetTablePointer
			call	PeekTableCell

			call	TestIfCellValid
			skpc
			goto	GetCodeAndConfExitNotFound

			movf	Read_Conf_Mode, W
			setc
			goto    GetCodeAndConfExit


GetNextCodeAndConf

			incf	TempIndex, F
			 ;
			decfsz	Counter, F
			goto	GFCC_NextWord
			 ;
GetCodeAndConfExitNotFound
			clrc

GetCodeAndConfExit

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
	    	endif	; -------------------------------------

			return ; GetNextCodeAndConf




		; -------------------------------------------------------------------
		; CheckForRadioAssignment
		; -------------------------------------------------------------
		; Check if the specified pin has a radio assignment
		; Input  : Pin number in W
		; Output : Succes, C = 1 and W = configuration byte (incl pin)
		;          Read_Low & Read_High will contain IR code
		;	       Fail, C = 0
		;

CheckForRadioAssignment
		ifdef CONF_RADIO_MODE
	        movwf   SubCallParameter

			; Prepare to sweep the entire assignments table
			movlw	TABLE_NOF_CELLS
			movwf	Counter2
			clrf	TempIndex2

CFR_NextWord
			movf	TempIndex2, W
			call	SetTablePointer
			call	PeekTableCellPin

			 ; Check if pin number matches
			 ;
			movf	Read_Conf_Pin, W
			subwf   SubCallParameter, W
			skpz
			goto	CFR_BadPin

			 ; Match. Check if radio
			 ;
			movf	TempIndex2, W
			call	SetTablePointer
			call	PeekTableCell

			movf	Read_Conf_Mode, W
			call	ModeConf2Int
			sublw	CONF_RADIO_MODE
			skpz
			goto	CFR_BadPin

			 ; Yes, its a radio
			setc
			goto    CFR_Exit

CFR_BadPin
			incf	TempIndex2, F
			 ;
			decfsz	Counter2, F
			goto	CFR_NextWord
			 ;
		endif
			clrc
CFR_Exit
			return



		; SetOutputsFromMasks
		; ---------------------------------------------------
		; Sets the 16 output pin levels as given by "OutputsMaskXxx"
		;
SetOutputsFromMasks
	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			SetPgeF	FAR_CODE_PAGE
			goto	Far_SetOutputsFromMasks & __PAGE__




; -----------------------------------------------------------------------------
;    U T I L I T Y    C O D E    P A G E       ("Far_")
; -----------------------------------------------------------------------------


			org		FAR_CODE_PAGE


CheckIfProhibited	macro pin
		ifdef CONF_RADIO_NEXT
			btfss	RadioModeNextPin, BIT7
			goto	$+.6	; check_prev
			movf	RadioModeNextPin, W
			andlw	0x7F
			sublw	pin
			skpnz
			goto	$+.9	; prohibited
;check_prev
			btfss	RadioModePrevPin, BIT7
			goto	$+.9	; not_prohibited
			movf	RadioModePrevPin, W
			andlw	0x7F
			sublw	pin
			skpnz
			goto	$+.2	; prohibited
			goto	$+.3	; not_prohibited
;prohibited
			setc
			goto	$+.3	; check_exit
;not_prohibited
			nop				; FIXIT
			clrc
;check_exit
		endif
			endm




SetOutputPin macro	port, bit, pin
			btfss	mask, bit
			goto	$ + 3
			bsf		port, pin
			goto	$ + 2
			bcf		port, pin
			endm

		; Far_SetOutputsFromMask
		; ---------------------------------------------------
		; See SetOutputsFromMasks
		;

Far_SetOutputsFromMasks

mask 		set  OutputsMask32+0

		; For pin0 and pin1 check if they are running as RadioNext and
		; RadioPrev. If they are, then skip them, they are configured as
		; inputs with external pullups.
		;
		ifdef CONF_RADIO_NEXT
			CheckIfProhibited PIN0
			skpnc
			goto	SkipBit1
		endif
			SetOutputPin	PIN0_PORT, BIT0, PIN0
SkipBit1

		ifdef PIN1_PORT
		ifdef CONF_RADIO_PREV
		jkjl
			CheckIfProhibited PIN1
			skpnc
			goto	SkipBit2
		endif
			SetOutputPin	PIN1_PORT, BIT1, PIN1
		endif
SkipBit2

		ifdef PIN2_PORT
			SetOutputPin	PIN2_PORT, BIT2, PIN2
			SetOutputPin	PIN3_PORT, BIT3, PIN3
		endif
		ifdef PIN4_PORT
			SetOutputPin	PIN4_PORT, BIT4, PIN4
			SetOutputPin	PIN5_PORT, BIT5, PIN5
			SetOutputPin	PIN6_PORT, BIT6, PIN6
			SetOutputPin	PIN7_PORT, BIT7, PIN7

mask 		set  OutputsMask32+1

			SetOutputPin	PIN8_PORT, BIT0, PIN8
			SetOutputPin	PIN9_PORT, BIT1, PIN9
			SetOutputPin	PIN10_PORT, BIT2, PIN10
			SetOutputPin	PIN11_PORT, BIT3, PIN11

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			btfss   Flags, PROGRAM_MODE_ON
            goto    SetOutputsNormalMode

     		; Programming mode, all are outputs
      		;
			SetOutputPin	PIN12_PORT, BIT4, PIN12
		if ICE == false
			SetOutputPin	PIN13_PORT, BIT5, PIN13
			SetOutputPin	PIN14_PORT, BIT6, PIN14
			SetOutputPin	PIN15_PORT, BIT7, PIN15
		endif
			goto    SetBiToggleEnds

SetOutputsNormalMode
			; If any of 8-11 are running in bitoggle mode, i.e. having 12-15 as
       		; slave inputs, then dont write to slave inputs (which are inputs
       		; with pullups to vcc)
			;
            btfsc   BiToggleRegister, BIT0
			goto	no_12_set
			SetOutputPin	PIN12_PORT, BIT4, PIN12
no_12_set

		if ICE == false
			btfsc   BiToggleRegister, BIT1
			goto	no_13_set
			SetOutputPin	PIN13_PORT, BIT5, PIN13
no_13_set

            btfsc   BiToggleRegister, BIT2
			goto	no_14_set
			SetOutputPin	PIN14_PORT, BIT6, PIN14
no_14_set

            btfsc   BiToggleRegister, BIT3
			goto	no_15_set
			SetOutputPin	PIN15_PORT, BIT7, PIN15
no_15_set
		endif


SetBiToggleEnds

		endif

	       	if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
	    	endif	; -------------------------------------

			SetPage0
			return

			; Far_SetOutputsFromMask
			
			
