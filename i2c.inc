;
; ========================================================================
; i2c.inc
; i2c library file
; 001 27-09-02 Source from http://www.piclist.com/techref/microchip/i2c87xjs.htm (John Sand)
; 002
; ========================================================================
;

;
; See i2c_def.inc
;
; External provides :
; I2C_Start
; I2C_Address
; I2C_Status (flags)
;
; The terms sync and async are used here to distinguish between a busy
; waiting implementation (exits when all data ready) and a state machine
; implementation to be inserted in-the-loop (i.e. typically as would be
; nescessary in a ISR).
;
;
; Remember to have SDA and SCL tris'd to inputs.
;-----------------------------------------------------------------------


CODE_SIZE_I2C_START



			; ---------------------------------------------------
			; M A S T E R    S Y N C
			; ---------------------------------------------------


        ifdef Iwer2C_MASTER_SYNC_2

I2C_Start
			SetBank0
			bcf     PIR1,SSPIF
			SetBank1
			bsf     SSPCON2 & FORCE_7_BIT_ADR, SEN     ; send i2c START condition [S] bit
			SetBank0

			if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
			endif	; -------------------------------------

            btfss   PIR1,SSPIF         ; start bit cycle complete?
            goto    $-1

            bcf     PIR1,SSPIF

            return




I2C_Stop
            SetBank0
            bcf     PIR1,SSPIF
            SetBank1
            bsf     SSPCON2 & FORCE_7_BIT_ADR, PEN         ;send i2c STOP [P] bit
            SetBank0

			if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
			endif	; -------------------------------------

            btfss   PIR1,SSPIF      ; Stop clock complete ?
            goto    $-1

			if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
			endif	; -------------------------------------

            bcf     PIR1,SSPIF
            return



I2C_AckWait
            SetBank0

			if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
			endif	; -------------------------------------

            btfss   PIR1, SSPIF     ; 9th clock complete ?
            goto    $-1

			if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
			endif	; -------------------------------------

            bcf     PIR1, SSPIF

            return



I2C_MasterInit

			; SSPCON<5>   SSPEN
			; SSPCON<3:0> '0110' = I2C Slave mode, 7-bit address
			; SSPCON<3:0> '1000' = I2C Master mode, clock = FOSC / (4 * (SSPADD+1))
			;
			movlw   b'00101000'     ; Set SSPEN
			movwf   SSPCON     ;=0x028

			bcf     PIR1, SSPIF

			SetBank1
			movlw   b'10000000'
			movwf   SSPSTAT & FORCE_7_BIT_ADR

			movlw   b'01100000'
			movwf   SSPCON2 & FORCE_7_BIT_ADR

			movlw   b'00001010'
			movwf   SSPADD & FORCE_7_BIT_ADR

			SetBank0
			return



I2C_Test

			if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
			endif	; -------------------------------------

            SetBnkF SSPCON2
            setc
            btfss   SSPCON2 & FORCE_7_BIT_ADR, ACKSTAT
            clrc
            SetBank0
             ;
            skpnc
            bsf     I2C_TxStatus, I2C_ERROR_NACK
            return





		; I2C_MasterSyncWrite
		; ---------------------------------------------------
		; Transmits databyte to specified address.

I2C_MasterSyncWrite
            call    I2C_Start
            movf    I2C_TxAddress, W
            movwf   SSPBUF

            call    I2C_AckWait
            call    I2C_Test

			if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
			endif	; -------------------------------------

            movf    I2C_TxCommand, W
            movwf   SSPBUF
            call    I2C_AckWait
            call    I2C_Test

			if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
			endif	; -------------------------------------

            decfsz  I2C_TxNofBytes, F
            goto    tx_data

MSW_Ends
			if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
			endif	; -------------------------------------

            call    I2C_Stop
            return

tx_data
            movf    I2C_TxData, W
            movwf   SSPBUF
            call    I2C_AckWait
            call    I2C_Test
            goto    MSW_Ends

	        endif




        ifdef I2C_MASTER_SYNC

I2C_MasterSyncInit

             ; SSPCON<5>   SSPEN
             ; SSPCON<3:0> '0110' = I2C Slave mode, 7-bit address
             ; SSPCON<3:0> '1000' = I2C Master mode, clock = FOSC / (4 * (SSPADD+1))
             ;
            movlw   b'00101000'     ; Set SSPEN
            movwf   SSPCON     ;=0x028

            bcf     PIR1, SSPIF

            SetBank1
            movlw   b'10000000'
            movwf   SSPSTAT & FORCE_7_BIT_ADR

            movlw   b'01100000'
            movwf   SSPCON2 & FORCE_7_BIT_ADR

            movlw   b'00001010'
            movwf   SSPADD & FORCE_7_BIT_ADR

            SetBank0
            return


#define I2C_MASTER_ASYNC


I2C_MasterSyncWrite

			bsf     I2C_Public, I2C_TX_START

MSW_loop
            clrwdt
            call    I2C_MasterAsyncEngine

            btfsc   I2C_Public, I2C_TX_START
            goto    MSW_loop

            return

	        endif


		; ---------------------------------------------------
                ; M A S T E R    A S Y N C
		; ---------------------------------------------------


        ifdef I2C_MASTER_ASYNC

; Storage used :
;I2C_Public
;I2C_TxJumpVector
;I2C_TxIndex
;I2C_TxNofBytes         ; Nof bytes out including the address, I2C_TxNofBytes < 128d
;I2C_TxAddress
;I2C_TxCommand          ; Indirectly for I2C_TxNofBytes > 1
;I2C_TxData             ; Indirectly for I2C_TxNofBytes > 2


I2C_MasterAsyncInit

             ; SSPCON<5>   SSPEN
             ; SSPCON<3:0> '0110' = I2C Slave mode, 7-bit address
             ; SSPCON<3:0> '1000' = I2C Master mode, clock = FOSC / (4 * (SSPADD+1))
             ;
            movlw   b'00101000'     ; Set SSPEN
            movwf   SSPCON     ;=0x028

            bcf     PIR1, SSPIF

            SetBank1
            movlw   b'10000000'
            movwf   SSPSTAT & FORCE_7_BIT_ADR

            movlw   b'01100000'
            movwf   SSPCON2 & FORCE_7_BIT_ADR

            movlw   b'00001010'
            movwf   SSPADD & FORCE_7_BIT_ADR

            SetBank0
            return



                ; I2C_MasterAsyncEngine
		; ---------------------------------------------------
		;
			ForcePageAlign	.10

I2C_MasterAsyncEngine

I2C_MAW		movlw	I2C_MAW / 0x100
			movwf	PCLATH
			 ;
			movf	I2C_TxJumpVector, W
			addwf	PCL, F
			 ;


I2C_MAW_IDLE	        equ	.0
		        		goto	I2C_MawIdle
I2C_MAW_BUSY	        equ	.1
		        		goto	I2C_MawBusy
I2C_MAW_FINISHING       equ     .2
                        goto    I2C_MawFinishing

		if I2C_MAW / 0x0100 != $ / 0x0100
		  	error "***** Page fault #0276 *****"
		endif

                 ; I2C_MawIdle
                 ; ----------------------------------------
                 ; Wait for client flags to start
I2C_MawIdle

            btfss   I2C_Public, I2C_TX_START
            goto    I2C_MawEnds

             ; User go. Initiate i2c start

            bcf     PIR1,SSPIF
            SetBank1
            bsf     SSPCON2 & FORCE_7_BIT_ADR, SEN     ; i2c START condition
            SetBank0

			if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
			endif	; -------------------------------------

            clrf    I2C_TxIndex

            movlw   I2C_MAW_BUSY
            movwf   I2C_TxJumpVector

            goto    I2C_MawEnds



             ; I2C_MawBusy
             ; ----------------------------------------
             ; Transmitting and waiting....

I2C_MawBusy
            btfss   PIR1,SSPIF         ;cycle complete?
            goto    I2C_MawEnds

            bcf     PIR1,SSPIF

             ; Figure out if there are more data bytes
             ;
            decf    I2C_TxNofBytes, F
            btfsc   I2C_TxNofBytes, BIT7    ; Stop when reaching -1 (this is the 128 limit)
            goto    setup_stop


             ; Load databyte and write to transmitter register "SSPBUF"
             ;
            movf    I2C_TxIndex, W
            addlw   I2C_TxAddress
            movwf   FSR
            movf    INDF, W
            movwf   SSPBUF                  ; Data transmission starts


setup_next
            incf    I2C_TxIndex, F
            goto    I2C_MawEnds



setup_stop
             ; No more data, start i2c STOP condition

            SetBank1
            bsf     SSPCON2 & FORCE_7_BIT_ADR, PEN         ;send i2c STOP [P] bit
            SetBank0

            movlw   I2C_MAW_FINISHING
            movwf   I2C_TxJumpVector
            goto    I2C_MawEnds




             ; I2C_MawFinishing
             ; ----------------------------------------
             ; Waiting for end of STOP, and then cleaning up.
             ; Clears "I2C_TX_START" to signal tx complete to host.
             ;
I2C_MawFinishing

            btfss   PIR1,SSPIF      ; STOP complete ?
            goto    I2C_MawEnds

             ; STOP is complete
            bcf     PIR1,SSPIF
            bcf     I2C_Public, I2C_TX_START
             ;
            movlw   I2C_MAW_IDLE
            movwf   I2C_TxJumpVector

I2C_MawEnds
			return


        endif ; ifdef I2C_MASTER_ASYNC









		; ---------------------------------------------------
		; S L A V E    A S Y N C
		; ---------------------------------------------------

        ifdef   I2C_SLAVE_ASYNC

			; I2C_SlaveInit
			; ---------------------------------------------------

I2C_SlaveInit

			movlw   b'00110110'                     ; 7bit address slave
			movwf   SSPCON     ;=0x028

			bcf     PIR1, SSPIF

			SetBank1
			movlw   b'10000000'
			movwf   SSPSTAT & FORCE_7_BIT_ADR

			movlw   b'01100000'
			movwf   SSPCON2 & FORCE_7_BIT_ADR

			movlw   I2C_SLAVE_ADRESS
			movwf   SSPADD & FORCE_7_BIT_ADR        ; 7 bit address

			SetBank0Auto

			movf    SSPBUF, W                       ; Clear any buffer full (BF) flag.
			bcf     SSPCON, SSPOV                   ; Clear any overflow flag.

			 ; State machine
			 ;
			clrf    I2C_SarJumpvector
			clrf    I2C_Public

			return




        ; I2C_SlaveAsyncRead
		; ---------------------------------------------------
		; Supports 1 or 2 data bytes after address.
		; ToDo : Slave write
		; ToDo : Support for length field after command (Command with bit 7 set)
		;

I2C_SlaveAsyncRead



I2C_SAR		movlw	I2C_SAR / 0x100
			movwf	PCLATH
			 ;
			movf	I2C_SarJumpvector, W
			addwf	PCL, F
			 ;


I2C_SAR_STOPPED	        equ		.0
				        goto	I2C_SarStopped
I2C_SAR_IDLE	        equ		.1
				        goto	I2C_SarIdle
I2C_SAR_CMD             equ     .2
                        goto    I2C_SarCmd
I2C_SAR_DATA            equ     .3
                        goto    I2C_SarData
I2C_SAR_WAIT_STOP       equ     .4
                        goto    I2C_SarWaitStop

		if I2C_SAR / 0x0100 != $ / 0x0100
		  messg "Page fault #0225"
		endif


			; I2C_SarStopped
			; ----------------------------------------
			; Wait for client ready
I2C_SarStopped

			 ; Stop receiving until previous data are processed by subscriber
			 ;
			btfsc   I2C_Public, I2C_RX_COMPLETE
			goto    I2C_SarEnds

			 ; Cleanup any junk flags (FIXIT)
			 ;
			movf    SSPBUF, W                       ; Clear any buffer full (BF) flag.
			bcf     SSPCON, SSPOV                   ; Clear any overflow flag.
			bcf     PIR1, SSPIF

			movlw   I2C_SAR_IDLE
			movwf   I2C_SarJumpvector

			goto    I2C_SarEnds


			 ; I2C_SarIdle
			 ; ----------------------------------------
			 ; Wait for address match
I2C_SarIdle

			 ; Stop receiving until previous data are released
			 ;
			btfss   PIR1, SSPIF
			goto    I2C_SarEnds

			 ; Address match
			 ;
			bcf     PIR1, SSPIF
			movf    SSPBUF, W                       ; Clear any buffer full (BF) flag.
			 ;
			movlw   I2C_SAR_CMD
			movwf   I2C_SarJumpvector




			 ; I2C_SarCmd
			 ; ----------------------------------------
			 ; Load first data byte
I2C_SarCmd

			btfss   PIR1, SSPIF
			goto    I2C_SarEnds

			bcf     PIR1, SSPIF

			 ; Got command

			movf    SSPBUF, W                       ; Clears BF
			movwf   I2C_RxCommand

			 ; More bytes ??
			 ;
			call    I2C_GetNofBytesFromCommand
			movwf   I2C_RxNofBytes

			decfsz  I2C_RxNofBytes, F
			goto    more_bytes

			 ; No, we are done.
			 ;
			movlw   I2C_SAR_WAIT_STOP
			movwf   I2C_SarJumpvector
			goto    I2C_SarEnds


more_bytes
			movlw   I2C_SAR_DATA
			movwf   I2C_SarJumpvector
			goto    I2C_SarEnds



			; I2C_SarData
			; ----------------------------------------
			; Load first data byte
I2C_SarData

			btfss   PIR1, SSPIF
			goto    I2C_SarEnds

			 ; Got data

			movf    SSPBUF, W                       ; Clears BF
			movwf   I2C_RxData

			bcf     PIR1, SSPIF
			movlw   I2C_SAR_WAIT_STOP
			movwf   I2C_SarJumpvector
			goto    I2C_SarEnds



			 ; I2C_SarWaitStop
			 ; ----------------------------------------
I2C_SarWaitStop

			SetBnkF SSPSTAT
			btfss   SSPSTAT & FORCE_7_BIT_ADR, P
			goto    I2C_SarEndsBank0
			SetBank0Auto

			 ; There were a stop condition, publish to user
			 ;
			bsf     I2C_Public, I2C_RX_COMPLETE

			MDisableI2CIntr

			movlw   I2C_SAR_STOPPED
			movwf   I2C_SarJumpvector

			goto    I2C_SarEnds


I2C_SarEndsBank0
			SetBank0Auto
I2C_SarEnds

			return

        endif



		; ---------------------------------------------------
                ; S L A V E    A S Y N C
		; ---------------------------------------------------

        ifdef   I2C_SLAVE_ASYNC_ISR

			; I2C_SlaveInit
			; ---------------------------------------------------

I2C_SlaveInit

            movlw   b'00110110'                     ; 7bit address slave
            movwf   SSPCON     ;=0x028

            bcf     PIR1, SSPIF

            SetBank1
            movlw   b'10000000'
            movwf   SSPSTAT & FORCE_7_BIT_ADR

            movlw   b'01100000'
            movwf   SSPCON2 & FORCE_7_BIT_ADR

            movlw   I2C_SLAVE_ADRESS * 2
            movwf   SSPADD & FORCE_7_BIT_ADR        ; 7 bit address

            SetBank0

             ; State machine
             ;
            clrf    I2C_SarJumpvector
            clrf    I2C_Public

I2C_Restart

            movf    SSPBUF, W                       ; Clear any buffer full (BF) flag.
            bcf     SSPCON, SSPOV                   ; Clear any overflow flag.

            return






		; I2C_SlaveAsyncRead
		; ---------------------------------------------------
		; Supports 1 or 2 data bytes after address.
		; ToDo : Slave write
		; ToDo : Support for length field after command (Command with bit 7 set)
		;

I2C_SlaveAsyncRead


I2C_SAR		movlw	I2C_SAR / 0x100
			movwf	PCLATH
			 ;
			movf	I2C_SarJumpvector, W
			addwf	PCL, F
			 ;


I2C_SAR_IDLE	        equ		.0
				        goto	I2C_SarIdle
I2C_SAR_CMD             equ     .1
                        goto    I2C_SarCmd
I2C_SAR_DATA            equ     .2
                        goto    I2C_SarData
I2C_SAR_WAIT_STOP       equ     .3
                        goto    I2C_SarWaitStop

		if I2C_SAR / 0x0100 != $ / 0x0100
		  messg "Page fault #0225"
		endif



			; I2C_SarIdle
			; ----------------------------------------
			; Wait for address match
I2C_SarIdle

             ; Stop receiving until previous data are released
             ;
            btfss   PIR1, SSPIF
            goto    I2C_SarEnds

             ; Address match
             ;
            bcf     PIR1, SSPIF
            movf    SSPBUF, W                       ; Clear any buffer full (BF) flag.
             ;

            SetBnkF SSPSTAT
            bcf     SSPSTAT & FORCE_7_BIT_ADR, P
            SetBank0

            movlw   I2C_SAR_CMD
            movwf   I2C_SarJumpvector




			; I2C_SarCmd
			; ----------------------------------------
			; Load first data byte
I2C_SarCmd

            btfss   PIR1, SSPIF
            goto    I2C_SarEnds

             ; Got command
            bcf     PIR1, SSPIF
            movf    SSPBUF, W                       ; Clears BF
            movwf   I2C_RxCommand

             ; More bytes ??
             ;
            call    I2C_GetNofBytesFromCommand
            movwf   I2C_RxNofBytes

            decfsz  I2C_RxNofBytes, F
            goto    more_bytes

             ; No, we done.
             ;
            movlw   I2C_SAR_WAIT_STOP
            movwf   I2C_SarJumpvector
            goto    I2C_SarEnds


more_bytes
            movlw   I2C_SAR_DATA
            movwf   I2C_SarJumpvector
            goto    I2C_SarEnds



			; I2C_SarData
			; ----------------------------------------
			; Load first data byte
I2C_SarData

            btfss   PIR1, SSPIF
            goto    I2C_SarEnds


            bcf     PIR1, SSPIF
             ; Got data

            movf    SSPBUF, W                       ; Clears BF
            movwf   I2C_RxData


            bsf     I2C_Public, I2C_RX_COMPLETE
            MDisableI2CIntr


            movlw   I2C_SAR_IDLE            ; I2C_SAR_WAIT_STOP
            movwf   I2C_SarJumpvector
            goto    I2C_SarEnds



             ; I2C_SarWaitStop
             ; ----------------------------------------
I2C_SarWaitStop

            SetBnkF SSPSTAT
            btfss   SSPSTAT & FORCE_7_BIT_ADR, P
            goto    I2C_SarEndsBank0
            SetBank0

             ; There were a stop condition, publish to user
             ;



             ; delete
            btfss   PIR1, SSPIF
            goto    NoFuzz2

            bcf     PIR1, SSPIF

			if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
			endif	; -------------------------------------

NoFuzz2
			movlw   I2C_SAR_IDLE
			movwf   I2C_SarJumpvector

            goto    I2C_SarEnds


I2C_SarEndsBank0
            SetBank0
I2C_SarEnds

            return





        endif






        ifdef   I2C_RxCommand

      	; I2C_GetNofBytesFromCommand
		; ---------------------------------------------------
		; Returns nof bytes matching value in "I2C_RxCommand"
		; ToDo : Switch table as function of adress.

I2C_GetNofBytesFromCommand

			if ICE	; -------------------------------------
			nop		; Breakpoint :
			nop		;
			endif	; -------------------------------------

		if I2C_GetNofBytesFromCommand / 0x100 != I2C_SAR / 0x100
        	movlw	I2C_GetNofBytesFromCommand / 0x100
			movwf	PCLATH
		endif

            movf    I2C_RxCommand, W
            andlw   b'00000111'

			addwf	PCL, F

            retlw I2C_725_LEVEL_CHANGED_CMD_LEN - 1		;I2C_725_CMD_LEVEL_LOW
            retlw I2C_725_LEVEL_CHANGED_CMD_LEN - 1		;I2C_725_CMD_LEVEL_HIGH
            retlw 0x01
            retlw 0x01
            retlw 0x01
            retlw 0x01
            retlw 0x01
            retlw 0x01

            if I2C_GetNofBytesFromCommand / 0x0100 != $ / 0x0100
              error "Page fault"
            endif

	endif




CODE_SIZE_I2C	equ	$ - CODE_SIZE_I2C_START


