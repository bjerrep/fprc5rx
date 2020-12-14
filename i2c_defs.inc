;;;-----------------------------------------------------------------------
;;; i2c_defs.inc
;;; Global defines for i2c library file "i2c.inc" and a common exchange
;;; file for device adresses.
;;;-----------------------------------------------------------------------
;;; Author  : Claus Bjerre
;;; Target  : PIC16F87x series.
;;; Licence : Free for noncommercial use. Not free for commercial use,
;;; Version : 1
;;; Contact : -
;;;-----------------------------------------------------------------------
;;; See i2c.inc
;;;
;;; I2C_Status are I2C_Public are flag registers that might share same
;;  physical adress.
;;;
;;; Remember to have SDA and SCL tris'd to inputs.
;;;-----------------------------------------------------------------------
;;;
;;; History
;;; 001 27-09-02        Initial version
;;;
;;;-----------------------------------------------------------------------

; I2C_Status
I2C_ERROR_NACK          	equ	BIT0

; I2C_Public
I2C_RX_COMPLETE         	equ	BIT1
I2C_TX_START            	equ	BIT2

; Addresses
I2C_725_BROADCAST_ADDRESS	equ	0x34    ;12

; Commands (remember to update I2C_GetNofBytesFromCommand() in i2c.inc)

I2C_725_CMD_LEVEL_LOW		equ	.0
I2C_725_CMD_LEVEL_HIGH		equ	.1
I2C_725_LEVEL_CHANGED_CMD_LEN   equ	.3


