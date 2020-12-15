;
; ========================================================================
; 1 6 F 8 7 3 . I N C	                 	   Claus Bjerre 2002-04-22
;
; Version history
; 001	Initial merge of old include files.
; ========================================================================
;
	NOLIST

RESET				equ	0
INTERUPT			equ	4
ROMSTART			equ	8
RAM_BANK0_START		equ	20h
RAM_BANK0_LAST		equ	7Fh
RAM_BANK0_SIZE		equ	RAM_BANK0_LAST - RAM_BANK0_START + 1


RAM_BANK1_START		equ	0xA0
RAM_BANK1_LAST		equ	0xFF
RAM_BANK1_SIZE		equ	RAM_BANK1_LAST - RAM_BANK1_START + 1

FILE_BANKS			equ .4


EEP_START			equ	.0
EEP_SIZE			equ	.128
EEP_MEMORY_ADR		equ	0x2100

RAM_BANKS			equ	.2


	LIST
