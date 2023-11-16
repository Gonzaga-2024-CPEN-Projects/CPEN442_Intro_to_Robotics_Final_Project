SYSCTL_RCGCGPIO_R       EQU 	0x400FE608
SYSCTL_RCC_R            EQU 	0x400FE060
GPIO_PORTA_DIR_R        EQU  	0x40004400
GPIO_PORTA_DATA_R       EQU  	0x400043FC
GPIO_PORTA_DEN_R        EQU  	0x4000451C
GPIO_PORTC_DIR_R        EQU  	0x40006400
GPIO_PORTC_DATA_R       EQU  	0x400063FC
GPIO_PORTC_DEN_R        EQU  	0x4000651C

GPIO_PORTE_DIR_R        EQU 	0x40024400
GPIO_PORTE_DATA_R       EQU 	0x400243FC
GPIO_PORTE_DEN_R        EQU		0x4002451C




	THUMB
	AREA	|.text|, CODE, READONLY, ALIGN=2
		
	EXPORT SYSCTL_RCGCGPIO_R
	EXPORT SYSCTL_RCC_R	
	EXPORT GPIO_PORTA_DATA_R
	
	
	EXPORT  Delay1ms	
	EXPORT	Set_Position
	EXPORT	Display_Msg
	EXPORT	Display_Char 
    EXPORT  Set_Blink_ON
    EXPORT  Set_Blink_OFF
;	EXPORT	Init_Clock
	EXPORT	Init_LCD_Ports
	EXPORT	Init_LCD

		
Init_LCD_Ports
	; Initialize PORTs A, C and E. Note: this initialization uses an UNFRIENDLY code
	PUSH	{LR, R2, R3}
	MOV		R3, #0x15; Activating the clocks for the three ports.
	LDR		R2,=SYSCTL_RCGCGPIO_R
	STR		R3, [R2]
	NOP
	NOP
	MOV		R3, #0x3C; Pins PTA2-PTA5 are outputs
	LDR		R2, =GPIO_PORTA_DIR_R
	STR		R3, [R2]
	MOV		R3, #0x40; Pin PC6 is output
	LDR		R2, =GPIO_PORTC_DIR_R
	STR		R3, [R2]
	MOV		R3, #0x01; Pin PE0 is output
	LDR		R2,	=GPIO_PORTE_DIR_R
	STR		R3, [R2]
	
	MOV		R3, #0xFF; PORTA's signals are digital
	LDR		R2, =GPIO_PORTA_DEN_R
	STR		R3, [R2]
	LDR		R2, =GPIO_PORTC_DEN_R; PORTC's signals are digital
	STR		R3, [R2]
	LDR		R2, =GPIO_PORTE_DEN_R; PORTC's signals are digital
	STR		R3, [R2]
	POP		{LR, R2, R3}
 	BX		LR	

; Data is pointed to by R0
Display_Msg
	PUSH	{LR, R0, R1}
	MOV		R1, R0
disp_again
	LDRB	R0, [R1] 		; R1 <- ASCII data
	CMP		R0,	#0x00		; check for the end of the string
	BEQ		disp_end			  
	BL		Display_Char
	ADD		R1, R1, #1		; increment R0
	B		disp_again	
disp_end
	POP		{LR, R0, R1}
	BX		LR
		
; Display_Char - writes an ASCII value in R0 to LCD

Display_Char
	PUSH	{LR, R1, R0}
	BL		SplitNum	;
	BL		WriteData	; write upper 4 bits of ASCII byte
	MOV		R0, R1
	BL		WriteData	; write lower 4 bits of ASCII byte
	MOV		R0, #0x01	
	BL		Delay1ms	; wait for 1ms
	POP		{LR, R1, R0}
	BX		LR

; Set_Position - sets the position in R0 for displaying data in LCD

Set_Position
	PUSH	{LR, R1, R0}
	ORR		R0,	#0x80	; set b7 of R1
	BL		SplitNum	
	BL		WriteCMD	; write upper 4 bits of the command
	MOV		R0,	R1
	BL		WriteCMD	; write lower 4 bits of the command
	MOV		R0, #0x01		
	BL		Delay1ms	; wait for 1ms
	POP		{LR, R1, R0}
	BX		LR

; WriteData - sends a data (lower 4 bits) in R0 to LCD

WriteData
	PUSH	{LR, R1, R0}
	LSL		R0, R0, #2		; data from bits 2 - 5
	LDR		R1, =GPIO_PORTA_DATA_R
	STRB	R0, [R1]
	LDR		R1, =GPIO_PORTE_DATA_R
	MOV		R0, #0x01	; Sending data
	STRB	R0, [R1]
	MOV		R0, #0x00	; Enabling the LCD (falling edge)
	LDR		R1,	=GPIO_PORTC_DATA_R
	STRB	R0, [R1]
	NOP
	NOP
	MOV		R0, #0x40	; Raising the edge in preparation for the next write 
	STRB	R0, [R1]
	POP		{LR, R1,R0}
	BX		LR			

; WriteCMD - sends a command (lower 4 bits) in R0 to LCD

WriteCMD 
	PUSH	{LR, R1, R0}
	LSL		R0, R0, #2		; data from bits 2 - 5
	LDR		R1, =GPIO_PORTA_DATA_R
	STRB	R0, [R1]
	MOV		R0, #0x00;		; RS=0 for sending a command
	LDR		R1, =GPIO_PORTE_DATA_R
	STRB	R0, [R1]
	MOV		R0, #0x00	; Enabling the LCD
	LDR		R1, =GPIO_PORTC_DATA_R
	STRB	R0, [R1]
	NOP
	NOP
	MOV		R0, #0x40	; Raising PC6
	STRB	R0, [R1]
	POP		{LR, R1, R0}
	BX		LR

; SlipNum - separates hex numbers in R0
;	  R1 <- LS digit
;	  R0 <- MS digit

SplitNum
	PUSH	{LR}
	MOV		R1, R0
	AND		R1, #0x0F		; mask the upper 4 bits
	LSR		R0,	R0, #4 	
	POP		{LR}
	BX		LR

; Init_LCD - initializes LCD according to the initializing sequence indicated
;	  by the manufacturer

Init_LCD
	PUSH	{LR, R1, R0}
	MOV		R0, #30		;
	BL		Delay1ms		; wait 30ms for LCD to power up
	
	; send byte 1 of code to LCD
	MOV		R0,	#0x30		; R1 <- byte #1 of code: $30
	BL		SplitNum	;
	BL		WriteCMD	; write byte #1
	MOV		R0,	#5		;
	BL		Delay1ms	; wait for 5 ms
	
	; send byte 2 of code to LCD
	MOV		R0,	#0x30		; R1 <- byte #2 of code: $30
	BL		SplitNum	;
	BL		WriteCMD	; write byte #2
	MOV		R0, #1		;
	BL		Delay1ms	; wait for 1ms
	
	; send byte 3 of code to LCD
	MOV		R0,	#0x30		; R1 <- byte #3 of code: $30
	BL		SplitNum	;
	BL		WriteCMD	; write byte #3
	MOV		R0,	#1		;
	BL		Delay1ms	; wait for 1ms
	
	; send byte 4 of code to LCD
	MOV		R0,	#0x20		; R1 <- byte #4 of code: $20
	BL		SplitNum	;
	BL		WriteCMD	; write byte #4
	MOV		R0,	#1		;
	BL		Delay1ms	; wait for 1ms
	
	; send byte 5 of code to LCD
	MOV		R0,	#0x28		; R1 <- byte #5 of code: $28
				;  db5 = 1, db4 = 0 (DL = 0 - 4 bits), 
				;  db3 = 1 (N = 1 - 2 lines),
				;  db2 = 0 (F = 0 - 5x7 dots).
	BL		SplitNum	;
	BL		WriteCMD	; write upper 4 bits of byte #5
	MOV		R10,R1
	BL		WriteCMD	; write lower 4 bits of byte #5
	MOV		R0,	#1		;
	BL		Delay1ms	; wait for 1ms
	
	; send byte 6 of code to LCD
	MOV		R0,	#0x0C		; R1 <- byte #6 of code: $0C
				;  db3 = 1, db2 = 1 (D = 1 - display ON)
				;  db1 = 0 (C = 0 - cursor OFF)
				;  db0 = 0 (B = 0 - blink OFF)
	BL		SplitNum	;
	BL		WriteCMD	; write upper 4 bits of byte #6
	MOV		R0,R1
	BL		WriteCMD	; write lower 4 bits of byte #6
	MOV		R0,	#1		;
	BL		Delay1ms	; wait for 1ms
	
	; send byte 7 of code to LCD
	MOV		R0,	#0x01		; R1 <- byte #7 of code: $01
				;  db0 = 1 (clears display and returns
				;	the cursor home).		 
	BL		SplitNum	;
	BL		WriteCMD	; write upper 4 bits of byte #8
	MOV		R0,R1
	BL		WriteCMD	; write lower 4 bits of byte #8
	MOV		R0,	#3		;
	BL		Delay1ms	; wait for 3ms
	
	; send byte 8 of code to LCD
	MOV		R0,	#0x06		; R1 <- byte #8 of code: $06
				;  db2 = 1,
				;  db1 = 1 (I/D = 1 - increment cursor)
				;  db0 = 0 (S = 0 - no display shift)
	BL		SplitNum	;
	BL		WriteCMD	; write upper 4 bits of byte #7
	MOV		R0,R1
	BL		WriteCMD	; write lower 4 bits of byte #7
	MOV		R0,	#1		;
	BL		Delay1ms	; wait for 1ms
	POP		{LR, R1, R0}
	BX		LR
	
; Subroutine Set_Blink_ON: sets the blink on at the character indicated by R0
Set_Blink_ON
	PUSH	{LR, R1, R0}
	MOV		R0, #0x0D
	BL		SplitNum
	BL		WriteCMD
	MOV		R0, R1
	BL		WriteCMD
	MOV		R0, #0x01
	BL		Delay1ms
	POP		{LR, R1, R0}
	BX		LR

; Subroutine Set_Blink_OFF: sets the blink off 
Set_Blink_OFF
	PUSH	{LR, R1, R0}
	MOV		R0, #0x0C
	BL		SplitNum
	BL		WriteCMD
	MOV		R0, R1
	BL		WriteCMD
	MOV		R0, #0x01
	BL		Delay1ms
	POP		{LR, R1, R0}
	BX		LR

;Delay milliseconds
Delay1ms
	PUSH	{LR, R0, R3, R4}
	MOVS	R3, R0
	BNE		L1; if n=0, return
	BX		LR; return

L1	LDR		R4, =5334
			; do inner loop 5336 times (16 MHz CPU clock)
L2	SUBS	R4, R4,#1
	BNE		L2
	SUBS	R3, R3, #1
	BNE		L1
	POP		{LR, R0, R3, R4}
	BX		LR
	
	END