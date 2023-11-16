; Functions that convert to and from ASCII
; John Tadrous
; September 27, 2020

Size_Cal    EQU     4;              ; 4 digits per number
    
    AREA    |.text|, CODE, READONLY, ALIGN=2;
    EXPORT  Hex2ASCII
    EXPORT  ASCII2Hex   
        
; Subroutine HEX2ASCII- Converts a hex value in R1 into an ASCII string of BCD characters in location
; pointed to by R0
Hex2ASCII
	PUSH	{R0-R5, LR}
	MOV		R2, #0x20    			; blanking the content before writing the string the BCD digits
	MOV		R3, #Size_Cal
Blank_digit
	SUB		R3, #1
	STRB	R2, [R0, R3]			; writing a blank
	CMP		R3, #0x00
	BHI		Blank_digit
	STRB	R3, [R0, #Size_Cal]		; here R3 is NULL so we simply NULL the last byte of the string
	MOV		R3, #Size_Cal
Attach_digit	
	MOV		R2, R1					; quotient in R2
	CMP		R2, #10
	BLO		Last_digit
	MOV		R4, #10					; R4 is temporarily used to hold #10
	UDIV	R1, R2, R4				; R1=floor(R2/10)
	MUL		R5, R1, R4
	SUB		R4, R2, R5				; remainder in R4
	ADD		R4, #0x30				; ASCII
	SUB		R3, #1
	STRB	R4, [R0, R3]			; store the ASCII code of BCD digit
	CMP		R3, #0x00
	BEQ		End_Hex2ASCII		; typically this will not be executed unless we have an overflow (c.f. Lab7)
	B		Attach_digit
Last_digit							; here we store the quotient as the most significant BCD digit
	ADD		R2, #0x30
	SUB		R3, #1
	STRB	R2, [R0, R3]
End_Hex2ASCII
	POP		{R0-R5, LR}
	BX		LR
 
 
 ; Subroutine ASCII2HEX- Converts a string of BCD characters pointed to by R0
; to a hex equivalent value in R0
ASCII2Hex
	PUSH	{R1,R2, LR}
	MOV		R2, #0x00    ; initialize R2
ASCII2HEX_Again
	MOV		R1, #0x00						; initialize R1
	LDRB	R1, [R0]						; R1 <- digit in the string
	ADD		R0, #1							; increment base address
	CMP		R1, #0x00
	BEQ		End_ASCII2HEX	;
	SUB		R1,	#0x30						; convert to decimal digit
	ADD		R2, R1							; R2 <-- R2 + R1
	MOV		R1, R2							; R1 <-- R2
	MOV		R3, #0x00
	LDRB	R3, [R0]
	CMP		R3, #0x00						; check the next digit in the string before multiplying by 10
	BEQ		End_ASCII2HEX	;
	MOV		R2, #10							; R2 <- 10
	MUL		R1, R2							; R1=10*R1
	MOV		R2, R1							; copy the result in R2
	B		ASCII2HEX_Again
End_ASCII2HEX
    MOV     R0, R1
	POP		{R1,R2, LR}
	BX		LR
 
    END
    
    END