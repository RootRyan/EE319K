; Print.s
; Student names: change this to your names or look very silly
; Last modification date: change this to the last modification date or look very silly
; Runs on LM4F120 or TM4C123
; EE319K lab 7 device driver for any LCD
;
; As part of Lab 7, students need to implement these LCD_OutDec and LCD_OutFix
; This driver assumes two low-level LCD functions
; ST7735_OutChar   outputs a single 8-bit ASCII character
; ST7735_OutString outputs a null-terminated string 

    IMPORT   ST7735_OutChar
    IMPORT   ST7735_OutString
    EXPORT   LCD_OutDec
    EXPORT   LCD_OutFix

    AREA    |.text|, CODE, READONLY, ALIGN=2
    THUMB

  

;-----------------------LCD_OutDec-----------------------
; Output a 32-bit number in unsigned decimal format
; Input: R0 (call by value) 32-bit unsigned number
; Output: none
; Invariables: This function must not permanently modify registers R4 to R11
LCD_OutDec
length EQU 4
	SUB SP, #8
	MOV R12, SP
	LDR R3, [R12, #length]
	MOV R3, #0
	STR R3, [R12, #length]
	
	MOV R1, R0
Loop
	MOV R2, R1
	MOV R3, #10
	UDIV R1, R3
	MUL R1, R3
	SUB R3, R2, R1 ; R2 (value before division) - R1 (value after division multiplied by 10) = R3 (remainder)
	PUSH {R3, R12}
	MOV R3, #10
	UDIV R1, R3
	LDR R3, [R12, #length]
	ADD R3, #1
	STR R3, [R12, #length]
	CMP R1, #0
	BHI Loop
Output
	POP {R0, R12}
	CMP R0, #0
	BHI check1
	MOV R0, #0x30
	B done
check1
	CMP R0, #1
	BHI check2
	MOV R0, #0x31
	B done
check2
	CMP R0, #2
	BHI check3
	MOV R0, #0x32
	B done
check3
	CMP R0, #3
	BHI check4
	MOV R0, #0x33
	B done
check4
	CMP R0, #4
	BHI check5
	MOV R0, #0x34
	B done
check5
	CMP R0, #5
	BHI check6
	MOV R0, #0x35
	B done
check6
	CMP R0, #6
	BHI check7
	MOV R0, #0x36
	B done
check7
	CMP R0, #7
	BHI check8
	MOV R0, #0x37
	B done
check8
	CMP R0, #8
	BHI check9
	MOV R0, #0x38
	B done
check9
	MOV R0, #0x39
done
	PUSH {R12, LR}
	BL ST7735_OutChar
	POP {R12, LR}
	LDR R3, [R12, #length]
	SUB R3, #1
	STR R3, [R12, #length]
	CMP R3, #0
	BHI Output
	ADD SP, #8
	
    BX LR
;* * * * * * * * End of LCD_OutDec * * * * * * * *

; -----------------------LCD _OutFix----------------------
; Output characters to LCD display in fixed-point format
; unsigned decimal, resolution 0.001, range 0.000 to 9.999
; Inputs:  R0 is an unsigned 32-bit number
; Outputs: none
; E.g., R0=0,    then output "0.000 "
;       R0=3,    then output "0.003 "
;       R0=89,   then output "0.089 "
;       R0=123,  then output "0.123 "
;       R0=9999, then output "9.999 "
;       R0>9999, then output "*.*** "
; Invariables: This function must not permanently modify registers R4 to R11
LCD_OutFix
lengthFix EQU 4 ;Creates local variable lengthFix
	SUB SP, #8
	MOV R12, SP
	LDR R3, [R12, #lengthFix]
	MOV R3, #0
	STR R3, [R12, #lengthFix]
	
	MOV R1, R0
LoopFix
	MOV R2, R1
	MOV R3, #10
	UDIV R1, R3
	MUL R1, R3
	SUB R3, R2, R1 ;R2 (value before division) - R1 (value after division multiplied by 10) = R3 (remainder)
	PUSH {R3, R12} ;Stores remainder
	MOV R3, #10
	UDIV R1, R3
	LDR R3, [R12, #lengthFix]
	ADD R3, #1
	STR R3, [R12, #lengthFix]
	CMP R1, #0
	BHI LoopFix
	
;Checks length of decimal number (Does nothing if lengthFix >= 4, Outputs 0. if lengthFix = 3, Outputs 0.0 if lengthFix = 2, Outputs 0.00 if lengthFix = 1)
	MOV R1, #1
	LDR R3, [R12, #lengthFix]
	CMP R3, #3
	BHI OutputFix
	
	MOV R1, #0
	MOV R0, #0x30
	PUSH {R1, LR}
	BL ST7735_OutChar
	POP {R1, LR}
	MOV R0, #0x2E
	PUSH {R1, LR}
	BL ST7735_OutChar
	POP {R1, LR}
	
	LDR R3, [R12, #lengthFix]
	CMP R3, #2
	BHI OutputFix
	
	MOV R0, #0x30
	PUSH {R1, LR}
	BL ST7735_OutChar
	POP {R1, LR}
	
	LDR R3, [R12, #lengthFix]
	CMP R3, #1
	BHI OutputFix
	
	MOV R0, #0x30
	PUSH {R1, LR}
	BL ST7735_OutChar
	POP {R1, LR}

OutputFix
	LDR R3, [R12, #lengthFix]
	CMP R3, #4
	BHI Stars
	POP {R0, R12}
	CMP R0, #0
	BHI check1Fix
	MOV R0, #0x30
	B doneFix
check1Fix
	CMP R0, #1
	BHI check2Fix
	MOV R0, #0x31
	B doneFix
check2Fix
	CMP R0, #2
	BHI check3Fix
	MOV R0, #0x32
	B doneFix
check3Fix
	CMP R0, #3
	BHI check4Fix
	MOV R0, #0x33
	B doneFix
check4Fix
	CMP R0, #4
	BHI check5Fix
	MOV R0, #0x34
	B doneFix
check5Fix
	CMP R0, #5
	BHI check6Fix
	MOV R0, #0x35
	B doneFix
check6Fix
	CMP R0, #6
	BHI check7Fix
	MOV R0, #0x36
	B doneFix
check7Fix
	CMP R0, #7
	BHI check8Fix
	MOV R0, #0x37
	B doneFix
check8Fix
	CMP R0, #8
	BHI check9Fix
	MOV R0, #0x38
	B doneFix
check9Fix
	MOV R0, #0x39
doneFix
	PUSH {R1, LR}
	BL ST7735_OutChar
	POP {R1, LR}
	CMP R1, #0
	BEQ afterDone
	MOV R1, #0
	MOV R0, #0x2E
	PUSH {R1, LR}
	BL ST7735_OutChar
	POP {R1, LR}
afterDone	
	LDR R3, [R12, #lengthFix]
	SUB R3, #1
	STR R3, [R12, #lengthFix]
	CMP R3, #0
	BHI OutputFix
	B Ending
Stars ;Action performed if length > 4
	MOV R0, #0x2A
	PUSH {R1, LR}
	BL ST7735_OutChar
	POP {R1, LR}
	MOV R0, #0x2E
	PUSH {R1, LR}
	BL ST7735_OutChar
	POP {R1, LR}
	MOV R0, #0x2A
	PUSH {R1, LR}
	BL ST7735_OutChar
	POP {R1, LR}
	MOV R0, #0x2A
	PUSH {R1, LR}
	BL ST7735_OutChar
	POP {R1, LR}
	MOV R0, #0x2A
	PUSH {R1, LR}
	BL ST7735_OutChar
	POP {R1, LR}
StarsLoop
	POP {R0, R12}
	LDR R3, [R12, #lengthFix]
	SUB R3, #1
	STR R3, [R12, #lengthFix]
	CMP R3, #0
	BHI StarsLoop
Ending
	ADD SP, #8
	
    BX LR
 
     ALIGN
;* * * * * * * * End of LCD_OutFix * * * * * * * *

     ALIGN                           ; make sure the end of this section is aligned
     END                             ; end of file
