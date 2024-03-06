; ISR_example.asm: a) Increments/decrements a BCD variable every half second using
; an ISR for timer 2; b) Generates a 2kHz square wave at pin P1.7 using
; an ISR for timer 0; and c) in the 'main' loop it displays the variable
; incremented/decremented using the ISR for timer 2 on the LCD.  Also resets it to 
; zero if the 'CLEAR' push button connected to P1.5 is pressed.
$NOLISTx
$MODN76E003
$LIST

;  N76E003 pinout:
;                               -------
;       PWM2/IC6/T0/AIN4/P0.5 -|1    20|- P0.4/AIN5/STADC/PWM3/IC3
;               TXD/AIN3/P0.6 -|2    19|- P0.3/PWM5/IC5/AIN6
;               RXD/AIN2/P0.7 -|3    18|- P0.2/ICPCK/OCDCK/RXD_1/[SCL]
;                    RST/P2.0 -|4    17|- P0.1/PWM4/IC4/MISO
;        INT0/OSCIN/AIN1/P3.0 -|5    16|- P0.0/PWM3/IC3/MOSI/T1
;              INT1/AIN0/P1.7 -|6    15|- P1.0/PWM2/IC2/SPCLK
;                         GND -|7    14|- P1.1/PWM1/IC1/AIN7/CLO
;[SDA]/TXD_1/ICPDA/OCDDA/P1.6 -|8    13|- P1.2/PWM0/IC0
;                         VDD -|9    12|- P1.3/SCL/[STADC]
;            PWM5/IC7/SS/P1.5 -|10   11|- P1.4/SDA/FB/PWM1
;                               -------
;

CLK           EQU 16600000 ; Microcontroller system frequency in Hz
BAUD          EQU 115200 ; Baud rate of UART in bps
TIMER0_RATE   EQU 4096     ; 2048Hz squarewave (peak amplitude of CEM-1203 speaker)
TIMER0_RELOAD EQU ((65536-(CLK/TIMER0_RATE)))
TIMER2_RATE   EQU 1000     ; 1000Hz, for a timer tick of 1ms
TIMER2_RELOAD EQU ((65536-(CLK/TIMER2_RATE)))
TIMER1_RELOAD     EQU (0x100-(CLK/(16*BAUD)))
TIMER0_RELOAD_1MS EQU (0x10000-(CLK/1000))

SOUND_OUT     equ P1.7
PWM_OUT equ P1.0

; Reset vector
org 0x0000
    ljmp main

; External interrupt 0 vector (not used in this code)
org 0x0003
	reti

; Timer/Counter 0 overflow interrupt vector
org 0x000B
	ljmp Timer0_ISR

; External interrupt 1 vector (not used in this code)
org 0x0013
	reti

; Timer/Counter 1 overflow interrupt vector (not used in this code)
org 0x001B
	reti

; Serial port receive/transmit interrupt vector (not used in this code)
org 0x0023 
	reti
	
; Timer/Counter 2 overflow interrupt vector
org 0x002B
	ljmp Timer2_ISR

; In the 8051 we can define direct access variables starting at location 0x30 up to location 0x7F
dseg at 0x30
Count1ms:     ds 2 ; Used to determine when half second has passed
BCD_counter:  ds 2 ; The BCD counter incrememted in the ISR and displayed in the main loop
BCD_minutes:  ds 1
x:   ds 4
y:   ds 4
bcd: ds 5
VLED_ADC: ds 2
stime: ds 2
stemp: ds 2	;0x05 0x43
rtime: ds 2
rtemp: ds 2
temp: ds 1
pwm_counter: ds 1 ; Free running counter 0, 1, 2, ..., 100, 0
pwm: ds 1 ; pwm percentage
seconds: ds 1 ; a seconds counter attached to Timer 2 ISR
current_state: ds 1
soak_temp: ds 1
soak_time: ds 1
temp_state3: ds 1
reflow_temp: ds 1
reflow_time: ds 1
cooling_temp: ds 1
cooling_time: ds 1
max_soak_time: ds 1
temporary: ds 4


; In the 8051 we have variables that are 1-bit in size.  We can use the setb, clr, jb, and jnb
; instructions with these variables.  This is how you define a 1-bit variable:
bseg
half_seconds_flag: dbit 1 ; Set to one in the ISR every time 500 ms had passed
mf: dbit 1
PB0: dbit 1
PB1: dbit 1
PB2: dbit 1
PB3: dbit 1
PB4: dbit 1
fsm: dbit 1
abort: dbit 1

cseg
; These 'equ' must match the hardware wiring
LCD_RS equ P1.3
;LCD_RW equ PX.X ; Not used in this code, connect the pin to GND
LCD_E  equ P1.4
LCD_D4 equ P0.0
LCD_D5 equ P0.1
LCD_D6 equ P0.2
LCD_D7 equ P0.3

$NOLIST
$include(LCD_4bit.inc) ; A library of LCD related functions and utility macros
$include(math32.inc)
$LIST



soak_time_msg: db 'St:', 0
soak_temp_msg: db 'ST:', 0
      
reflow_time_msg:      db 'Rt:', 0
reflow_temp_msg:      db 'RT:', 0

start_msg: db 'STARTING...', 0
clear: db '                ', 0
fsm_message: db 'FSM', 0
state0: db 'state 0     ', 0
state1: db '  Ramp -> Soak  ', 0
state2: db '      Soak      ', 0
state3: db ' Ramp -> Reflow ', 0
state4: db '     Reflow     ', 0
state5: db '  Cooling Down  ', 0
complete: db 'Reflow Complete', 0
Abort_msg: db ' Aborting... ', 0
stop: db ' Stoping...', 0

Init_All:
	; Configure all the pins for biderectional I/O
	mov	P3M1, #0x00
	mov	P3M2, #0x00
	mov	P1M1, #0x00
	mov	P1M2, #0x00
	mov	P0M1, #0x00
	mov	P0M2, #0x00
	
	orl	CKCON, #0x10 ; CLK is the input for timer 1
	orl	PCON, #0x80 ; Bit SMOD=1, double baud rate
	mov	SCON, #0x52
	anl	T3CON, #0b11011111
	anl	TMOD, #0x0F ; Clear the configuration bits for timer 1
	orl	TMOD, #0x20 ; Timer 1 Mode 2
	mov	TH1, #TIMER1_RELOAD ; TH1=TIMER1_RELOAD;
	setb TR1
	
	; Using timer 0 for delay functions.  Initialize here:
	clr	TR0 ; Stop timer 0
	orl	CKCON,#0x08 ; CLK is the input for timer 0
	anl	TMOD,#0xF0 ; Clear the configuration bits for timer 0
	orl	TMOD,#0x01 ; Timer 0 in Mode 1: 16-bit timer
	
	; Initialize the pins used by the ADC (P1.1, P1.7) as input.
	orl	P1M1, #0b10000010
	anl	P1M2, #0b01111101
	
	; Initialize and start the ADC:
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x07 ; Select channel 7
	; AINDIDS select if some pins are analog inputs or digital I/O:
	mov AINDIDS, #0x00 ; Disable all analog inputs
	orl AINDIDS, #0b10000001 ; Activate AIN0 and AIN7 analog inputs
	orl ADCCON1, #0x01 ; Enable ADC
	
	ret

; Send a character using the serial port

putchar:
    jnb TI, putchar
    clr TI
    mov SBUF, a
    ret

; Send a constant-zero-terminated string using the serial port
SendString:
    clr A
    movc A, @A+DPTR
    jz SendStringDone
    lcall putchar
    inc DPTR
    sjmp SendString
SendStringDone:
    ret
 
Hello_World:
    DB  '\r', '\n', 0
    
wait_1ms:
	clr	TR0 ; Stop timer 0
	clr	TF0 ; Clear overflow flag
	mov	TH0, #high(TIMER0_RELOAD_1MS)
	mov	TL0,#low(TIMER0_RELOAD_1MS)
	setb TR0
	jnb	TF0, $ ; Wait for overflow
	ret

; Wait the number of miliseconds in R2
waitms:
	lcall wait_1ms
	djnz R2, waitms
	ret

Read_ADC:
	clr ADCF
	setb ADCS ;  ADC start trigger signal
    jnb ADCF, $ ; Wait for conversion complete
    
    ; Read the ADC result and store in [R1, R0]
    mov a, ADCRL
    anl a, #0x0f
    mov R0, a
    mov a, ADCRH   
    swap a
    push acc
    anl a, #0x0f
    mov R1, a
    pop acc
    anl a, #0xf0
    orl a, R0
    mov R0, A
	ret

Send_BCD mac 
	push ar0 
	mov r0, %0 
	lcall ?Send_BCD 
	pop ar0 
endmac 
	
?Send_BCD: 
	push acc 
	; Write most significant digit 
	mov a, r0 
	swap a 
	anl a, #0fh 
	orl a, #30h 
	lcall putchar 
	; write least significant digit 
	mov a, r0 
	anl a, #0fh 
	orl a, #30h 
	lcall putchar 
	pop acc 
	ret 

;---------------------------------;
; Routine to initialize the ISR   ;
; for timer 0                     ;
;---------------------------------;
Timer0_Init:
	orl CKCON, #0b00001000 ; Input for timer 0 is sysclk/1
	mov a, TMOD
	anl a, #0xf0 ; 11110000 Clear the bits for timer 0
	orl a, #0x01 ; 00000001 Configure timer 0 as 16-timer
	mov TMOD, a
	mov TH0, #high(TIMER0_RELOAD)
	mov TL0, #low(TIMER0_RELOAD)
	; Enable the timer and interrupts
    setb ET0  ; Enable timer 0 interrupt
    setb TR0  ; Start timer 0
	ret

;---------------------------------;
; ISR for timer 0.  Set to execute;
; every 1/4096Hz to generate a    ;
; 2048 Hz wave at pin SOUND_OUT   ;
;---------------------------------;
Timer0_ISR:
	;clr TF0  ; According to the data sheet this is done for us already.
	; Timer 0 doesn't have 16-bit auto-reload, so
	clr TR0
	mov TH0, #high(TIMER0_RELOAD)
	mov TL0, #low(TIMER0_RELOAD)
	setb TR0
	cpl SOUND_OUT ; Connect speaker the pin assigned to 'SOUND_OUT'!
	reti

;---------------------------------;
; Routine to initialize the ISR   ;
; for timer 2                     ;
;---------------------------------;
Timer2_Init:
	mov T2CON, #0 ; Stop timer/counter.  Autoreload mode.
	mov TH2, #high(TIMER2_RELOAD)
	mov TL2, #low(TIMER2_RELOAD)
	; Set the reload value
	orl T2MOD, #0x80 ; Enable timer 2 autoreload
	mov RCMP2H, #high(TIMER2_RELOAD)
	mov RCMP2L, #low(TIMER2_RELOAD)
	; Init One millisecond interrupt counter.  It is a 16-bit variable made with two 8-bit parts
	clr a
	mov Count1ms+0, a
	mov Count1ms+1, a
	; Enable the timer and interrupts
	orl EIE, #0x80 ; Enable timer 2 interrupt ET2=1
    setb TR2  ; Enable timer 2
	mov pwm_counter, #0
	ret

;---------------------------------;
; ISR for timer 2                 ;
;---------------------------------;
Timer2_ISR:
	clr TF2  ; Timer 2 doesn't clear TF2 automatically. Do it in the ISR.  It is bit addressable.
	cpl P0.4 ; To check the interrupt rate with oscilloscope. It must be precisely a 1 ms pulse.
	
	; The two registers used in the ISR must be saved in the stack
	push acc
	push psw
	
; Increment the 16-bit one mili second counter
	inc Count1ms+0    ; Increment the low 8-bits first
	mov a, Count1ms+0 ; If the low 8-bits overflow, then increment high 8-bits
	jnz Inc_Done
	inc Count1ms+1


	inc pwm_counter
	clr c
	mov a, pwm
	subb a, pwm_counter ; If pwm_counter <= pwm then c=1
	cpl c
	mov PWM_OUT, c
	mov a, pwm_counter
	cjne a, #100, Timer2_ISR_done
	mov pwm_counter, #0

	

Inc_Done:
	; Check if half second has passed
	mov a, Count1ms+0
	cjne a, #low(1000), Timer2_ISR_done ; Warning: this instruction changes the carry flag!
	mov a, Count1ms+1
	cjne a, #high(1000), Timer2_ISR_done
	
	; 500 milliseconds have passed.  Set a flag so the main program knows
	setb half_seconds_flag ; Let the main program know half second had passed
	cpl TR0 ; Enable/disable timer/counter 0. This line creates a beep-silence-beep-silence sound.
	; Reset to zero the milli-seconds counter, it is a 16-bit variable
	clr a
	mov Count1ms+0, a
	mov Count1ms+1, a
	

	; Increment the BCD counter
	mov a, BCD_counter+0
	
	add a, #0x01
	sjmp Timer2_ISR_da

Timer2_ISR_da:
	da a ; Decimal adjust instruction.  Check datasheet for more details!
	mov BCD_counter, a

	cjne a, #0x00, Timer2_ISR_done

	mov a, BCD_Counter+1
    add a, #0x01        ; Increment time.
	da a 
	mov BCD_Counter+1,a 

Timer2_ISR_done:
	pop psw
	pop acc
	reti

LCD_PB:
	; Set variables to 1: 'no push button pressed'
	setb PB0
	setb PB1
	setb PB2
	setb PB3
	setb PB4
	; The input pin used to check set to '1'
	setb P1.5
	
	; Check if any push button is pressed
	clr P0.0
	clr P0.1
	clr P0.2
	clr P0.3
	clr P1.3
	jb P1.5, LCD_PB_Done

	; Debounce
	mov R2, #50
	lcall waitms
	jb P1.5, LCD_PB_Done

	; Set the LCD data pins to logic 1
	setb P0.0
	setb P0.1
	setb P0.2
	setb P0.3
	setb P1.3
	
	; Check the push buttons one by one
	clr P1.3
	mov c, P1.5
	mov PB4, c
	setb P1.3

	clr P0.0
	mov c, P1.5
	mov PB3, c
	setb P0.0
	
	clr P0.1
	mov c, P1.5
	mov PB2, c
	setb P0.1
	
	clr P0.2
	mov c, P1.5
	mov PB1, c
	setb P0.2
	
	clr P0.3
	mov c, P1.5
	mov PB0, c
	setb P0.3

LCD_PB_Done:		
	ret

reset:
	Set_Cursor(1, 1)
	Send_Constant_String(#clear)

	Set_Cursor(2, 1)
	Send_Constant_String(#clear)
	ret	

; We can display a number any way we want.  In this case with
; four decimal places.
Display_formated_BCD:
	Set_Cursor(2, 2)
	Display_BCD(bcd+3)
	Display_BCD(bcd+2)
	Display_char(#'.')
	Display_BCD(bcd+1)
	;Display_BCD(bcd+0)
	;Set_Cursor(2, 10)
	;Display_char(#'=')
	ret
	
update_temp:
	;Set_Cursor(1, 7)
    ;Display_BCD(BCD_counter)

	; Read the 2.08V LED voltage connected to AIN0 on pin 6
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x00 ; Select channel 0
	lcall Read_ADC
	; Save result for later use
	mov VLED_ADC+0, R0
	mov VLED_ADC+1, R1
	; Read the signal connected to AIN7
	anl ADCCON0, #0xF0
	orl ADCCON0, #0x07 ; Select channel 7
	lcall Read_ADC
	; Convert to voltage
	mov x+0, R0
	mov x+1, R1
	; Pad other bits with zero
	mov x+2, #0
	mov x+3, #0
	Load_y(20740) ; The MEASURED LED voltage: 2.074V, with 4 decimal places
	lcall mul32
	; Retrive the ADC LED value
	mov y+0, VLED_ADC+0
	mov y+1, VLED_ADC+1
	; Pad other bits with zero
	mov y+2, #0
	mov y+3, #0
	lcall div32
	; Convert to BCD and display
	
	Load_y(100000)
	lcall mul32
	
	Load_y(1353)
	lcall div32

	Load_y(205000)
	lcall add32
	
	; Convert to BCD and send to serial port for graphing
	lcall hex2bcd
	Send_BCD(bcd+3)
	Send_BCD(bcd+2)
	mov a, #'.'
	lcall putchar
	Send_BCD(bcd+1)
	;Send_BCD(bcd+0)
	mov a, #'\r'
	lcall putchar
	mov a, #'\n'
	lcall putchar
	

	;mov temp, x
	ret
	
;---------------------------------;
; Main program. Includes hardware ;
; initialization and 'forever'    ;
; loop.                           ;
;---------------------------------;
main:
	; Initialization
    mov SP, #0x7F
    mov P0M1, #0x00
    mov P0M2, #0x00
    mov P1M1, #0x00
    mov P1M2, #0x00
    mov P3M2, #0x00
    mov P3M2, #0x00
    mov sp, #0x7f
	lcall Init_All
    lcall LCD_4BIT 
    lcall Timer0_Init
    lcall Timer2_Init
    setb EA   ; Enable Global interrupts
	setb fsm
    lcall LCD_4BIT

	mov pwm, #0
	mov max_soak_time, #0x50
    ; For convenience a few handy macros are included in 'LCD_4bit.inc':

	;mov BCD_counter, #0x00
	;mov BCD_minutes, #0x00
	mov rtemp+0, #0x17
	mov rtemp+1, #0x02
	mov stemp+0, #0x48
	mov stemp+1, #0x01
	mov stime+0, #0x78
	mov stime+1, #0x00
	mov rtime+0, #0x39
	mov rtime+1, #0x00

	setb half_seconds_flag
	

	
	; After initialization the program stays in this 'forever' loop
loop_a:
	mov pwm, #0
	lcall LCD_PB
	lcall update_temp
	
	jb PB0, loop_b  ; if the 'CLEAR' button is not pressed skip
	Wait_Milli_Seconds(#80)	; Debounce delay.  This macro is also in 'LCD_4bit.inc'
	jb PB0, loop_b  ; if the 'CLEAR' button is not pressed skip
	mov a, stemp+0  ; Load the least significant byte of stemp
	add a, #0x01    ; Increment the LSB by 1
	da a            ; Decimal adjust the result to handle carry
	
	mov stemp+0, a
	cjne a, #0x00, loop_b

	mov a, stemp+1
	add a, #0x01
	da a
	mov stemp+1, a
	ljmp loop_b
	
loop_b:
	lcall LCD_PB
	
	jb PB1, loop_c ; if the 'CLEAR' button is not pressed skip
	Wait_Milli_Seconds(#80)	; Debounce delay.  This macro is also in 'LCD_4bit.inc'
	jb PB1, loop_c ; if the 'CLEAR' button is not pressed skip

	mov a, rtemp+0
	add a, #0x01
	da a
	mov rtemp+0, a

	cjne a, #0x00, loop_c

	mov a, rtemp+1
	add a, #0x01
	da a
	mov rtemp+1, a
	ljmp loop_c
	 
loop_c:
	lcall LCD_PB
	
	jb PB2, loop_d ; if the 'CLEAR' button is not pressed skip
	Wait_Milli_Seconds(#80)	; Debounce delay.  This macro is also in 'LCD_4bit.inc'
	jb PB2, loop_d ; if the 'CLEAR' button is not pressed skip

	mov a, stime+0
	add a, #0x01
	da a
	mov stime+0, a

	cjne a, #0x00, loop_d

	mov a, stime+1
    add a, #0x01        ; Increment time.
	da a 
	mov stime+1,a 
	ljmp loop_d

loop_d:
	lcall LCD_PB
	
	jb PB3, loop_e ; if the 'CLEAR' button is not pressed skip
	Wait_Milli_Seconds(#80)	; Debounce delay.  This macro is also in 'LCD_4bit.inc'
	jb PB3, loop_e ; if the 'CLEAR' button is not pressed skip

	mov a, rtime
	add a, #0x01
	da a
	mov rtime, a

	cjne a, #0x00, loop_e

	mov a, rtime+1
    add a, #0x01        ; Increment time.
	da a 
	mov rtime+1,a 
	sjmp loop_e

loop_e:
	lcall LCD_PB
	
	jb PB4, loop_f ; if the 'CLEAR' button is not pressed skip
	Wait_Milli_Seconds(#80)	; Debounce delay.  This macro is also in 'LCD_4bit.inc'
	jb PB4, loop_f ; if the 'CLEAR' button is not pressed skip


	ljmp fsm_start
loop_f: 
	clr half_seconds_flag ; We clear this flag in the main loop, but it is set in the ISR for timer 2
	Set_Cursor(1, 1)
    Send_Constant_String(#soak_time_msg)
    Set_Cursor(1, 10)
    Send_Constant_String(#soak_temp_msg)
	Set_Cursor(2, 1)
    Send_Constant_String(#reflow_time_msg)
    Set_Cursor(2, 10)
	Send_Constant_String(#reflow_temp_msg)

    
	Set_Cursor(1, 4)
    Display_BCD(stime+1)
	Display_BCD(stime+0)
	Set_Cursor(1, 13)
    Display_BCD(stemp+1)
	Display_BCD(stemp+0)
	Set_Cursor(2, 4)
    Display_BCD(rtime+1)
	Display_BCD(rtime+0)
	Set_Cursor(2, 13)
    Display_BCD(rtemp+1)
	Display_BCD(rtemp+0)

    ljmp loop_a

fsm_start:
	cpl fsm
	lcall reset

	Set_Cursor(1, 1)
	Send_Constant_String(#start_msg)
	mov current_state, #1
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	mov BCD_counter, #0x00
	mov BCD_Counter+1, #0x00
	sjmp FSM1_state1
	
;Finite State Machine

set_abort:
	setb abort
	ret

soak_time_check:
	mov a, max_soak_time
	clr c
	subb a, BCD_Counter
	jc temperature_check
	ret
temperature_check:
	lcall update_temp
	mov a, bcd+2
	clr c
	subb a, #0x50
	jc hundredcheck6
	ret

hundredcheck6:
	mov a, bcd+3
	cjne a, #0, return
	ljmp set_abort

return:
	ret

FSM1_state1: ;ramp to soak
    lcall LCD_PB

	jb PB4, here ; if the 'CLEAR' button is not pressed skip
	Wait_Milli_Seconds(#50)
	jb PB4, here ; if the 'CLEAR' button is not pressed skip
	lcall reset
	ljmp loop_a
here:	
	clr abort
	;mov a, current_state
	;cjne a, #1, FSM1_State2
	Set_Cursor(1,1)
	Send_Constant_String(#state1)
	Send_BCD(#3)
	Send_BCD(#0)
	mov a, #'\r'
	lcall putchar
	mov a, #'\n'
	lcall putchar
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	lcall update_temp
	lcall Display_formated_BCD
	
	Set_Cursor(2, 10)
    Display_BCD(stemp+1)
	Display_BCD(stemp+0)

	mov pwm, #100 ;set power to 100%
	
	lcall soak_time_check ;check to see if the maximum time for soaking has been reached
	jb abort, aborthere ;abort if abort bit set to 1

	mov a, stemp+0 
	clr c
	subb a, bcd+2 ;check if temperature has been exceeded threshold
	jnc stateone

hundredcheck1:
	mov a, stemp+1
	subb a, bcd+3 ;check if time has been exceeded threshold
	jnc stateone
	mov BCD_Counter, #0
	mov BCD_Counter+1, #0
	lcall reset
	ljmp FSM1_state2

stateone:
	ljmp FSM1_State1

aborthere:
	lcall reset
	ljmp loop_a
	
FSM1_state2:
	;mov a, current_state
	;cjne a, #2, FSM1_state3
	lcall LCD_PB

	jb PB4, here2 ; if the 'CLEAR' button is not pressed skip
	Wait_Milli_Seconds(#50)
	jb PB4, here2 ; if the 'CLEAR' button is not pressed skip
	lcall reset
	ljmp loop_a

here2:
	Set_Cursor(1,1)
	Send_Constant_String(#state2)
	Set_Cursor(2,10)
	Display_BCD(stime+1)
	Display_BCD(stime+0)

	Send_BCD(#3)
	Send_BCD(#1)
	mov a, #'\r'
	lcall putchar
	mov a, #'\n'
	lcall putchar
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	lcall update_temp
	
	mov pwm, #20 ;set power to 20%
	Set_Cursor(2, 1)
    Display_BCD(BCD_counter+1)
	Display_BCD(BCD_counter)
	mov a, stime
	clr c
	subb a, BCD_Counter ;check if time has been exceeded threshold
	jnc statetwo

hundredcheck2:
	mov a, stime+1
	subb a, BCD_Counter+1 ;check if time has been exceeded threshold
	jnc statetwo
	mov current_state, #3
	lcall reset
	ljmp FSM1_state3

statetwo:
	ljmp FSM1_State2

	
FSM1_state3:
	;mov a, current_state
	;cjne a, #3, FSM1_state4
	lcall LCD_PB

	jb PB4, here3 ; if the 'CLEAR' button is not pressed skip
	Wait_Milli_Seconds(#50)
	jb PB4, here3 ; if the 'CLEAR' button is not pressed skip
	lcall reset
	ljmp loop_a
here3:
	Set_Cursor(1,1)
	Send_Constant_String(#state3)
	
	Send_BCD(#3)
	Send_BCD(#2)
	mov a, #'\r'
	lcall putchar
	mov a, #'\n'
	lcall putchar
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	lcall update_temp
	lcall Display_formated_BCD

	Set_Cursor(2,10)
	Display_BCD(rtemp+1)
	Display_BCD(rtemp+0)
	
	mov pwm, #100 ;set power to 100%
	mov a, rtemp+0
	clr c
	subb a, bcd+2 ;check if temperature has been exceeded threshold
	jnc statethree

hundredcheck3:
	mov a, rtemp+1
	subb a, BCD+3 ;check if time has been exceeded threshold
	jnc statethree
	lcall reset
	mov BCD_Counter, #0 ;set seconds to 0
	mov BCD_Counter+1, #0 ;set seconds to 0
	ljmp FSM1_state4

statethree:
	ljmp FSM1_State3

FSM1_state4:


	;mov a, current_state
	;cjne a, #4, FSM1_state5
	lcall LCD_PB

	jb PB4, here4 ; if the 'CLEAR' button is not pressed skip
	Wait_Milli_Seconds(#50)
	jb PB4, here4 ; if the 'CLEAR' button is not pressed skip
	lcall reset
	ljmp loop_a
here4:
	Set_Cursor(1,1)
	Send_Constant_String(#state4)
	Set_Cursor(2,10)
	Display_BCD(rtime+1)
	Display_BCD(rtime+0)

	Send_BCD(#3)
	Send_BCD(#3)
	mov a, #'\r'
	lcall putchar
	mov a, #'\n'
	lcall putchar
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	lcall update_temp

	mov pwm, #20 ;set power to 20%
	Set_Cursor(2, 1)
    ;Display_BCD(BCD_counter+1)
	Display_BCD(BCD_counter+0)
	mov a, rtime
	clr c
	subb a, BCD_Counter ;check if time has been exceeded threshold
	jnc statefour

hundredcheck4:
	mov a, rtime+1
	subb a, BCD_Counter+1 ;check if time has been exceeded threshold
	jnc statefour
	lcall reset
	ljmp FSM1_state5

statefour:
	ljmp FSM1_State4


FSM1_state5:

	;mov a, current_state
	;cjne a, #5, FSM1_complete
	lcall LCD_PB

	jb PB4, here5 ; if the 'CLEAR' button is not pressed skip
	Wait_Milli_Seconds(#50)
	jb PB4, here5 ; if the 'CLEAR' button is not pressed skip
	lcall reset
	ljmp loop_a
here5:
	Set_Cursor(1,1)
	Send_Constant_String(#state5)
	Send_BCD(#3)
	Send_BCD(#4)
	mov a, #'\r'
	lcall putchar
	mov a, #'\n'
	lcall putchar
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	lcall update_temp
	lcall Display_formated_BCD
	
	mov pwm, #0 ;set power to 0%

	mov a, #96
	clr c
	subb a, bcd+2 ;check if temperature is below threshold
	jc statefive
	
hundredcheck5:
	mov a, #0
	subb a, bcd+3 ;check if time has been exceeded threshold
	jc statefive
	lcall reset
	ljmp FSM1_complete

statefive:
	ljmp FSM1_State5

FSM1_complete:
	lcall reset
	Set_Cursor(1,1)
	Send_Constant_String(#complete)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Wait_Milli_Seconds(#250)
	Send_BCD(#3)
	Send_BCD(#5)
	mov a, #'\r'
	lcall putchar
	mov a, #'\n'
	lcall putchar
	lcall reset
	ljmp loop_a
END