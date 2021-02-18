.include "tn861Adef.inc"
.include "icakb.inc"

.def zero		= r0
.def i_sreg		= r1
.def msecl_r	= r2
.def msech_r	= r3
.def sec_r		= r4
.def min_r		= r5
.def hour_r		= r6

.def temp1 		= r16
.def temp2		= r17
.def temp3 		= r18
.def temp4		= r19
.def temp_delay	= r20

.dseg
.org			SRAM_START
hour:			.byte 1
minute:			.byte 1
second:			.byte 1
msecond_l:		.byte 1
msecond_h:		.byte 1
lcd_buf_h:		.byte 1
lcd_buf_l:		.byte 1

.cseg
.org		0	

				rjmp		RESET 			; Reset Handler
	reti		;rjmp		EXT_INT0 		; IRQ0 Handler
	reti		;rjmp		PCINT			; PCINT Handler
	reti		;rjmp		TIM1_COMPA		; Timer1 CompareA Handler
	reti		;rjmp		TIM1_COMPB		; Timer1 CompareB Handler
	reti		;rjmp		TIM1_OVF		; Timer1 Overflow Handler
	reti		;rjmp		TIM0_OVF		; Timer0 Overflow Handler
	reti		;rjmp		USI_START		; USI Start Handler
	reti		;rjmp		USI_OVF			; USI Overflow Handler
	reti		;rjmp		EE_RDY			; EEPROM Ready Handler
	reti		;rjmp		ANA_COMP		; Analog Comparator Handler
				rjmp		ADC_INT			; ADC Conversion Handler
	reti		;rjmp		WDT				; WDT Interrupt Handler
	reti		;rjmp		EXT_INT1		; IRQ1 Handler
				rjmp		TIM0_COMPA		; Timer0 CompareA Handler
	reti		;rjmp		TIM0_COMPB		; Timer0 CompareB Handler
	reti		;rjmp		TIM0_CAPT		; Timer0 Capture Event Handler
	reti		;rjmp		TIM1_COMPD		; Timer1 CompareD Handler
	reti		;rjmp		FAULT_PROTECT ; Timer1 Fault Protection

;********* Подпрограммы обработки прерываний ********
ADC_INT:

	reti

TIM0_COMPA:
	in		i_sreg,	SREG
	push	temp1
	push	temp2
	push	temp3
	
	lds		ZL,		msecond_l
	lds		ZH,		msecond_h
	adiw	ZH:ZL,	1					; инкремент миллисекунд
	cpi		ZL,			low(1000)
	ldi		temp1,		high(1000)
	cpc		ZH,		temp1
	brne	out_msec
	clr		ZL
	clr		ZH

	in		temp1,	PORTA
	ldi		temp2, (1<<LED)
	eor		temp1,	temp2
	out		PORTA,	temp1

	lds		temp1,		second
	inc		temp1						; инкремент секунд
	cpi		temp1,		60
	brne	out_sec
	clr		temp1

	lds		temp2,		minute
	inc		temp2						; инкремент минут
	cpi		temp2,		60
	brne	OUT_min
	clr		temp2

	lds		temp3,		hour
	inc		temp3						; инкремент часов
	cpi		temp3,		24
	brne	OUT_hour
	clr		temp3

OUT_hour:
	sts		hour,		temp3
OUT_min:
	sts		minute,		temp2
OUT_sec:
	sts		second,		temp1
OUT_msec:
	sts		msecond_l,		ZL
	sts		msecond_h,		ZH

	pop		temp3
	pop		temp2
	pop		temp1
	out		SREG,	i_sreg
	reti

;********* Подпрограммы *******

ADC_init:
;	ldi		temp1,	(1<<ADLAR)
;	out		ADMUX,	temp1

	ldi		temp1,	(1<<ADEN)|(1<<ADIE)|(1<<ADPS2)|(1<<ADPS1)
	out		ADCSRA,	temp1

	out		ADCSRB, zero
	
	ldi		temp1,	(1<<ADC0D)|(1<<ADC1D)
	out		DIDR0,	temp1

	out		DIDR1, zero
	
	ret

PORT_init:
	ldi		temp1,	(1<<BUZ)|(1<<LED)|(1<<RS)|(1<<E)
	out		DDRA,	temp1
	out		PORTA,	zero

	ldi		temp1,	(1<<DB4)|(1<<DB5)|(1<<DB6)|(1<<DB7)
	out		DDRB,	temp1
	out		PORTB,	zero

;	ldi		temp1,	(1<<SB1)
;	out		PORTB,	temp1
	sbi		PORTB,	SB1
	out		MCUCR,	zero

	ret

TIM0_init:
	ldi		temp1, (1<<WGM00)
	out		TCCR0A,	temp1
	
	ldi		temp1,	249
	out		OCR0A,	temp1

	ldi		temp1,	(1<<OCIE0A)
	out		TIMSK,	temp1
	
;	ldi		temp1, (1<<CS00)
	ldi		temp1, (1<<CS01)|(1<<CS00)						; предделитель 64
	out		TCCR0B,	temp1

	ret

TIM1_init:
	
	ret

LCD_init:
	ldi		temp_delay,	1											; задержка 1 с
	rcall	delay_s

	in		temp1,		PORTB
	andi	temp1,		0xF0
	sbr		temp1,		(1<<DB5)|(1<<DB4)
	out		PORTB,		temp1
	ldi		temp_delay,	5											; задержка 255 мс
	rcall	delay_us
	rcall	LCD_write												; 0011

	ldi		temp_delay,	255											; задержка 255 мс
	rcall	delay_ms
;	rcall	LCD_write												; 0011

;	ldi		temp_delay,	50											; задержка 200 мкс
;	rcall	delay_ms
;	rcall	LCD_write												; 0011
	cbi		PORTB,		DB4
	ldi		temp_delay,	200											; задержка 100 мкс
	rcall	delay_ms


	
	rcall	LCD_write												; 0010

	ldi		temp_delay,	200											; задержка 100 мкс
	rcall	delay_ms

	ldi		temp2,	0x28
	rcall	LCD_send_byte
	ldi		temp_delay,	200
	rcall	delay_us

	ldi		temp2,	0x08
	rcall	LCD_send_byte
	ldi		temp_delay,	200
	rcall	delay_us

	ldi		temp2,	0x01
	rcall	LCD_send_byte
	ldi		temp_delay,	200
	rcall	delay_us


	ret

LCD_send_byte:
	ldi		temp4,	2
LCD_send_L_tetr:
	swap	temp2
	mov		temp3,		temp2
	andi	temp3,		0x0F
	in		temp1,		PORTB
	andi	temp1,		0xF0
	or		temp1,		temp3
	out		PORTB,		temp1
	rcall	LCD_write
	dec		temp4
	brne	LCD_send_L_tetr
	ret

LCD_set_cursor:

	ret

LCD_on:
	ldi		temp_delay,	200
	rcall	delay_us
	ldi		temp2,	0x0C
	rcall	LCD_send_byte
	ret

LCD_clr:
	ldi		temp_delay,	200
	rcall	delay_us
	ldi		temp2,	0x01
	rcall	LCD_send_byte
	ret

LCD_ret_home:
	ldi		temp_delay,	200
	rcall	delay_us
	ldi		temp2,	0x02
	rcall	LCD_send_byte
	ret

LCD_write:
	ldi		temp_delay,	5
	rcall	delay_ms
	sbi		PORTA,		E
	ldi		temp_delay, 5				; 1 цикл
	rcall	delay_ms					; 3 цикла
	cbi		PORTA,		E
	ldi		temp_delay,	5
	rcall	delay_ms
	ret
;********* Подпрограммы задержки *******
delay_us:
	ldi		temp1,		4				; 1 цикл
delay_us_loop:
	subi	temp1,		1				; 1 цикл
	brne	delay_us_loop				; 2 цикла (1 при выходе)
	subi	temp_delay,	1				; 1 цикл
	nop									; 1 цикл
	brne	delay_us					; 2 цикла (1 при выходе)
	nop									; 1 цикл
	ret									; delay_us_loop при значении 4 выполняется за 16 циклов

delay_ms:
	lds		temp1, msecond_l
	add		temp1, temp_delay
delay_ms_loop:
	lds		temp2, msecond_l
	cp		temp1,	temp2
	brne	delay_ms_loop
	ret

delay_s:
	lds		temp1, second
	add		temp1, temp_delay
delay_s_loop:
	lds		temp2, second
	cp		temp1,	temp2
	brne	delay_s_loop
	ret

;********* Точка входа в программу *******
RESET:
	clr		zero

	ldi		ZH,		high (RAMEND)
	out		SPH,	ZH
	ldi		ZL,		low (RAMEND)
	out		SPL,	ZL

SRAM_clr:
	st		-Z,		zero
	cpi		ZL,		SRAM_START
	cpc		ZH,		zero
	brne	SRAM_clr1
	ldi		ZL,		30
SRAM_clr1:
	cp		ZL, zero
	cpc		ZH, zero
	brne	SRAM_clr
;********* Инициализация переферии *******	
	rcall	PORT_init
	rcall	ADC_init
	rcall	TIM0_init
	sei
;	rcall	LCD_init
;	rcall	LCD_clr
;	rcall	LCD_on
;	rcall	LCD_ret_home

	sbi		PORTB,	DB4
	ldi		temp_delay,	5
	rcall	delay_us
	cbi		PORTB,	DB4

	sbi		PORTB,	DB5
	ldi		temp_delay,	20
	rcall	delay_us
	cbi		PORTB,	DB5
	
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop
	nop

	sbi		PORTB,	DB5
	ldi		temp_delay,	100
	rcall	delay_us
	cbi		PORTB,	DB5

	sbi		PORTB,	DB6
	ldi		temp_delay,	5
	rcall	delay_ms
	cbi		PORTB,	DB6

	sbi		PORTB,	DB7
	ldi		temp_delay,	100
	rcall	delay_ms
	cbi		PORTB,	DB7
	
	sbi		PORTA,	RS
	ldi		temp_delay,	5
	rcall	delay_s
	cbi		PORTA,	RS
	
;********* Основной цикл *******
main_loop:
	ldi		temp_delay,	1
	rcall	delay_s
	ldi		temp2,	0x30
	rcall	LCD_send_byte
	rjmp main_loop




;	rcall	LCD_write												; 0010
;	in		temp1,		PORTB
;	cbr		temp1,		(1<<DB7)|(1<<DB6)|(1<<DB5)|(1<<DB4)
;	sbr		temp1,		(1<<DB4)	
;	out		PORTB,		temp1										; 1000
;	rcall	LCD_write

;	ldi		temp_delay,	200
;	rcall	delay_us

;	in		temp1,		PORTB
;	cbr		temp1,		(1<<DB7)|(1<<DB6)|(1<<DB5)|(1<<DB4)	
;	out		PORTB,		temp1
;	rcall	LCD_write												; 0000
;	in		temp1,		PORTB
;	sbr		temp1,		(1<<DB7)	
;	out		PORTB,		temp1
;	rcall	LCD_write												; 1000

;	ldi		temp_delay,	200
;	rcall	delay_us

;	in		temp1,		PORTB
;	cbr		temp1,		(1<<DB7)|(1<<DB6)|(1<<DB5)|(1<<DB4)	
;	out		PORTB,		temp1
;	rcall	LCD_write												; 0000
;	in		temp1,		PORTB
;	sbr		temp1,		(1<<DB4)	
;	out		PORTB,		temp1										; 0001
;	rcall	LCD_write

;	ldi		temp_delay,	200
;	rcall	delay_us

;	in		temp1,		PORTB
;	cbr		temp1,		(1<<DB7)|(1<<DB6)|(1<<DB5)|(1<<DB4)	
;	out		PORTB,		temp1
;	rcall	LCD_write												; 0000
;	in		temp1,		PORTB
;	sbr		temp1,		(1<<DB6)|(1<<DB5)	
;	out		PORTB,		temp1										; 0110
;	rcall	LCD_write
	
;	ldi		temp_delay,	200
;	rcall	delay_us

