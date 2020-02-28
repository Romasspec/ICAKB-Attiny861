.include "tn861Adef.inc"
.include "icakb.inc"

;***********************************************************
;����������: �������� 20 ���, 10 ���. 12 � - 833 ������� ���.
;���: ����������� ������� ���� 50 ��/�. ��� = 100.
;� ������ ��� ����������� ���� 0,5 �/�
.dseg
.org				SRAM_START
hour:				.byte 1						; ����
minute:				.byte 1						; ������
second:				.byte 1						; �������
msecond_l:			.byte 1						; ������������ (������� ����)
msecond_h:			.byte 1						; ������������ (������� ����)
systimer_l:			.byte 1
systimer_h:			.byte 1
vtimer1_l:			.byte 1						; ������ ����� ���������
vtimer1_h:			.byte 1
vtimer2_l:			.byte 1						; ������ ������� ������
vtimer2_h:			.byte 1
vtimer3_l:			.byte 1
vtimer3_h:			.byte 1
lcd_1:				.byte 1
cursor_pos:			.byte 1						; 765 4 3210	- ������ ���
.equ				N_line	= 4					;	  4 		- 0 - ������ ������, 1 - ������ ������		
												;		xxxx	- ����� ������� �� 0 �� F
U_akb:				.byte 1						; ���������� ���
I_akb:				.byte 1						; ��� ���
volume_akb_l:		.byte 1						; ������� ��� (������� ����)
volume_akb_h:		.byte 1						; ������� ��� (������� ����)
U_adc_l:			.byte 1						; ADC ��������� (������� ����)
U_adc_h:			.byte 1						; ADC ��������� (������� ����)
I_adc_l:			.byte 1						; ADC ���� (������� ����)
I_adc_h:			.byte 1						; ADC ���� (������� ����)
adci_counter:		.byte 1						; ����������� ���������� ������� ���������������� �������������� ���
lcd_buf5:			.byte 5						; ����� LCD ��� ������ ����� �� 0 �� 0xFFFF
prev_msecond_l:		.byte 1						; ���������� ����� ���������� (������� ����)
prev_msecond_h:		.byte 1						; ���������� ����� ���������� (������� ����)
key_count:			.byte 1
lcd_loop:			.byte 1
flags:				.byte 1
.equ				key_push_flag	=	0
.equ				key_onoff_flag	=	1
.equ				start_stop_CAKB =	2
.equ				adc_complete	=	3
.equ				key_timeout		=	4

UU_akb_l:			.byte 1
UU_akb_h:			.byte 1
II_akb_l:			.byte 1
II_akb_h:			.byte 1
CC_akb_l:			.byte 1
CC_akb_h:			.byte 1
C_akb:				.byte 4
C_akb_count:		.byte 1
beep_time:			.byte 1



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

;*********************************************************************
;********* ������������ ��������� ���������� ��� ********
;*********************************************************************
.include "math.inc"

ADC_INT:
	in		i_sreg,			SREG
	push	temp1
	push	temp2
	push	temp3
	push	temp4

	in		temp3,			ADCL
	in		temp4,			ADCH

	in		temp1,			ADMUX
	andi	temp1,			ADMUX_nmask
	cpi		temp1,			I_CH
	brne	adc_u_ch

	lds		temp1,			I_adc_l
	lds		temp2,			I_adc_h
	add		temp1,			temp3
	adc		temp2,			temp4
	sts		I_adc_l,		temp1
	sts		I_adc_h,		temp2

	in		temp3,			ADMUX
	andi	temp3,			ADMUX_mask
	ori		temp3,			U_CH
	out		ADMUX,			temp3
	rjmp	adc_int_out1

adc_u_ch:
	cpi		temp1,			U_CH
	brne	adc_int_out

	lds		temp1,			U_adc_l
	lds		temp2,			U_adc_h
	add		temp1,			temp3
	adc		temp2,			temp4
	sts		U_adc_l,		temp1
	sts		U_adc_h,		temp2

	in		temp3,			ADMUX
	andi	temp3,			ADMUX_mask
	ori		temp3,			I_CH
	out		ADMUX,			temp3

	lds		temp1,			adci_counter
	subi	temp1,			1
	sts		adci_counter,	temp1
	brne	adc_int_out1
	lds		temp1,			flags
	sbr		temp1,			(1<<adc_complete)
	sts		flags,			temp1
	rjmp	adc_int_out

adc_int_out1:
	sbi		ADCSRA,			ADSC
adc_int_out:
	pop		temp4
	pop		temp3
	pop		temp2	
	pop		temp1
	out		SREG,			i_sreg
	reti

;*********************************************************************
;********* ������������ ��������� ���������� ������� ********
;*********************************************************************

TIM0_COMPA:
	in		i_sreg,			SREG
	push	temp1
	push	temp2
	push	temp3
	push	ZL
	push	ZH

;	in		temp1,	PORTA
;	ldi		temp2, (1<<LED)
;	eor		temp1,	temp2
;	out		PORTA,	temp1

	lds		ZL,				vtimer1_l
	lds		ZH,				vtimer1_h
	adiw	ZH:ZL,			1
	sts		vtimer1_l,		ZL
	sts		vtimer1_h,		ZH

	lds		ZL,				systimer_l
	lds		ZH,				systimer_h
	adiw	ZH:ZL,			1
	sts		systimer_l,		ZL
	sts		systimer_h,		ZH
	
	lds		ZL,				msecond_l
	lds		ZH,				msecond_h
	adiw	ZH:ZL,			1						; ��������� �����������
	cpi		ZL,				low(1000)
	ldi		temp1,			high(1000)
	cpc		ZH,				temp1
	brne	out_msec
	clr		ZL
	clr		ZH

	lds		temp1,			second
	inc		temp1									; ��������� ������
	cpi		temp1,			60
	brne	out_sec
	clr		temp1

	lds		temp2,			minute
	inc		temp2									; ��������� �����
	cpi		temp2,			60
	brne	OUT_min
	clr		temp2

	lds		temp3,			hour
	inc		temp3									; ��������� �����
	cpi		temp3,			24
	brne	OUT_hour
	clr		temp3

OUT_hour:
	sts		hour,			temp3
OUT_min:
	sts		minute,			temp2
OUT_sec:
	sts		second,			temp1
OUT_msec:
	sts		msecond_l,		ZL
	sts		msecond_h,		ZH
	
	pop		ZH
	pop		ZL
	pop		temp3
	pop		temp2
	pop		temp1
	out		SREG,	i_sreg
	reti

;*********************************************************************
;********* ������������ ������������� ���������
;*********************************************************************

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
	
	ldi		temp1, (1<<CS00)
	ldi		temp1, (1<<CS01)|(1<<CS00)				; ������������ 64
	out		TCCR0B,	temp1

	ret

TIM1_init:
	
	ret

;*********************************************************************
;********* ������������ ������ � ��������
;*********************************************************************

;***************************************************
;������������ ������������� LCD
;����� 4-���, 2 ������, ������� 5�8
;***************************************************
LCD_init:
	ldi		temp_delay,		200						; �������� 200 ��
	rcall	delay_ms

	in		temp1,			PORT_LCD_DB
	andi	temp1,			0xF0
	sbr		temp1,			(1<<DB5)|(1<<DB4)
	out		PORT_LCD_DB,	temp1
	rcall	LCD_write								; 0011
	ldi		temp_delay,		40						; �������� �� ����� 37 ���
	rcall	delay_us

	ldi		temp2,			0x28					; 0010	��������� ������ 4 ���,
	rcall	LCD_send_byte							; 1000	2 ������, ������� 5�8
	ldi		temp_delay,		40						; �������� �� ����� 37 ���
	rcall	delay_us

	ldi		temp2,			0x28					; 0010
	rcall	LCD_send_byte							; 1000
	ldi		temp_delay,		40						; �������� �� ����� 37 ���
	rcall	delay_us

	ldi		temp2,			0x08					; 0000	��������� �������,
	rcall	LCD_send_byte							; 1000	������, �����
	ldi		temp_delay,		40						; �������� �� ����� 37 ���
	rcall	delay_us

	ldi		temp2,			0x0C					; 0000	�������� �������
	rcall	LCD_send_byte							; 1100	������ ��������, ����� �� ������
	ldi		temp_delay,		40						; �������� �� ����� 37 ���
	rcall	delay_us
	ret

;***************************************************
;������������ �������� ������ ����� �� LCD
;���� ������ ��������� � ���������� ������� "temp4"
;***************************************************
LCD_send_byte:
	ldi		temp4,			2						; 2 - �.�. ���� �������� ������� ������� �������, ����� �������
LCD_send_L_tetr:
	swap	temp2									; �������� ������� �������
	mov		temp3,			temp2
	andi	temp3,			0x0F					;�������� ������� ������� ����� �� �������� ������ � �����
	in		temp1,			PORT_LCD_DB
	andi	temp1,			0xF0
	or		temp1,			temp3
	out		PORT_LCD_DB,	temp1
	rcall	LCD_write
	dec		temp4
	brne	LCD_send_L_tetr
	ret

;***************************************************
;������������ ������ �������� �� flash ������ �� LCD
;����� ������ �������� ��������� � ������� Z
;***************************************************
Message_send:
	lpm		temp1, 			Z+
	cpi		temp1, 			'\0'
	breq	Message_send_out
	mov		temp2, 			temp1
	rcall	LCD_send_byte
	ldi		temp_delay,		40						; �������� �� ����� 37 ���
	rcall	delay_us
	rjmp	Message_send
Message_send_out:
	ret

;***************************************************
;������������ ��������� ������� �� LCD 16�2
;��������� ������� ������� � ���������� "cursor_pos"
;� ������� 0�YX. Y = 0 - ������ ������
;				 Y = 1 - ������ ������
;				 � = 0...F ����� �������
;***************************************************
LCD_set_cursor:
	cbi		PORT_LCD_RS,	RS
	lds		temp1,			cursor_pos
	ldi		temp2,			0x80					; ��� ������� ��������� �������
	sbrc	temp1,			N_line
	sbr		temp2,			(1<<6)
	andi	temp1,			0x0F
	or		temp2,			temp1
	rcall	LCD_send_byte
	ldi		temp_delay,		40
	rcall	delay_us
	sbi		PORT_LCD_RS,	RS
	ret

;***************************************************
;������������ ��������� ����������� �� LCD
;***************************************************
LCD_on:
	cbi		PORT_LCD_RS,	RS
	ldi		temp2,			0x0C
	rcall	LCD_send_byte
	sbi		PORT_LCD_RS,	RS
	ldi		temp_delay,		40
	rcall	delay_us
	ret

;***************************************************
;������������ ������� LCD � ����������� �������
;� ������� 0 - ������, 0 - �������
;***************************************************
LCD_clr:
	cbi		PORT_LCD_RS,	RS
	ldi		temp2,			0x01
	rcall	LCD_send_byte
	ldi		temp_delay,		2
	rcall	delay_ms
	sbi		PORT_LCD_RS,	RS
	ret

;***************************************************
;������������ ����������� ������� LCD
;� ������� 0 - ������, 0 - �������
;***************************************************
LCD_ret_home:
	cbi		PORT_LCD_RS,	RS
	ldi		temp2,			0x02
	rcall	LCD_send_byte
	ldi		temp_delay,		2
	rcall	delay_ms
	sbi		PORT_LCD_RS,	RS
	ret

;***************************************************
;������������ ������ � LCD
;***************************************************
LCD_write:
	sbi		PORT_LCD_E,		E
	ldi		temp_delay,		1						; 1 ����
	rcall	delay_us								; 3 �����
	cbi		PORT_LCD_E,		E
	ldi		temp_delay,		2
	rcall	delay_us
	ret

;***************************************************
;������������ ��������� ������, ����������� � ������
; "lcd_buf5" �� LCD. ����� ��������� ��������
;����������� � ���������� "length_buf"
;***************************************************
Indications:
	ldi		ZL,				low(lcd_buf5)
	ldi		ZH,				high(lcd_buf5)
	ldi		temp1,			length_buf
	mov		n_pos_lcd,		temp1
Indications_loop:
	ld		temp2, 			Z+
	ldi		temp1,			'0'
	add		temp2,			temp1
	rcall	LCD_send_byte
	ldi		temp_delay,		30						; �������� �� ����� 37 ���
	rcall	delay_us
	ldi		temp1,			1
	sub		n_pos_lcd,		temp1
	brne	Indications_loop
	ret
;*********************************************************************
;********* ������������ ������������ ������
;*********************************************************************
Scan_key:
;	cbi		PORT_BUZ,		BUZ
	sbic	PORT_SB1,		SB1
	rjmp	scan_key_out
	lds		temp1,			flags
	sbrc	temp1,			key_push_flag
	rjmp	scan_key_push_time
	
	lds		temp1,			key_count
	subi	temp1,			0xFF
	sts		key_count,		temp1
	cpi		temp1,			10
	breq	scan_key_out2
	rjmp	scan_key_out1
scan_key_out2:										; ���������� ��� ������ ������
	lds		temp1,			flags
	sbr		temp1,			(1<<key_push_flag)		; ���������� ���� ������� ������
	ldi		temp2,			(1<<key_onoff_flag)		; ���������/����� ����� ������
	eor		temp1,			temp2
	sts		flags,			temp1
	
	sts		key_count,		zero
	sbi		PORT_BUZ,		BUZ
	ldi		temp2,			1
	sts		beep_time,		temp2
	
	cli
	lds		temp3,			systimer_h
	lds		temp2,			systimer_l				; �������� �������� �������
	in		temp4,			TIMSK
	sbrs	temp4,			OCIE0A
	rjmp	NO_OCIE0A1
	subi	temp2,			0xFF
	sbci	temp3,			0xFF
NO_OCIE0A1:
	sei
	sts		vtimer2_l,		temp2					; ��������� ����� ������� ��������
	sts		vtimer2_h,		temp3
	rjmp	scan_key_out1

scan_key_push_time:
	sbrc	temp1,			key_timeout
	rjmp	scan_key_out1
	lds		temp5,			vtimer2_h
	lds		temp4,			vtimer2_l				; �������� ���������� �������
	cli
	lds		temp3,			systimer_h
	lds		temp2,			systimer_l				; �������� �������� �������
	in		temp4,			TIMSK
	sbrs	temp4,			OCIE0A
	rjmp	NO_OCIE0A2
	subi	temp2,			0xFF
	sbci	temp3,			0xFF
NO_OCIE0A2:
	sei
	
	sub		temp2,			temp4
	sbc		temp3,			temp5
	cpi		temp2,			low (key_timeout_val)
	ldi		temp4,			high(key_timeout_val)
	cpc		temp3,			temp4
	brlo	scan_key_out1
	
	lds		temp1,			flags
	sbr		temp1,			(1<<key_timeout)
	ldi		temp2,			(1<<start_stop_CAKB)
	eor		temp1,			temp2
	sts		flags,			temp1
	sbi		PORT_BUZ,		BUZ
	ldi		temp2,			10
	sts		beep_time,		temp2
	rjmp	scan_key_out1

scan_key_out:
	sts		key_count,		zero
	lds		temp1,  		flags
	cbr		temp1,			(1<<key_push_flag)|(1<<key_timeout)
	sts		flags,			temp1
scan_key_out1:
	ret

;*********************************************************************
;********* ������������ ������� �������
;*********************************************************************
Estim_vol_akb:
	lds		temp1,			flags
	sbrs	temp1,			adc_complete
	rjmp	Estim_vol_akb_out

	cbr		temp1,			(1<<adc_complete)
	sts		flags,			temp1

	lds		temp1,			U_adc_l
	lds		temp2,			U_adc_h

	rcall	div_u16_4
	mov		XL,				temp1
	mov		XH,				temp2
	ldi		temp1,			low(3700)			;3700
	ldi		temp2,			high(3700)
	rcall	mul_16x16
;	rcall	div_u32_256							; ������ �� 256 ��� ����� ��� �������� ������� ����
		
	sts		UU_akb_l,	temp2
	sts		UU_akb_h,	temp3

	lds		temp1,			I_adc_l
	lds		temp2,			I_adc_h

	rcall	div_u16_4
	mov		XL,				temp1
	mov		XH,				temp2
	ldi		temp1,			low(2327*4)
	ldi		temp2,			high(2327*4)
	rcall	mul_16x16
	rcall	div_u32_4

;	subi	temp2,			low(171)
;	sbci	temp3,			high(171)

;	brsh	estim_vol_akb1
;	mov		temp2,			zero
;	mov		temp3,			zero
estim_vol_akb1:
	sts		II_akb_l,		temp2
	sts		II_akb_h,		temp3

	ldi		ZL,				low(C_akb)
	ldi		ZH,				high(C_akb)

	lds		temp1,			flags
	sbrs	temp1,			start_stop_CAKB
	rjmp	Estim_vol_akb3

	lds		temp1,			C_akb_count
	inc		temp1
	sts		C_akb_count,	temp1
	cpi		temp1,			100									; ��������� �� 100 �������� �� ������ 10 ��
	brlo	estim_vol_akb2
	sts		C_akb_count,	zero
	
	ld		temp4,			Z+
	ld		temp5,			Z+
	ld		temp6,			Z+
	ld		temp7,			Z+

	add		temp4,			temp2
	adc		temp5,			temp3
	adc		temp6,			zero
	adc		temp7,			zero
	st		-Z,				temp7
	st		-Z,				temp6
	st		-Z,				temp5
	st		-Z,				temp4

																; ����� �� 3600 �������� ��/�
																; ��������� �������
	ldi		temp1,			4									; ������� ����� �� 16
estim_vol_div32:
	lsr		temp7
	ror		temp6
	ror		temp5
	ror		temp4
	dec		temp1
	brne	estim_vol_div32
	
	ldi		temp1,		(3600/16)
	rcall	div32u

	sts		CC_akb_l,	temp4
	sts		CC_akb_h,	temp5

	rjmp	estim_vol_akb2
estim_vol_akb3:
	st		Z+,				zero
	st		Z+,				zero
	st		Z+,				zero
	st		Z,				zero
	
estim_vol_akb2:
	sts		u_adc_l,		zero
	sts		u_adc_h,		zero
	sts		I_adc_l,		zero
	sts		I_adc_h,		zero


Estim_vol_akb_out:
	ret

;*********************************************************************
;********* ������������ ������� �������������� ���
;*********************************************************************
start_adc:
	in		temp1,			ADMUX
	andi	temp1,			ADMUX_mask
	out		ADMUX,			temp1
	lds		temp1,			adci_counter
	ldi		temp1,			adc_counter
	sts		adci_counter,	temp1
	
	sbi		ADCSRA,			ADSC
	ret

;*********************************************************************
;********* ������������ �������� ������������
;*********************************************************************
beep_warning:
	lds		temp1,			beep_time
	tst		temp1
	breq	beep_warning_out1
	dec		temp1
	sts		beep_time,		temp1
	rjmp	beep_warning_out
beep_warning_out1:
	cbi		PORT_BUZ,		BUZ
beep_warning_out:
	ret

beep_start:
	sbi		PORT_BUZ,		BUZ
	ldi		temp_delay,		20
	rcall	delay_ms								; �������� 20 �� ��� "���"
	cbi		PORT_BUZ,		BUZ
	ret

;*********************************************************************
;********* "bin16ASCII5"- �������������� 16-������� ���������
;********* �������� � ������������� BCD ������, ������ � ASII ������.
;*********************************************************************
;	1. ������ ����� � tmpASCII_h, tmpASCII_L
;	2. ��������� ���������� � �����

bin16ASCII5:
	ldi		temp1,			low(10000)
	ldi		temp2,			high(10000)
	rcall	bin16ASCII5_digit
	sts		lcd_buf5,		temp3					; 1_0000
	
	ldi		temp1,			low(1000)
	ldi		temp2,			high(1000)
	rcall	bin16ASCII5_digit
	sts		lcd_buf5+1,		temp3					; 0_1_000

	ldi		temp1,			low(100)
	ldi		temp2,			high(100)
	rcall	bin16ASCII5_digit
	sts		lcd_buf5+2,		temp3					; 00_1_00

	ldi		temp1,			low(10)
	ldi		temp2,			high(10)
	rcall	bin16ASCII5_digit
	sts		lcd_buf5+3,		temp3					; 000_1_0
	sts		lcd_buf5+4,		tmpASCII_L				; 0000_1
	ret

bin16ASCII5_digit:
	ldi		temp3,	-1
bin16ASCII5_digit_loop:
	inc		temp3
	sub		tmpASCII_L,		temp1
	sbc		tmpASCII_h,		temp2
	brsh	bin16ASCII5_digit_loop
	add		tmpASCII_l,		temp1
	adc		tmpASCII_h,		temp2
	ret
;*********************************************************************
;********* ������������ �������� *******
;*********************************************************************
;�������� �������� ��������� � ������� "temp_delay"
;������� ������������ ������� delay_us, delay_ms, delay_s
;*********************************************************************
delay_us:
	ldi		temp1,			4						; 1 ����
delay_us_loop:
	subi	temp1,			1						; 1 ����
	brne	delay_us_loop							; 2 ����� (1 ��� ������)
	subi	temp_delay,		1						; 1 ����
	nop												; 1 ����
	brne	delay_us								; 2 ����� (1 ��� ������)
	nop												; 1 ����
	ret												; delay_us_loop ��� �������� 4 ����������� �� 16 ������

delay_ms:
	lds		temp1, 			msecond_l
	add		temp1, 			temp_delay
delay_ms_loop:
	lds		temp2, 			msecond_l
	cp		temp1,			temp2
	brne	delay_ms_loop
	ret

delay_s:
	lds		temp1, 			second
	add		temp1, 			temp_delay
	cpi		temp1,			60
	brlo	delay_s_loop
	subi	temp1,			60
delay_s_loop:
	lds		temp2, 			second
	cp		temp1,			temp2
	brne	delay_s_loop
	ret

;*********************************************************************
;********* ����� ����� � ��������� *******
;*********************************************************************
RESET:
	clr		zero

	ldi		ZH,				high (RAMEND)			; ��������� �����
	out		SPH,			ZH
	ldi		ZL,				low (RAMEND)
	out		SPL,			ZL

SRAM_clr:											; ������� SRAM � ���
	st		-Z,				zero
	cpi		ZL,				SRAM_START
	cpc		ZH,				zero
	brne	SRAM_clr1
	ldi		ZL,				30
SRAM_clr1:
	cp		ZL, zero
	cpc		ZH, zero
	brne	SRAM_clr

;*********************************************************************
;********* ������������� ��������� *******
;*********************************************************************
	rcall	PORT_init
	rcall	ADC_init
	rcall	TIM0_init
	
;	ldi		temp1,		byte1 (250000001)
;	ldi		temp2,		byte2 (250000001)
;	ldi		temp3,		byte3 (250000001)
;	ldi		temp4,		byte4 (250000001)

;	ldi		temp5,		220
;	ldi		temp4,		high(500)
;	rcall	div32u

	sei
	rcall	LCD_init
	rcall	LCD_clr
	sbi		PORT_LED,		LED
	rcall	beep_start
	
	ldi		ZL,				low(Message1*2)			; ����� ������
	ldi		ZH,				high(Message1*2)
	rcall	Message_send

	ldi		temp1,	0x10							; ��������� ������� 
	sts		cursor_pos,	temp1
	rcall	LCD_set_cursor

	ldi		ZL,				low(Message2*2)			; ����� ������
	ldi		ZH,				high(Message2*2)
	rcall	Message_send

;*********************************************************************
;********* �������� ���� *******
;*********************************************************************
main_loop:
	lds		temp1,			vtimer1_l				; �������� ������� �����������
	lds		temp2,			vtimer1_h
	cpi		temp1,			vtimer1					; �������� ������� ������������ �������
	brlo	main_ind_loop_out						; ������� ���� ������� ����� ��� �� ����� �� ������ ������� ����������

	sts		vtimer1_l,		zero	
	sts		vtimer1_h,		zero
	
	rcall	start_adc
	
	rcall	scan_key
	rcall	beep_warning
	rcall	estim_vol_akb

	lds		temp1,			lcd_loop
	inc		temp1
	sts		lcd_loop,		temp1
	cpi		temp1,			(Ind_loop/vtimer1)
	brne	main_ind_loop_out
	sts		lcd_loop,		zero

	ldi		temp1,			0x04					; ��������� ������� 
	sts		cursor_pos,		temp1
	rcall	LCD_set_cursor

	lds		temp1,			UU_akb_l
	lds		temp2, 			UU_akb_h
	mov		tmpASCII_l, 	temp1
	mov		tmpASCII_h, 	temp2
	rcall	bin16ASCII5
	rcall	indications

	ldi		temp1,			0x14					; ��������� ������� 
	sts		cursor_pos,		temp1
	rcall	LCD_set_cursor

;	lds		temp1,			II_akb_l
;	lds		temp2, 			II_akb_h
	lds		temp1,			CC_akb_l
	lds		temp2,			CC_akb_h

	mov		tmpASCII_l, 	temp1
	mov		tmpASCII_h, 	temp2
	rcall	bin16ASCII5
	rcall	indications

	ldi		temp1,			0x1B					; ��������� ������� 
	sts		cursor_pos,		temp1
	rcall	LCD_set_cursor

	lds		temp1,			II_akb_l
	lds		temp2, 			II_akb_h

	mov		tmpASCII_l, 	temp1
	mov		tmpASCII_h, 	temp2
	rcall	bin16ASCII5
	rcall	indications

main_ind_loop_out:
;	rcall	estim_vol_akb
;	rcall	indications

;	rcall	beep_warning
	lds		temp1,		flags
	sbrs	temp1,		start_stop_CAKB
	rjmp	off_led
	sbi		PORT_LED,		LED
	rjmp	on_led
off_led:
	cbi		PORT_LED,		LED
on_led:

	rjmp	main_loop

Message1:	.db "U = ",'\0',0
Message2:	.db	"I = ",'\0',0
Message3:	.db "C = ",'\0',0
Message4:	.db 0xA2,0xBC,0xBA,0x6F,0x63,0xBF,0xC4, '=', '\0', 0x00
