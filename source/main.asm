.include "tn861Adef.inc"
.include "icakb.inc"

;***********************************************************
;Напряжение: делитель 20 кОм, 10 кОм. 12 В - 833 отсчета АЦП.
;Ток: коэффициент датчика тока 50 мВ/А. Кус = 100.
;С учетом Кус коэффициент тока 0,5 В/А
.dseg
.org				SRAM_START
hour:				.byte 1						; часы
minute:				.byte 1						; минуты
second:				.byte 1						; секунды
msecond_l:			.byte 1						; миллисекунды (младший байт)
msecond_h:			.byte 1						; миллисекунды (старший байт)
systimer_l:			.byte 1
systimer_h:			.byte 1
vtimer1_l:			.byte 1						; таймер цикла индикации
vtimer1_h:			.byte 1
vtimer2_l:			.byte 1						; таймер нажатой кнопки
vtimer2_h:			.byte 1
vtimer3_l:			.byte 1
vtimer3_h:			.byte 1
lcd_1:				.byte 1
cursor_pos:			.byte 1						; 765 4 3210	- номера бит
.equ				N_line	= 4					;	  4 		- 0 - первая строка, 1 - вторая строка		
												;		xxxx	- номер столбца от 0 до F
U_akb:				.byte 1						; напряжение АКБ
I_akb:				.byte 1						; ток АКБ
volume_akb_l:		.byte 1						; емкость АКБ (младший байт)
volume_akb_h:		.byte 1						; емкость АКБ (старший байт)
U_adc_l:			.byte 1						; ADC напряжени (младший байт)
U_adc_h:			.byte 1						; ADC напряжени (старший байт)
I_adc_l:			.byte 1						; ADC тока (младший байт)
I_adc_h:			.byte 1						; ADC тока (старший байт)
adci_counter:		.byte 1						; Колличество повторений запуска последовательных преобразований АЦП
lcd_buf5:			.byte 5						; буфер LCD для вывода чисел от 0 до 0xFFFF
lcd_buf2:			.byte 2
prev_msecond_l:		.byte 1						; промежуток между индикацией (младший байт)
prev_msecond_h:		.byte 1						; промежуток между индикацией (старший байт)
key_count:			.byte 1
lcd_loop:			.byte 1
flags:				.byte 1
.equ				key_push_flag	=	0
.equ				key_onoff_flag	=	1
.equ				start_stop_CAKB =	2
.equ				adc_complete	=	3
.equ				key_timeout		=	4
.equ				start_reset		=	5

UU_akb_l:			.byte 1
UU_akb_h:			.byte 1
II_akb_l:			.byte 1
II_akb_h:			.byte 1
CC_akb_l:			.byte 1
CC_akb_h:			.byte 1
C_akb:				.byte 4
C_akb_count:		.byte 1
beep_time:			.byte 1
hour_out:			.byte 1
minute_out:			.byte 1
second_out:			.byte 1



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
;********* Подпрограммы обработки прерываний АЦП ********
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
;********* Подпрограммы обработки прерываний таймера ********
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

;	lds		ZL,				vtimer1_l
;	lds		ZH,				vtimer1_h
;	adiw	ZH:ZL,			1
;	sts		vtimer1_l,		ZL
;	sts		vtimer1_h,		ZH

	lds		ZL,				systimer_l
	lds		ZH,				systimer_h
	adiw	ZH:ZL,			1
	sts		systimer_l,		ZL
	sts		systimer_h,		ZH

	lds		ZL,				msecond_l
	lds		ZH,				msecond_h
	adiw	ZH:ZL,			1						; инкремент миллисекунд
	cpi		ZL,				low(1000)
	ldi		temp1,			high(1000)
	cpc		ZH,				temp1
	brne	out_msec
	clr		ZL
	clr		ZH

	lds		temp1,		flags
	sbrs	temp1,		start_reset
	rjmp	OUT_msec

clr_time:
	lds		temp1,			second
	inc		temp1									; инкремент секунд
	cpi		temp1,			60
	brne	out_sec
	clr		temp1

	lds		temp2,			minute
	inc		temp2									; инкремент минут
	cpi		temp2,			60
	brne	OUT_min
	clr		temp2

	lds		temp3,			hour
	inc		temp3									; инкремент часов
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

TIM0_COMPA_out:
	pop		ZH
	pop		ZL
	pop		temp3
	pop		temp2
	pop		temp1
	out		SREG,	i_sreg
	reti

;*********************************************************************
;********* Подпрограммы инициализации переферии
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
	ldi		temp1,	(1<<LED)|(1<<RS)|(1<<E);|(1<<BUZ)
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
	ldi		temp1, (1<<CS01)|(1<<CS00)				; предделитель 64
	out		TCCR0B,	temp1

	ret

TIM1_init:
	
	ret

;*********************************************************************
;********* Подпрограммы работы с димплеем
;*********************************************************************

;***************************************************
;Подпрограмма инициализации LCD
;Режим 4-бит, 2 строки, символы 5х8
;***************************************************
LCD_init:
	ldi		temp_delay,		200						; задержка 200 мс
	rcall	delay_ms

	in		temp1,			PORT_LCD_DB
	andi	temp1,			0xF0
	sbr		temp1,			(1<<DB5)|(1<<DB4)
	out		PORT_LCD_DB,	temp1
	rcall	LCD_write								; 0011
	ldi		temp_delay,		40						; задержка не менее 37 мкс
	rcall	delay_us

	ldi		temp2,			0x28					; 0010	установка режима 4 бит,
	rcall	LCD_send_byte							; 1000	2 строки, символы 5х8
	ldi		temp_delay,		40						; задержка не менее 37 мкс
	rcall	delay_us

	ldi		temp2,			0x28					; 0010
	rcall	LCD_send_byte							; 1000
	ldi		temp_delay,		40						; задержка не менее 37 мкс
	rcall	delay_us

	ldi		temp2,			0x08					; 0000	выключить дисплей,
	rcall	LCD_send_byte							; 1000	курсор, место
	ldi		temp_delay,		40						; задержка не менее 37 мкс
	rcall	delay_us

	ldi		temp2,			0x0C					; 0000	включить дисплей
	rcall	LCD_send_byte							; 1100	курсор выключен, место не мигает
	ldi		temp_delay,		40						; задержка не менее 37 мкс
	rcall	delay_us
	ret

;***************************************************
;Подпрограмма отправки одного байта на LCD
;Байт данных загрузить в переменный регистр "temp4"
;***************************************************
LCD_send_byte:
	ldi		temp4,			2						; 2 - т.к. надо передать сначала старшую тетраду, затем младшую
LCD_send_L_tetr:
	swap	temp2									; поменять тетрады местами
	mov		temp3,			temp2
	andi	temp3,			0x0F					;очистить старшую тетраду чтобы не затереть данные в порту
	in		temp1,			PORT_LCD_DB
	andi	temp1,			0xF0
	or		temp1,			temp3
	out		PORT_LCD_DB,	temp1
	rcall	LCD_write
	dec		temp4
	brne	LCD_send_L_tetr
	ret

;***************************************************
;Подпрограмма вывода символов из flash памяти на LCD
;Адрес начала символов загрузить в регистр Z
;***************************************************
Message_send:
	lpm		temp1, 			Z+
	cpi		temp1, 			'\0'
	breq	Message_send_out
	mov		temp2, 			temp1
	rcall	LCD_send_byte
	ldi		temp_delay,		40						; задержка не менее 37 мкс
	rcall	delay_us
	rjmp	Message_send
Message_send_out:
	ret

;***************************************************
;Подпрограмма установки курсора на LCD 16х2
;загрузить позицию курсора в переменную "cursor_pos"
;в формате 0хYX. Y = 0 - первая строка
;				 Y = 1 - вторая строка
;				 Х = 0...F номер столбца
;***************************************************
LCD_set_cursor:
	cbi		PORT_LCD_RS,	RS
	lds		temp1,			cursor_pos
	ldi		temp2,			0x80					; код команды установки курсора
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
;Подпрограмма включения отображения на LCD
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
;Подпрограмма очистки LCD и возвращения курсора
;в позицию 0 - строка, 0 - стодбец
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
;Подпрограмма возвращения курсора LCD
;в позицию 0 - строка, 0 - стодбец
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
;Подпрограмма записи в LCD
;***************************************************
LCD_write:
	sbi		PORT_LCD_E,		E
	ldi		temp_delay,		1						; 1 цикл
	rcall	delay_us								; 3 цикла
	cbi		PORT_LCD_E,		E
	ldi		temp_delay,		2
	rcall	delay_us
	ret

;***************************************************
;Подпрограмма индикации данных, размещенных в буфере
; "lcd_buf5" на LCD вызывается по адресу "Indications". Число выводимых символов
;определено константой "length_buf".
;Индикация четырех чисел с разлением зяпятой в формате ХХ,ХХ
;***************************************************
Indications_point:
	ldi		temp5,			length_buf - 2
	rjmp	Indications1
Indications:
	ldi		temp5,			length_buf + 1
Indications1:
	ldi		ZL,				low(lcd_buf5)
	ldi		ZH,				high(lcd_buf5)
	ldi		temp1,			length_buf
	mov		n_pos_lcd,		temp1
Indications_loop:
	cpse	n_pos_lcd,		temp5
	rjmp	Indications_loop1
	ldi		temp2,			','
	rjmp	Indications_loop2
Indications_loop1:
	ld		temp2, 			Z+
	ldi		temp1,			'0'
	add		temp2,			temp1
Indications_loop2:
	rcall	LCD_send_byte
	ldi		temp_delay,		30						; задержка не менее 37 мкс
	rcall	delay_us
	ldi		temp1,			1
	sub		n_pos_lcd,		temp1
	brne	Indications_loop
	ret

;***************************************************
;Подпрограмма индикации часо, минут, секунд, размещенных в буфере
; "lcd_buf2" на LCD. Число выводимых символов
;расположено в переменной "length_buf"
;***************************************************
indications_time:
	ldi		ZL,				low(lcd_buf2)
	ldi		ZH,				high(lcd_buf2)
	ldi		temp1,			2
	mov		n_pos_lcd,		temp1
Indications_time_loop:
	ld		temp2, 			Z+
	ldi		temp1,			'0'
	add		temp2,			temp1
	rcall	LCD_send_byte
	ldi		temp_delay,		30						; задержка не менее 37 мкс
	rcall	delay_us
	ldi		temp1,			1
	sub		n_pos_lcd,		temp1
	brne	Indications_time_loop
	ret

indications_razd:
	ldi		temp2,			':'
	rcall	LCD_send_byte
	ldi		temp_delay,		30						; задержка не менее 37 мкс
	rcall	delay_us
	ret

;*********************************************************************
;********* Подпрограмма сканирования кнопки
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
scan_key_out2:										; определили что кнопка нажата
	lds		temp1,			flags
	sbr		temp1,			(1<<key_push_flag)		; Установили флаг нажатой кнопки
	ldi		temp2,			(1<<key_onoff_flag)		; Установка/сброс флага кнопки
	eor		temp1,			temp2
	sts		flags,			temp1
	
	sts		key_count,		zero
	sbi		PORT_BUZ,		BUZ
	ldi		temp2,			1
	sts		beep_time,		temp2
	
	cli
	lds		temp3,			systimer_h
	lds		temp2,			systimer_l				; загрузка текущего времени
	in		temp4,			TIFR
	sbrs	temp4,			OCF0A
	rjmp	NO_OCIE0A1
	subi	temp2,			0xFF
	sbci	temp3,			0xFF
NO_OCIE0A1:
	sei
	sts		vtimer2_l,		temp2					; Стартовое время отсчета таймаута
	sts		vtimer2_h,		temp3
	rjmp	scan_key_out1

scan_key_push_time:
	sbrc	temp1,			key_timeout
	rjmp	scan_key_out1
	lds		temp5,			vtimer2_h
	lds		temp4,			vtimer2_l				; загрузка стартового времени
	cli
	lds		temp3,			systimer_h
	lds		temp2,			systimer_l				; загрузка текущего времени
	in		temp4,			TIFR
	sbrs	temp4,			OCF0A
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
;********* Подпрограмма расчета емкости
;*********************************************************************
Estim_vol_akb:
	lds		temp1,			flags
	sbrs	temp1,			adc_complete
	rjmp	Estim_vol_akb_out

	cbr		temp1,			(1<<adc_complete)
	sts		flags,			temp1

	lds		temp1,			U_adc_l
	lds		temp2,			U_adc_h

	rcall	div_u16_64
	mov		XL,				temp1
	mov		XH,				temp2
	ldi		temp1,			low(3772)			;3700
	ldi		temp2,			high(3772)
	rcall	mul_16x16
;	rcall	div_u32_256							; делить на 256 все равно что откинуть младший байт
		
	sts		UU_akb_l,	temp2
	sts		UU_akb_h,	temp3

	lds		temp1,			I_adc_l
	lds		temp2,			I_adc_h

	rcall	div_u16_64
	mov		XL,				temp1
	mov		XH,				temp2
	ldi		temp1,			low(5052*4)
	ldi		temp2,			high(5052*4)
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
	cpi		temp1,			100									; суммируем по 100 значений за каждые 10 мс
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

																; делим на 3600 получаем ма/ч
																; индикация емкости
	ldi		temp1,			4									; сначала делим на 16
estim_vol_div32:
	lsr		temp7
	ror		temp6
	ror		temp5
	ror		temp4
	dec		temp1
	brne	estim_vol_div32
	
	ldi		temp1,		(3600/16)								; затем делим на 3600/16=225
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
;********* Подпрограмма счета времени
;*********************************************************************
calc_time:
	lds		temp1,			flags
	sbrs	temp1,			start_stop_CAKB
	rjmp	calc_time_clr
	lds		temp2,			hour
	lds		temp3,			minute
	lds		temp4,			second
	sts		hour_out,		temp2
	sts		minute_out,		temp3
	sts		second_out,		temp4
	
	sbrc	temp1,			start_reset
	rjmp	calc_time_out
	sbr		temp1,			(1<<start_reset)
	sts		flags,			temp1
	rjmp	calc_time_out

calc_time_clr:
	sts		hour,			zero
	sts		minute,			zero
	sts		second,			zero
	cbr		temp1,			(1<<start_reset)
	sts		flags,			temp1

calc_time_out:
	ret

;*********************************************************************
;********* Подпрограмма запуска преобразования АЦП
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
;********* Подпрограмма звуковой сигнализации
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
	rcall	delay_ms								; задержка 20 мс для "БИП"
	cbi		PORT_BUZ,		BUZ
	ret

;*********************************************************************
;********* "bin16ASCII5"- преобразование 16-битного двоичного
;********* значения в неупакованный BCD формат, тоесть в ASII формат.
;*********************************************************************
;	1. ввести число в tmpASCII_h, tmpASCII_L
;	2. результат копируется в буфер lcd_buf5

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
;********* "bin8ASCII2"- преобразование 8-битного (до 99) двоичного
;********* значения в неупакованный BCD формат, тоесть в ASII формат.
;*********************************************************************
;	1. ввести число в tmpASCII_L
;	2. результат копируется в буфер lcd_buf2
bin8ASCII2:
	ldi		temp1,			10
	ldi		temp3,	-1
bin8ASCII2_digit_loop:
	inc		temp3
	sub		tmpASCII_L,		temp1
	brsh	bin8ASCII2_digit_loop
	add		tmpASCII_l,		temp1
	sts		lcd_buf2,		temp3					; 1_0
	sts		lcd_buf2+1,		tmpASCII_L				; 0_1
	ret


;*********************************************************************
;********* Подпрограммы задержки *******
;*********************************************************************
;величину задержки загрузить в резистр "temp_delay"
;вызвать родпрограмму задерки delay_us, delay_ms, delay_s
;*********************************************************************
delay_us:
	ldi		temp1,			4						; 1 цикл
delay_us_loop:
	subi	temp1,			1						; 1 цикл
	brne	delay_us_loop							; 2 цикла (1 при выходе)
	subi	temp_delay,		1						; 1 цикл
	nop												; 1 цикл
	brne	delay_us								; 2 цикла (1 при выходе)
	nop												; 1 цикл
	ret												; delay_us_loop при значении 4 выполняется за 16 циклов

delay_ms:
	lds		temp1, 			msecond_l
	add		temp1, 			temp_delay
delay_ms_loop:
	lds		temp2, 			msecond_l
	cp		temp1,			temp2
	brne	delay_ms_loop
	ret

;delay_s:
;	lds		temp1, 			second
;	add		temp1, 			temp_delay
;	cpi		temp1,			60
;	brlo	delay_s_loop
;	subi	temp1,			60
;delay_s_loop:
;	lds		temp2, 			second
;	cp		temp1,			temp2
;	brne	delay_s_loop
;	ret

;*********************************************************************
;********* Точка входа в программу *******
;*********************************************************************
RESET:
	clr		zero

	ldi		ZH,				high (RAMEND)			; установка стека
	out		SPH,			ZH
	ldi		ZL,				low (RAMEND)
	out		SPL,			ZL

SRAM_clr:											; очистка SRAM и РОН
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
;********* Инициализация переферии *******
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
	
	ldi		ZL,				low(Message1*2)			; вывод текста	"U="
	ldi		ZH,				high(Message1*2)
	rcall	Message_send

	ldi		temp1,	0x10							; установка курсора 
	sts		cursor_pos,	temp1
	rcall	LCD_set_cursor

	ldi		ZL,				low(Message3*2)			; вывод текста	"I="
	ldi		ZH,				high(Message3*2)
	rcall	Message_send

	ldi		temp1,	0x19							; установка курсора 
	sts		cursor_pos,	temp1
	rcall	LCD_set_cursor

	ldi		ZL,				low(Message2*2)			; вывод текста "C="
	ldi		ZH,				high(Message2*2)
	rcall	Message_send

;*********************************************************************
;********* Основной цикл *******
;*********************************************************************
;main_loop:
;	lds		temp1,			vtimer1_l				; загрузка текущих миллисекунд
;	lds		temp2,			vtimer1_h
;	cpi		temp1,			vtimer1					; проверка времени виртуального таймера
;	brsh	main_loop1	;pc+2						; переход если текущее время еще не дошло до нового времени обновления
;	rjmp	main_ind_loop_out
;main_loop1:
;	sts		vtimer1_l,		zero	
;	sts		vtimer1_h,		zero
	
main_loop:
	cli
	lds		temp2,			systimer_h
	lds		temp1,			systimer_l				; загрузка текущего времени
	in		temp3,			TIFR
	sbrs	temp3,			OCF0A
	rjmp	main_loop1
	subi	temp1,			0xFF
	sbci	temp2,			0xFF
main_loop1:
	sei
	mov		temp10,			temp1
	mov		temp11,			temp2
	lds		temp4,			vtimer1_h
	lds		temp3,			vtimer1_l
	sub		temp1,			temp3
	sbc		temp2,			temp4
	cpi		temp1,			vtimer1
	cpc		temp2,			zero
	brsh	main_loop2
	rjmp	main_ind_loop_out
main_loop2:
	sts		vtimer1_l,		temp10	
	sts		vtimer1_h,		temp11

	rcall	start_adc
	rcall	scan_key
	rcall	beep_warning
	rcall	estim_vol_akb
	rcall	calc_time

	lds		temp1,			lcd_loop
	inc		temp1
	sts		lcd_loop,		temp1
	cpi		temp1,			(Ind_loop/vtimer1)
	brne	main_ind_loop_out
	sts		lcd_loop,		zero

	ldi		temp1,			0x02					; установка курсора 
	sts		cursor_pos,		temp1
	rcall	LCD_set_cursor

	lds		temp1,			UU_akb_l
	lds		temp2, 			UU_akb_h
	mov		tmpASCII_l, 	temp1
	mov		tmpASCII_h, 	temp2
	rcall	bin16ASCII5
	rcall	indications_point

	ldi		temp1,			0x08					; установка курсора 
	sts		cursor_pos,		temp1
	rcall	LCD_set_cursor

	lds		temp1,			hour_out
	mov		tmpASCII_l, 	temp1
	rcall	bin8ASCII2
	rcall	indications_time

	rcall	indications_razd

	lds		temp1,			minute_out
	mov		tmpASCII_l, 	temp1
	rcall	bin8ASCII2
	rcall	indications_time

	rcall	indications_razd

	lds		temp1,			second_out
	mov		tmpASCII_l, 	temp1
	rcall	bin8ASCII2
	rcall	indications_time

	ldi		temp1,			0x12					; установка курсора 
	sts		cursor_pos,		temp1
	rcall	LCD_set_cursor

	lds		temp1,			CC_akb_l
	lds		temp2,			CC_akb_h

	mov		tmpASCII_l, 	temp1
	mov		tmpASCII_h, 	temp2
	rcall	bin16ASCII5
	rcall	indications

	ldi		temp1,			0x1B					; установка курсора 
	sts		cursor_pos,		temp1
	rcall	LCD_set_cursor

	lds		temp1,			II_akb_l
	lds		temp2, 			II_akb_h

	mov		tmpASCII_l, 	temp1
	mov		tmpASCII_h, 	temp2
	rcall	bin16ASCII5
	rcall	indications_point

main_ind_loop_out:

	lds		temp1,		flags
;	sbrs	temp1,		start_stop_CAKB
	rjmp	off_led
	sbi		PORT_LED,		LED
	rjmp	on_led
off_led:
	cbi		PORT_LED,		LED
on_led:

	rjmp	main_loop

Message1:	.db "U=",'\0', 0x00
Message2:	.db	"I=",'\0', 0x00
Message3:	.db "C=",'\0', 0x00
Message4:	.db 0xA2,0xBC,0xBA,0x6F,0x63,0xBF,0xC4, '=', '\0', 0x00
