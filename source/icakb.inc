;.def dataFlash		= r0
.def zero			= r1
.def i_sreg			= r2
.def tmpASCII_l		= r3
.def tmpASCII_h		= r4
.def n_pos_lcd		= r5
.def temp10			= r6
.def temp11			= r7

.def temp1 			= r16
.def temp2			= r17
.def temp3 			= r18
.def temp4			= r19
.def temp5			= r20
.def temp6			= r21
.def temp7			= r22
.def temp8			= r23
.def temp9			= r24
.def temp_delay		= r25


.equ BUZ			= PA3
.equ LED			= PA5
.equ RS				= PA6
.equ E				= PA7
.equ DB4			= PB0
.equ DB5			= PB1
.equ DB6			= PB2
.equ DB7			= PB3
.equ SB1			= PB6

.equ PORT_BUZ		= PORTA
.equ PORT_LED		= PORTA
.equ PORT_LCD_RS	= PORTA
.equ PORT_LCD_E		= PORTA

.equ PORT_LCD_DB	= PORTB
.equ PORT_SB1		= PINB

.equ I_CH					= 0
.equ U_CH					= 1
.equ ADC2_CH				= 2
.equ ADC3_CH				= 3
.equ ADMUX_mask				= 0b11100000
.equ ADMUX_nmask			= 0b00011111
.equ length_buf				= 5
.equ Ind_loop				= 500			; цикл индикации 500 мс
.equ vtimer1				= 10			; цикл 10 мс
.equ adc_counter			= 4
.equ key_timeout_val		= 1500			; длительное нажатие кнопки более ххх мс
