;***************************************************
;Подпрограмма умножения чисел 16х8
;temp3:temp2:temp1 = XH:XL*temp1
;***************************************************
mul_16x8:
	clr		temp3
	clr		temp2
	ldi		temp4,			8
	clc
mul_16x8_1:
	sbrs	temp1,				0
	rjmp	mul_16x8_2
	add		temp2,			XL
	adc		temp3,			XH
mul_16x8_2:
	ror		temp3
	ror		temp2
	ror		temp1
	clc
	dec		temp4
	brne	mul_16x8_1
	ret

;***************************************************
;Подпрограмма умножения чисел 16х16
;temp4:temp3:temp2:temp1 = XH:XL*temp2:temp1
;***************************************************
mul_16x16:
	clr		temp4
	clr		temp3
	ldi		temp7,			16
	clc
mul_16x16_1:
	sbrs	temp1,				0
	rjmp	mul_16x16_2
	add		temp3,			XL
	adc		temp4,			XH
mul_16x16_2:
	ror		temp4
	ror		temp3
	ror		temp2
	ror		temp1
	clc
	dec		temp7
	brne	mul_16x16_1
	ret

;***************************************************
;Подпрограмма деления числа u16 на 4
;temp2:temp1 = temp2:temp1/4
;***************************************************
div_u16_4:
	lsr		temp2
	ror		temp1
	lsr		temp2
	ror		temp1
	ret

;***************************************************
;Подпрограмма деления числа u32 на 256					бесполезная, просто нужно
;temp4:temp3:temp2:temp1 = temp4:temp3:temp2:temp1/256	откинуть младший байт
;***************************************************
div_u32_4:
	ldi		temp7,			2
	rjmp	div_u32_start
div_u32_256:
	ldi		temp7,			8
div_u32_start:
	clc
div_u32_256_1:
	lsr		temp4
	ror		temp3
	ror		temp2
	ror		temp1
	dec		temp7
	brne	div_u32_256_1
	ret

;***************************************************
;Подпрограмма деления чисел u16/u16
;temp4:temp3:temp2:temp1 = temp7:temp6:temp5:temp4/temp1
.def	drem16u0	=	r12
.def	drem16u1	=	r13
.def	drem16u2	=	r14	;		остаток_L
.def	drem16u3	=	r15	;		остаток_H
;.def	dres16uL=r16	;temp1
;.def	dres16uH=r17	;temp2
;.def	dd16u0	=r16	;temp1	делимое_0
;.def	dd16u1	=r17	;temp2	делимое_1
;.def	dd16u2	=r18	;temp3	делимое_2
;.def	dd16u3	=r19	;temp4	делимое_3

;.def	dv16u	=r20	;temp5	делитель
;.def	dv16uH	=r19	;temp4	делитель_H
;.def	dcnt16u	=r21	;temp6	счетчик
;***************************************************
div32u:
	clr		drem16u0				;очистить остаток LB
	clr		drem16u1
	clr		drem16u2
	sub		drem16u3,	drem16u3	;очистить остаток HB и флаг переноса
	ldi		temp8,33				;инициализация счетчика циклов
d32u_1:
	rol		temp4					;сдвиг влево делимого
	rol		temp5
	rol		temp6
	rol		temp7
	dec		temp8					;decrement counter
	brne	d32u_2					;if done
	ret								;return
d32u_2:
	rol		drem16u0				;сдвиг старшего бита из флага переноса
	rol		drem16u1
	rol		drem16u2
	rol		drem16u3

	sub		drem16u0,	temp1		;остаток = остаток - делитель
	sbc		drem16u1,	zero	;dv16uH		;
	sbc		drem16u2,	zero
	sbc		drem16u3,	zero
	brcc	d32u_3					;если вычитание меньше 0
	add		drem16u0,	temp1		;восстанавливаем остаток
	adc		drem16u1,	zero	;dv16uH
	adc		drem16u2,	zero	
	adc		drem16u3,	zero
	clc								;clear carry to be shifted into result
	rjmp	d32u_1					;else
d32u_3:
	sec								;set carry to be shifted into result
	rjmp	d32u_1

;***************************************************
;Подпрограмма деления чисел u32/u8
;temp4:temp3:temp2:temp1 = temp7:temp6:temp5:temp4/temp1
;***************************************************
;mul_16x16:
;	clr		temp4
;	clr		temp3
;	clr		temp1
;	ldi		temp7,			16
;	clc
;mul_16x16_1:
;	sbrs	temp1,				0
;	rjmp	mul_16x16_2
;	add		temp3,			XL
;	adc		temp4,			XH
;mul_16x16_2:
;	ror		temp4
;	ror		temp3
;	ror		temp2
;	ror		temp1
;	clc
;	dec		temp7
;	brne	mul_16x16_1
;	ret
