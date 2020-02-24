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
