@ECHO OFF
"C:\Program Files (x86)\Atmel\AVR Tools\AvrAssembler2\avrasm2.exe" -S "D:\AT26\asm\ICAKB_1\labels.tmp" -fI -W+ie -C V2 -o "D:\AT26\asm\ICAKB_1\ICAKB_1.hex" -d "D:\AT26\asm\ICAKB_1\ICAKB_1.obj" -e "D:\AT26\asm\ICAKB_1\ICAKB_1.eep" -m "D:\AT26\asm\ICAKB_1\ICAKB_1.map" "D:\AT26\asm\ICAKB_1\source\main.asm"
