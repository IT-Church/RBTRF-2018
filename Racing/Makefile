# MPLAB IDE generated this makefile for use with GNU make.
# Project: Race.mcp
# Date: Fri Feb 09 10:06:58 2018

AS = pic30-as.exe
CC = pic30-gcc.exe
LD = pic30-ld.exe
AR = pic30-ar.exe
HX = pic30-bin2hex.exe
RM = rm

Race.hex : Race.cof
	$(HX) "Race.cof"

Race.cof : main.o
	$(CC) -mcpu=30F4011 "main.o" -o"Race.cof" -Wl,-Tp30F4011.gld,--defsym=__MPLAB_BUILD=1,-Map="Race.map",--report-mem

main.o : ../../../../../program\ files\ (x86)/microchip/mplab\ c30/support/generic/h/libpic30.h ../../../../../program\ files\ (x86)/microchip/mplab\ c30/include/float.h ../../../../../program\ files\ (x86)/microchip/mplab\ c30/include/math.h ../../../../../program\ files\ (x86)/microchip/mplab\ c30/include/stdarg.h ../../../../../program\ files\ (x86)/microchip/mplab\ c30/include/stddef.h ../../../../../program\ files\ (x86)/microchip/mplab\ c30/include/stdio.h ../../../../../program\ files\ (x86)/microchip/mplab\ c30/support/dsPIC30F/h/p30f4011.h main.c
	$(CC) -mcpu=30F4011 -x c -c "main.c" -o"main.o" -g -Wall

clean : 
	$(RM) "main.o" "Race.cof" "Race.hex"

