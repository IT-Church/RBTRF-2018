# MPLAB IDE generated this makefile for use with GNU make.
# Project: RBTRF.mcp
# Date: Thu Dec 14 17:29:40 2017

AS = pic30-as.exe
CC = pic30-gcc.exe
LD = pic30-ld.exe
AR = pic30-ar.exe
HX = pic30-bin2hex.exe
RM = rm

RBTRF.hex : RBTRF.cof
	$(HX) "RBTRF.cof"

RBTRF.cof : main.o
	$(CC) -mcpu=30F4011 "main.o" -o"RBTRF.cof" -Wl,-Tp30F4011.gld,--defsym=__MPLAB_BUILD=1,--defsym=__MPLAB_DEBUG=1,-Map="RBTRF.map",--report-mem

main.o : ../../../../program\ files\ (x86)/microchip/mplabc30/v3.31/support/generic/h/libpic30.h ../../../../program\ files\ (x86)/microchip/mplabc30/v3.31/include/float.h ../../../../program\ files\ (x86)/microchip/mplabc30/v3.31/include/math.h ../../../../program\ files\ (x86)/microchip/mplabc30/v3.31/include/stdarg.h ../../../../program\ files\ (x86)/microchip/mplabc30/v3.31/include/stddef.h ../../../../program\ files\ (x86)/microchip/mplabc30/v3.31/include/stdio.h ../../../../program\ files\ (x86)/microchip/mplabc30/v3.31/support/dsPIC30F/h/p30f4011.h main.c
	$(CC) -mcpu=30F4011 -x c -c "main.c" -o"main.o" -D__DEBUG -g -Wall

clean : 
	$(RM) "main.o" "RBTRF.cof" "RBTRF.hex"

