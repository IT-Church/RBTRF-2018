#include<p30f4011.h>
#include<stdio.h>
#include<math.h>
#include<float.h>
#include"libpic30.h"
#define SCL_line RC3
#define SDA_line RC4
#define SCL_tris TRISC3
#define SDA_tris TRISC4
#define N 8
#define M 5
#define ACK  1
#define NACK 0
_FOSC(CSW_FSCM_OFF & XT_PLL8);
_FWDT(WDT_OFF);

unsigned char flag = 0x00, i = 0, init = 1, f = 1, traffic_data, y = 1;

unsigned int  ch = 0, w = 0, pwm = 0x0200, test = 0xd356, temp, dir=1, mode=0, black=850, stoptimer = 6000, flag1, add = 0,curservo = 2650, prevservo = 2650, firstirda = 0, num1, num2;

float        sl3 = 0,   /* AN0, stripe sensor -> left 3 */
             sl2 = 0,   /* AN1, stripe sensor -> left 2 */
             sl1 = 0,   /* AN2, stripe sensor -> left 1 */
             sc = 0,    /* AN3, stripe sensor -> centre black stip on/off */
             sr1 = 0,   /* AN4, stripe sensor -> right 1 */
             sr2 = 0,   /* AN5, stripe sensor -> right 2 */
             sr3 = 0,   /* AN6, stripe sensor -> right 3 */
             ss = 0,    /* AN7, stripe sensor -> stop */ 
/* */             
             fl,    /* AN8, left front distance sensor */
             fr,    /* AN9, right front distance sensor */
             rl,    /* AN10, left rear distance sensor */
             rr,    /* AN11, right rear distance sensor */
			 turn,
			turns, srval, masssum, momsum;
          

	        
             
unsigned char h,l;

unsigned int    received_data [0x0a] = {0};
unsigned char   transmitted_data [0x0c] = {0};
unsigned char   a2d_channels [0x0c] = {0x86, 0x8e, 0x96, 0x9e, 0xa6, 0xae, 0xb6, 0xbe, 0xc6, 0xce, 0xd6, 0xde};
unsigned int   	a2d_data [0x0c] = {0};


void initialization (void);
void a2d_sensors (void);
void SPI1_A2D_init (void);
void uart1A_IrDA_init (void);
void uart2_rs232_init (void);
void putch (char byte);

/*======================================================*/

float mapper(float val, float smin,float smax, float dmin, float dmax){
	return ((val - smin) / (smax - smin)) * (dmax- dmin) + dmin;
}

int mins[] = {
	56, 83, 79, 83, 71, 93, 67, 63 	
};
int maxs[] = {
	1781, 1800, 1273, 1049, 1317, 1716, 1934, 1693
}; 
float vals[N][M];
float datch[N];
float dists[] = {40, 30, 17, 0, -17, -30, -40};

float max(float a, float b){
	if(a>b)return a;
	return b;
}
float min(float a, float b){
	if(a<b) return a;
	return b;
}

void debug_output_convertation(){
			printf( "SL3 %d - %f \n", a2d_data[0x00],sl3);
			printf( "SL2 %d - %f \n", a2d_data[0x01],sl2);
			printf( "SL1 %d - %f \n", a2d_data[0x02],sl1);
			printf( "SC %d - %f \n", a2d_data[0x03],sc);
			printf( "SR1 %d - %f \n",a2d_data[0x04],sr1);
			printf( "SR2 %d - %f \n", a2d_data[0x05],sr2);
			printf( "SR3 %d - %f \n", a2d_data[0x06],sr3);
			printf( "SS %d - %f \n", a2d_data[0x07],ss);
}

void debug_print_datch(){
			printf( " %.03f \t", sl3);
			printf( " %.03f \t", sl2);
			printf( " %.03f \t", sl1);
			printf( " %.03f \t", sc);
			printf( " %.03f \t", sr1);
			printf( " %.03f \t", sr2);
			printf( " %.03f \t", sr3);
			printf( " %.03f \n", ss);
}

void recalc(){
	sl3 = datch[0];
	sl2 = datch[1];
	sl1 = datch[2];
	sc = datch[3];
	sr1 = datch[4];
	sr2 = datch[5];
	sr3 = datch[6];
	ss = datch[7];
}


void initialization (void)
	{
    	
/* Interrupt control register */

        SRbits.IPL      = 1;    /* CPU Interrupt Priority Level Status bits, 100 = CPU interrupt priority level is 4 (12) */
        CORCONbits.IPL3 = 1;    /* CPU interrupt priority level is 7 or less */

        INTCON1     = 0x0000;
        INTCON2     = 0x0000;

/* init TMR1 */

		TMR1		= 0x0000;	/* Timer1 register for the Timer0 module  */
		PR1 		= 0x30;		/* Period register  for the Timer1 module  */
    	T1CON		= 0x8030;	/* 16-bit 1:1 prescale value */	
    	
        IPC0bits.T1IP   = 2;    		/* Priority */
        IEC0bits.T1IE 	= 1;		    /* Enables the TMR1 overflow interrupt */
        IFS0bits.T1IF 	= 0;		    /* TMR1 Overflow Interrupt Flag bit */

/* init TMR2 */

        T2CON       = 0x8000;
        TMR2        = 0x0000;
        PR2         = 0x3ff;

        IPC1bits.T2IP   = 2;    		/* Priority */
        IEC0bits.T2IE 	= 1;		    /* Enables the TMR2 overflow interrupt */
        IFS0bits.T2IF 	= 0;		    /* TMR2 Overflow Interrupt Flag bit */

/* init TMR3 */

        T3CON       = 0x8010;
        TMR3        = 0x0000;
        PR3         = 0x77ff;
        
        IPC1bits.T3IP   = 3;    		/* Priority */
        IEC0bits.T3IE 	= 1;		    /* Enables the TMR2 overflow interrupt */
        IFS0bits.T3IF 	= 0;		    /* TMR2 Overflow Interrupt Flag bit */

/* init servo motor control PWM output */

        PTCON               = 0x8008;
        PTMR                = 0x0000;
        PTPER               = 0x47fe; 
        SEVTCMP             = 0x0000;
        PWMCON1             = 0x0001;
        PWMCON2             = 0x0000;
        OVDCON              = 0x0101;
        
/* compare output PWM  */

        OC1CON      = 0x0000;
        OC3CON      = 0x0006;
        IPC4bits.OC3IP = 2;
        OC3R        = 0x0000;
        IEC1bits.OC3IE	= 1;
        IFS1bits.OC3IF	= 0;
 
                        
/* I/O init */
        
        ADPCFG  = 0xffff;    /* 1 = Analog input pin in Digital mode */
        I2CCON  = 0x0000;
        
        TRISE = 0x01fc;     /* servo->RE0  pin_38 (output servo_motor) */
        PDC1 = 0x0add;      /* servo motor PWM */            

        TRISDbits.TRISD0 = 0; /* dir */
        TRISDbits.TRISD2 = 0; /* pwm */
        
        
        TRISCbits.TRISC13 = 0;    /* TX1 */
        TRISCbits.TRISC14 = 1;    /* RX1 */
        TRISFbits.TRISF4  = 1;    /* RX2 */
        TRISFbits.TRISF5  = 0;    /* TX2 */
        
        TRISFbits.TRISF6  = 0;    /*  sck */
        TRISFbits.TRISF3  = 0;    /*  sdo */
        TRISFbits.TRISF2  = 1;    /*  sdi */
        TRISFbits.TRISF0  = 0;    /*  cs  */
        TRISFbits.TRISF1  = 1;    /*  oec */
       
       
        TRISBbits.TRISB2  = 1;    /*  run => 1, stop =>0  */
        
        
        
        TRISEbits.TRISE1 = 0;     /* test point */
       
        //PORTEbits.RE1 = 0;
  
        dir=1;
        OC3RS = 0x0001;      /* DC motor PWM */        
         
	}


/*=================================================================================PRINT FUNCTIONS=======================================================================================================================================*/

void _ISR  __attribute__((auto_psv))  _T1Interrupt( void)  
	{


		if( y == 1)
    	{
//	    	printf("Robotraffic 2012, A2D test \n");
	    	y = 0;

    	}  
    	 	
		if( y == 0)  
		{ 
			uart1A_IrDA_init(); 
			SPI1_A2D_init();
			y = 2;
		}
		if( y== 2)
		{
			a2d_sensors();
		
		}

		if ( flag1 == 0x01ff)
		{
//			printf( " irDa = {%d}  \n",(int)received_data [0]);
//			debug_print_datch();
//			printf( " FL = %d \t", a2d_data[0x08]);  //fl - center
//			printf( " FR = %d \t", a2d_data[0x09]);  //fr - left
//		printf( " RL = %d \t", a2d_data[0x0a]);      // rl - right
//		printf( " RR = %d \t", a2d_data[0x0b]);

//			printf( " U1RXREG = %d \t", U1RXREG);
		//	printf( " rmzr = %d \t", ramzor);
		//	printf( " mode = %d \t", (int)mode);
		//	printf( " smode = %d \n", (int)smode);
            //printf( " turns = %d\t", (int)turns);
//			printf("mode = %d \t",(int) mode);
//			printf("ztime = %d \t",(int) ztime);
			flag1 = 0;

		}
		
			
			
/*
Traffic Light ID;
RED             = 0x00,
RED + Yellow    = 0x01,
Green           = 0x02,
Green + Flash   = 0x03,
Yellow          = 0x04.

Traffic signal ID;

Pedestrian crossing = 0x05,
Stop                 = 0x06,
Go Straight         = 0x07, *
Go Left             = 0x08, *
Go Right            = 0x09. *

*/
uart1A_IrDA_init();

                /*switch ((int)received_data [0])
                {
                             
                        case 0x00:  //if(mode != 1) mode = 2; break; 
						case 0x01:  //if(mode != 1) mode = 2; break;  
                        case 0x02: // mode = 0; break;
                        case 0x03: // mode = 0; break;
                        case 0x04: // if(mode != 1) mode = 2; break;
						case 0x06: // mode = 3; break;
                        default:  //if(!(rl > 1100 || rr > 1100 || fl > 1100 || fr > 1100))mode = 0; break;
                }*/
		//	if(!firstirda && mode != 1) mode = 2; 
			if( flag == 0x07){	flag = 0x0;}		
		

/*===================================CODE========================================*/
	masssum = 0;
	momsum = 0; 
	for(num1 = 0; num1 < N-1; num1++){
		masssum += datch[num1];
		momsum += dists[num1] * datch[num1];
	}
	turn = 0;
	turns = 0;
	add=0;
	if(sl3 > black) {turn += 9; turns++;}
	if(sl2 > black) {turn += 6; turns++;}
	if(sl1 > black) {turn += 3; turns++;}
	if(sr1 > black) {turn -= 3; turns++;}
	if(sr2 > black) {turn -= 6; turns++;}
	if(sr3 > black) {turn -= 9; turns++;}
	if(sc > black) {turns++;}
	if(turns == 1 && sl3 > black) add = 500;
	if(turns == 1 && sr3 > black) add = -500;
	//printf("%0.2f %0.2f %0.2f\n", momsum,masssum, momsum / masssum); 
/* ================================= Send data to PC Terminal ===============================*/ 

/* ================================= Car driving controll =================================*/ 





/*================================ENDCOGE========================================*/
        if ( pwm > 0x3ff) pwm = 0x3ff; OC3RS = pwm;
		TRISDbits.TRISD0=dir;
		flag1++;
		
		
        IFS0bits.T1IF 	= 0;
    }
    
/*==============================================================================PID FUNCTION==========================================================================================================================================*/

void _ISR  __attribute__((auto_psv))  _T2Interrupt( void)  
	{
	

	IFS0bits.T2IF 	= 0;
	}    

/*================================================================================TRAFIC FUNCTIONS========================================================================================================================================*/

void _ISR  __attribute__((auto_psv))  _T3Interrupt( void)  
	{
			IFS0bits.T3IF 	= 0;
	}    
/*======================================================*/
 void _ISR  __attribute__((auto_psv))  _U1RXInterrupt( void)  
	{
		
    	if(flag == 0x00)
	    {
    	    for(i=0;i<0x01;i++)
			{
    			while(!IFS0bits.U1RXIF){	
				/* set when register is not empty */
				continue;
				}
				firstirda = 1;
				received_data [i] = U1RXREG;
			//	f4++;
			}
				
		    flag = 0x07;
	    }		
	    
	    IFS0bits.U1RXIF	= 0;	
	}
/*======================================================*/      
/*======================================================*/
 
void uart2_rs232_init (void)
	{
    	/* Fcy 7.3728 MHZ  */
    	
		U2MODE		= 0x8000;		/* Transmit status and control register */
		U2STA		= 0x84c0;		/* Receve status and control register */
		U2BRG		= 0x0007;     	/* Baund rate control register  115200 bps */
	
		IPC6bits.U2RXIP      = 2;   /* Receiver Interrupt Priority bits */	
		IEC1bits.U2RXIE 	 = 1;	/* Enable RS232 interrupts */
		IFS1bits.U2RXIF 	 = 0;	
	}
	
/*======================================================*/
 
void uart1A_IrDA_init (void)
	{
    	/* Fcy 7.3728 MHZ  */
   	
		U1MODE		= 0x8400;		/* U1ATX Transmit status and control register */
		U1STA		= 0x8400;		/* Receve status and control register */
		U1BRG		= 0x0007;     	/* Baund rate control register  115200 bps */
		
//		IPC2bits.U1RXIP      = 3;   /* Receiver Interrupt Priority bits */
		IPC2bits.U1RXIP      = 3;   /* Receiver Interrupt Priority bits */

		IEC0bits.U1RXIE 	 = 1;	/* Enable RS232 interrupts */
		IFS0bits.U1RXIF  	 = 0;	
	}
			
/*======================================================*/ 
void putch (char byte) 
{
	while(!IFS1bits.U2TXIF)		
	continue;
	U2TXREG = byte;
	IFS1bits.U2TXIF = 0;
}  

/*======================================================*/
void a2d_sensors (void)
    {

/***** R E A D  Ch_ 0 ************************************/

    for(ch=0;ch<0xc;ch++)
    { 
        /***********************************/
        SPI1CON     = 0x0323;
        SPI1CONbits.MODE16 = 0;
               
        PORTFbits.RF0	= 0; //cs		= 0;  
		SPI1BUF	= 0x18;
		while(!IFS0bits.SPI1IF)
		continue;
        temp = SPI1BUF;  	    
		IFS0bits.SPI1IF      = 0;
        PORTFbits.RF0	= 1; //		cs = 1;     

        for(w=0;w<0xb;w++) { asm("NOP"); }
        
        /***********************************/
        SPI1CON     = 0x0323;
        SPI1CONbits.MODE16 = 1;
               
        PORTFbits.RF0	= 0; //        cs		= 0;  
		SPI1BUF	= 0x6a00;
		while(!IFS0bits.SPI1IF)
		continue;
        temp = SPI1BUF;  	    
		IFS0bits.SPI1IF      = 0;
        PORTFbits.RF0	= 1; //		cs = 1;     

        for(w=0;w<0xb;w++) { asm("NOP"); }
        
        /***********************************/
        SPI1CON     = 0x0323;
        SPI1CONbits.MODE16 = 0;
               
        PORTFbits.RF0	= 0; //        cs		= 0;  
		SPI1BUF	= a2d_channels[ch];
		while(!IFS0bits.SPI1IF)
		continue;
        temp = SPI1BUF;  	    
		IFS0bits.SPI1IF      = 0;
        PORTFbits.RF0	= 1; //		cs = 1;     

        for(w=0;w<0xb;w++) { asm("NOP"); }
        
        SPI1CON     = 0x0323;          
        SPI1CONbits.MODE16 = 1;
		SPI1CONbits.DISSDO = 1;
		
        PORTFbits.RF0	= 0; //		cs  = 0;
		SPI1BUF = 0x0000;
	    while(!IFS0bits.SPI1IF)
		continue;
		a2d_data[ch] = SPI1BUF;
		IFS0bits.SPI1IF      = 0;  
        PORTFbits.RF0	= 1; //        cs = 1;
        asm("NOP"); asm("NOP"); asm("NOP"); asm("NOP"); 
        
   }    
/***********************************************************/ 
		
		for(num1 = 0; num1 < N; num1++){
			for(num2 = 0; num2 < M-1; num2++){
				vals[num1][num2] = vals[num1][num2+1];
			}	
			vals[num1][M-1] = a2d_data[num1];
			srval = 0;	
			for(num2 = 0; num2 < M; num2++){
				srval += vals[num1][num2];
			}
			int myvar;
			srval /= M;
			datch[num1] = 1000 - mapper(srval,mins[num1], maxs[num2], 0, 1000);
		}
		recalc();
        /*sl3 = mapper(a2d_data[0x00], mins[0],maxs[0],0,1000);// * 0.8571;
        sl2 = mapper(a2d_data[0x01], mins[1],maxs[1],0,1000);// *  0.787;
        sl1 = mapper(a2d_data[0x02], mins[2],maxs[2],0,1000);// * 1.0714;
        sc  = mapper(a2d_data[0x03], mins[3],maxs[3],0,1000);// * 0.00122;
        sr1 = mapper(a2d_data[0x04], mins[4],maxs[4],0,1000);// * 1.2765;;
        sr2 = mapper(a2d_data[0x05], mins[5],maxs[5],0,1000);// * 0.80808;
        sr3 = mapper(a2d_data[0x06], mins[6],maxs[6],0,1000);// * 0.9756;
        ss =  mapper(a2d_data[0x07], mins[7],maxs[7],0,1000);// * 0.00122;*/
/* */              
        fl = a2d_data[0x08];//* 0.00122;
        fr = a2d_data[0x09];// * 0.00122;
        rl = a2d_data[0x0a];// *0.00122;
        rr = a2d_data[0x0b];// *0.00122;
           
/***********************************************************/       
    }
/*======================================================*/

void SPI1_A2D_init (void)
{
/***** R E S E T ****************************************/     
        SPI1STAT    = 0x8000;
        SPI1CON     = 0x0323;
        
	    IFS0bits.SPI1IF      = 0;	    
        IEC0bits.SPI1IE      = 1; 
        IPC2bits.SPI1IP     = 2;     

        PORTFbits.RF0	= 0; //        cs		= 0;    /* A2D chip select enable */
		SPI1BUF	= 0x10;   /* reset */
		while(!IFS0bits.SPI1IF)
		continue;
		IFS0bits.SPI1IF      = 0;
        PORTFbits.RF0	= 1; //		cs = 1;
		
/********** S E T U P *********************************************/        
        SPI1STAT    = 0x8000;
        SPI1CON     = 0x0323;
        
        SPI1CONbits.MODE16 = 1; 
        
	    IFS0bits.SPI1IF      = 0;	    
        IEC0bits.SPI1IE      = 1; 
        IPC2bits.SPI1IP     = 2; 
             
        PORTFbits.RF0	= 0; //        cs = 0;
		SPI1BUF	= 0x6a00;   /* setup */
		while(!IFS0bits.SPI1IF)
		continue;
		IFS0bits.SPI1IF      = 0;
        PORTFbits.RF0	= 1; //		cs = 1;

/********** A V E R A G I N G *******************************************/        
        SPI1STAT    = 0x8000;
        SPI1CON     = 0x0323;
        SPI1CONbits.MODE16 = 0; 
	    IFS0bits.SPI1IF      = 0;	    
        IEC0bits.SPI1IE      = 1; 
        IPC2bits.SPI1IP     = 2;     
        PORTFbits.RF0	= 0; //        cs = 0;
		SPI1BUF	= 0x20;
		while(!IFS0bits.SPI1IF)
		continue;
		IFS0bits.SPI1IF      = 0;
        PORTFbits.RF0	= 1; //		cs = 1;
						
/******** R E S E T  F I F O ********************************/        		
        SPI1STAT    = 0x8000;
        SPI1CON     = 0x0323;
        
	    IFS0bits.SPI1IF      = 0;	    
        IEC0bits.SPI1IE      = 1; 
        IPC2bits.SPI1IP     = 2;     
        PORTFbits.RF0	= 0; //        cs = 0;
		SPI1BUF	= 0x18;   
		while(!IFS0bits.SPI1IF)
		continue;
		IFS0bits.SPI1IF      = 0;
        PORTFbits.RF0	= 1; //		cs = 1;	
				
/***********************************************************/
        SPI1STAT    = 0x8000;
        SPI1CON     = 0x0323;
               
	    IFS0bits.SPI1IF      = 0;	    
        IEC0bits.SPI1IE      = 1; 
        IPC2bits.SPI1IP     = 2;     

        PORTFbits.RF0	= 0; //        cs		= 0;   
        
		SPI1BUF	= 0x00de;
		while(!IFS0bits.SPI1IF)
		continue;
        temp = SPI1BUF;  	    
		IFS0bits.SPI1IF      = 0;
		
        PORTFbits.RF0	= 1; //		cs = 1;     

        for(w=0;w<0x80;w++)
		{
   		    asm("NOP"); 
        }

        SPI1STAT    = 0x8000;
        SPI1CON     = 0x0323;          
        SPI1CONbits.MODE16 = 1; 

        PORTFbits.RF0	= 0; //		cs  = 0;
		SPI1CONbits.DISSDO = 1;
		PORTFbits.RF3       = 0;

		asm ("CLR _SPI1BUF");
	    while(!IFS0bits.SPI1IF)
		continue;
		temp = SPI1BUF;
		IFS0bits.SPI1IF      = 0;  
        PORTFbits.RF0	= 1; //        cs = 1;  
 
/***********************************************************/       
	    IFS0bits.SPI1IF      = 0;	    
        IEC0bits.SPI1IE      = 1; 
        IPC2bits.SPI1IP     = 2;   
}

/*======================================================*/	
/*======================================================*/ 



void recount(void){
	if(masssum > 1500)
		curservo = max(2650-690, min(2650 + 690, 2650 + mapper(momsum / masssum, -20, 20, -690, 690) + add));		
	else {
		pwm = 850;
		curservo = prevservo;
		PDC1 = curservo;
		return;
	}
	pwm = 850;
	if(abs(curservo - 2650) > 200){
		curservo = (turns ? 2650 + (int) 750 * (turn / turns) / 9 : prevservo) + add;
		pwm = 800;	
	}
	prevservo = curservo;
	PDC1 = curservo;
}

int main (void)
{
	__C30_UART = 2;
	initialization ();
	uart2_rs232_init ();
	mode = 0;
	pwm = 800;
	printf( " SL3 \t");
	printf( " SL2 \t");
	printf( " SL1 \t");
	printf( " SC \t");
	printf( " SR1 \t");
	printf( " SR2 \t");
	printf( " SR3 \t");
	printf( " SS \t");
	printf("\n");
	while(1)
    {
		recount();
	}

       
}
/*======================================================*/
/*======================================================*/
