//mûködõ verzió


// 128 128 128 (ÇÇÇ) : adat következik
// 128 128 129 (ÇÇü) : státusz info következik
// státusz info : 	0: ok
//					1: limit
//					2: overload
//					3: timeout
//					4: overrange
//					5: home error
//					6: bad cmd

#include	<p18f4320.h>					// Register definitions
#include  	<stdlib.h>
#include 	<string.h>
#include 	<pwm.h>						// PWM library functions
#include	<portb.h>					// PORTB library function
#include 	<timers.h>					// Timer library functions
#include 	<spi.h>

//---------------------------------------------------------------------
//Constant Definitions
//---------------------------------------------------------------------

#define	BUSY	PORTBbits.RB5		// ADC ready input
#define	CS		PORTBbits.RB2		// ADC chip select
#define	CONVERT	PORTBbits.RB3		// ADC read / convert
#define	BYTE	PORTBbits.RB4		// ADC byte select

#define	LIMIT	PORTBbits.RB1		// 
#define	HOME	PORTBbits.RB0		// 
#define LDAC	PORTEbits.RE0		// DAC load
#define CSDAC   PORTEbits.RE1		// DAC chip select


//---------------------------------------------------------------------
// Variable declarations
//---------------------------------------------------------------------

const rom char ready[] = "ÇÇü0\n\r";	// ascii 128+128+129 (send status) + "0" (ok)
const rom int homespeed = 0;

char inpbuf[8];							// Input command buffer
char data[9];

unsigned char	
parameter,i,comcount,udata,			//soros komm változók						
var,
tempp								// SPI komm-hoz						
;

struct {										// Holds status bits for servo
    unsigned	overload:1;		
    unsigned    inrange:1;	
    unsigned 	motion:1;		
    unsigned 	saturated:1;	
    unsigned    ok:1;
    unsigned 	timeout:1;		
    unsigned    send:1;			
    unsigned    limit:1;	
} stat ;

int
kp,ki,kd,						// PID gain constants
oldad,
reqforce,
inrange,
delay,delay1,timeout
;

union INT
{
int i;
char b[2];
};

union INT
adin,
vlim;

union LNG									
{
long l;
unsigned long ul;
int i[2];
unsigned int ui[2];
char b[4];
unsigned char ub[4];
};

union LNG
temp,							// Temporary storage
error,							// PID error value
integral,               // PID integral value
ypid,							// Holds output of PID calculation
reqforce1,
position,
mposition
;

long
mvelocity
//, ypid1
;

//---------------------------------------------------------------------
// Function Prototypes
//---------------------------------------------------------------------

void servo_isr(void);		// Does servo calculations
void isrhandler(void);		// Located at high interrupt vector

void DoCommand(void);		// Processes command input strings
void Setup(void);				// Configures peripherals and variables
void UpdPos(void);			// Gets new measured position for motor
void CalcError(void);		// Calculates position error
void CalcPID(void);			// Calculates new PWM duty cycle
void UpdTraj(void);			// Calculates new commanded position
void SetupMove(void);		// Gets new parameters for motion profile
void HomeMove(void);
void StopMotor(void);


// Writes a string from ROM to the USART
void putrsUSART(const rom char *data);
// Writes a string from RAM to the USART
void putsUSART(char *data);

//---------------------------------------------------------------------
// Interrupt Code
//---------------------------------------------------------------------

// Designate servo_isr as an interrupt function and save key registers

#pragma interrupt servo_isr save = temp

// Locate ISR handler code at interrupt vector

#pragma code isrcode=0x0808

void isrhandler(void)	// This function directs execution to the
{								// actual interrupt code										
_asm
goto servo_isr
_endasm
}

#pragma code

//---------------------------------------------------------------------
// servo_isr()
// Performs the servo calculations
//---------------------------------------------------------------------

void servo_isr(void)
	{
	if (timeout != 0) timeout--;
	if (!LIMIT) 
		{
		stat.limit = 1;	
		stat.motion = 0;
		}

	UpdTraj();								// Get new commanded position
	UpdPos();								// Get new measured position

	if (stat.motion)
		{
		if (adin.i > inrange) stat.inrange = 1;
		if (stat.inrange)
			{
			delay-=1;
			if (delay == 0) stat.ok = 1;
			}	

		CalcError();							// Calculate new position error
		CalcPID();
		}
		PIR1bits.TMR2IF = 0;					// Clear Timer2 Interrupt Flag.
	}											// timing

//---------------------------------------------------------------------
// UpdTraj()
//	Computes the next required value 
//---------------------------------------------------------------------

void UpdTraj(void)
{
	if(stat.motion && !stat.saturated)
	    {   
		position.l += (long)vlim.i;
   		}
	if (position.l > reqforce1.l)
		{
		position.l = reqforce1.l;	
		}
}

//---------------------------------------------------------------------
// UpdPos()
// Gets the new measured position of the motor based on values 
// accumulated in Timer0 and Timer1
//---------------------------------------------------------------------

void UpdPos(void)
{

		CONVERT = 0;						// start new conversion
		CONVERT = 1;
		
		while (!BUSY);

		adin.b[0] = PORTD;
		BYTE = 1;
		adin.b[1] = PORTD;
		BYTE = 0;
	
		mvelocity = adin.i - oldad;
		oldad = adin.i;

		if (adin.i > 32700) stat.overload = 1;
	
		mposition.i[0] = adin.i;

		mposition.b[2] = mposition.b[1];		// Rotate reqforce1 one byte
		mposition.b[1] = mposition.b[0];		// to the left.
		mposition.b[0] = 0;
		if(mposition.b[2] & 0x80) 				// Sign-extend value
  			mposition.b[3] = 0xff;
		else 
   		mposition.b[3] = 0;

}

//---------------------------------------------------------------------
// CalcError()
// Calculates position error and limits to 16 bit result
//---------------------------------------------------------------------

void CalcError(void)
	{
	temp.l = position.l;							// Put commanded pos. in temp
	temp.b[0] = 0;									// Mask out fractional bits
	error.l = temp.l - mposition.l;					// Get error
	error.b[0] = error.b[1];							// from desired position and
	error.b[1] = error.b[2];							// shift to the right to discard
	error.b[2] = error.b[3];							// lower 8 bits.

	if (error.b[2] & 0x80)							// If error is negative.
		{
		error.b[3] = 0xff;							// Sign-extend to 32 bits.

		if(error.l < -32768) error.l = -32768;						// Limit error to 16-bits.
		}

	else												// If error is positive.
		{
		error.b[3] = 0x00;

		if(error.l > 32767) error.l = 32767;						// Limit error to 16-bits.
		}
	}

//---------------------------------------------------------------------
// CalcPID()
// Calculates PID compensator algorithm and determines new value for
// PWM duty cycle
//---------------------------------------------------------------------

void CalcPID(void)
	{
	if(!stat.saturated)	// If output is not saturated,
    integral.l += error.l;  // modify the integral value.
 
// Calculate the PID compensator.

	ypid.l = (long)error.i[0]*(long)kp + integral.l*(long)ki + mvelocity*(long)kd;					

	if(ypid.ub[3] & 0x80)					// If PID result is negative
		{
		if((ypid.ub[3] < 0xff) || !(ypid.ub[2] & 0x80))
		ypid.ul = 0xff800000;			// Limit result to 24-bit value
		}

	else											// If PID result is positive
		{
		if(ypid.ub[3] || (ypid.ub[2] > 0x7f))
		ypid.ul = 0x007fffff;			// Limit result to 24-bit value
		}
	
	ypid.b[0] = ypid.b[1];					// Shift PID result right to 
	ypid.b[1] = ypid.b[2];					// get upper 16 bits.

	stat.saturated = 0;						// Clear saturation flag and see
	if(ypid.i[0] > 2046)						// if present duty cycle output
		{											// exceeds limits.
		ypid.i[0] = 2046;
		stat.saturated = 1;
		}
	
	if(ypid.i[0] < -2046)
		{
		ypid.i[0] = -2046;
		stat.saturated = 1;
		}

	ypid.i[0] += 2048;							// Add offset to get positive 
												// duty cycle value.
	CSDAC = 0;
	tempp = (ypid.b[1] & 0x0f);
	tempp = (tempp | 0x30);
	PORTAbits.RA0 = 1;
	var = putcSPI(tempp);
	var = putcSPI(ypid.b[0]);
	PORTAbits.RA0 = 0;
	CSDAC = 1;
	LDAC = 0;
	LDAC = 1;

	}

//---------------------------------------------------------------------
//	Setup() initializes program variables and peripheral registers
//---------------------------------------------------------------------

void Setup(void)
{

timeout = 0;
parameter = 0;							
i = 0;									// Receive buffer index
comcount = 0;							// Input command index
udata = 0;								// Holds USART received data

delay = 1;

stat.overload = 0;
stat.inrange = 0;
stat.motion = 0;
stat.saturated = 0;
stat.ok = 0;
stat.timeout = 0;
stat.send = 0;
stat.limit = 0;

integral.l = 0;
vlim.i = 30000;
mvelocity = 0;
temp.l = 0;
error.l = 0;
ypid.l = 0;
reqforce1.l = 0;
reqforce = 15000;
position.l = 0;
mposition.l = 0;

memset(inpbuf,0,8);		   // clear the input buffer

OpenPWM1(235);				

OpenTimer2(T2_PS_1_4 & T2_POST_1_4 & TIMER_INT_ON);   // 1.22 kHz int.

SetDCPWM1(512);				// 50% duty cycle for sw. cap. filter

OpenSPI(SPI_FOSC_16, MODE_00, SMPEND);

ADCON1 = 0x0f;				// all analog in disabled

PORTA = 0;
PORTB = 0;
PORTC = 0;						// Clear PORTC
PORTD = 0;						//	Clear PORTD
PORTE = 0;					//	

TRISA = 0xfe;
TRISB = 0xe3;					// PORTB all inputs
TRISC = 0xc3;					// 
TRISD = 0xff;					// PORTD all inputs (ADC data bus)
TRISE = 0;						// PORTE all outputs.

CONVERT = 1;
CS = 0;
CSDAC = 1;
LDAC = 1;
BYTE = 0;

// Setup the USART for 19200 baud @ 20MHz
SPBRG = 64;						// 19200 baud @ 20MHz
//SPBRG = 129;
TXSTA = 0x24;					// setup USART transmit
//TXSTA = 0x20;
RCSTA = 0x90;					// setup USART receive


kp = 60;			// Get PID gain constants
ki = 0;							// from data EEPROM
kd = 0;

StopMotor();
										
INTCONbits.PEIE = 1;					// Enable peripheral interrupts
INTCONbits.GIE = 1;					// Enable all interrupts
}

//---------------------------------------------------------------------
// main()
//---------------------------------------------------------------------

void main(void)
	{
	Setup();									// Setup peripherals and software
	putrsUSART("\r\nPain servo v1.0\n\r");
	putrsUSART(ready);

	while(1)                    		// Loop forever
    	{										
   		//ClrWdt();							// Clear the WDT

			if (stat.limit) 
				{
				stat.motion = 0;
				stat.limit = 0;
				HomeMove();
				putrsUSART("ÇÇü1\r\n");				//limit error
				}	
			if (stat.overload) 
				{
				stat.motion = 0;
				stat.overload = 0;
				HomeMove();
				putrsUSART("ÇÇü2\r\n");			//overload error
				}
			if (stat.ok) 
				{
				stat.motion = 0;
				stat.ok = 0;
				HomeMove();
				putrsUSART("ÇÇü0\r\n");				//ok
				}

	if (stat.motion	&& !timeout)
		{
		StopMotor();
		stat.timeout = 1;
		stat.motion = 0;
		putrsUSART("ÇÇü3\r\n");									// timeout error
		}

	if (stat.send)
			{
			putrsUSART("ÇÇ");		// adat következik
				
		//	itoa(adin.i, data);
          //  putsUSART(data);
		
			while(!(TXSTA & 0x02));
			TXREG = adin.b[0];
		
			while(!(TXSTA & 0x02));
			TXREG = adin.b[1];

			while(!(TXSTA & 0x02));
			TXREG = 13;
			}

		if(PIR1bits.RCIF)									// Check for USART interrupt
   			{
   			switch(udata = RCREG)
	    		{
	    		

case 'g':	
				if (stat.timeout)
					{	
					putrsUSART("ÇÇü3\r\n");		// timeout error
					break;
					}
				reqforce1.l = (long)reqforce+(long)adin.i;
				if ((reqforce1.l > 32700) | (reqforce1.l < 2000)) 
					{
					putrsUSART("ÇÇü4\r\n");    //overrange error
					break;						
					}

				inrange = reqforce1.l - (reqforce1.l / 10);

				reqforce1.b[2] = reqforce1.b[1];		// Rotate reqforce1 one byte
				reqforce1.b[1] = reqforce1.b[0];		// to the left.
				reqforce1.b[0] = 0;
				if(reqforce1.b[2] & 0x80) 				// Sign-extend value
  					reqforce1.b[3] = 0xff;
				else 
   					reqforce1.b[3] = 0;
					
				if (!HOME)
					{				
					position.i[0] = adin.i;

					position.b[2] = position.b[1];		// Rotate reqforce1 one byte
					position.b[1] = position.b[0];		// to the left.
					position.b[0] = 0;
					if(position.b[2] & 0x80) 				// Sign-extend value
  						position.b[3] = 0xff;
					else 
   						position.b[3] = 0;
					integral.l = 0;
					putrsUSART("ÇÇü0");						//ok, szúrás
					stat.inrange = 0;
					stat.ok = 0;
					delay = delay1;
					timeout = 1221;							// timeout 1 sec
					stat.motion = 1;						//enable motion
					}
				else
					{
					putrsUSART("ÇÇü5");						//home error, nem lehet szúrni
					stat.motion = 0;
					}
	
			break;
		
			case 'h':   						//go home
				if (stat.timeout)				
					{	
					putrsUSART("ÇÇü3\r\n");		// timeout error
					break;
					}
				stat.motion = 0;
				HomeMove();
			break;
				
			case 'm':   						//send measured data
				if(!stat.send) stat.send = 1;
				else stat.send = 0;
			break;





				case ',':	DoCommand();				// process the string
	    			memset(inpbuf,0,8);		// clear the input buffer
	               	i = 0;						// clear the buffer index
	               	comcount++;					// increment comma count
	               	TXREG = udata;				// echo the character
	            break;
	      				
	    		case 0x0d:  DoCommand();				// process the string
	               	memset(inpbuf,0,8);		// clear the input buffer
	               	i = 0;						// clear the buffer index
	               	comcount = 0;				// clear comma count
					parameter = 0;				// clear paramater 
					putrsUSART(ready);		// put prompt to USART 
	           	break;
	                  
	      		default:    inpbuf[i] = udata;		// get received char 
	               	i++;							// increment buffer index
	               	if(i > 7)					// If more than 8 chars
	                	{							// received before getting
	                   	putrsUSART(ready); 	// a <CR>, clear input
	                   	memset(inpbuf,0,8);	// buffer
	                   	i = 0;					// the buffer index
	                   	}
	               	else TXREG = udata; 		// echo character
	              	break;						//
         
         		}     										//end switch(udata)
	   	    }       										//end if(RCIF)
    	}          										//end while(1)
	}
											// variables.

//-------------------------------------------------------------------
// DoCommand()
// Processes incoming USART data.
//-------------------------------------------------------------------

void DoCommand(void)
	{
	if(comcount == 0)		// If this is the first parameter of the input
		{						// command...
		switch(inpbuf[0])
			{
			case 'v':	parameter = 'V';		// velocity ch ange
			break;

			case 'f':	parameter = 'F';		// force change
			break;
					
			case 'p':	parameter = 'P';		// Change proportional gain
			break;
	
			case 'i':	parameter = 'I';		// Change integral gain
			break;

			case 'd':	parameter = 'D';		// Change differential gain
			break;
		
			case 't':	parameter = 'T';		// szuras ideje
			break;

			case 's': 	putrsUSART("  ADin = ");						// Send all parameters to host.
            	itoa(adin.i, data);
               	putsUSART(data);
               
				putrsUSART("  req = ");						// Send all parameters to host.
               	itoa(reqforce, data);
               	putsUSART(data);	

               	putrsUSART("  Kp = ");
               	itoa(kp, data);
               	putsUSART(data);

				putrsUSART("  Ki = ");
               	itoa(ki, data);
               	putsUSART(data);
               
               	putrsUSART("  Kd = ");
               	itoa(kd, data);
               	putsUSART(data);

				putrsUSART("  vlim = ");
               	itoa(vlim.i, data);
               	putsUSART(data);

				putrsUSART("  delay = ");
               	itoa(delay, data);
               	putsUSART(data);

				putrsUSART("  range = ");
               	itoa(inrange, data);
               	putsUSART(data);

         	break; 
		
			case 'g':	
				if (stat.timeout)
					{	
					putrsUSART("ÇÇü3\r\n");		// timeout error
					break;
					}
				reqforce1.l = (long)reqforce+(long)adin.i;
				if ((reqforce1.l > 32700) | (reqforce1.l < 2000)) 
					{
					putrsUSART("ÇÇü4\r\n");    //overrange error
					break;						
					}

				inrange = reqforce1.l - (reqforce1.l / 10);

				reqforce1.b[2] = reqforce1.b[1];		// Rotate reqforce1 one byte
				reqforce1.b[1] = reqforce1.b[0];		// to the left.
				reqforce1.b[0] = 0;
				if(reqforce1.b[2] & 0x80) 				// Sign-extend value
  					reqforce1.b[3] = 0xff;
				else 
   					reqforce1.b[3] = 0;
					
				if (!HOME)
					{				
					position.i[0] = adin.i;

					position.b[2] = position.b[1];		// Rotate reqforce1 one byte
					position.b[1] = position.b[0];		// to the left.
					position.b[0] = 0;
					if(position.b[2] & 0x80) 				// Sign-extend value
  						position.b[3] = 0xff;
					else 
   						position.b[3] = 0;
					integral.l = 0;
					putrsUSART("ÇÇü0");						//ok, szúrás
					stat.inrange = 0;
					stat.ok = 0;
					delay = delay1;
					timeout = 1221;							// timeout 1 sec
					stat.motion = 1;						//enable motion
					}
				else
					{
					putrsUSART("ÇÇü5");						//home error, nem lehet szúrni
					stat.motion = 0;
					}
	
			break;
		
			case 'h':   						//go home
				if (stat.timeout)				
					{	
					putrsUSART("ÇÇü3\r\n");		// timeout error
					break;
					}
				stat.motion = 0;
				HomeMove();
			break;
				
			case 'm':   						//send measured data
				if(!stat.send) stat.send = 1;
				else stat.send = 0;
			break;

			case 'r':   						// reset error
				stat.timeout = 0;				
			break;

      	    default:    
				if(inpbuf[0] != '\0') putrsUSART("ÇÇü6/r/n");		// bad cmd
	        break;
		}
	}

	else if(comcount == 1)		// If this is the second parameter of the
		{								// input command.
		switch(parameter)									
			{
			case 'P':   kp = atoi(inpbuf);			// proportional gain change
		    break;

			case 'I':   ki = atoi(inpbuf);			// integral gain change
		    break;

			case 'D':   kd = atoi(inpbuf);			// differential gain change
		    break;
      
			case 'V':	vlim.i = atoi(inpbuf);
			break;	
		
			case 'T':	delay1 = atoi(inpbuf);
				if (delay1<1) delay1 =1;
			break;									
												
			case 'F':	reqforce = atoi(inpbuf);
				if (reqforce < 1000)  reqforce = 1000;
			break;
			     
			default:    break;
     	}
	}
}

//---------------------------------------------------------------------
// putrsUSART()
// Writes a string of characters in program memory to the USART
//---------------------------------------------------------------------

void putrsUSART(const rom char *data)
	{
	do
		{				
		while(!(TXSTA & 0x02));
		TXREG = *data;
		} 
	while( *data++ );
	}


//---------------------------------------------------------------------
// putsUSART()
// Writes a string of characters in data memory to the USART
//---------------------------------------------------------------------

void putsUSART(char *data)
	{
	do
		{				
		while(!(TXSTA & 0x02));
		TXREG = *data;
		} 
	while( *data++ );
	}

//---------------------------------------------------------------------
// homemove
// 
//---------------------------------------------------------------------

void HomeMove(void)
	{
	if (HOME)
			{
			if (stat.timeout)
				{
				StopMotor();
				putrsUSART("ÇÇü3\r\n");		// timeout error
				}
			else
				{
		
				ypid.i[0] = homespeed;

				CSDAC = 0;
				tempp = (ypid.b[1] & 0x0f);
				tempp = (tempp | 0x30);
				PORTAbits.RA0 = 1;
				var = putcSPI(tempp);
				var = putcSPI(ypid.b[0]);
				PORTAbits.RA0 = 0;
				CSDAC = 1;
				LDAC = 0;
				LDAC = 1;

				timeout = 1221; 			//1 sec timeout
				while (HOME)
					{
					if (timeout == 0) 
						{
						StopMotor();
						stat.timeout = 1;	
						stat.motion = 0;
						putrsUSART("ÇÇü3\r\n");									// timeout error
						break;
						}	
					}
				StopMotor();
				}		
			}
		}

//---------------------------------------------------------------------
// stopmotor
// 
//---------------------------------------------------------------------

void StopMotor(void)
	{
	ypid.i[0] = 2047;

	CSDAC = 0;
	tempp = (ypid.b[1] & 0x0f);
	tempp = (tempp | 0x30);
	PORTAbits.RA0 = 1;
	var = putcSPI(tempp);
	var = putcSPI(ypid.b[0]);
	PORTAbits.RA0 = 0;
	CSDAC = 1;
	LDAC = 0;
	LDAC = 1;
	}
