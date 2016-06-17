#define NOTE_ON	0x91
#define DELAY 100

//
// HEADER FILES
//
#include <system.h>
#include <rand.h>
#include <eeprom.h>

// PIC CONFIG BITS
// - RESET INPUT DISABLED
// - WATCHDOG TIMER OFF
// - INTERNAL OSC
#pragma DATA _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF &_CLKOUTEN_OFF
#pragma DATA _CONFIG2, _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_19 & _LVP_OFF
#pragma CLOCK_FREQ 16000000

//
// TYPE DEFS
//
typedef unsigned char byte;

//
// MACRO DEFS
//


// outputs
#define P_LED0		porta.1
#define P_LED1		porta.2 

#define TIMER_0_INIT_SCALAR		5	// Timer 0 is an 8 bit timer counting at 250kHz
									// using this init scalar means that rollover
									// interrupt fires once per ms

//
// GLOBAL DATA
//

// timer stuff
volatile byte tick_flag = 0;
volatile unsigned int timer_init_scalar = 0;
volatile unsigned long systemTicks = 0; // each system tick is 1ms
////////////////////////////////////////////////////////////
// INTERRUPT HANDLER CALLED WHEN CHARACTER RECEIVED AT 
// SERIAL PORT OR WHEN TIMER 1 OVERLOWS
void interrupt( void )
{
	// timer 0 rollover ISR. Maintains the count of 
	// "system ticks" that we use for key debounce etc
	if(intcon.2)
	{
		tmr0 = TIMER_0_INIT_SCALAR;
		systemTicks++;
		intcon.2 = 0;
	}

}


////////////////////////////////////////////////////////////
// INITIALISE SERIAL PORT FOR MIDI
void initUSART()
{
	pir1.1 = 1;		//TXIF 		
	pir1.5 = 0;		//RCIF
	
	pie1.1 = 0;		//TXIE 		no interrupts
	pie1.5 = 0;		//RCIE 		enable
	
	baudcon.4 = 0;	// SCKP		synchronous bit polarity 
	baudcon.3 = 1;	// BRG16	enable 16 bit brg
	baudcon.1 = 0;	// WUE		wake up enable off
	baudcon.0 = 0;	// ABDEN	auto baud detect
		
	txsta.6 = 0;	// TX9		8 bit transmission
	txsta.5 = 1;	// TXEN		transmit enable
	txsta.4 = 0;	// SYNC		async mode
	txsta.3 = 0;	// SEDNB	break character
	txsta.2 = 0;	// BRGH		high baudrate 
	txsta.0 = 0;	// TX9D		bit 9

	rcsta.7 = 1;	// SPEN 	serial port enable
	rcsta.6 = 0;	// RX9 		8 bit operation
	rcsta.5 = 0;	// SREN 	!enable receiver
	rcsta.4 = 1;	// CREN 	continuous receive enable
		
	spbrgh = 0;		// brg high byte
	spbrg = 31;		// brg low byte (31250)	
	
}

////////////////////////////////////////////////////////////
// SEND A BYTE ON SERIAL PORT
void send(byte c)
{
	txreg = c;
	while(!txsta.1);
}


////////////////////////////////////////////////////////////
// MAIN
void main()
{ 
	// osc control / 16MHz / internal
	osccon = 0b01111010;
	
	apfcon.7=1; // RX on RA5
	apfcon.2=1;	// TX on RA4	
	// configure io
	trisa = 0b00110000;              	
	ansela = 0b00000000;
	porta=0;
	// initialise MIDI comms
	initUSART();
/*

	// Configure timer 0 (controls systemticks)
	// 	timer 0 runs at 4MHz
	// 	prescaled 1/16 = 250kHz
	// 	rollover at 250 = 1kHz
	// 	1ms per rollover	
	option_reg.5 = 0; // timer 0 driven from instruction cycle clock
	option_reg.3 = 0; // timer 0 is prescaled
	option_reg.2 = 0; // }
	option_reg.1 = 1; // } 1/16 prescaler
	option_reg.0 = 1; // }
	intcon.5 = 1; 	  // enabled timer 0 interrrupt
	intcon.2 = 0;     // clear interrupt fired flag
	
	// enable interrupts	
	intcon.7 = 1; //GIE
	intcon.6 = 1; //PEIE

*/
	// App loop
	byte note = 1;
	for(;;)
	{	
		send(NOTE_ON);
		send(note);
		send(0x7f);
		P_LED0 = 1;
		delay_ms(DELAY);
		send(NOTE_ON);
		send(note);
		send(0x00);
		P_LED0 = 0;
		delay_ms(DELAY);
		P_LED1 = 0;		
		++note;
		if(note > 127) {
			note =1;
			P_LED1 = 1;		
		}
	}
}