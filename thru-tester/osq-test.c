//////////////////////////////////////////////
// ORANGE SQUEEZE THRU TEST
// RUNS ON ORANGE SQUEEZE HARDWARE
//////////////////////////////////////////////


//
// HEADER FILES
//
#include <system.h>
#include <rand.h>
#include <eeprom.h>

#define BEAT_DELAY 30
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


#define P_LED1		latc.2 	// MIDI input red LED
#define P_LED2		latc.3 	// Blue LED
#define P_LED3		lata.2 	// Yellow LED
#define P_SWITCH	porta.5

#define P_BLUE		P_LED2
#define P_YELLOW	P_LED3
#define P_RED		P_LED1


//#define P_LEDR 	lata.0
//#define P_LEDG  lata.1
//#define P_LEDY  lata.2
//#define P_LEDB  latc.0
//#define P_BUTTON  portc.3

// MIDI beat clock messages
#define MIDI_SYNCH_TICK     	0xf8
#define MIDI_SYNCH_START    	0xfa
#define MIDI_SYNCH_CONTINUE 	0xfb
#define MIDI_SYNCH_STOP     	0xfc


#define TIMER_0_INIT_SCALAR		5	// Timer 0 is an 8 bit timer counting at 250kHz
									// using this init scalar means that rollover
									// interrupt fires once per ms

// Tempo defs
//#define BPM_MIN					30
//#define BPM_MAX					300
//#define BPM_DEFAULT				120

// EEPROM usage
//#define EEPROM_ADDR_MAGIC_COOKIE 9
//#define EEPROM_ADDR_OPTIONS	10
//#define EEPROM_MAGIC_COOKIE 0xC5

// Menu size
//#define MENU_SIZE 6 

//
// GLOBAL DATA
//

// timer stuff
volatile byte tick_flag = 0;
volatile unsigned int timer_init_scalar = 0;
volatile unsigned long systemTicks = 0; // each system tick is 1ms

// define the buffer used to receive MIDI input
#define SZ_RXBUFFER 0x40
#define SZ_RXBUFFER_MASK	0x3f
volatile byte rx_buffer[SZ_RXBUFFER];
volatile byte rx_head = 0;
volatile byte rx_tail = 0;

byte midi_status;					// current MIDI message status (running status)
byte midi_num_params;				// number of parameters needed by current MIDI message
byte midi_params[2];					// parameter values of current MIDI message
char midi_param;					// number of params currently received

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

	// timer 1 rollover ISR. Responsible for timing
	// the tempo of the MIDI clock
	if(pir1.0)
	{
		tmr1l=(timer_init_scalar & 0xff); 
		tmr1h=(timer_init_scalar>>8); 
		tick_flag = 1;
		pir1.0 = 0;
	}
		
	// serial rx ISR
	if(pir1.5)
	{	
		// get the byte
		byte b = rcreg;
		
		// calculate next buffer head
		byte nextHead = (rx_head + 1);
		if(nextHead >= SZ_RXBUFFER) 
		{
			nextHead -= SZ_RXBUFFER;
		}
		
		// if buffer is not full
		if(nextHead != rx_tail)
		{
			// store the byte
			rx_buffer[rx_head] = b;
			rx_head = nextHead;
		}		
	}
}



////////////////////////////////////////////////////////////
// INITIALISE SERIAL PORT FOR MIDI
void initUSART()
{
	pir1.1 = 1;		//TXIF 		
	pir1.5 = 0;		//RCIF
	
	pie1.1 = 0;		//TXIE 		no interrupts
	pie1.5 = 1;		//RCIE 		enable
	
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
	rcsta.5 = 1;	// SREN 	enable receiver
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
// GET MESSAGES FROM MIDI INPUT
byte midi_in()
{
	// loop until there is no more data or
	// we receive a full message
	for(;;)
	{
		// usart buffer overrun error?
		if(rcsta.1)
		{
			rcsta.4 = 0;
			rcsta.4 = 1;
		}
		
		// check for empty receive buffer
		if(rx_head == rx_tail)
			return 0;
		
		// read the character out of buffer
		byte ch = rx_buffer[rx_tail];
		++rx_tail;
		rx_tail&=SZ_RXBUFFER_MASK;

		// SYSTEM MESSAGE
		if((ch & 0xf0) == 0xf0)
		{
			return 0;
		}
		// STATUS BYTE
		else if(!!(ch & 0x80))
		{
			midi_param = 0;
			midi_status = ch; 
			switch(ch & 0xF0)
			{
			case 0xC0: //  Patch change  1  instrument #   
			case 0xD0: //  Channel Pressure  1  pressure  
				midi_num_params = 1;
				break;    
			case 0xA0: //  Polyphonic aftertouch  2  key  touch  
			case 0x80: //  Note-off  2  key  velocity  
			case 0x90: //  Note-on  2  key  veolcity  
			case 0xB0: //  Continuous controller  2  controller #  controller value  
			case 0xE0: //  Pitch bend  2  lsb (7 bits)  msb (7 bits)  
			default:
				midi_num_params = 2;
				break;        
			}
		}    
		else if(midi_status)
		{
			// gathering parameters
			midi_params[midi_param++] = ch;
			if(midi_param >= midi_num_params)
			{
				// we have a complete message.. is it one we care about?
				midi_param = 0;
				switch(midi_status&0xF0)
				{
				case 0x80: // note off
				case 0x90: // note on
				case 0xE0: // pitch bend
				case 0xB0: // cc
				case 0xD0: // channel pressure
					return midi_status; 
				}
			}
		}
	}
	// no message ready yet
	return 0;
}


////////////////////////////////////////////////////////////
// RUN MIDI THRU
byte do_test()
{
	rx_head = rx_tail;
	midi_status = 0;
	midi_num_params = 0;
	midi_param = 0;
	
	
	for(int i=0; i<80; i++)
	{		
		byte status = (0x90 | (i & 0x0F));
		byte note = i;
		byte velocity = 127-i;


		send(status);
		send(note);
		send(velocity);
		delay_ms(30);
		systemTicks = 0;
		byte msg = 0;
		do {
			msg = midi_in();
		}
		while(!msg && systemTicks < 200);
		if(!msg)
			return 0;
			
		if(midi_num_params != 2)
			return 0;
		if(midi_status != status)
			return 0;
		if(midi_params[0] != note)
			return 0;
		if(midi_params[1] != velocity)
			return 0;	
	}		
	return 1;
}


////////////////////////////////////////////////////////////
// MAIN
void main()
{ 
	
	// osc control / 16MHz / internal
	osccon = 0b01111010;
	
	// configure io
	//trisa = TRIS_A;              	
    //trisc = TRIS_C;              
	//ansela = 0b00000000;
	//anselc = 0b00000000;
	//porta=0;
	//portc=0;
	//wpuc.3=1;
	//option_reg.7=0;

	// configure io
	trisa = 0b00100000;              	
    trisc = 0b00110000;   	
	ansela = 0b00000000;
	anselc = 0b00000000;
	porta=0;
	portc=0;
	wpua.5 = 1; // weak pullup on switch RA5
	option_reg.7 = 0;	// enable weak pull ups on port a
		
	// initialise MIDI comms
	initUSART();

	// Configure timer 1 (controls tempo)
	// Input 4MHz
	// Prescaled to 500KHz
	tmr1l = 0;	 // reset timer count register
	tmr1h = 0;
	t1con.7 = 0; // } Fosc/4 rate
	t1con.6 = 0; // }
	t1con.5 = 1; // } 1:8 prescale
	t1con.4 = 1; // }
	t1con.0 = 1; // timer 1 on
	pie1.0 = 1;  // timer 1 interrupt enable
	
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
	
	delay_ms(200);
	P_RED=0;
	P_BLUE=0;
	P_YELLOW=0;
	// App loop
	for(;;)
	{	
		while(P_SWITCH) {
//			P_YELLOW = !beat;
//			if(++beat == 24) beat = 0;
//			send(MIDI_SYNCH_TICK);			
//			delay_ms(BEAT_DELAY);
		}		
		P_RED=1;
		byte result = do_test();
		P_RED=0;
		if(result) {
				P_BLUE=1;
				P_YELLOW=1;
				while(P_SWITCH);
				P_BLUE=0;
				P_YELLOW=0;
		}
		else {
				P_BLUE=0;
				while(P_SWITCH) {
					P_YELLOW=1; delay_ms(200); P_YELLOW=0; delay_ms(200);
				}
		}
	}
}