//
// HEADER FILES
//
#include <system.h>
#include <rand.h>
#include <eeprom.h>
#include <memory.h>
#include "mmmerge.h"

// PIC CONFIG BITS
// - RESET INPUT DISABLED
// - WATCHDOG TIMER OFF
// - INTERNAL OSC
#pragma DATA _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF &_CLKOUTEN_OFF
#pragma DATA _CONFIG2, _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_19 & _LVP_OFF
#pragma CLOCK_FREQ 16000000

typedef unsigned char byte;

//
// INPUT/OUTPUT DEFS
//
#define P_LED1		latc.2 // MIDI input red LED
#define P_LED2		latc.3 // Blue LED
#define P_LED3		lata.2 // Yellow LED
#define P_SWITCH	porta.5

#define TRIS_A	0b00100000
#define TRIS_C	0b00110000

#define SLAVE_I2C_ADDR 		0x70	// slave address #0


#define TIMER_0_INIT_SCALAR		5	// Timer 0 is an 8 bit timer counting at 250kHz

#define SZ_TXBUFFER 16
#define TXBUFFER_INDEX_MASK 0x0F
volatile byte tx_buffer[SZ_TXBUFFER];
volatile byte tx_head = 0;
volatile byte tx_tail = 0;

#define SZ_RXBUFFER 64
#define RXBUFFER_INDEX_MASK 0x3F
volatile byte rx_buffer[SZ_RXBUFFER];
volatile byte rx_head = 0;
volatile byte rx_tail = 0;


volatile byte led_1_timeout = 0;
volatile byte led_2_timeout = 0;
volatile byte led_3_timeout = 0;

#define INITIAL_PULSE				200
#define MIDI_INPUT_PULSE			5
#define MIDI_OVERFLOW				100
#define MIDI_OUTPUT_PULSE			5

// macros used to signal the 3 LEDs
#define SIGNAL_LED1(t) { P_LED1 = 1; led_1_timeout = (t); }
#define SIGNAL_LED2(t) { P_LED2 = 1; led_2_timeout = (t); }
#define SIGNAL_LED3(t) { P_LED3 = 1; led_3_timeout = (t); }

// Definitions for the flags byte in input status
#define IS_ONLINE 		0x01	// Slave responded to initial handshake
#define INSIDE_SYSEX   	0x02	// Slave is passing sysex

typedef struct 
{
	byte flags;
	byte sysex_timeout;	
	
	byte running_status;		// MIDI running status for this slave
	byte param[2];				// MIDI parameters
	byte num_params;			// Number of parameters
	byte param_index;			// Current parameter (0..1)
} INPUT_STATUS;

#define NUM_SLAVES 3
INPUT_STATUS slave_status[NUM_SLAVES] = {0};
INPUT_STATUS master_status = {0};
byte out_running_status = 0;


////////////////////////////////////////////////////////
//
// INTERRUPT HANDLER
//
////////////////////////////////////////////////////////
void interrupt( void )
{
	byte next_head;
	
	////////////////////////////////////////////////
	// TIMER0 OVERFLOW INTERRUPT
	// This is used for timing the length of time
	// the various LEDs are on for 
	if(intcon.2)
	{
		tmr0 = TIMER_0_INIT_SCALAR;
		if(led_1_timeout) {
			if(!--led_1_timeout)
				P_LED1 = 0;
		}
		if(led_2_timeout) {
			if(!--led_2_timeout)
				P_LED2 = 0;
		}
		if(led_3_timeout) {
			if(!--led_3_timeout)
				P_LED3 = 0;
		}
		intcon.2 = 0;		
	}
	
	////////////////////////////////////////////////
	// SERIAL PORT RECEIVE INTERRUPT
	// Fires when traffic arrives at the 
	// primary MIDI port
	if(pir1.5)
	{	
		// get the byte
		byte b = rcreg;
				
		// check for realtime message 0xF8..0xFF
		if((b&0xF8) == 0xF8) {
			// place it in the transmit buffer 
			// for transmission asap (tx buffer is
			// only used for realtime messages so
			// just assume it will never fill up!)
			next_head = (tx_head + 1)&TXBUFFER_INDEX_MASK;			
			if(next_head == tx_tail) {
				SIGNAL_LED3(MIDI_OVERFLOW);
			}
			else {
				tx_buffer[tx_head] = b;
				tx_head = next_head;
				SIGNAL_LED1(MIDI_INPUT_PULSE);
			}
		}
		else 
		{
			// place the byte in the receive buffer
			// for standard processing
			next_head = (rx_head + 1)&RXBUFFER_INDEX_MASK;
			if(next_head == rx_head) {
				SIGNAL_LED3(MIDI_OVERFLOW);				
			}
			else {
				rx_buffer[rx_head] = b;
				rx_head = next_head;
				SIGNAL_LED1(MIDI_INPUT_PULSE);
			}			
		}
		pir1.5 = 0;
	}		
}

////////////////////////////////////////////////////////////
// INIT TIMER
// Timer used to provide a millisecond tick
////////////////////////////////////////////////////////////
void init_timer() {
	// Configure timer 0 (controls systemticks)
	// 	timer 0 runs at 2MHz
	// 	prescaled 1/8 = 250kHz
	// 	rollover at 250 = 1kHz
	// 	1ms per rollover	
	option_reg.5 = 0; // timer 0 driven from instruction cycle clock
	option_reg.3 = 0; // timer 0 is prescaled
	option_reg.2 = 0; // }
	option_reg.1 = 1; // } 1/16 prescaler
	option_reg.0 = 0; // }
	intcon.5 = 1; 	  // enabled timer 0 interrrupt
	intcon.2 = 0;     // clear interrupt fired flag	
}

////////////////////////////////////////////////////////////
// INITIALISE SERIAL PORT FOR MIDI
void init_usart()
{
	
	pie1.1 = 0;		//TXIE 		no tx interrupts
	
	pie1.5 = 1;		//RCIE 		enable rx interrupt
	pir1.5 = 0;		//RCIF
	
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
//
// FLUSH BUFFERED REALTIME MESSAGES
//
////////////////////////////////////////////////////////////
void flush_tx_buffer() 
{
	while(tx_head != tx_tail) {
		txreg = tx_buffer[tx_tail];
		tx_tail = (tx_tail + 1)&TXBUFFER_INDEX_MASK;
		SIGNAL_LED2(MIDI_OUTPUT_PULSE);	
		while(!txsta.1);
	}
}

////////////////////////////////////////////////////////////
//
// TRANSMIT A SINGLE MIDI BYTE
//
////////////////////////////////////////////////////////////
void transmit(byte d) 
{
//	if(tx_head != tx_tail) 
//		flush_tx_buffer();
	txreg = d;
	SIGNAL_LED2(MIDI_OUTPUT_PULSE);	
	while(!txsta.1);	
}

////////////////////////////////////////////////////////////
//
// INIT I2C MASTER
//
////////////////////////////////////////////////////////////
void init_i2c() {
	// disable output drivers on i2c pins
	trisc.0 = 1;
	trisc.1 = 1;
	
	//ssp1con1.7 = 
	//ssp1con1.6 = 
	ssp1con1.5 = 1; // Enable synchronous serial port
	ssp1con1.4 = 1; // Enable SCL
	ssp1con1.3 = 1; // }
	ssp1con1.2 = 0; // }
	ssp1con1.1 = 0; // }
	ssp1con1.0 = 0; // } I2C Master with clock = Fosc/(4(SSPxADD+1))
	
	ssp1stat.7 = 1;	// slew rate disabled	
	ssp1add = 19;	// 100kHz baud rate
}

////////////////////////////////////////////////////////////
//
// SEND BYTE TO I2C BUS
//
////////////////////////////////////////////////////////////
void i2c_send(byte data) 
{
	ssp1buf = data;
	while((ssp1con2 & 0b00011111) || // SEN, RSEN, PEN, RCEN or ACKEN
		(ssp1stat.2)); // data transmit in progress	
}

////////////////////////////////////////////////////////////
//
// BEGIN I2C MASTER WRITE TRANSACTION
//
////////////////////////////////////////////////////////////
void i2c_begin_write(byte address) 
{
	pir1.3 = 0; // clear SSP1IF
	ssp1con1.7 = 0; // clear WCOL
	ssp1con2.0 = 1; // signal start condition
	while(!pir1.3); // wait for it to complete
	i2c_send(address<<1); // address + WRITE(0) bit
}

////////////////////////////////////////////////////////////
//
// BEGIN I2C MASTER READ TRANSACTION
//
////////////////////////////////////////////////////////////
byte i2c_begin_read(byte address) 
{
	pir1.3 = 0; // clear SSP1IF
	ssp1con2.0 = 1; // signal start condition
	while(!pir1.3); // wait for it to complete
	i2c_send(address<<1|1); // address + READ(1) bit
	
	pir1.3 = 0; // clear SSP1IF
	ssp1con2.3 = 1; // start receiving
	while(!ssp1stat.0); // wait for it to complete	
	return ssp1buf;
}

////////////////////////////////////////////////////////////
//
// CONTINUE I2C MASTER READ TRANSACTION
//
////////////////////////////////////////////////////////////
byte i2c_continue_read() 
{
	pir1.3 = 0; // clear SSP1IF
	ssp1con2.5 = 0; // set ack state
	ssp1con2.4 = 1; // send ack
	while(!pir1.3); // wait for it to complete		

	ssp1con2.3 = 1; // start receiving
	while(!ssp1stat.0); // wait for data to arrive
	return ssp1buf;
}

byte i2c_end_read() 
{
	pir1.3 = 0; // clear SSP1IF
	ssp1con2.5 = 1; // set NAK state
	ssp1con2.4 = 1; // send ack
	while(!pir1.3); // wait for it to complete		

	pir1.3 = 0; // clear SSP1IF
	ssp1con2.2 = 1; // signal stop condition
	while(!pir1.3); // wait for it to complete
}

////////////////////////////////////////////////////////////
//
// END AN I2C MASTER TRANSACTION
//
////////////////////////////////////////////////////////////
void i2c_end() 
{
	pir1.3 = 0; // clear SSP1IF
	ssp1con2.2 = 1; // signal stop condition
	while(!pir1.3); // wait for it to complete
}

////////////////////////////////////////////////////////////
//
// PROCESS AN INCOMING MIDI BYTE (MASTER OR SLAVE)
// Keeps record of any partial MIDI messages that have 
// been received and when a full message is ready it is 
// sent to the MIDI output
//
////////////////////////////////////////////////////////////
void handle_input_byte(byte d, INPUT_STATUS *pstatus)
{
	// Check if we have a command/status byte
	if(d & 0x80)
	{
		switch(d & 0xF0)
		{
			case 0xF0:
				// system wide and realtime are skipped... they
				// are not valid to be passed into this function!
				return;
			case 0xA0: //  Aftertouch  1  key  touch  
			case 0xC0: //  Patch change  1  instrument #   
			case 0xD0: //  Channel Pressure  1  pressure  
				pstatus->num_params = 1;
				break;    
			case 0x80: //  Note-off  2  key  velocity  
			case 0x90: //  Note-on  2  key  veolcity  
			case 0xB0: //  Continuous controller  2  controller #  controller value  
			case 0xE0: //  Pitch bend  2  lsb (7 bits)  msb (7 bits)  
			default:
				pstatus->num_params = 2;
				break;        
		}
		pstatus->running_status = d;
		pstatus->param_index = 0;
	}
	else 
	{
		// store the parameter
		pstatus->param[pstatus->param_index++] = d;
		
		// do we have all the parameters for the message?
		if(pstatus->running_status && 
			(pstatus->param_index >= pstatus->num_params)) 
		{
			// Send the running status if changed
			if(pstatus->running_status != out_running_status) {
				transmit(pstatus->running_status);
				out_running_status = pstatus->running_status;
			}
			// first parameter
			if(pstatus->num_params > 0) {
				transmit(pstatus->param[0]);
			}
			// second parameter
			if(pstatus->num_params > 1) {
				transmit(pstatus->param[1]);
			}
			// ready to start again
			pstatus->param_index = 0;
		}
	}
}

////////////////////////////////////////////////////////////
//
// PROCESS MIDI DATA RECEIVED FROM 
//
////////////////////////////////////////////////////////////
void master_run()
{
	// is there more info received from serial?
	while(rx_tail != rx_head)
	{	
		// read next byte from serial receive buffer
		byte d = rx_buffer[rx_tail];
		rx_tail = (rx_tail + 1)&RXBUFFER_INDEX_MASK;

		// is this a system wide or realtime message?
		if((d & 0xF0) == 0xF0)
		{
			// check when we are inside a sysex, since this
			// will ensure the slaves are blocked to stop
			// interlacing of data!
			switch(d) 
			{
			case 0xF0: // Start of system exclusive
				master_status.flags |= INSIDE_SYSEX;
				break;
			case 0xF7: // End of System Exclusive
				master_status.flags &= ~INSIDE_SYSEX;
				break;
			}
			
			// single byte message without an effect on
			// running status so transmit it directly
			transmit(d);
		}
		else if(master_status.flags & INSIDE_SYSEX) 
		{
			// send sysex byte
			transmit(d);
		}
		else
		{
			// process the input, updating running
			// status and the like...
			handle_input_byte(d, &master_status);
		}
	}
}	

////////////////////////////////////////////////////////////
//
// SEND A COMMAND TO AN I2C SLAVE
//
////////////////////////////////////////////////////////////
void slave_command(byte addr, byte cmd) 
{
  i2c_begin_write(addr);
  i2c_send(cmd);
  i2c_end();
}

////////////////////////////////////////////////////////////
//
// CHECK A SLAVE IS PRESENT 
// We get the slave to send us the magic cookie "MSLVx" 
// where x is A,B or C. This confirms the slave is alive
//
////////////////////////////////////////////////////////////
byte slave_handshake(byte addr)
{
	// send identify command to the client
	slave_command(addr, CMD_IDENTIFY);
	
	byte version = i2c_begin_read(addr);
	byte result = 1;
	if('M' != i2c_continue_read())
		result = 0;
	if('S' != i2c_continue_read())
		result = 0;
	if('L' != i2c_continue_read())
		result = 0;
	if('V' != i2c_continue_read())
		result = 0;
	byte name = i2c_continue_read();
	i2c_end();	
	
	// make sure the slave buffer is cleared
	slave_command(addr, CMD_CLEAR);
	return result;	
}

////////////////////////////////////////////////////////////
//
// PULL IN DATA FROM A SLAVE
//
////////////////////////////////////////////////////////////
void slave_run(byte addr, INPUT_STATUS *pstatus)
{
	byte i;
	// get count of bytes remaining
	byte count = i2c_begin_read(addr);	
	
	// mask out slave overflow bit
//	if(count & OVERFLOW_BIT) 
//	{
//		count &= ~OVERFLOW_BIT;
//		SIGNAL_LED3(MIDI_OVERFLOW);
//	}

//	if(!count) 
//		SIGNAL_LED3(MIDI_OVERFLOW);

	// get all data from slave
	byte buffer[64];
	for(i=0; i<count; ++i)  
		buffer[i] = i2c_continue_read();
	i2c_end_read();	

	for(i=0; i<count; ++i)  
	{
		byte d = buffer[i];

		//transmit(d);
		
		// No system common or real time messages 
		// will be processed from a slave. This means
		// we need to track when we are within a 
		// system exclusive message
		if((d & 0xF0) == 0xF0)
		{
			switch(d) 
			{
			case 0xF0: // Start of system exclusive
				pstatus->flags |= INSIDE_SYSEX;
				break;
			case 0xF7: // End of System Exclusive
				pstatus->flags &= ~INSIDE_SYSEX;
				break;
			}
		}
		// Other messages will be processed (as long
		// as they are not part of a sysex)
		else if(!(pstatus->flags & INSIDE_SYSEX))
		{
			handle_input_byte(d, pstatus);
		}
	}
}	

void init_status(INPUT_STATUS *pstatus)
{
	memset(pstatus,0,sizeof(pstatus));
}	

////////////////////////////////////////////////////////////
//
// MAIN
//
////////////////////////////////////////////////////////////
void main()
{ 	
	char i;
	byte slave_addr;

	// osc control / 16MHz / internal
	osccon = 0b01111010;

	// configure io
	trisa = 0b00100000;              	
    trisc = 0b00110000;   	
	ansela = 0b00000000;
	anselc = 0b00000000;
	porta=0;
	portc=0;

	wpua.5 = 1; // weak pullup on switch RA5
	option_reg.7 = 0;	// enable weak pull ups on port a

	init_status(&master_status);
	for(i=0; i<NUM_SLAVES; ++i) 
		init_status(&slave_status[i]);

	// initialise timer
	init_timer();
	
	// initialise MIDI comms
	init_usart();
	
	// initialise I2C master
	init_i2c();

	SIGNAL_LED1(INITIAL_PULSE);
	SIGNAL_LED2(INITIAL_PULSE);
	SIGNAL_LED3(INITIAL_PULSE);
	
	// this delay makes sure all the slaves have time to initialise
	delay_ms(100);

	// enable interrupts	
	intcon.7 = 1; //GIE
	intcon.6 = 1; //PEIE

	
	while(1) 
	{
		// ensure receive buffer overrun 
		// errors are cleared
		if(rcsta.1)
		{
			rcsta.4 = 0;
			rcsta.4 = 1;
		}

		// poll the master
		//master_run();

		// master sysex transmission blocks slaves
		//if(!(master_status.flags & INSIDE_SYSEX)) 
		{
			// round-robin the slaves
			for(i=0; i<NUM_SLAVES; ++i) 
//			for(i=0; i<1; ++i) 
			{			
//				if(slave_status[i].flags & IS_ONLINE)
					slave_run(SLAVE_I2C_ADDR|i, &slave_status[i]);
			}
		}
		// ensure any buffered realtime messages are flushed 
		//flush_tx_buffer();
	}

}



