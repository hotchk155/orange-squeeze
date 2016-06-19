////////////////////////////////////////////////////////////
//
//           ///    //// /////  /////   /////   ////
//         //  // //        // //  // //   // //  //
//        //  // //     ///// //  //  ////// //////
//       //  // //    //  // //  //      // //
//        ////  //     ///// //  //   /////  /////   
//
//      ////  ///// //  //  ////   //// //////   //// 
//    //    //  // //  // //  // //  //    //  //  //
//    ////  ///// //  // ////// //////   //   //////
//      //    // //  // //     //      //    //
//  ////     /// /////  /////  ////  //////  /////
//
// 4:1 MIDI MERGE - MIDI TO I2C MASTER MODULE
// 2016/hotchk155         Sixty Four Pixels Limited
// 
// This code licensed under terms of creative commons BY-NC-SA
// http://creativecommons.org/licenses/by-nc-sa/4.0/
//
// Code for PIC12F1822, Compiled with SourceBoost C
//
// History
// Ver  Date 		Desc	
// 1    18/6/16		Initial version
//
#define FIRMWARE_VERSION 	1
////////////////////////////////////////////////////////////

//
// HEADER FILES
//
#include <system.h>
#include <memory.h>
#include "mmmerge.h"

// 
// PIC12F1822 CONFIG BYTES
//

// - RESET INPUT DISABLED
// - WATCHDOG TIMER OFF
// - INTERNAL OSC
#pragma DATA _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF &_CLKOUTEN_OFF
#pragma DATA _CONFIG2, _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_19 & _LVP_OFF
#pragma CLOCK_FREQ 16000000

//
// CONSTANTS
//

// io pins
#define P_LED1		latc.2 	// MIDI input red LED
#define P_LED2		latc.3 	// Blue LED
#define P_LED3		lata.2 	// Yellow LED
#define P_SWITCH	porta.5

// io direction bits
#define TRIS_A	0b00100000
#define TRIS_C	0b00110000

#define SLAVE_I2C_ADDR 		0x70	// slave address #0


#define TIMER_0_INIT_SCALAR		5	// Timer 0 is an 8 bit timer counting at 250kHz

#define SYSEX_TIMEOUT 1000	// Timeout (ms) to prevent an unfinished sysex from blocking system

#define SZ_RXBUFFER 64
#define RXBUFFER_INDEX_MASK 0x3F

#define SZ_TXBUFFER 64
#define TXBUFFER_INDEX_MASK 0x3F

#define INITIAL_PULSE				200
#define MIDI_INPUT_PULSE			1
#define MIDI_INPUT_OVERFLOW			200
#define MIDI_OUTPUT_PULSE			1

// macros used to signal the 3 LEDs
#define LED_1_SIGNAL(t) { P_LED1 = 1; if(t>led_1_timeout) led_1_timeout = t; }
#define LED_2_SIGNAL(t) { P_LED2 = 1; if(t>led_2_timeout) led_2_timeout = t; }
#define LED_3_SIGNAL(t) { P_LED3 = 1; if(t>led_3_timeout) led_3_timeout = t; }


#define SZ_DATACHUNK 64			// How many bytes to process on each pass 

#define NUM_SLAVES 3

//
// TYPE DEFINITIONS
//
typedef unsigned char byte;

typedef struct 
{
	byte enabled;				// TRUE if this slave exists
	byte running_status;		// MIDI running status for this slave
	byte is_system_common_msg;	// TRUE if this is not really running status but is system common msg
	byte param[2];				// MIDI parameters
	byte num_params;			// Number of parameters
	byte param_index;			// Current parameter (0..1)	
} INPUT_STATUS;

//
// GLOBAL DATA
//
volatile byte rx_buffer[SZ_RXBUFFER];
volatile byte rx_head = 0;
volatile byte rx_tail = 0;
volatile byte tx_buffer[SZ_TXBUFFER];
volatile byte tx_head = 0;
volatile byte tx_tail = 0;
volatile int sysex_timer = 0;
volatile INPUT_STATUS slave_status[NUM_SLAVES];
volatile INPUT_STATUS master_status;
volatile byte out_running_status;
volatile int led_1_timeout;
volatile int led_2_timeout;
volatile int led_3_timeout;

////////////////////////////////////////////////////////
//
// INTERRUPT HANDLER
//
////////////////////////////////////////////////////////
void interrupt( void )
{
	char next;	

	////////////////////////////////////////////////
	// SERIAL PORT RECEIVE INTERRUPT
	// Fires when traffic arrives at the primary 
	// MIDI port
	if(pir1.5)
	{			
		// read the value to clear rc status
		byte b = rcreg;		
		pir1.5 = 0;
		
		next = rx_head + 1;
		next &= RXBUFFER_INDEX_MASK;
		if(next == rx_tail) {
			// full receive buffer! data will be dropped
			LED_1_SIGNAL(MIDI_INPUT_OVERFLOW);
			LED_3_SIGNAL(MIDI_INPUT_OVERFLOW);
		}
		else {
			rx_buffer[rx_head] = b;
			rx_head = next;
			LED_1_SIGNAL(MIDI_INPUT_PULSE);
		}			
	}		
	
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
		if(sysex_timer) {
			--sysex_timer;
		}
		intcon.2 = 0;		
	}
	
	
	////////////////////////////////////////////////
	// SERIAL PORT TRANSMIT INTERRUPT
	// TXIF bit is high whenever there is no serial
	// transmit in progress. Low->High transition will
	// trigger an interrupt, meaning characters of the 
	// transmit buffer can be sent back to back
	if(pir1.4) 
	{	
		// more info in the receive fifo?
		if(tx_head != tx_tail) {
		
			// send next character
			txreg = tx_buffer[tx_tail];
			++tx_tail;
			tx_tail &= TXBUFFER_INDEX_MASK;
			LED_2_SIGNAL(MIDI_OUTPUT_PULSE);
		}
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

	rcsta.7 = 0;	// SPEN 	serial port initiall disabled
	rcsta.6 = 0;	// RX9 		8 bit operation
	rcsta.5 = 1;	// SREN 	enable receiver
	rcsta.4 = 1;	// CREN 	continuous receive enable
		
	spbrgh = 0;		// brg high byte
	spbrg = 31;		// brg low byte (31250)	
	
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
byte i2c_send(byte data) 
{
	ssp1buf = data;
	while((ssp1con2 & 0b00011111) || // SEN, RSEN, PEN, RCEN or ACKEN
		(ssp1stat.2)); // data transmit in progress	
	return !ssp1con2.6; // slave ack
}

////////////////////////////////////////////////////////////
//
// BEGIN I2C MASTER WRITE TRANSACTION
//
////////////////////////////////////////////////////////////
byte i2c_begin_write(byte address) 
{
	pir1.3 = 0; // clear SSP1IF
	ssp1con1.7 = 0; // clear WCOL
	ssp1con2.0 = 1; // signal start condition
	while(!pir1.3); // wait for it to complete
	return i2c_send(address<<1); // address + WRITE(0) bit
}

////////////////////////////////////////////////////////////
//
// END AN I2C MASTER WRITE TRANSACTION
//
////////////////////////////////////////////////////////////
void i2c_end_write() 
{
	pir1.3 = 0; // clear SSP1IF
	ssp1con2.2 = 1; // signal stop condition
	while(!pir1.3); // wait for it to complete
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
// CONTINUE I2C MASTER READ 
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

////////////////////////////////////////////////////////////
//
// END I2C MASTER READ TRANSACTION
//
////////////////////////////////////////////////////////////
void i2c_end_read() 
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
// SEND A COMMAND TO AN I2C SLAVE
//
////////////////////////////////////////////////////////////
byte slave_command(byte addr, byte cmd) 
{
  byte result = i2c_begin_write(addr);
  i2c_send(cmd);
  i2c_end_write();
  return result;
}

////////////////////////////////////////////////////////////
//
// QUEUE UP A BYTE FOR TRANSMISSION
//
////////////////////////////////////////////////////////////
void transmit(byte d) {

	// calculate next buffer head position
	byte next_head = tx_head + 1;
	next_head &= TXBUFFER_INDEX_MASK;
	
	// if the output buffer is full, we'll need to wait..
	while(next_head == tx_tail);
				
	// add the byte to the output buffer
	tx_buffer[tx_head] = d;
	tx_head = next_head;	
}

////////////////////////////////////////////////////////////
//
// PROCESS AN INCOMING MIDI BYTE (MASTER OR SLAVE)
// Keeps record of any partial MIDI messages that have 
// been received and when a full message is ready it is 
// sent to the MIDI output
//
////////////////////////////////////////////////////////////
void handle_input_byte(byte d, INPUT_STATUS *pstatus, byte *in_sysex)
{	
	// is this a MIDI command/status byte?
	if(d & 0x80) 
	{
		// set up the new command 
		pstatus->running_status = d;	
		pstatus->param_index = 0;			
		
		// assume not system common for now...							
		pstatus->is_system_common_msg = 0; 
		
		// handle command/status byte
		switch(d & 0xF0)
		{
			case 0xF0:
				switch(d) {
					////////////////////////////////////////////
					case 0xF0: // start of sysex
						*in_sysex = 1;
						pstatus->running_status = 0;
						transmit(d);							
						break;
					case 0xF7: // end of sysex
						*in_sysex = 0;
						pstatus->running_status = 0;
						transmit(d);							
						break;
					////////////////////////////////////////////
					case 0xF1: // MTC quarter frame
					case 0xF3: // song select
						*in_sysex = 0;
						pstatus->running_status = d;
						pstatus->is_system_common_msg = 1;
						pstatus->num_params = 1;
						break;
					////////////////////////////////////////////
					case 0xF2: // Song position pointer
						*in_sysex = 0;
						pstatus->running_status = d;
						pstatus->is_system_common_msg = 1;
						pstatus->num_params = 2;
						break;
					////////////////////////////////////////////
					case 0xF4: // reserved
					case 0xF5: // reserved
					case 0xF6: // tune request
						// System common message without params will cancel running and sysex status
						*in_sysex = 0;
						pstatus->running_status = 0;
						transmit(d);							
						break;
					////////////////////////////////////////////
					case 0xF8: // MIDI clock tick
					case 0xF9: // reserved
					case 0xFA: // Start
					case 0xFB: // Continue
					case 0xFC: // Stop
					case 0xFD: // Reserved
					case 0xFE: // Active sensing
					case 0xFF: // System reset
						// Realtime message - does not cancel running or sysex status
						transmit(d);							
						break;
				}
				break;
			case 0xA0: //  Aftertouch  1  key  touch  
			case 0xC0: //  Patch change  1  instrument #   
			case 0xD0: //  Channel Pressure  1  pressure  
				*in_sysex = 0; 					// cancel sysex status
				pstatus->running_status = d; 	// sets running status
				pstatus->num_params = 1;		// single param
				pstatus->param_index = 0;										
				break;    
			case 0x80: //  Note-off  2  key  velocity  
			case 0x90: //  Note-on  2  key  veolcity  
			case 0xB0: //  Continuous controller  2  controller #  controller value  
			case 0xE0: //  Pitch bend  2  lsb (7 bits)  msb (7 bits)  
			default:
				*in_sysex = 0; 					// cancel sysex status
				pstatus->running_status = d;	// sets running status
				pstatus->num_params = 2;		// single param								
				pstatus->param_index = 0;		
				break;        
		}
	}
	// 7-bit data during sysex transmission
	else if(*in_sysex) 
	{
		transmit(d);
		sysex_timer = SYSEX_TIMEOUT; 
	}
	// parameters for a MIDI message?
	else if(pstatus->running_status)
	{
		// store the parameter
		pstatus->param[pstatus->param_index++] = d;
		
		// do we have all the parameters for the message?
		if(pstatus->param_index >= pstatus->num_params)
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
			
			// System common messages don not use a running status and 
			// cancel any existing running status (but we try to make life 
			// easier for ourselves by use common running status handling 
			// to process them!)
			if(pstatus->is_system_common_msg) {
				pstatus->is_system_common_msg = 0;
				pstatus->running_status = 0;
			}
		}
	}
}


////////////////////////////////////////////////////////////
//
// RUN MASTER INPUT
//
////////////////////////////////////////////////////////////
void master_run()
{
	byte in_sysex = 0;
	do {
		for(byte count = 0; count < SZ_DATACHUNK; ++count) {
			if(rx_tail == rx_head) {
				break; // input buffer is empty
			}
			handle_input_byte(rx_buffer[rx_tail], &master_status, &in_sysex);
			++rx_tail;
			rx_tail&=RXBUFFER_INDEX_MASK;
		}
	} while(in_sysex && sysex_timer);
}

////////////////////////////////////////////////////////////
//
// RUN SLAVE INPUT
//
////////////////////////////////////////////////////////////
void slave_run(byte addr, INPUT_STATUS *pstatus)
{
	byte in_sysex = 0;
	do {
	
		// get count of bytes remaining
		byte count = i2c_begin_read(addr);				
		if(count & 0x80) {
			LED_3_SIGNAL(MIDI_INPUT_OVERFLOW);
			count &= 0x7F;
		}
		
		// read the data over I2C
		for(byte i=0; i<count; ++i) {
			byte d = i2c_continue_read();
			switch(d) {
				case 0xF8: // CLOCK TICK is filtered out
					break;
				default:
					handle_input_byte(d, pstatus, &in_sysex);
					break;
			}
		}
		i2c_end_read();	
	} while (in_sysex && sysex_timer);
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

	// initialise timer
	init_timer();
	
	// initialise MIDI comms
	init_usart();
	
	// initialise I2C master
	init_i2c();

	for(;;) {
	
		// disable interrupts	
		intcon.7 = 0; //GIE
		intcon.6 = 0; //PEIE
	
		// Initialise status
		out_running_status = 0;
		memset(&master_status,0,sizeof(INPUT_STATUS));
		for(i=0; i<NUM_SLAVES; ++i) {
			memset(&slave_status[i],0,sizeof(INPUT_STATUS));
		}
		led_1_timeout = 0;
		led_2_timeout = 0;
		led_3_timeout = 0;
	
		// LED test pulses
		P_LED1 = 1;
		P_LED2 = 1;
		P_LED3 = 1;
		
		// this delay makes sure all the slaves have time to initialise
		delay_ms(100);
	
		P_LED1 = 0;
		P_LED2 = 0;
		P_LED3 = 0;
	
		// enable interrupts	
		intcon.7 = 1; //GIE
		intcon.6 = 1; //PEIE
	
		// Reset each of the slaves
		for(i=0; i<NUM_SLAVES; ++i) {
			if(slave_command(SLAVE_I2C_ADDR|i, CMD_START)) {
				slave_status[i].enabled = 1;
			}
		}
	
		// Enable local MIDI receive
		rcsta.7 = 1;	
	
		// service the inputs in turn until the world ends
		for(;;)
		{
			// unblock the serial input if there is an overflow
			if(rcsta.1)	{
				rcsta.4 = 0;
				rcsta.4 = 1;
			}
			
			// run the master
			master_run();	
			
			// and each of the slaves in turn	
			for(byte i=0; i < NUM_SLAVES; ++i) {
				if(slave_status[i].enabled) {
					slave_run(SLAVE_I2C_ADDR|i, &slave_status[i]);
				}
			}				
			
			// reset if switch is pressed
			if(!P_SWITCH) 
				break;
		}
	}
}

//
// END
//