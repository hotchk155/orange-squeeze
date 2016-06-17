////////////////////////////////////////////////////////////
//
// MIDI MERGE MODULE
//
// MIDI INPUT SLAVE CONTROLLER FIRMWARE
//
// Code for PIC12F1822
// Compiled with SourceBoost C
//
// 2016/hotchk155
//
// History
// Ver  Date 	Desc
//   1  Mar16
//
#define FIRMWARE_VERSION 	1
////////////////////////////////////////////////////////////

#include <system.h>
#include "..\master\mmmerge.h"

// 16MHz internal oscillator block, reset disabled
#pragma DATA _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF &_CLKOUTEN_OFF
#pragma DATA _CONFIG2, _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_19 & _LVP_OFF
#pragma CLOCK_FREQ 16000000

typedef unsigned char byte;

// define the I/O pins. Note that 
// PORTA.5 is the UART RX pin
#define P_LED		porta.0
#define P_IDPIN0	porta.4
#define P_IDPIN1	porta.3

// PORTA I/O tristate mask
#define TRIS_A 			0b00100110

#define FLAG_OVERFLOW			0x01

// Define special LED override modes
#define LED_OVERRIDE_SIGNAL		1	// the LED will blink once then return to normal mode
#define LED_OVERRIDE_BLINK		2	// the LED will blink repeatedly

// Define time periods (milliseconds)
#define LED_ACTIVITY_PERIOD		2		// LED on when MIDI data received 	
#define LED_SIGNAL_PERIOD 		200		// LED on when master requests a "signal" flash
#define LED_OVERFLOW_PERIOD 	1000	// LED on when buffer overflow happens
#define LED_BLINK_PERIOD_ON		50		// LED on when blinking in error
#define LED_BLINK_PERIOD_OFF	100		// LED off when blinking in error

// Timer settings
#define TIMER_0_INIT_SCALAR		5	// Timer 0 is an 8 bit timer counting at 250kHz

// Timer ticked flag (set once per ms, main code path needs to reset it)
volatile byte timer_ticked = 0;		

// ID of the slave 
volatile byte slave_id = 0;

// slave status
volatile byte slave_status = 0;
volatile byte slave_command = 0;
volatile byte slave_flags = 0;

// The MIDI data buffer
#define SZ_RXBUFFER	64
#define MASK_RXBUFFER_INDEX	0x3F
volatile byte rx_buffer[SZ_RXBUFFER];
volatile byte rx_head = 0;
volatile byte rx_tail = 0;
volatile byte rx_count = 0;

// number of milliseconds till LED state changes (for blinking etc)
volatile int led_timeout = 0;
volatile int led_override = 0;

////////////////////////////////////////////////////////////
//
// INTERRUPT HANDLER 
//
////////////////////////////////////////////////////////////
void interrupt( void )
{
	byte d;

	//////////////////////////////////////////////////////////
	// I2C SLAVE INTERRUPT
	// Called when there is activity on the I2C bus
	if(pir1.3) 
	{	
		if(!ssp1stat.5) // ADDRESS
		{
			d = ssp1buf; // read and discard address to clear BF
		
			if(ssp1stat.2) // MASTER IS READING FROM SLAVE
			{ 						
				switch(slave_command) 
				{
					case CMD_GETSTATUS:
						ssp1buf = slave_status; // send the status byte
						break;
					case CMD_IDENTIFY:
						// place special identity string in the data buffer
						rx_tail = 0;
						rx_count = 0;
						rx_buffer[rx_count++] = 'M';
						rx_buffer[rx_count++] = 'S';;
						rx_buffer[rx_count++] = 'L';;
						rx_buffer[rx_count++] = 'V';;
						rx_buffer[rx_count++] = 'A' + slave_id;
						rx_head = rx_count;					
						ssp1buf = FIRMWARE_VERSION; // send firmware version
						break;
					default:
						d = rx_count;
						if(slave_flags & FLAG_OVERFLOW)
						{
							slave_flags &= ~FLAG_OVERFLOW;
							d |= OVERFLOW_BIT;
						}
						ssp1buf = d; // send the number of bytes available
						break;
				}
				slave_command = 0;
			}
		}
		else // DATA 
		{ 
			if(!ssp1stat.2) // MASTER IS WRITING TO SLAVE
			{
				d = ssp1buf; // read byte
				slave_command = 0;
				switch(d) 
				{
					case CMD_GETSTATUS:
					case CMD_IDENTIFY:
						slave_command = d;
						break;
					case CMD_ENABLE:
						rcsta.7 = 1;	// serial port enable
						slave_status |= STATUS_RECEIVING;
						break;
					case CMD_DISABLE:
						rcsta.7 = 0;	// serial port disable
						slave_status &= ~STATUS_RECEIVING;
						// Fall through to clear
					case CMD_CLEAR:
						rx_head = 0;
						rx_tail = 0;
						rx_count = 0;
						break;						
					// SLAVE LED SET TO NORMAL MODE (show MIDI activity)
					case CMD_LEDACTIVITY:
						led_override = 0;
						led_timeout = 0;
						P_LED = 0;
						break;
					// SLAVE LED SET TO BLINK ONCE
					case CMD_LEDSIGNAL:
						led_override = LED_OVERRIDE_SIGNAL;
						led_timeout = LED_SIGNAL_PERIOD;
						P_LED = 1;
						break;
					case CMD_LEDBLINK:
						led_override = LED_OVERRIDE_BLINK;
						led_timeout = LED_BLINK_PERIOD_ON;
						P_LED = 1;
						break;
				}
			}
			else // MASTER IS READING FROM SLAVE
			{ 						
				ssp1con1.7 = 0; // clear write collision bit
				if(rx_count <= 0) 
				{
					// data underflow
					ssp1buf = UNDERFLOW_DATA;	
					slave_status |= STATUS_UNDERFLOW;					
				}
				else 
				{
					// send the next byte
					ssp1buf = rx_buffer[rx_tail];	
					rx_tail = (rx_tail+1)&MASK_RXBUFFER_INDEX;
					--rx_count;
				}
			}
		}
		ssp1con1.4 = 1; // release clock
		pir1.3 = 0; // clear interrupt flag
	}

	//////////////////////////////////////////////////////////
	// SERIAL RX INTERRUPT
	if(pir1.5)
	{

		// get the byte
		d = rcreg;
		
		// calculate the next buffer position
		byte next_head = (rx_head+1)&MASK_RXBUFFER_INDEX;
		if(next_head == rx_tail) {
			// the buffer is full! byte will be lost
			slave_flags |= FLAG_OVERFLOW;
			slave_status |= STATUS_OVERFLOW;
			
			// signal overflow
			if(!led_override) {
				led_timeout = LED_OVERFLOW_PERIOD;
				P_LED = 1;
			}
		}
		else {
			// put the byte in the buffer
			rx_buffer[rx_head] = d;
			rx_head = next_head;
			++rx_count;
			
			// signal activity
			if(!led_override) {
				led_timeout = LED_ACTIVITY_PERIOD;
				P_LED = 1;
			}
		}
		pir1.5 = 0;		
	}
	
	//////////////////////////////////////////////////////////
	// TIMER0 OVERFLOW
	// Timer 0 overflow is used to create a once per millisecond
	// signal for blinking LEDs etc
	if(intcon.2)
	{
		tmr0 = TIMER_0_INIT_SCALAR;
		timer_ticked = 1;
		intcon.2 = 0;
	}		
}

////////////////////////////////////////////////////////////
//
// SERIAL PORT INITIALISATION
//
////////////////////////////////////////////////////////////
void uart_init()
{
	pir1.1 = 0;		//TXIF 		
	pir1.5 = 0;		//RCIF
	
	pie1.1 = 0;		//TXIE 		no interrupts
	pie1.5 = 1;		//RCIE 		enable
	
	baudcon.4 = 0;	// SCKP		synchronous bit polarity 
	baudcon.3 = 1;	// BRG16	enable 16 bit brg
	baudcon.1 = 0;	// WUE		wake up enable off
	baudcon.0 = 0;	// ABDEN	auto baud detect
		
	txsta.6 = 0;	// TX9		8 bit transmission
	txsta.5 = 0;	// TXEN		transmit enable
	txsta.4 = 0;	// SYNC		async mode
	txsta.3 = 0;	// SEDNB	break character
	txsta.2 = 0;	// BRGH		high baudrate 
	txsta.0 = 0;	// TX9D		bit 9

	rcsta.7 = 1;	// SPEN 	
	//rcsta.7 = 0;	// SPEN 	serial port initially disabled
	rcsta.6 = 0;	// RX9 		8 bit operation
	rcsta.5 = 1;	// SREN 	enable receiver
	rcsta.4 = 1;	// CREN 	continuous receive enable
		
	spbrgh = 0;		// brg high byte
	spbrg = 31;		// brg low byte (31250)	
	
}

////////////////////////////////////////////////////////////
//
// I2C SLAVE INITIALISATION
//
////////////////////////////////////////////////////////////
void i2c_init(byte addr) 
{
	//ssp1stat.7 = 0; // slew rate control disabled on high speed i2c mode
	
	ssp1con1.3 = 0; // }
	ssp1con1.2 = 1; // } I2C slave mode
	ssp1con1.1 = 1; // } with 7 bit address
	ssp1con1.0 = 0; // }
	
	ssp1con2.0 = 1; // enable clock stretching by slave
	
	//ssp1con3.6 = 0; // stop detection condition interrupt disabled
	//ssp1con3.5 = 0; // start detection condition interrupt disabled
	//ssp1con3.4 = 0; // ssp1bug only updated if ssp1ov is clear
	//ssp1con3.3 = 0; // 100ns min hold time on SDA after falling SCL
	//ssp1con3.2 = 0; // slave bus collision interrupt disabled
	//ssp1con3.1 = 0; // address hold by slave disabled
	//ssp1con3.0 = 0; // data hold by slave disabled
	
	ssp1msk = 0b01111111;	// address mask bits 0-6
	ssp1add = addr<<1;	// set slave address

	pie1.3 = 1;	// SSP1IE
	pir1.3 = 0; // SSP1IF
	
	ssp1con1.5 = 1; // enable i2c

}

////////////////////////////////////////////////////////////
//
// MILLISECOND TIMER INITIALISATION
//
////////////////////////////////////////////////////////////
void timer_init() 
{
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
}

////////////////////////////////////////////////////////////
//
// READ SLAVE ID (0,1 or 2 based on pins tied high and low)
//
////////////////////////////////////////////////////////////
byte get_slave_id() {
	byte id = 0;
	if(!!(P_IDPIN0))
		id |= 1;
	if(!!(P_IDPIN1))
		id |= 2;
	return id;
}

////////////////////////////////////////////////////////////
//
// MAIN ENTRY POINT
//
////////////////////////////////////////////////////////////
void main()
{ 
	// osc control / 16MHz / internal
	osccon = 0b01111010;
	
	// configure io
	trisa = TRIS_A;              	
	ansela = 0b00000000;
	porta=0;
	apfcon.7=1; // RX on RA5
	apfcon.2=1;	// TX on RA4

	// enable interrupts
	intcon.7 = 1;
	intcon.6 = 1;

	// initialise USART
	uart_init();

	// Get the id of this chip
	slave_id = get_slave_id();
	
	// initialise i2c
	i2c_init(I2C_ADDR|slave_id);
	
	// start millisecond timer
	timer_init(); 

	// Idle loop is used to control signal LED...
	// All the MIDI and I2C comms are handled
	// by interrupts
	for(;;)
	{	
		// wait for next ms
		while(!timer_ticked);
		timer_ticked = 0;

		// pending LED event?
		if(led_timeout) {
				
			// check if the event is due
			if(--led_timeout == 0) {
			
				// is LED blinking?
				if(LED_OVERRIDE_BLINK == led_override) {
				
					// Toggle the LED state
					if(!P_LED) {
						P_LED = 1;
						led_timeout = LED_BLINK_PERIOD_ON;
					}
					else {
						P_LED = 0;
						led_timeout = LED_BLINK_PERIOD_OFF;
					}
				}
				else {
					// timeout expired, turn off LED
					P_LED = 0;
					led_override = 0;
				}
			}
		}
	}
}