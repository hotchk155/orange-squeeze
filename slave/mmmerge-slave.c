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


// Define time periods (milliseconds)
#define LED_ACTIVITY_PERIOD		2		
#define LED_ERROR_PERIOD 		1000	
#define LED_SIGNAL_PERIOD 		200

// Timer settings
#define TIMER_0_INIT_SCALAR		5	// Timer 0 is an 8 bit timer counting at 250kHz


// ID of the slave 
volatile byte slave_id;

// slave status
//volatile byte note = 1;
//volatile byte ms = 0;

volatile int led_timeout;

// The MIDI data buffer
#define SZ_DATABUFFER 50

typedef struct {
	byte count;
	byte data[SZ_DATABUFFER];	
} DATA_BUFFER;

// One buffer is receiving MIDI data
// while the other transmitting to I2C
volatile DATA_BUFFER Buffer1;
volatile DATA_BUFFER Buffer2;

volatile DATA_BUFFER *rx_buf;
volatile DATA_BUFFER *tx_buf;
volatile byte tx_index;

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
		pir1.3 = 0; // clear interrupt flag
		if(!ssp1stat.D_NOT_A) // ADDRESS
		{
			d = ssp1buf; // read and discard address to clear BF
		
			if(ssp1stat.R_NOT_W) // MASTER IS READING FROM SLAVE
			{ 		
				// switch the buffers
				DATA_BUFFER *temp_buf = rx_buf;
				rx_buf = tx_buf;
				tx_buf = temp_buf;
				
				// clear the receive buffer ready for new data
				rx_buf->count = 0;
				
				// send the count of characters to master
				ssp1buf = tx_buf->count;
				tx_index = 0;
			}
		}
		else // DATA 
		{ 
			if(!ssp1stat.R_NOT_W) // MASTER IS WRITING TO SLAVE
			{
				d = ssp1buf; // read byte
			}
			else // MASTER IS READING FROM SLAVE
			{ 						
				ssp1con1.WCOL = 0; // clear write collision bit
				if(tx_index < tx_buf->count) 
				{
					ssp1buf = tx_buf->data[tx_index++];
				}
				else {
					ssp1buf=0x00; // buffer underflow should not happen!
				}
			}
		}
		ssp1con1.CKP = 1; // release clock
	}

	//////////////////////////////////////////////////////////
	// SERIAL RX INTERRUPT
	if(pir1.5)
	{
		// place the received MIDI byte in the receive buffer;
		if(rx_buf->count < SZ_DATABUFFER - 1) {
			rx_buf->data[rx_buf->count++] = rcreg;
			P_LED = 1;
			if(led_timeout < LED_ACTIVITY_PERIOD) {
				led_timeout = LED_ACTIVITY_PERIOD;
			}

		}
		else {
			P_LED = 1;
			led_timeout = LED_ERROR_PERIOD;
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
		if(led_timeout) {
			if(--led_timeout == 0) {
				P_LED = 0;
			}
		}
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
	
	//ssp1con2.SEN = 1; // enable clock stretching by slave
	
	//ssp1con3.6 = 0; // stop detection condition interrupt disabled
	//ssp1con3.5 = 0; // start detection condition interrupt disabled
	//ssp1con3.4 = 0; // ssp1buf only updated if ssp1ov is clear
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

	// prepare buffers
	rx_buf = &Buffer1;
	tx_buf = &Buffer2;
	tx_index = 0;

	// configure io
	trisa = TRIS_A;              	
	ansela = 0b00000000;
	porta=0;
	apfcon.7=1; // RX on RA5
	apfcon.2=1;	// TX on RA4

	P_LED = 1;
	led_timeout = LED_SIGNAL_PERIOD;

	// Get the id of this chip
	slave_id = get_slave_id();
	
	// initialise i2c
	i2c_init(I2C_ADDR|slave_id);
	
	// start millisecond timer
	timer_init(); 

	// initialise USART
	uart_init();

	// global enable interrupts
	intcon.7 = 1;
	intcon.6 = 1;

	// Idle loop is used to control signal LED...
	// All the MIDI and I2C comms are handled
	// by interrupts
	for(;;)
	{	
		if(rcsta.1)
		{
			rcsta.4 = 0;
			rcsta.4 = 1;
		}
	}
}