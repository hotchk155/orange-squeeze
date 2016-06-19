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
// 4:1 MIDI MERGE - MIDI TO I2C SLAVE MODULE
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
// INCLUDE FILES
//
#include <system.h>
#include "..\master\mmmerge.h"

//
// PIC12F1822 MCU CONFIG
//

// 16MHz internal oscillator block, reset disabled
#pragma DATA _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF &_CLKOUTEN_OFF
#pragma DATA _CONFIG2, _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_19 & _LVP_OFF
#pragma CLOCK_FREQ 16000000

//
// CONSTANTS
//

// define the I/O pins. Note that 
// PORTA.5 is the UART RX pin
#define P_LED		porta.0
#define P_IDPIN0	porta.4
#define P_IDPIN1	porta.3

// PORTA I/O tristate mask
#define TRIS_A 			0b00100110

// Define time periods (milliseconds)
#define LED_ACTIVITY_PERIOD		2		
#define LED_ERROR_PERIOD 		200
#define LED_SIGNAL_PERIOD 		200

// Timer settings
#define TIMER_0_INIT_SCALAR		5	// Timer 0 is an 8 bit timer counting at 250kHz

//
// TYPE DEFS
//

typedef unsigned char byte;

// Structure to hold a data buffer (for receiving serial MIDI
// and for sending it out to the master over I2C bus)
typedef struct {
	byte count;					// number of bytes in the buffer
	byte data[SZ_SLAVEDATA];	// data bytes
	byte error;					// error flag
} DATA_BUFFER;

//
// GLOBAL DATA
//

// Data buffers
// One buffer is receiving MIDI data
// while the other transmitting to I2C
volatile DATA_BUFFER Buffer1;
volatile DATA_BUFFER Buffer2;

// pointer to the current buffer for storing received MIDI bytes
volatile DATA_BUFFER *rx_buf;	

// pointer to the current buffer for sending data to master
volatile DATA_BUFFER *tx_buf;

// index into the transmit buffer data
volatile byte tx_index;

// ID of the slave 
volatile byte slave_id;

// used to time LED flashes
volatile int led_timeout;

// The master might send us a command byte
volatile byte master_command;

////////////////////////////////////////////////////////////
//
// INTERRUPT HANDLER 
//
////////////////////////////////////////////////////////////
void interrupt( void )
{
	//////////////////////////////////////////////////////////
	// I2C SLAVE INTERRUPT
	// Called when there is activity on the I2C bus
	if(pir1.3) 
	{	
		pir1.3 = 0; // clear interrupt flag
		if(!ssp1stat.D_NOT_A) // master has sent our slave address
		{
			byte d = ssp1buf; // read and discard address to clear BF flag
		
			// Is the master setting up a data READ?
			if(ssp1stat.R_NOT_W) 
			{ 		
				// switch the buffers, so that the current receive buffer 
				// data is ready to send to the master
				DATA_BUFFER *temp_buf = rx_buf;
				rx_buf = tx_buf;
				tx_buf = temp_buf;
				tx_index = 0;
				
				// clear the receive buffer ready for new MIDI data
				rx_buf->count = 0;
				rx_buf->error = 0;
				
				// now return the byte count to the master - this 
				// precedes the data byte. Set the top bit of the
				// count field if any error has occurred
				byte count = tx_buf->count;
				if(tx_buf->error) {
					count |= 0x80;
				}
				ssp1buf = count;
			}
			else 
			{
				// dummy data
				ssp1buf = 0;
			}
		}
		else // DATA 
		{ 
			if(!ssp1stat.R_NOT_W) // MASTER IS WRITING TO SLAVE
			{
				// Commands that the master might send
				switch(ssp1buf) {
					case CMD_START:	// Reset buffers and start receiving MIDI				
						rx_buf->count = 0;
						tx_buf->count = 0;
						tx_index = 0;						
						rcsta.7 = 1;
						P_LED = 1;
						led_timeout = LED_SIGNAL_PERIOD;
						break;
					case CMD_STOP:	// Stop receiving MIDI
						rcsta.7 = 0;
						break;
				}
			}
			else // MASTER IS READING FROM SLAVE
			{ 						
				ssp1con1.WCOL = 0; // clear write collision bit
				if(tx_index < tx_buf->count) 
				{
					// return the next data byte
					ssp1buf = tx_buf->data[tx_index++];
				}
				else {
					// buffer underflow.. should not happen!
					ssp1buf=0; 
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
		if(rx_buf->count < SZ_SLAVEDATA - 1) {
			rx_buf->data[rx_buf->count++] = rcreg;
			P_LED = 1;
			if(led_timeout < LED_ACTIVITY_PERIOD) {
				led_timeout = LED_ACTIVITY_PERIOD;
			}

		}
		else {
			// buffer overflow
			P_LED = 1;
			rx_buf->error = 1;
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

	rcsta.7 = 0;	// SPEN 	serial port initially disabled
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
	master_command = 0;
	
	// configure io
	trisa = TRIS_A;              	
	ansela = 0b00000000;
	porta=0;
	apfcon.7=1; // RX on RA5
	apfcon.2=1;	// TX on RA4

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
		// clear receive buffer overruns
		if(rcsta.1)
		{
			rcsta.4 = 0;
			rcsta.4 = 1;
		}
	}
}

//
// END
// 