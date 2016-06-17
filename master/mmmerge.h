////////////////////////////////////////////////////////////
//
// MIDI MERGE MODULE
//
// Common definitions for I2C master and slave
//
////////////////////////////////////////////////////////////
#define I2C_ADDR 			0x70	// slave address #0

#define UNDERFLOW_DATA		0xF9	// what will be sent in event of underflow
#define OVERFLOW_BIT		0x80	

#define STATUS_UNDERFLOW 	0x01	// master tried to read empty slave buffer
#define STATUS_BADCOMMAND 	0x02	// master sent a command slave did not recognise
#define STATUS_OVERFLOW 	0x04	// slave ran out of buffer space
#define STATUS_RECEIVING 	0x08	// serial port is enabled to read MIDI

#define CMD_DISABLE		1		// turn off MIDI reception
#define CMD_ENABLE		2		// turn on MIDI reception
#define CMD_CLEAR		3		// Clear the data buffer	
#define CMD_GETSTATUS	4		// Read the status byte and reset the status
#define CMD_IDENTIFY	5		// Return the id of this slave 
#define CMD_LEDACTIVITY	6		// LED indicates MIDI activity
#define CMD_LEDSIGNAL	7		// blink the LED once
#define CMD_LEDBLINK	8		// blink the LED repeatedly
