//
// HEADER FILES
//
#include <system.h>

// PIC CONFIG BITS
// - RESET INPUT DISABLED
// - WATCHDOG TIMER OFF
// - INTERNAL OSC
#pragma DATA _CONFIG1, _FOSC_INTOSC & _WDTE_OFF & _MCLRE_OFF &_CLKOUTEN_OFF
#pragma DATA _CONFIG2, _WRT_OFF & _PLLEN_OFF & _STVREN_ON & _BORV_19 & _LVP_OFF
#pragma CLOCK_FREQ 16000000

	
////////////////////////////////////////////////////////////
//
// MAIN
//
////////////////////////////////////////////////////////////
void main()
{ 	
	char i;

	// osc control / 16MHz / internal
	osccon = 0b01111010;

	for(;;);
}
	