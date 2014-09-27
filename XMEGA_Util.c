/*
* XMEGA_Serial_PROGMEM
 * XMEGA_Util.c
 * 
 * MIT LICENSE: Please be sure to leave the Author information and attribute the work publicly
 * not asking much for a little recognition for time spent
 *
 * Created: 9/26/2014 7:11:32 PM
 *  Author: Jacob Pagel
 */ 


#include <avr/io.h>
#include <stdio.h>
#include <util/delay.h>

#define F_CPU 32000000

#include <XMEGA_CLKSET.h>
#include <XMEGA_USART0.h>
#include <XMEGA_ANALOG.h>
#include "ProgMemData.h"
//#include <xmega_Freq_Set.h>
//#include <xmega_Serial_USART0.h>

int main(void)
{
	char buffer[10];
	
	set32MhzClock();
    //configureSerial2M();
	SerialBegin(BAUD_460800);
	setupADC();
	
		PORTA.DIRSET &= ~(1<<3) | (1<<4);
		PORTB.DIRSET = PIN7_bm;
		PORTC.DIRSET |= (1<<3) | (1<<0);
		PORTC.DIRSET &= ~(1<<2);
	
	for(;;)
	{
		uint16_t voltage = analogreadPA4();		
		sprintf(buffer, "%d\n\r", voltage + 11);
		sendString(buffer);
		PORTB.OUTTGL = PIN7_bm;
		_delay_ms(100);
		
		
	}
	
}