/*************************************************************
*       at328-0.c - Demonstrate simple I/O functions of ATmega328
*
*       Program loops turning PC0 on and off as fast as possible.
*
* The program should generate code in the loop consisting of
*   LOOP:   SBI  PORTC,0        (2 cycles)
*           CBI  PORTC,0        (2 cycles)
*           RJMP LOOP           (2 cycles)
*
* PC0 will be low for 4 / XTAL freq
* PC0 will be high for 2 / XTAL freq
* A 9.8304MHz clock gives a loop period of about 600 nanoseconds.
*
* Revision History
* Date     Author      Description
* 09/14/12 A. Weber    Initial Release
* 11/18/13 A. Weber    Renamed for ATmega328P
*************************************************************/

#include <avr/io.h>
#include <util/delay.h>

void serial_init ( unsigned short ubrr ) {
UBRR0 = ubrr;// Set baud rate
UCSR0B |= (1 << TXEN0 ); // Turn on transmitter
UCSR0B |= (1 << RXEN0 ); // Turn on receiver
UCSR0C = (3 << UCSZ00 ); // Set for async . operation , no parity ,
// one stop bit , 8 data bits
}


void serial_out ( char ch )
{
while (( UCSR0A & (1<<UDRE0 )) == 0);
UDR0 = ch ;
}


char serial_in ()
{
	while ( !( UCSR0A & (1 << RXC0 )) );
	return UDR0 ;
}

int main(void)
{

    serial_init(63);
    unsigned char test = 'a';
    
    
    

	while(1){
	    char in = serial_in();
	    if(in == 'a' || in == 'e' || in == 'i' || in == 'o' || in == 'u'){
		serial_out('v');
		}
	    else{
		serial_out('n');
	}
	    
	}


    return 0;   /* never reached */
}