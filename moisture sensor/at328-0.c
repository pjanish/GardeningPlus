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


void serial_string( char *string){

	int q = 0;
	while(string[q] != '\0')
	{
		serial_out(string[q]);
		q++;
	}
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


void set_up(){
	DDRD |= (1 << PD6);          /* set pin to output */ //PC2 -> D6
	PORTD &= ~(1 << PD6);		/* set to low pin */
}

void power_on(){
	PORTD |= (1 << PD6);		/* set to high pin */
	_delay_ms(10);
}

void power_off(){
	_delay_ms(10);
	PORTD &= ~(1 << PD6);		/* set to high pin */
}
int read_pin(){
	int x;

	DDRC &= ~(1 << PC3);          /* set pin to input */
	ADMUX |= (1 << REFS0 ); // Set reference to AVCC
	ADMUX &= ~(1 << REFS1 );
	ADMUX |= (1 << ADLAR ); // Left adjust the output
	ADMUX |= (3 << MUX0 ); // Select the channel
	ADMUX &= (0xf0 | (3 << MUX0 ));
	ADCSRA |= (7 << ADPS0 ); // Set the prescalar to 128
	ADCSRA |= (1 << ADEN ); // Enable the ADC

	while (1) {
	ADCSRA |= (1 << ADSC ); // Start a conversion
	while ( ADCSRA & (1 << ADSC )); // wait for conversion complete
	x = ADCH ; // Get converted value
	return x;
	}
}



void decimal_to_string( int decimal_val){
	char other[4];
	snprintf(other, 4, "%d", decimal_val);
	serial_string(other);
	
}

	
 

int main(void)
{
	
    serial_init(63);

	
	int moisture_value;


	set_up();
	

	while(1){
		power_on();
		moisture_value = read_pin();
		decimal_to_string(moisture_value);
		power_off();
		serial_out(' ');
		_delay_ms(5000);
		
	}


    return 0;   /* never reached */
}