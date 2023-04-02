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

void Request()			/* Microcontroller send request */
{
	DDRD |= (1 << PD2);          /* set pin to output */
	PORTD &= ~(1 << PD2);		/* set to low pin */
	_delay_ms(18);              /* delay */
	DDRD &= ~(1 << PD2);		/* set pin to input */

	while((PIND & (1 << PIND2)) == 0); /* check pin to see if low */
}

void Response()			/* Receive response from DHT11 */
{
	while((PIND & (1 << PIND2)) != 0); /* check pin to see if high */
	while((PIND & (1 << PIND2)) == 0); /* check pin to see if low */
	while((PIND & (1 << PIND2)) != 0); /* check pin to see if high */
}

int Receive_data()		/* Receive data */
{
	int q,c=0;

	int c1, c2, c3, c4, c5, c6, c7, c8;	
	for (q=0; q<8; q++)
	{
		while((PIND & (1 << PIND2)) == 0);/* check received bit 0 or 1 */
		_delay_us(50);
		if ( (PIND & (1 << PIND2)) != 0) {
		c = (c<<1)|(0x01); 
		} else {
		c = (c<<1); /* otherwise its logic LOW  */
		}
		while((PIND & (1 << PIND2)) != 0);

	}
	return c;
}


void decimal_to_string( int decimal_val){
	char other[4];
	snprintf(other, 4, "%d", decimal_val);
	serial_string(other);
	
}

	
 

int main(void)
{
	
    serial_init(63);

	
	int I_RH, D_RH, I_Temp, D_Temp, CS, count;


	_delay_ms(3000);
	Request();
	Response();

	I_RH = Receive_data();
	D_RH = Receive_data();
	I_Temp = Receive_data();
	D_Temp = Receive_data();
	CS = Receive_data();


	count = I_RH + D_RH + I_Temp + D_Temp;
	if(CS == count){
		serial_string("Counts Match!");
	}


	serial_string("Humidity: ");
	decimal_to_string(I_RH);
	serial_string(".");
	decimal_to_string(D_RH);
	serial_string("\n");

	serial_string("Temperature: ");
	decimal_to_string(I_Temp);
	serial_string(".");
	decimal_to_string(D_Temp);
	serial_string("\n");

	
	

	while(1){
		
	}


    return 0;   /* never reached */
}