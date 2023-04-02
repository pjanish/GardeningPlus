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

int main(void)
{
    PORTD = 0X00;
    PORTC = 0X00;
    PORTB = 0X00;

    DDRC = 0XFF; 
    DDRD = 0XFF;
    DDRB = 0XFF;

    while (1) {
	PORTD |= 1 << PD0;      // Set PC0 to a 1
	PORTD &= ~(1 << PD0);   // Set PC0 to a 0
	PORTD |= 1 << PD1;      // Set PC0 to a 1
	PORTD &= ~(1 << PD1);   // Set PC0 to a 0
	PORTD |= 1 << PD2;      // Set PC0 to a 1
	PORTD &= ~(1 << PD2);   // Set PC0 to a 0
	PORTD |= 1 << PD3;      // Set PC0 to a 1
	PORTD &= ~(1 << PD3);   // Set PC0 to a 0
	PORTD |= 1 << PD4;      // Set PC0 to a 1
	PORTD &= ~(1 << PD4);   // Set PC0 to a 0

	PORTB |= 1 << PB7;      // Set PC0 to a 1
	PORTB &= ~(1 << PB7);   // Set PC0 to a 0
	PORTD |= 1 << PD5;      // Set PC0 to a 1
	PORTD &= ~(1 << PD5);   // Set PC0 to a 0
	PORTD |= 1 << PD6;      // Set PC0 to a 1
	PORTD &= ~(1 << PD6);   // Set PC0 to a 0
	PORTD |= 1 << PD7;      // Set PC0 to a 1
	PORTD &= ~(1 << PD7);   // Set PC0 to a 0
	PORTB |= 1 << PB0;      // Set PC0 to a 1
	PORTB &= ~(1 << PB0);   // Set PC0 to a 0

	PORTC |= 1 << PC5;      // Set PC0 to a 1
	PORTC &= ~(1 << PC5);   // Set PC0 to a 0
	PORTC |= 1 << PC4;      // Set PC0 to a 1
	PORTC &= ~(1 << PC4);   // Set PC0 to a 0
	PORTC |= 1 << PC3;      // Set PC0 to a 1
	PORTC &= ~(1 << PC3);   // Set PC0 to a 0
	PORTC |= 1 << PC2;      // Set PC0 to a 1
	PORTC &= ~(1 << PC2);   // Set PC0 to a 0
	PORTC |= 1 << PC1;      // Set PC0 to a 1
	PORTC &= ~(1 << PC1);   // Set PC0 to a 0
	PORTC |= 1 << PC0;      // Set PC0 to a 1
	PORTC &= ~(1 << PC0);   // Set PC0 to a 0

	PORTB |= 1 << PB5;      // Set PC0 to a 1
	PORTB &= ~(1 << PB5);   // Set PC0 to a 0
	PORTB |= 1 << PB4;      // Set PC0 to a 1
	PORTB &= ~(1 << PB4);   // Set PC0 to a 0
	PORTB |= 1 << PB3;      // Set PC0 to a 1
	PORTB &= ~(1 << PB3);   // Set PC0 to a 0
	PORTB |= 1 << PB2;      // Set PC0 to a 1
	PORTB &= ~(1 << PB2);   // Set PC0 to a 0
	PORTB |= 1 << PB1;      // Set PC0 to a 1
	PORTB &= ~(1 << PB1);   // Set PC0 to a 0


    }

    return 0;   /* never reached */
}