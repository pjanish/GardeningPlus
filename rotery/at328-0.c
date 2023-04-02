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
#include <stdio.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>


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

int debounce_button = 0;
int flag = 0;
int prev_was_three = 1;
char ostr[20];	
volatile uint8_t new_value, old_value;
volatile uint8_t changed;
volatile int16_t count = 0;
volatile int isrs = 0;



void create_plant_database(void){

	int bytes_per_plant = 31;
	int i, j = 0;
	int position = 270;
	char *plant_selected;
	char print_position[10];

	// enter the number of plants you would like to add to the database
	int num_plants = 3;
	
	// flag will always be set to 0 since will start with no plants added so this leaves total of 31 bytes per each plant
	//char *plant_names[3] = {"Calendula", "Hibiscus", "Begonia"}; //length of 9
	//char *water_per_week[3] = {1,3,2}; //length of 1
	//char *low_temperature_range [3] = {"25", "60","60"}; //length of 2
	//char *high_temperature_range [3] = {"85","95","75"}; //length of 2
	//char *low_moisture_range[3] = {"100","200","200"}; //length of 3
	//char *high_moisture_range[3] = {"200","999","999"}; //length of 3
	//char *low_brightness_range[3] = {"01500","99999","00000"}; // length of 5
	//char *high brightness_range[3] = {"99999","99999","01500"}; //length of 5

	char *plant1 = "0Calendula125851002000150099999";
	char *plant2 = "0\0Hibiscus360952009999999999999";
	char *plant3 = "0\0\0Begonia260752009990000001500";

	for(i=0;i<num_plants;i++){
		serial_string(" new plant: ");
		if(i==0){
			plant_selected = plant1;
		}else if(i == 1){
			plant_selected = plant2;
		}else{
			plant_selected = plant3;
		}

		for(j=0; j<bytes_per_plant; j++){
			serial_out(plant_selected[j]);
			eeprom_write_byte((void *) position, plant_selected[j] );
			position += 1;
		}
	}
		
		
}

int main(void){
	
    serial_init(63);


	
	uint8_t r_bit, one, two;
	char input_eeprom = 't';
	char *teststring = "hello";
	char check_eeprom;

	/*
	int q = 0;
	while(teststring[q] != '\0')
	{
		serial_out(teststring[q]);
		eeprom_write_byte((void *) q, teststring[q] );
		q++;
	}
	
	serial_string(" space ");

	

	for (q=0; q<5; q++)
	{
		check_eeprom = eeprom_read_byte((void *) q );
		serial_out(check_eeprom);
	}
	*/

	//create_plant_database(); //only use this when completely remaking plant database
	char flag;
	char name = {"h","e","l","l","o"};
	char water_per_week;
	char t_low[2];
	char t_high[3];
	char b_low[4];
	char b_high[4];
	char m_low[4];
	char m_high[4];
	int q = 0;
	int starting_index = 0;
	int bytes_per_plant = 32;


	//eeprom_write_byte((void *) 271, '1' );
	serial_string(" Plant 1: ");
	
	serial_string("Flag: ");
	flag = eeprom_read_byte((void *) 271 );
	serial_out(flag);
	

	



	DDRB &= ~(1 << PB0);		/* set pin to input */
	DDRC &= ~(1 << PC2);		/* set pin to input */
	DDRC &= ~(1 << PC1);		/* set pin to input */

	PORTC |= (1 << PC2 | 1 << PC1); // Enable pull-ups on PD6, PD7

	PCICR |= (1 << PCIE1);  // Enable PCINT on Port C
	PCMSK1 |= (1 << PCINT9 | 1 << PCINT10); // Interrupt on PC5, PC1 ***PJ WHAT SHOULD THESE BE****

	sei();                  // Enable interrupts

	serial_string("begin loop");
	// Determine the intial state
    r_bit = PINC;
    one = r_bit & (1 << PC1);
    two = r_bit & (1 << PC2);


	serial_string("initial state: ");
    if (!two && !one){
	old_value = 0;
	serial_string(" 0 ");}
    else if (!two && one){
	old_value = 1;
	serial_string(" 1 ");}
    else if (two && !one){
	old_value = 2;
	serial_string(" 2 ");}
    else{
	old_value = 3;
	serial_string(" 3 ");}

    new_value = old_value;
	while(1){
		_delay_us(10000);
		if((PINB & (1 << PINB0)) == 0 && !debounce_button){
			serial_string(" push button! ");
			debounce_button = 1;
		}else if((PINB & (1 << PINB0)) == 1){
			debounce_button = 0;
		}
		
		
	}


    return 0;   /* never reached */
}



ISR(PCINT1_vect)
{
	direction();
	_delay_us(10000);
}

void direction(){
	uint8_t bits, one, two;
	bits = PINC;		// Read the two encoder input at the same time
	one = bits & (1 << PC1);
	two = bits & (1 << PC2);

	if(flag == 0){
		if (!two && one){
		old_value = 1;
		serial_string(" 1 ");
		flag = 1;
		prev_was_three = 0;}
		else if (two && !one){
		old_value = 2;
		serial_string(" 2 ");
		flag = 1;
		prev_was_three = 0;}
		else if( !two && !one){
			prev_was_three = 0;
		}
	}
	if( two && one){
			prev_was_three = 1;
			flag = 0;
	}
}