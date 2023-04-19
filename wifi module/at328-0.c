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

#define FOSC 7372800            // Clock frequency = Oscillator freq.
#define BAUD 115200               // wifi module baud rate
#define MYUBRR FOSC/16/BAUD-1   // Value for wifi module buad rate

#define BDIV_LCD (FOSC / 50000 - 16) / 2 + 1    // Puts I2C rate just below 100kHz

/* Address of the LCD on the I2C bus */
#define LCD_ADDR  0x50

uint8_t i2c_io(uint8_t device_addr, uint8_t *ap, uint16_t an, 
               uint8_t *wp, uint16_t wn, uint8_t *rp, uint16_t rn)
{
    uint8_t status, send_stop, wrote, start_stat;

    status = 0;
    wrote = 0;
    send_stop = 0;

    if (an > 0 || wn > 0) {
        wrote = 1;
        send_stop = 1;
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);  // Send start condition
        while (!(TWCR & (1 << TWINT)));     // Wait for TWINT to be set
        status = TWSR & 0xf8;
        if (status != 0x08)  {               // Check that START was sent OK
            return(status);}


        TWDR = device_addr & 0xfe;          // Load device address and R/W = 0;
        TWCR = (1 << TWINT) | (1 << TWEN);  // Start transmission
        while (!(TWCR & (1 << TWINT)));     // Wait for TWINT to be set
        status = TWSR & 0xf8;
        if (status != 0x18) {               // Check that SLA+W was sent OK
            if (status == 0x20)             // Check for NAK
                //program exiting here
                goto nakstop;               // Send STOP condition

            return(status);                 // Otherwise just return the status
        }


        // Write "an" data bytes to the slave device
        while (an-- > 0) {
            TWDR = *ap++;                   // Put next data byte in TWDR
            TWCR = (1 << TWINT) | (1 << TWEN); // Start transmission
            while (!(TWCR & (1 << TWINT))); // Wait for TWINT to be set
            status = TWSR & 0xf8;
            if (status != 0x28) {           // Check that data was sent OK
                if (status == 0x30)         // Check for NAK
                    goto nakstop;           // Send STOP condition
                return(status);             // Otherwise just return the status
            }
        }


        // Write "wn" data bytes to the slave device
        while (wn-- > 0) {
            TWDR = *wp++;                   // Put next data byte in TWDR
            TWCR = (1 << TWINT) | (1 << TWEN); // Start transmission
            while (!(TWCR & (1 << TWINT))); // Wait for TWINT to be set
            status = TWSR & 0xf8;
            if (status != 0x28) {           // Check that data was sent OK
                if (status == 0x30)         // Check for NAK
                    goto nakstop;           // Send STOP condition
                return(status);             // Otherwise just return the status
            }
        }

        status = 0;                         // Set status value to successful
    }
    if (rn > 0) {
        send_stop = 1;

        // Set the status value to check for depending on whether this is a
        // START or repeated START
        start_stat = (wrote) ? 0x10 : 0x08;

        // Put TWI into Master Receive mode by sending a START, which could
        // be a repeated START condition if we just finished writing.
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTA);
                                            // Send start (or repeated start) condition
        while (!(TWCR & (1 << TWINT)));     // Wait for TWINT to be set
        status = TWSR & 0xf8;
        if (status != start_stat)           // Check that START or repeated START sent OK
            return(status);

        TWDR = device_addr  | 0x01;         // Load device address and R/W = 1;
        TWCR = (1 << TWINT) | (1 << TWEN);  // Send address+r
        while (!(TWCR & (1 << TWINT)));     // Wait for TWINT to be set
        status = TWSR & 0xf8;
        if (status != 0x40) {               // Check that SLA+R was sent OK
            if (status == 0x48)             // Check for NAK
                goto nakstop;
            return(status);
        }

        // Read all but the last of n bytes from the slave device in this loop
        rn--;
        while (rn-- > 0) {
            TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // Read byte and send ACK
            while (!(TWCR & (1 << TWINT))); // Wait for TWINT to be set
            status = TWSR & 0xf8;
            if (status != 0x50)             // Check that data received OK
                return(status);
            *rp++ = TWDR;                   // Read the data
        }

        // Read the last byte
        TWCR = (1 << TWINT) | (1 << TWEN);  // Read last byte with NOT ACK sent
        while (!(TWCR & (1 << TWINT)));     // Wait for TWINT to be set
        status = TWSR & 0xf8;
        if (status != 0x58)                 // Check that data received OK
            return(status);
        *rp++ = TWDR;                       // Read the data

        status = 0;                         // Set status value to successful
    }
    
nakstop:                                    // Come here to send STOP after a NAK
    if (send_stop)
        TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWSTO);  // Send STOP condition

    return(status);
}


/*
  i2c_init - Initialize the I2C port
*/
void i2c_init(uint8_t bdiv)
{
    TWSR = 0;                           // Set prescalar for 1
    TWBR = bdiv;                        // Set bit rate register
}

void clear_lcd(void){
	uint8_t status;


    i2c_init(BDIV_LCD);             // Initialize the I2C port

	char buff[2];
    buff[0] = 0xFE;
    buff[1] = 0x51;

	status = i2c_io(LCD_ADDR, NULL, 0, buff, 2, NULL, 0);
    _delay_ms(10);
}



void first_line(void){
	char buff[3];
	char status;
	buff[0] = 0xFE;
    buff[1] = 0x45;
    buff[2] = 0x00;

	status = i2c_io(LCD_ADDR, NULL, 0, buff, 3, NULL, 0);
    _delay_ms(10);
}

void second_line(void){
	char buff[3];
	char status;
	buff[0] = 0xFE;
    buff[1] = 0x45;
    buff[2] = 0x40;

	status = i2c_io(LCD_ADDR, NULL, 0, buff, 3, NULL, 0);
    _delay_ms(10);


}

void third_line(void){
	char buff[3];
	char status;

	buff[0] = 0xFE;
    buff[1] = 0x45;
    buff[2] = 0x14;

	status = i2c_io(LCD_ADDR, NULL, 0, buff, 3, NULL, 0);
    _delay_ms(10);
}

void fourth_line(void){
	char buff[3];
	char status;

	buff[0] = 0xFE;
    buff[1] = 0x45;
    buff[2] = 0x54;
	status = i2c_io(LCD_ADDR, NULL, 0, buff, 3, NULL, 0);
    _delay_ms(10);
}


void write_line(char *tester){
	uint8_t status;

	status = i2c_io(LCD_ADDR, NULL, 0, tester, strlen(tester), NULL, 0); 

	_delay_ms(10);
}


void write_char(char *tester, int len){
	uint8_t status;

	status = i2c_io(LCD_ADDR, NULL, 0, tester, len, NULL, 0); 

	_delay_ms(10);
}


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
	serial_init(MYUBRR); //change to MYUBBR or 4
	clear_lcd();
	
	char letter_in1[1];
	char letter_in2[1];
	char letter_in3[1];
	char letter_in4[1];
	char l1;
	char l2;
	char l3;
	char l4;
	char l5;
	//letter_in1[0] = 'a';
	

	
	DDRD |= (1 << PD3);          //PC2 -> D6
	PORTD |= (1 << PD3);		

	write_line("starting ");
	
	
    
	while(1){
		char val = 0x55;

		serial_out('A');
		serial_out('T');
		serial_out(0x0d);
		serial_out(0x0a);

		l1 = serial_in();
		l2 = serial_in();
		l3 = serial_in();
		l4 = serial_in();

		if(l3 == 'O'){
			write_line("O");
		}


		letter_in1[0]= l1;
		letter_in2[0]= l2;
		letter_in3[0]= l3;
		letter_in4[0]= l4;


		write_char(letter_in1,1);
		write_char(letter_in2,1);

	}


    return 0;   /* never reached */
}