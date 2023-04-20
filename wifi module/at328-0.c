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

char IP[11];

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

void serial_string( char *string){

	int q = 0;
	while(string[q] != '\0')
	{
		serial_out(string[q]);
		q++;
	}
}

void find_ok(char *at_command){

	int len = strlen(at_command) + 8;
	char flag_o = '0';
	char flag_ok = '0';
	char letter;
	int j;

	for(j=0;j<len;j++){
		letter = serial_in();
		if(letter == 'O'){
			flag_o = '1';
		}

		if(flag_o && letter == 'K'){
			flag_ok = '1';
			break;
		}

	}

	if(flag_ok){
		write_line("OK");
	}else{
		write_line("NOT OK");
	}



}

void get_ip(void)

{
  	char ch='0';
	char full_IP[55];

	char letter;
	char flag_first = '0';

	int j = 0;
	int count = 0;

	clear_lcd();
	write_line(" CIFSR ");
	serial_string("AT+CIFSR");
	serial_out(0x0d);
	serial_out(0x0a);

	for(j=0;j<55;j++){
		letter = serial_in();
		full_IP[j] = letter;
		if(letter == '\"' && flag_first != '1'){
			flag_first = '1';
		}else if(letter == '\"' && flag_first == '1'){
			break;
		}else if(flag_first == '1'){
			IP[count] = letter;
			count += 1;
		}
	}
	write_char(IP,11);

	write_line(" IP Address: ");
	serial_string("IP Address:");
	serial_out(0x0d);
	serial_out(0x0a);
	find_ok("IP Address:");

	write_line(" IP ");
	for(j=0;j<11;j++){
		serial_out(IP[j]);
	}
	serial_out(0x0d);
	serial_out(0x0a);
	find_ok("IP Address:");

	write_line(" Port: ");
	serial_string("Port:");
	serial_out(0x0d);
	serial_out(0x0a);
	serial_out(0x0d);
	serial_out(0x0a);
	find_ok("Port:");

	write_line(" 80 ");
	serial_string("80");
	serial_out(0x0d);
	serial_out(0x0a);
	find_ok("80");

}


int main(void)
{
	serial_init(MYUBRR); //change to MYUBBR or 4
	clear_lcd();
	
	char letter_in1[50];
	char letter_in2[62];
	char letter_in3[30];
	char letter_in4[22];

	char letter;

	int j = 0;
	//letter_in1[0] = 'a';
	

	
	DDRD |= (1 << PD3);          //PC2 -> D6
	PORTD |= (1 << PD3);		

	

	write_line("AT ");
	serial_string("AT");
	serial_out(0x0d);
	serial_out(0x0a);
	find_ok("AT");
	_delay_ms(100);


	write_line(" AT+CWMODE=3 ");	
	serial_string("AT+CWMODE=3");
	serial_out(0x0d);
	serial_out(0x0a);
	find_ok("AT+CWMODE=3");
	_delay_ms(100);

	write_line(" AT+CWQAP ");	
	serial_string("AT+CWQAP");
	serial_out(0x0d);
	serial_out(0x0a);
	find_ok("AT+CWQAP");
	_delay_ms(100);


	write_line(" AT+RST ");	
	serial_string("AT+RST");
	serial_out(0x0d);
	serial_out(0x0a);
	find_ok("AT+RST");
	_delay_ms(5000);

	/*
	

	

	

	write_line(" AT+CIPSERVER=1,80 ");
	serial_string("AT+CIPSERVER=1,80");
	serial_out(0x0d);
	serial_out(0x0a);
	find_ok("AT+CIPSERVER=1,80");
	_delay_ms(100);

	_delay_ms(2000);
	*/

	write_line("AT+CWJAP");
	serial_string("AT+CWJAP=\"USC Guest Wireless\",\"\"");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("AT+CWJAP=\"USC Guest Wireless\",\"\"");
	_delay_ms(2000);

	write_line("Check Connect");
	serial_string("AT+CWJAP?");
	serial_out(0x0d);
	serial_out(0x0a);	

	for(j=0;j<50;j++){
		letter = serial_in();
		letter_in1[j] = letter;
	}

	clear_lcd();
	write_char(letter_in1,50);
	_delay_ms(5000);

	

	get_ip();

	_delay_ms(5000);

	write_line(" AT+CIPMUX=1 ");
	serial_string("AT+CIPMUX=1");
	serial_out(0x0d);
	serial_out(0x0a);
	find_ok("AT+CIPMUX=1");
	_delay_ms(100);


	write_line(" AT+CIPSTART ");
	serial_string("AT+CIPSTART=0,\"TCP\",\"mail.smtp2go.com\",2525");
	serial_out(0x0d);
	serial_out(0x0a);	
	
	for(j=0;j<62;j++){
		letter = serial_in();
		letter_in2[j] = letter;
	}

	clear_lcd();
	write_char(letter_in2,62);

	write_line(" AT+CIPSEND=0,18 ");
	serial_string("AT+CIPSEND=0,18");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("AT+CIPSEND=0,18");
	_delay_ms(2000);

	clear_lcd();
	write_line(" EHLO ");
	write_char(IP,11);
	serial_string("EHLO ");
	for(j=0;j<11;j++){
		serial_out(IP[j]);
	}
	serial_out(0x0d);
	serial_out(0x0a);	

	for(j=0;j<30;j++){
		letter = serial_in();
		letter_in3[j] = letter;
	}

	
	write_char(letter_in3,30);


	_delay_ms(2000);
	clear_lcd();

	write_line(" AT+CIPSEND=0,12 ");
	serial_string("AT+CIPSEND=0,12");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("AT+CIPSEND=0,12");

	
	write_line(" AUTH LOGIN ");
	serial_string("AUTH LOGIN");
	serial_out(0x0d);
	serial_out(0x0a);	

	for(j=0;j<22;j++){
		letter = serial_in();
		letter_in4[j] = letter;
	}

	
	write_char(letter_in4,22);

	_delay_ms(5000);

	clear_lcd();
	write_line(" AT+CIPSEND=0,22 ");
	serial_string("AAT+CIPSEND=0,22");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("AT+CIPSEND=0,22");
	_delay_ms(2000);
	
	write_line(" email ");
	serial_string("cGphbmlzaEB1c2MuZWR1");
	serial_out(0x0d);
	serial_out(0x0a);	
	for(j=0;j<22;j++){
		letter = serial_in();
		letter_in4[j] = letter;
	}
	for(j=0;j<30;j++){
		letter = serial_in();
		letter_in1[j] = letter;
	}

	
	write_char(letter_in1,30);

	
	write_char(letter_in4,22);
	_delay_ms(5000);

	clear_lcd();
	write_line(" AT+CIPSEND=0,26 ");
	serial_string("AAT+CIPSEND=0,26");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("AT+CIPSEND=0,26");
	_delay_ms(2000);
	
	write_line(" password ");
	//serial_string("Z2FyZGVuaW5ncGx1czQ1OQ0K");
	serial_string("R2FyZGVuaW5ncGx1czIwMjM=");
	serial_out(0x0d);
	serial_out(0x0a);	
	for(j=0;j<20;j++){
		letter = serial_in();
		letter_in1[j] = letter;
	}
	for(j=0;j<50;j++){
		letter = serial_in();
		letter_in1[j] = letter;
	}

	
	write_char(letter_in1,50);

	_delay_ms(5000);

	clear_lcd();
	write_line(" AT+CIPSEND=0,29 ");
	serial_string("AAT+CIPSEND=0,29");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("AT+CIPSEND=0,29");
	_delay_ms(2000);
	
	write_line(" send from ");
	serial_string("MAIL FROM:<pjanish@usc.edu>");
	serial_out(0x0d);
	serial_out(0x0a);	
	for(j=0;j<20;j++){
		letter = serial_in();
		letter_in1[j] = letter;
	}
	
	write_char(letter_in1,20);

	_delay_ms(5000);

	clear_lcd();
	write_line(" AT+CIPSEND=0,27 ");
	serial_string("AAT+CIPSEND=0,27");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("AT+CIPSEND=0,27");
	_delay_ms(2000);
	
	write_line(" send to ");
	serial_string("RCPT TO:<pjanish@usc.edu>");
	serial_out(0x0d);
	serial_out(0x0a);	
	for(j=0;j<20;j++){
		letter = serial_in();
		letter_in1[j] = letter;
	}
	for(j=0;j<40;j++){
		letter = serial_in();
		letter_in1[j] = letter;
	}

	
	write_char(letter_in1,40);
	_delay_ms(5000);

	clear_lcd();
	write_line(" AT+CIPSEND=0,6");
	serial_string("AT+CIPSEND=0,6");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("AT+CIPSEND=0,6");
	_delay_ms(2000);

	write_line(" DATA");
	serial_string("DATA");
	serial_out(0x0d);
	serial_out(0x0a);	
	_delay_ms(2000);

	_delay_ms(5000);

	clear_lcd();
	write_line(" AT+CIPSEND=0,19");
	serial_string("AT+CIPSEND=0,19");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("AT+CIPSEND=0,19");
	_delay_ms(2000);

	write_line(" Subject: hi bitch");
	serial_string("Subject: hi bitch");
	serial_out(0x0d);
	serial_out(0x0a);	
	_delay_ms(5000);

	clear_lcd();
	write_line(" AT+CIPSEND=0,3");
	serial_string("AT+CIPSEND=0,3");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("AT+CIPSEND=0,3");
	_delay_ms(2000);

	write_line(" . ");
	serial_string(".");
	serial_out(0x0d);
	serial_out(0x0a);	
	_delay_ms(5000);

	clear_lcd();
	write_line(" AT+CIPSEND=0,6");
	serial_string("AT+CIPSEND=0,6");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("AT+CIPSEND=0,6");
	_delay_ms(2000);

	write_line(" QUIT ");
	serial_string("QUIT");
	serial_out(0x0d);
	serial_out(0x0a);
	for(j=0;j<30;j++){
		letter = serial_in();
		letter_in2[j] = letter;
	}
	for(j=0;j<50;j++){
		letter = serial_in();
		letter_in2[j] = letter;
	}
	write_char(letter_in2,50);
	_delay_ms(5000);
	



/*
	

	

	

	clear_lcd();

	

	

	write_line(" AT+CIPSEND=4,18 ");
	serial_string("AT+CIPSEND=4,18");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("AT+CIPSEND=4,18");
	_delay_ms(2000);

	write_line(" pass ");
	serial_string("VGVzdGluZ3Bhc3N3b3JkMTIzIQ==");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("VGVzdGluZ3Bhc3N3b3JkMTIzIQ==");
	_delay_ms(2000);

	write_line(" AT+CIPSEND=4,34 ");
	serial_string("AT+CIPSEND=4,34");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("AT+CIPSEND=4,34");
	_delay_ms(2000);

	write_line(" mail from ");
	serial_string("MAIL FROM:<pjanish@usc.edu>");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("MAIL FROM:<pjanish@usc.edu>");
	_delay_ms(2000);

	write_line(" AT+CIPSEND=4,32 ");
	serial_string("AT+CIPSEND=4,32");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("AT+CIPSEND=4,32");
	_delay_ms(2000);

	write_line(" rcpt to");
	serial_string("RCPT To:<prjjanish@gmail.com>");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("RCPT To:<prjjanish@gmail.com>");
	_delay_ms(2000);

	write_line(" AT+CIPSEND=4,6");
	serial_string("AT+CIPSEND=4,6");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("AT+CIPSEND=4,6");
	_delay_ms(2000);

	write_line(" DATA");
	serial_string("DATA");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("DATA");
	_delay_ms(2000);

	write_line(" AT+CIPSEND=4,24");
	serial_string("AT+CIPSEND=4,24");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("AT+CIPSEND=4,24");
	_delay_ms(2000);

	write_line(" body data ");
	serial_string("body data");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("body data");
	_delay_ms(2000);


	write_line(" AT+CIPSEND=4,3");
	serial_string("AT+CIPSEND=4,3");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("AT+CIPSEND=4,3");
	_delay_ms(2000);

	write_line(" .");
	serial_string(".");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok(".");
	_delay_ms(1000);

	write_line(" AT+CIPSEND=4,6");
	serial_string("AT+CIPSEND=4,6");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("AT+CIPSEND=4,6");
	_delay_ms(2000);

	write_line(" QUIT");
	serial_string("QUIT");
	serial_out(0x0d);
	serial_out(0x0a);	
	find_ok("QUIT");
	_delay_ms(2000);


	
*/


	/*
	

	for(j=0;j<80;j++){
		letter = serial_in();
		letter_in1[j] = letter;
	}

	for(j=0;j<80;j++){
		letter = serial_in();
		letter_in2[j] = letter;
	}

	for(j=0;j<80;j++){
		letter = serial_in();
		letter_in3[j] = letter;
	}

	for(j=0;j<80;j++){
		letter = serial_in();
		letter_in4[j] = letter;
	}

	write_char(letter_in1,80);
	_delay_ms(10000);
	write_char(letter_in2,80);
	_delay_ms(10000);
	write_char(letter_in3,80);
	_delay_ms(10000);
	write_char(letter_in4,80);
	_delay_ms(10000);
	l1 = serial_in();
	l2 = serial_in();
	l3 = serial_in();
	l4 = serial_in();
	l5 = serial_in();
	l6 = serial_in();
	l7 = serial_in();
	l8 = serial_in();
	l9 = (char) serial_in();
	l10 = (char) serial_in();

	


	letter_in1[0]= l1;
	letter_in2[0]= l2;
	letter_in3[0]= l3;
	letter_in4[0]= l4;
	letter_in5[0]= l5;
	letter_in6[0]= l6;
	letter_in7[0]= l7;
	letter_in8[0]= l8;
	letter_in9[0]= l9;
	letter_in10[0]= l10;

	write_line("1");
	write_char(letter_in1,1);
	write_line("2");
	write_char(letter_in2,1);
	write_line("3");
	write_char(letter_in3,1);
	write_line("4");
	write_char(letter_in4,1);
	write_line("5");
	write_char(letter_in5,1);
	write_line("6");
	write_char(letter_in6,1);
	write_line("7");
	write_char(letter_in7,1);
	write_line("8");
	write_char(letter_in8,1);
	write_line("9");
	write_char(letter_in9,1);
	write_line("10");
	write_char(letter_in10,1);
	*/
	
	
	
    
	while(1){
		
	

		
	}


    return 0;   /* never reached */
}