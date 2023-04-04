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

// state defines

#define POWER_ON_STATE '1'
#define CALIBRATION_STATE '2'
#define SELECT_FUNCTION_STATE '3'
#define EDIT_PLANTS_STATE '4'
#define RESULTS_STATE '5'
#define WARNING_STATE '6'
#define MONITER_STATE '7'

// moisture sensor defines
// Find divisors for the UART0 and I2C baud rates
#define FOSC 9830400            // Clock frequency = Oscillator freq.
#define BAUD 9600               // UART0 baud rate
#define MYUBRR FOSC/16/BAUD-1   // Value for UBRR0 register
#define BDIV (FOSC / 100000 - 16) / 2 + 1    // Puts I2C rate just below 100kHz

/* Address of the Moisture Sensor on the I2C bus */
#define MOISTURE_ADDR  0x52

/* Address where we store test data in the EEPROM */
#define DATA_ADDR  500


// variables to facilitate state transitions and device status
char state;
char flag_max_plants_reached_flag = '0';

// variables to facilitate rotery encoder turns and presses
int debounce_button = 0;
int flag = 0;
int prev_was_three = 1;
char ostr[20];	
volatile uint8_t new_value, old_value;
char changed = '0';
volatile int16_t count = 0;
volatile int isrs = 0;
int rotery_state = 0;
int max_num_options_rotery = 0;

// flag will always be set to 0 since will start with no plants added so this leaves total of 33 bytes per each plant
	//char *plant_names[3] = {"Calendula", "Hibiscus", "Begonia"}; //length of 9
	//char *water_per_week[3] = {1,3,2}; //length of 1
	//char *low_temperature_range [3] = {"25", "60","60"}; //length of 2
	//char *high_temperature_range [3] = {"85","95","75"}; //length of 2
	//char *low_moisture_range[3] = {"100","200","200"}; //length of 3
	//char *high_moisture_range[3] = {"200","999","999"}; //length of 3
	//char *low_brightness_range[3] = {"01500","99999","00000"}; // length of 5
	//char *high brightness_range[3] = {"99999","99999","01500"}; //length of 5
	//char *rank[3] = ["0","0","0"]; // unset rank at beginning
	//char *score[3] = ["0","0","0"]; // unset score at beginning


// TO DO: remove these plant defines, just to simulate reading from EEPROM
int num_plants_in_database = 3;
char *plant1 = "0Calendula12585100200015009999900";
char *plant2 = "0\0Hibiscus36095200999999999999900";
char *plant3 = "0\0\0Begonia26075200999000000150000";
int temp_ave, brightness_ave, moisture_ave;



// serial setup functions
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




// conversion functions 


void decimal_to_string( int decimal_val){
	char other[4];
	snprintf(other, 4, "%d", decimal_val);
	serial_string(other);
	
}

void celcius_to_farenhiet(int i, int d){
	float i_result, d_result;
	char str[100];

	

	i_result = ((float) i * 9)/5;
	d_result = ((float) d * 9)/5;

	i_result += (d_result / 10);

	i_result += 32;

	int int_i_result = i_result;                  // Get the integer (678).
	float fract_i_result = i_result - int_i_result;      // Get fraction (0.0123).
	int int_fract_i_result = trunc(fract_i_result * 10);  // Turn into integer (123).

	sprintf (str, "%d.%01d\n", int_i_result, int_fract_i_result);
	serial_string(str);


	// TO DO: update value in EEPROM with temp average
	temp_ave = int_i_result;

	

}


// temperature sensor functions 

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

void poll_temperature(void){
	int I_Temp, D_Temp, I_RH, D_RH, CS, count;


	_delay_ms(3000);
	Request();
	Response();

	I_RH = Receive_data();
	D_RH = Receive_data();
	I_Temp = Receive_data();
	D_Temp = Receive_data();
	CS = Receive_data();


	count = I_RH + D_RH + I_Temp + D_Temp;

	// values are only valid if CS == count
	if(CS == count){
		serial_string("Temperature: ");
		celcius_to_farenhiet(I_Temp, D_Temp);
	}

	/*
	serial_string("Humidity: ");
	decimal_to_string(I_RH);
	serial_string(".");
	decimal_to_string(D_RH);
	serial_string("\n");
	*/

}

// moisture sensor functions

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

void poll_moisture(void){
	int moisture_value;
	set_up();
	power_on();
	moisture_value = read_pin();
	serial_string("Moisture: ");
	decimal_to_string(moisture_value);
	power_off();

	// TO DO: update moisture value in the EEPROM
	moisture_ave = moisture_value;
}

// brightness sensor functions

void poll_brightness(void){
	uint8_t status;
	int adata[] =  {0b10110010};           // Array to hold the byte for commanding a read ID
    int p_enable[] = {0b10100000};
    int set_gain_a[] = {0b10100001};
    int status_a[] = {0b10110011};
    int c14[] = {0b10110100};



    int wr_p_enable[] = {0b00000001};
    int wr_p_disable[] = {0b00000000};
    int set_gain_w[] = {0b00000011};
    int convert_w[] = {0b00000011};

    int read_status[10]; 
    int rdata[10];
    int check_po[10];
    int check_params[10];


    int codatal[10];
    int codatah[10];
    int c1datal[10];
    int c1datah[10]; 

    int num_times = 0;


	serial_init(63);
    i2c_init(BDIV);             // Initialize the I2C port
    _delay_ms(3000);  // delay to allow for I2C to be visible on the scope
	
	serial_init(63);
    i2c_init(BDIV);             // Initialize the I2C port
    _delay_ms(3000);  // delay to allow for I2C to be visible on the scope
	status = i2c_io(MOISTURE_ADDR, adata, 1, NULL, 0, rdata, 1); //bytes, reading id from device

    // checks to make sure the ID is correct 
    if(rdata[0] == 0x50){
        serial_string("   found");
    }
    status = i2c_io(MOISTURE_ADDR, p_enable, 1, wr_p_enable, 1, check_po, 1); //bytes, powers on device


    status = i2c_io(MOISTURE_ADDR, set_gain_a, 1, set_gain_w, 1, NULL, 0); //bytes, sets params


	_delay_ms(3000);
	unsigned char avalid = 0x30;
	unsigned char buff[4];
	status = i2c_io(MOISTURE_ADDR, p_enable, 1, convert_w, 1, check_params, 1); //bytes, set power on and AEN to start conversion

	_delay_ms(3000);
	while(avalid == 0x30){
		status = i2c_io(MOISTURE_ADDR, status_a, 1, NULL, 0, read_status, 1); //bytes, check the status of a valid
		avalid = read_status[0];
	}


	
	status = i2c_io(MOISTURE_ADDR, c14, 1, NULL, 0, buff, 4); 

	status = i2c_io(MOISTURE_ADDR, p_enable, 1, wr_p_enable, 1, NULL, 0); //bytes, set power on and leave AEN 0 

	serial_string(" Brightness: ");
	unsigned int x = buff[1];
	unsigned char other[10];
	x = x<<8;
	x = x | buff[0];
	
	snprintf(other, 10, "%d", x);
	serial_string(other);
	serial_string("\n");


    status = i2c_io(MOISTURE_ADDR, set_gain_a, 1, wr_p_disable, 1, NULL, 0); //bytes, powers off device

	// TO DO: update brightness in EEPROM 
	brightness_ave = x;
}

// algorithm
void rank_plants(void){
	int i = 0;
	unsigned char tester[10];
	char *plant_selected;
	float score1, score2, score3, score;
	char rank1, rank2, rank3;
	unsigned int low_range_t, high_range_t, range_ave;

	// TO DO: cycle through all plants in the eeprom, using fixed value
	for(i=0;i<3;i++){
		if(i == 0){
			plant_selected = plant1;
			serial_string(" plant1 ");
		}else if(i == 1){
			plant_selected = plant2;
			serial_string(" plant2 ");
		}else{
			plant_selected = plant3;
			serial_string(" plant3 ");
		}

		// first calculate the temperature correctness

		// low range temperature values
		serial_string(" TEMP ");
		serial_string("low range: ");
		low_range_t = ((int)(plant_selected[11] - '0')) * 10;
		low_range_t += (int)(plant_selected[12] - '0');


		snprintf(tester, 10,"%d", low_range_t);
		serial_string(tester);
		serial_out(' ');
		serial_out(plant_selected[11]);
		serial_out(plant_selected[12]);
		

		// high range temperature values
		serial_string("high range: ");
		serial_out(plant_selected[13] );
		serial_out(plant_selected[14] );
		high_range_t = (( unsigned int)(plant_selected[13] - '0')) * 10;
		high_range_t += (unsigned int)(plant_selected[14] - '0');

		range_ave = (high_range_t + low_range_t) /2;
		if( low_range_t <=temp_ave && temp_ave <= high_range_t ){
			serial_string(" IN RANGE ");
			if((range_ave - 5) <=temp_ave && temp_ave <= (range_ave + 5) ){
				serial_string(" BEST ");
				score = 4;
				
			}else if((range_ave - 10) <=temp_ave && temp_ave <= (range_ave + 10)){
				serial_string(" SECOND BEST");
				score = 3.5;

			}else{
				serial_string(" THIRD BEST ");
				score = 3;
			}
		}else{
			serial_string(" OUTSIDE RANGE ");
			score = 0.5;
		}

		// then calculate moisture values

		// low range temperature values
		serial_string(" MOISTURE ");
		serial_string("low range: ");
		low_range_t = ((unsigned int)(plant_selected[15] - '0')) * 100;
		low_range_t += ((unsigned int)(plant_selected[16] - '0')) * 10;
		low_range_t += (unsigned int)(plant_selected[17] - '0');


		snprintf(tester, 10,"%d", low_range_t);
		serial_string(tester);
		serial_out(' ');
		serial_out(plant_selected[15]);
		serial_out(plant_selected[16]);
		serial_out(plant_selected[17]);
		

		// high range temperature values
		serial_string(" high range: ");
		serial_out(plant_selected[18] );
		serial_out(plant_selected[19] );
		serial_out(plant_selected[20] );
		high_range_t = ((unsigned int)(plant_selected[18] - '0')) * 100;
		high_range_t += ((unsigned int)(plant_selected[19] - '0')) * 10;
		high_range_t += (unsigned int)(plant_selected[20] - '0');

		range_ave = (high_range_t + low_range_t) /2;
		if( low_range_t <=moisture_ave && moisture_ave <= high_range_t ){
			serial_string(" IN RANGE ");
			if((range_ave - 50) <=moisture_ave && moisture_ave <= (range_ave + 50) ){
				serial_string(" BEST ");
				score += 4;
				
			}else if((range_ave - 150) <=moisture_ave && moisture_ave <= (range_ave + 150)){
				serial_string(" SECOND BEST");
				score += 3.5;

			}else{
				serial_string(" THIRD BEST ");
				score += 3;
			}
		}else{
			serial_string(" OUTSIDE RANGE ");
			score += 0.5;
		}

		// brightness sensor

		// low range temperature values
		serial_string(" BRIGHTNESS ");
		serial_string("low range: ");
		low_range_t = ((unsigned int)(plant_selected[21] - '0')) * 10000;
		low_range_t += ((unsigned int)(plant_selected[22] - '0')) * 1000;
		low_range_t += (unsigned int)(plant_selected[23] - '0')* 100;
		low_range_t += (unsigned int)(plant_selected[24] - '0') * 10;
		low_range_t += (unsigned int)(plant_selected[25] - '0');


		snprintf(tester, 10,"%d", low_range_t);
		serial_string(tester);
		serial_out(' ');
		serial_out(plant_selected[21]);
		serial_out(plant_selected[22]);
		serial_out(plant_selected[23]);
		serial_out(plant_selected[24]);
		serial_out(plant_selected[25]);
		

		// high range temperature values
		serial_string(" high range: ");

		

		serial_out(plant_selected[26] );
		serial_out(plant_selected[27] );
		serial_out(plant_selected[28] );
		serial_out(plant_selected[29] );
		serial_out(plant_selected[30] );

		high_range_t = ((unsigned int)(plant_selected[26] - '0')) * 10000;
		high_range_t += ((unsigned int)(plant_selected[27] - '0')) * 1000;
		high_range_t += (unsigned int)(plant_selected[28] - '0')* 100;
		high_range_t += (unsigned int)(plant_selected[29] - '0') * 10;
		high_range_t += (unsigned int)(plant_selected[30] - '0');

		snprintf(tester, 10,"%u", high_range_t);
		serial_string(tester);
		serial_out(' ');

		range_ave = (high_range_t + low_range_t) /2;
		if( low_range_t <=brightness_ave && brightness_ave <= high_range_t ){
			serial_string(" IN RANGE ");
			if((range_ave - 50) <=brightness_ave && brightness_ave <= (range_ave + 50) ){
				serial_string(" BEST ");
				score += 2;
				
			}else if((range_ave - 150) <=brightness_ave && brightness_ave <= (range_ave + 150)){
				serial_string(" SECOND BEST");
				score += 1.5;

			}else{
				serial_string(" THIRD BEST ");
				score += 1;
			}
		}else{
			serial_string(" OUTSIDE RANGE ");
			score += 0.25;
		}

		if(i == 0){
			score1 = score;
		}else if(i == 1){
			score2 = score;
		}else{
			score3 = score;
		}
	}

	if(score1 >= score2){
		if(score1 >= score3){
			rank1 = '1';
			if(score2 >= score3){
				rank2 = '2';
				rank3 = '3';
			}else{
				rank2 = '3';
				rank3 = '2';
			}
		}else{
			rank3 = '1';
			rank1 = '2';
			rank2 = '3';
		}
	}else{
		if(score2 >= score3){
			rank2 = '1';
			if(score3 >= score1){
				rank3 = '2';
				rank1 = '3';
			}else{
				rank1 = '2';
				rank3 = '3';
			}
		}else{
			rank3 = '1';
			rank1 = '3';
			rank2 = '2';
		}
	}

	unsigned int int_i_result = score1;                  // Get the integer (678).
	float fract_i_result = score - int_i_result;      // Get fraction (0.0123).
	unsigned int int_fract_i_result = trunc(fract_i_result * 10);  // Turn into integer (123).
	char str[100];

	sprintf (str, "%d.%01d\n", int_i_result, int_fract_i_result);
	

	serial_string("plant 1: ");
	for(i=1; i<10; i++){
		serial_out(plant1[i]);
	}
	serial_out(' ');
	serial_string(str);
	serial_out(' ');
	serial_out(rank1);
	serial_out(' ');

	int_i_result = score2;                  // Get the integer (678).
	fract_i_result = score - int_i_result;      // Get fraction (0.0123).
	int_fract_i_result = trunc(fract_i_result * 10);  // Turn into integer (123).
	str[100];

	sprintf (str, "%d.%d\n", int_i_result, int_fract_i_result);
	

	serial_string("plant 2: ");
	for(i=1; i<10; i++){
		serial_out(plant2[i]);
	}
	serial_out(' ');
	serial_string(str);
	serial_out(' ');
	serial_out(rank2);
	serial_out(' ');

	int_i_result = score3;                  // Get the integer (678).
	fract_i_result = score - int_i_result;      // Get fraction (0.0123).
	int_fract_i_result = trunc(fract_i_result * 10);  // Turn into integer (123).
	str[100];

	sprintf (str, "%d.%01d", int_i_result, int_fract_i_result);
	serial_out('\n');

	serial_string("plant 3: ");
	for(i=1; i<10; i++){
		serial_out(plant3[i]);
	}
	serial_out(' ');
	serial_string(str);
	serial_out(' ');
	serial_out(rank3);
	serial_out(' ');

	// TO DO: update plant ranks and score in the EEPROM 
	plant1[31] = rank1;
	plant2[31] = rank2;
	plant3[31] = rank3;

	plant1[32] = (int) score1 + '0';
	plant2[32] = (int) score2 + '0';
	plant3[32] = (int) score3 + '0';

}


// state functions
void power_on_state(char first_po_flag, char num_plants_selected){
	serial_string(" in power on state function ");

	if(first_po_flag == '1'){
		state = CALIBRATION_STATE;
	}else{
		if(num_plants_selected != '0'){
			state = SELECT_FUNCTION_STATE;
		}else{
			state = RESULTS_STATE;
		}
		
	}
}

void calibration_state(void){
	serial_string(" in calibration state function ");

	// TO DO: LCD Display loading screen with calibration and perhaps progress bar
	// LCD SCREEN 2

	// stay in this state just until initial values are read

	//poll sensors and create averages
	// TO DO: poll sensors and add averages
	// TO DO: add sensor functions and integrate the polling
	int i = 0;

	poll_temperature();
	poll_moisture();
	poll_brightness();

	// TO DO: store average values into the EEPROM




	// State transition solely based on time, afterwards go to results state to see the result
	state = RESULTS_STATE;	
	
}

void select_function_state(void){
	serial_string(" in select function state function ");

	// TO DO: LCD Display three options : add new plant, edit current plants, or monitor
	// LCD SCREEN 3

	// rotery encoder to cycle through the options, on screen order will be add new plants, edit plants, then monitor from left to right
	//TO DO: add rotery encoder selection
	rotery_state = 0;

	// TO DO: add interrupt to change rotery state
	uint8_t r_bit, one, two;
	max_num_options_rotery = 3;

	DDRB &= ~(1 << PB0);		/* set pin to input */
	DDRC &= ~(1 << PC2);		/* set pin to input */
	DDRC &= ~(1 << PC1);		/* set pin to input */

	PORTC |= (1 << PC2 | 1 << PC1); // Enable pull-ups on PD6, PD7

	PCICR |= (1 << PCIE1);  // Enable PCINT on Port C
	PCMSK1 |= (1 << PCINT9 | 1 << PCINT10); // Interrupt on PC5, PC1 ***PJ WHAT SHOULD THESE BE****

	sei();                  // Enable interrupts

	// Determine the intial state
    r_bit = PINC;
    one = r_bit & (1 << PC1);
    two = r_bit & (1 << PC2);


    if (!two && !one){
	old_value = 0;}
    else if (!two && one){
	old_value = 1;}
    else if (two && !one){
	old_value = 2;}
    else{
	old_value = 3;}

    new_value = old_value;


	char *plant_selected;
	char  select_function_flag = '0';
	while((PINB & (1 << PINB0)) == 0);
	


	serial_string(" function selected: ");
	if(rotery_state == 0){
		serial_string(" add new plant ");
	}else if(rotery_state == 1){
		serial_string(" edit current plants ");
	}else{
		serial_string(" monitor current plants ");
	}
	
	while(select_function_flag == '0'){
		if((PINB & (1 << PINB0)) == 0 && !debounce_button){
			serial_string(" FUNCTION SELECTED ");
			select_function_flag = '1';
			debounce_button = 1;
		}else if((PINB & (1 << PINB0)) == 1){
			debounce_button = 0;
		}

		if(changed == '1'){
			serial_string(" function selected: ");
			if(rotery_state == 0){
				serial_string(" add new plant ");
			}else if(rotery_state == 1){
				serial_string(" edit current plants ");
			}else{
				serial_string(" monitor current plants ");
			}
			changed = '0';
		}
	}

	serial_string(" Function Chosen = ");
	if(rotery_state == 0){
		serial_string(" add new plant ");
	}else if(rotery_state == 1){
		serial_string(" edit current plants ");
	}else{
		serial_string(" monitor current plants ");
	}
	
	// State transitions listed below
	if(rotery_state ==2){
		state =  MONITER_STATE;
	}else if(rotery_state == 1){
		state = EDIT_PLANTS_STATE;
	} else if(  rotery_state == 0 && flag_max_plants_reached_flag == '0'){
		state = RESULTS_STATE;
	} else if( rotery_state == 0 && flag_max_plants_reached_flag == '1' ){
		state = WARNING_STATE;
	}
}

void results_state(void){

	serial_string(" in results state function ");
	int score1, score2, score3 = 0;
	char temp[5];
	int i,j = 0;

	// algorithm taking in current plant averages if first time callibrated use that data else use long term averages
	serial_string(" scores 1: ");
	serial_out(plant1[32]);
	rank_plants();
	serial_out(plant1[32]);

	serial_string("\n\nPLANT RANKING: 1. ");
	char *plant_selected1;
	if(plant1[31] == '1'){
		plant_selected1 = plant1;
	}else if (plant2[31] == '1'){
		plant_selected1 = plant2;
	}else if(plant3[31] == '1'){
		plant_selected1 = plant3;
	}

	for(j=1;j<10;j++){
		serial_out(plant_selected1[j]);
	}
	serial_string("  ");
	serial_out(plant_selected1[32]);
	serial_string(" / 10");

	char *plant_selected2;
	serial_string("\n\n 2. ");
	if(plant1[31] == '2'){
		plant_selected2 = plant1;
	}else if (plant2[31] == '2'){
		plant_selected2 = plant2;
	}else if(plant3[31] == '2'){
		plant_selected2 = plant3;
	}

	for(j=1;j<10;j++){
		serial_out(plant_selected2[j]);
	}
	serial_string("  ");
	serial_out(plant_selected2[32]);
	serial_string(" / 10");

	char *plant_selected3;
	serial_string("\n\n 3. ");
	if(plant1[31] == '3'){
		plant_selected3 = plant1;
	}else if (plant2[31] == '3'){
		plant_selected3 = plant2;
	}else if(plant3[31] == '3'){
		plant_selected3 = plant3;
	}

	for(j=1;j<10;j++){
		serial_out(plant_selected3[j]);
	}
	serial_string("  ");
	serial_out(plant_selected3[32]);
	serial_string(" / 10");

	// TO DO: LCD Display options from algorithm ranked, give option to select plant to moniter only if plant is not already being monitered
	//LCD SCREEN 4
	
	// TO DO: rotery encoder to cycle through selecting available plants
	char  select_plant_flag = '0';

	uint8_t r_bit, one, two;
	max_num_options_rotery = 3;

	DDRB &= ~(1 << PB0);		/* set pin to input */
	DDRC &= ~(1 << PC2);		/* set pin to input */
	DDRC &= ~(1 << PC1);		/* set pin to input */

	PORTC |= (1 << PC2 | 1 << PC1); // Enable pull-ups on PD6, PD7

	PCICR |= (1 << PCIE1);  // Enable PCINT on Port C
	PCMSK1 |= (1 << PCINT9 | 1 << PCINT10); // Interrupt on PC5, PC1 ***PJ WHAT SHOULD THESE BE****

	sei();                  // Enable interrupts

	// Determine the intial state
    r_bit = PINC;
    one = r_bit & (1 << PC1);
    two = r_bit & (1 << PC2);


    if (!two && !one){
	old_value = 0;}
    else if (!two && one){
	old_value = 1;}
    else if (two && !one){
	old_value = 2;}
    else{
	old_value = 3;}

    new_value = old_value;

	char *plant_selected;

	serial_string(" plant selected: ");
	if(rotery_state == 0){
		plant_selected = plant_selected1;
	}else if(rotery_state == 1){
		plant_selected = plant_selected2;
	}else{
		plant_selected = plant_selected3;
	}

	for(j=1;j<10;j++){
		serial_out(plant_selected[j]);
	}
	
	while(select_plant_flag == '0'){
		if((PINB & (1 << PINB0)) == 0 && !debounce_button){
			serial_string(" PLANT SELECTED ");
			select_plant_flag = '1';
			debounce_button = 1;
		}else if((PINB & (1 << PINB0)) == 1){
			debounce_button = 0;
		}

		if(changed == '1'){
			serial_string(" plant selected: ");
			if(rotery_state == 0){
				plant_selected = plant_selected1;
			}else if(rotery_state == 1){
				plant_selected = plant_selected2;
			}else{
				plant_selected = plant_selected3;
			}

			for(j=1;j<10;j++){
				serial_out(plant_selected[j]);
			}
			changed = '0';
		}
	}

	serial_string(" Plant Chosen = ");
	for(j=1;j<10;j++){
		serial_out(plant_selected[j]);
	}
	// TO DO: if plant is selected then update the EEPROM, if plant is selected update the select plant flag
	if(plant_selected == plant1){
		plant1[0] = '1';
	}else if(plant_selected = plant2){
		plant2[0] = '1';
	}else{
		plant3[0] = '1';
	}
	// if all three plants are selected than change the flag to 1 so no more plants can be added
	if(plant1[0] == '1' & plant2[0] == '1' & plant3[0] == '1'){
		flag_max_plants_reached_flag = '1';
	} 

	// TO DO: State transitions listed below
	// if plant is selected go back to select function state
	state = SELECT_FUNCTION_STATE;

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
		if(rotery_state != (max_num_options_rotery - 1)){
			rotery_state += 1;
		}else{
			rotery_state = 0;
		}
		changed = '1';
		flag = 1;
		prev_was_three = 0;}
		else if (two && !one){
		old_value = 2;
		if(rotery_state != 0){
			rotery_state -= 1;
		}else{
			rotery_state = max_num_options_rotery - 1;
		}
		changed = '1';
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


void edit_plants_state(void){
	serial_string(" in edit plants state function ");


	// TO DO: go into EEPROM and view which plants have been selected

	// TO DO: if all the plants were selected and one was removed change the max plants added flag to 0 again
	
	// TO DO: LCD display the selected plants with x's so these can be removed, back button 
	// TO DO: rotery encoder to cycle through the available plants

	rotery_state = 0; // rotate through all available plants and back button at bottom 
	char removed_all_plants_flag = '0';
	char  select_back_flag = '0';
	char flag_plant_removed = '0';
	max_num_options_rotery = 4;

	DDRB &= ~(1 << PB0);		/* set pin to input */
	DDRC &= ~(1 << PC2);		/* set pin to input */
	DDRC &= ~(1 << PC1);		/* set pin to input */

	PORTC |= (1 << PC2 | 1 << PC1); // Enable pull-ups on PD6, PD7

	PCICR |= (1 << PCIE1);  // Enable PCINT on Port C
	PCMSK1 |= (1 << PCINT9 | 1 << PCINT10); // Interrupt on PC5, PC1 ***PJ WHAT SHOULD THESE BE****
	while((PINB & (1 << PINB0)) == 0); // wait for button to become depressed

	sei();                  // Enable interrupts

	// Determine the intial state
	uint8_t r_bit, one, two;
    r_bit = PINC;
    one = r_bit & (1 << PC1);
    two = r_bit & (1 << PC2);


    if (!two && !one){
	old_value = 0;}
    else if (!two && one){
	old_value = 1;}
    else if (two && !one){
	old_value = 2;}
    else{
	old_value = 3;}

    new_value = old_value;

	char *option_selected;
	char *back = "\0\0\0\0\0\0back";
	int j = 0;
	int count = 1;
	char print_count[5];

	serial_string(" OPTIONS TO REMOVE ");
	if(plant1[0] == '1'){
		snprintf(print_count,5,"%d",count);
		serial_string(print_count);
		for(j=1;j<10;j++){
				serial_out(plant1[j]);
		}
		count += 1;
	}

	if(plant2[0] == '1'){
		snprintf(print_count,5,"%d",count);
		serial_string(print_count);
		for(j=1;j<10;j++){
				serial_out(plant2[j]);
		}
		count += 1;
	}

	if(plant3[0] == '1'){
		snprintf(print_count,5,"%d",count);
		serial_string(print_count);
		for(j=1;j<10;j++){
				serial_out(plant3[j]);
		}
		count += 1;
	}

	snprintf(print_count,5,"%d",count);
	serial_string(print_count);
	for(j=1;j<10;j++){
			serial_out(back[j]);
	}

	serial_string("\n\n plant selected: ");
	if(rotery_state == 0){
		option_selected = back;
	}else if(rotery_state == 1){
		option_selected = plant1;
		
	}else if(rotery_state == 2){
		option_selected = plant2;
		
	}else{
		option_selected = plant3;
	}

	for(j=1;j<10;j++){
		serial_out(option_selected[j]);
	}
	
	while(select_back_flag == '0'){
		if((PINB & (1 << PINB0)) == 0 && !debounce_button){
			serial_string(" OPTION SELECTED ");
			
			debounce_button = 1;
			// TO DO: if plant is selected then update the EEPROM, if plant is selected update the select plant flag
			serial_string("  ");
			for(j=1;j<10;j++){
				serial_out(option_selected[j]);
			}
			if(option_selected == back){
				select_back_flag = '1';
			}else if(option_selected == plant1){
				plant1[0] = '0';
				flag_plant_removed = '1';
			}else if(option_selected == plant2){
				plant2[0] = '0';
				flag_plant_removed = '1';
			}else if(option_selected == plant3){
				plant3[0] = '0';
				flag_plant_removed = '1';
			}


			serial_string(" Option Chosen = ");
			for(j=1;j<10;j++){
				serial_out(option_selected[j]);
			}


		}else if((PINB & (1 << PINB0)) == 1){
			debounce_button = 0;
		}

		if(changed == '1'){
			serial_string(" option selected: ");
			if(rotery_state == 0){
				option_selected = back;
			}else if(rotery_state == 1){
				option_selected = plant1;
				
			}else if(rotery_state == 2){
				option_selected = plant2;
				
			}else if(rotery_state == 3){
				option_selected = plant3;
			}

			for(j=1;j<10;j++){
				serial_out(option_selected[j]);
			}
			changed = '0';
		}
	}

	
	// if removed a plant annd used to have maximum plants now reset that flag
	if(flag_plant_removed == '1' && flag_max_plants_reached_flag == '1'){
		flag_max_plants_reached_flag = '0';
	} 
	if(plant1[0] == '0' & plant2[0] == '0' & plant3[0] == '0'){
		removed_all_plants_flag = '1';
	} 


	// TO DO: state transitions listed below
	// if back button selected: go to select functionality unless removed all of the plants then go to reccomended state 
	if(select_back_flag == '1' && removed_all_plants_flag){ // make back button selected 0 
		state = RESULTS_STATE;
	}else if (select_back_flag == '1' ){
		state = SELECT_FUNCTION_STATE;
	}
}


void monitor_state(void){
	serial_string(" in monitor state function ");

	char warning_flag, button_pressed_flag = '0';

	//TO DO: set up timer to poll sensors 4 times a day and then update the running averages and flags if watered
	// update the long term averages once a week? 

	// if there is something that brings plant outside of ranges or needs watering turn on diode

	// TO DO: LCD display to display monitoring symbol, back button 

	//TO DO: state transitions listed below
	// if back button pressed go back to the select functions state
	// if warning alert triggered go to warning state
	// prioritize warning alert over back button
	if(warning_flag == '1'){
		state = WARNING_STATE;
	} else if(button_pressed_flag == '1'){
		state = SELECT_FUNCTION_STATE;
	}
}

void warning_state(void){
	serial_string(" in warning state function ");

	char button_pressed_flag = '0';

	// TO DO: display the warning depending on the warning given

	// TO DO: LCD display to display warning message and an ok button

	// state transitions listed below
	// if okay button pressed go back to the select function state
	if(button_pressed_flag == '1'){
		state = SELECT_FUNCTION_STATE;
	}
}

int main(void){

	// for testing purposes only
	// TO DO: end of project remove
	serial_init(63);
	int i = 0;

	// TO DO: LCD Display Boot Up Screen with Name for a set amount of time 
	// LCD SCREEN 1

	// READ FLAG FROM EEPROM - is this the first time the device has been powered on since its been programmed 
	char first_po_flag = '1'; // TO DO: read this value from EEPROM	
	char num_plants_selected = '0';


	// only look for the number of plants if it is not the first power on
	if(first_po_flag != '1'){
		num_plants_selected = '3'; // TO DO: iterate through all of the plants and find this number of the plants currently planted 
		if(num_plants_selected == 3){
			flag_max_plants_reached_flag = '1';
		}
	}
	
	
	//state is initially set to the POWER_ON state 
	state = POWER_ON_STATE;


	for(i=0; i<10; i++){
		switch(state){
			case POWER_ON_STATE:
				power_on_state(first_po_flag, num_plants_selected);
				break;
			case CALIBRATION_STATE:
				calibration_state();
				break;
			case SELECT_FUNCTION_STATE:
				select_function_state();
				break;
			case RESULTS_STATE:
				results_state();
				break;
			case EDIT_PLANTS_STATE:
				edit_plants_state();
				break;
			case WARNING_STATE:
				warning_state();
				break;
			case MONITER_STATE:
				monitor_state();
				break;
			default:
				serial_string(" not in state ");
		}
	}
	
	
    return 0;  
}


