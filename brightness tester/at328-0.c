/*********************************************************************
*       i2c - I/O routines for the ATmega168 TWI interface,
*       which is the same functionally as I2C.
*
*       Note: These routines were written to help students in EE459Lx
*       at USC.  They may contain errors, and definitely are not
*       ready for use in a real product.  Use at your own risk.
*
* Revision History
* Date     Author      Description
* 04/14/11 A. Weber    First release
* 02/07/12 A. Weber    Added i2c_write and i2c_read1 routines
* 02/07/12 A. Weber    Added i2c_write1 routine
* 02/17/12 A. Weber    Changes to comments and types
* 04/19/12 A. Weber    Combined write and read routines into one function
* 05/08/12 A. Weber    Added code to handle NAKs better
* 04/09/15 A. Weber    More comments
*********************************************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>


uint8_t i2c_io(uint8_t, uint8_t *, uint16_t, uint8_t *, uint16_t, uint8_t *, uint16_t);
void i2c_init(unsigned char);


// Find divisors for the UART0 and I2C baud rates
#define FOSC 9830400            // Clock frequency = Oscillator freq.
#define BAUD 9600               // UART0 baud rate
#define MYUBRR FOSC/16/BAUD-1   // Value for UBRR0 register
#define BDIV (FOSC / 100000 - 16) / 2 + 1    // Puts I2C rate just below 100kHz

/* Address of the Moisture Sensor on the I2C bus */
#define MOISTURE_ADDR  0x52

/* Address where we store test data in the EEPROM */
#define DATA_ADDR  500


void serial_init ( unsigned short ubrr ) {
UBRR0 = ubrr;// Set baud rate
UCSR0B |= (1 << TXEN0 ); // Turn on transmitter
UCSR0B |= (1 << RXEN0 ); // Turn on receiver
UCSR0C = (3 << UCSZ00 ); // Set for async . operation , no parity ,
// one stop bit , 8 data bits
}


void serial_string_special( char *string){

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


int main(void) {
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
	status = i2c_io(MOISTURE_ADDR, adata, 1, NULL, 0, rdata, 1); //bytes, reading id from device

    // checks to make sure the ID is correct 
    if(rdata[0] == 0x50){
        serial_string_special("   found");
    }
    status = i2c_io(MOISTURE_ADDR, p_enable, 1, wr_p_enable, 1, check_po, 1); //bytes, powers on device
    if(check_po[0] == 1){
        serial_string_special(" powered on ");
    }else{
        serial_string_special(" powered off ");
    }


    status = i2c_io(MOISTURE_ADDR, set_gain_a, 1, set_gain_w, 1, NULL, 0); //bytes, sets params


    while(1){
        serial_string_special(" incoming data array ");
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

        serial_string_special(" visible light ");
        unsigned int x = buff[1];
        unsigned char other[10];
        x = x<<8;
        x = x | buff[0];
        
        snprintf(other, 10, "%d", x);
        serial_string_special("\n");
        serial_string_special(other);

        serial_string_special(" infared light ");
        x = buff[3];
        other[10];
        x = x<<8;
        x = x | buff[2];
        
        snprintf(other, 10, "%d", x);
        serial_string_special("\n");
        serial_string_special(other);

    }
    status = i2c_io(MOISTURE_ADDR, set_gain_a, 1, wr_p_disable, 1, NULL, 0); //bytes, powers off device

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
