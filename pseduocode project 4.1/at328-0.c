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

#define POWER_ON_STATE 0000001
#define CALIBRATION_STATE 0000010
#define SELECT_FUNCTION_STATE 0000100
#define EDIT_PLANTS_STATE 0001000
#define RESULTS_STATE 0010000
#define WARNING_STATE 0100000
#define MONITER_STATE 100000

char state;
char flag_max_plants_reached_flag = '0';



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

	// stay in this state for 2 min

	//poll sensors every 10 seconds and create averages
	// TO DO: poll sensors and add averages
	// TO DO: add sensor functions and integrate the polling
	int i = 0;

	for(i=0;i<12;i++){
		_delay_ms(10000);
		serial_string(" poll ")
	}


	// State transition solely based on time, afterwards go to results state to see the result
	state = RESULTS_STATE;	
	
}

void select_function_state(void){
	serial_string(" in select function state function ");

	// TO DO: LCD Display three options : add new plant, edit current plants, or monitor
	// LCD SCREEN 3

	// rotery encoder to cycle through the options, on screen order will be add new plants, edit plants, then monitor from left to right
	//TO DO: add rotery encoder selection
	char rotery_state = '0';

	// TO DO: add interrupt to change rotery state
	
	// TO DO: State transitions listed below
	// if select moniter go to moniter state
	// if select edit plants go to edit plants state
	// if select add new plant and not max number of plants reached then go to results state
	// if select add new plant and max number of plants is reached then go to warning state
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

	//TO DO: algorithm taking in current plant averages if first time callibrated use that data else use long term averages
	// perhaps have 1st values saved in long term averages be after the callibration stage

	// TO DO: LCD Display options from algorithm ranked, give option to select plant to moniter only if plant is not already being monitered
	//LCD SCREEN 4
	
	// TO DO: rotery encoder to cycle through selecting available plants
	char rotery_state, select_plant_flag = '0';

	// TO DO: if plant is selected then update the EEPROM, if plant is selected update the select plant flag

	// TO DO: State transitions listed below
	// if plant is selected go back to select function state
	if(select_plant_flag){
		state = SELECT_FUNCTION_STATE;
	}
}


void edit_plants_state(void){
	serial_string(" in edit plants state function ");

	// TO DO: go into EEPROM and view which plants have been selected
	// TO DO: LCD display the selected plants with x's so these can be removed, back button 
	// TO DO: rotery encoder to cycle through the available plants

	char rotery_state = '1'; // rotate through all available plants and back button at bottom 
	char removed_all_plants_flag = '0';


	// TO DO: state transitions listed below
	// if back button selected: go to select functionality unless removed all of the plants then go to reccomended state 
	if(rotery_state == 0 && removed_all_plants_flag){ // make back button selected 0 
		state = RESULTS_STATE;
	}else if ( rotery_state == 0 && ! removed_all_plants_flag){
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


	for(i=0; i<5; i++){
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
			case RESULTS_STATE:
				results_state();
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


