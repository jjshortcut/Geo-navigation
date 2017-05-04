/*
* Main.c
* BikeNav project
* Phone connected Bluetooth bike navigation with digital compass and Neoleds ring
* Created: 2-4-2016 20:19:54
*  Author: J-J_LAPTOP
* Version: 0.1
*/

/*
TODO:
- Baudrate much to fast for not calibrated internal oscillator
- Changed lat/lon etc. to float (was double)
- Read battery voltage
- Startup code (maybe add delay for startup?)
- Set ADC correct (reference)
- Percent function with color change (to 3 is red etc.)
- myloc-groningen: b52.15,5.3833;53.2194,6.5665e (143km, 33bearing)
- Function per color (arrow color =..)
- check OCR0A = (((F_CPU/1024)*(DISPLAY_REFRESH_MS/1000))-1); ->16ms = 62.5hz
- can 62.5hz/16ms too fast for other processes to keep up?
- on reset the display should be emptied faster
- changed: data_in is now char6 (not unsigned char)
- Extra: flashlight mode (bike mode)
- Led percentage does not take in account the angle/north
- Button press freezes (due to other pins? not 128?
- Write startup calibration procedure (rotate until....), save in eeprom
- Calibrate...
- Filter outliers from compass
- enum maken van statuses!
- Gain of compass OK? if reading = -4096 the gain is to high (overflow)
- Save calibration in EEPROM?
- fix setLedValue 0's between values
- Error is now calculated, make sure to check on this error
*/

//#include "global.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>

#include <stdlib.h>
//#include <string.h>
#include <stdio.h>
//#include <avr/pgmspace.h>
#include "defines.h"
//#include <avr/eeprom.h>
//#include "math.h"
//#include "uart/uart.h"
#include "uart_handler.h"
//#include "ui.h"
#include "LSM303_simple.h"
//#include "i2chw/i2cmaster.h"

//#include "hmc5883l/hmc5883l.h"

#define OSCILLATOR_CAL_VAL 156	/* Calibrate the internal oscillator*/

#define MAGN_DOCALIBRATION 0 //enable or disable magnetometer calibration
#define magncal_getrawdata(mxraw, myraw, mzraw) hmc5883l_getrawdata(mxraw, myraw, mzraw); //set the function that return magnetometer raw values
#include "magn_docalibration.h"

// Prototypes
void init_io(void);
void init_int(void);
void init_adc(void);
void init_bluetooth(void);

uint8_t codeSeen(char code, char *line, char **str_ptr);

void do_button_action(void);	

void print_status(void);
uint16_t filtered_compass_reading(uint16_t heading_current);
int rollingAverage(int newValue);


double read_compass(void);
void read_axis(void);

/* Settings stored in EEPROM */
uint16_t EEMEM eeprom_first_startup, eeprom_brightness, eeprom_displaymode, eeprom_showdistance, eeprom_shownorth;
uint32_t EEMEM eeprom_navigationcolor;

//uint8_t new_data = 0;
//uint8_t device_status = FALSE;

// Compass vars
/*
int16_t mxraw = 0;
int16_t myraw = 0;
int16_t mzraw = 0;
double mx = 0;
double my = 0;
double mz = 0;*/

ISR (TIMER0_COMPA_vect)  // timer0 overflow interrupt
{
	volatile static uint8_t refresh_counter;
	refresh_counter++;
	if (refresh_counter>=(DISPLAY_REFRESH_MS/INTERRUPT_DISPLAY_MS))
	{
		UI.refresh = TRUE;
		refresh_counter = 0;
		//TEST_TOGGLE;
	}
}

int main(void)
{
	//OSCCAL = OSCILLATOR_CAL_VAL;		/* Calibrate the internal oscillator*/
	init_io();			/* Init IO*/
	init_int();			/* Init int for resfresh display every x ms	*/
	init_adc();			/* Init ADC */
	
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );	/* Init Uart */
	sei();						/* Enable global interrupts for uart*/
	uart_puts("Init Uart OK\n");
	
	//init_bluetooth();			/* Setup the bluetooth device (1 time only)	*/
	init_lsm303();				/* Init the acc/mag sensor				*/
	uart_puts("Init LSM303 OK\n");
	//buzzer(BUZZER_SHORT);
	
	/* Set initial values of device */
	set_initial_device_status();			/* Start condition */
	device.compass = get_heading_avg(25);	/* Get initial value */
	
	if(eeprom_read_word(&eeprom_first_startup))
	{
		eeprom_update_word(&eeprom_first_startup, FALSE);
		reset_factory_settings();	/* Set device to initial values*/
		uart_puts("First time startup, doing factory presets\n");
	}
	
	/* Load initial values of device */	
	load_eeprom_settings();		/* Load program settings from EEPROM	*/
	
	/* Now the device is ready! */
	uart_puts("BikeNav 2017 V");		/* Print version number					*/
	print_float(VERSION,1);

	UI.navigationcolor = YELLOW;	/* Set color of navigation */
	clearLeds();					/* Clear leds */
	
	device.battery = read_battery();
	uart_puts("Battery = ");
	print_int(device.battery,0);
	uart_puts("%\n");
	//setLedPercentage(device.battery,MULTIPLE,GREEN,100);	/* Show battery percentage on display */
	//_delay_ms(750);	
	clearLeds();					/* Clear leds */
	
	device.temperature = get_temp();
	uart_puts("Temperature = ");
	print_int(device.temperature,0);
	uart_puts(" deg. C.\n");
	//setLedValue(device.temperature,100);
	//_delay_ms(750);		/* Wait for the user to be able to see the battery percentage */
	//uart_puts("TODO: FIX UART receive coordinates!!\n");
	while(1)
	{	
		get_serial();
		if (command_ready)
		{
			//process_command();
		}
		//process_serial();							/* Check for serial messages */	
		
		//device.compass = rollingAverage(get_heading_avg(10));	// filter out noise spikes or quick changes
		//device.compass = get_heading_avg(10);
		device.compass = filtered_compass_reading(device.compass); // Has a threshold for new compass sensor data
		//device.compass = rollingAverage(device.compass);	// filter out noise spikes or quick changes
		//device.compass = get_heading_avg(10);		/* get compass values (north) */
		
		
		if (UI.refresh)	// Refresh display
		{
			print_status();
			check_device_status();						/* Check status of device for time-out's and button etc. */
			refreshDisplay(device.compass, device.heading, read_device_status());	
		}
		
		if (device.buttonaction)	// button action active
		{
			do_button_action();	
		}
		
		//uart_puts("NORTH = ");
		//print_int(device.compass,TRUE);
				
		//i = (i>0) ? i-1 : 15;
		//device.distance=i;
		//a = (a<360) ? a+10 : 0;
		
		//refreshDisplay(heading, 0);	/* Refresh display */
		//uart_puts("Battery = ");
		//device.battery = read_battery();
		//print_int(device.battery,0);

		_delay_ms(REFRESH_LOOP_MS);
	}
}

void init_io(void)
{
	//DDRC &= ~(1 << PORTC4) | (1 << PORTC5);
	//PORTC = (1 << PORTC4) | (1 << PORTC5); // enable pull-ups on SDA and SCL, respectively
	
	//PWR_ON_INIT;				// Init power on pin
	
	SW_PIN_DDR &= ~(1<<SW_PIN);	// Buttons/switches as input
	SW_PIN_PORT |= (1<<SW_PIN);	// With pullups on
	
	CHARGE_ST_DDR &= ~(1<<CHARGE_ST_PIN);	// Charge status pin as input
	
	DDRC |= (1<<TEST_PIN);	// Init test pin as output
	DDRB |= (1<<0);	// PB0 AS OUTPUT
	BATT_PWR_INIT;	// Leds as output
	BATT_PWR_OFF;	// Leds off
	
	//AT_PIN_INIT;
	//AT_ON;
}

void init_int(void)
{
	//OCRn =  [ (clock_speed / Prescaler_value) * Desired_time_in_Seconds ] - 1
	
	//print_int((((F_CPU/1024)*(DISPLAY_REFRESH_MS/1000))-1),TRUE);
	TCCR0A |= (1 << WGM01);		// Set the Timer Mode to CTC
	//OCR0A = 125;				// Set the value that you want to count to 16ms
	OCR0A = (((((F_CPU/1024)*INTERRUPT_DISPLAY_MS)/1000))-1);
	
	TIMSK0 |= (1 << OCIE0A);			//Set the ISR COMPA vect
	TCCR0B |= (1 << CS00)|(1<<CS02);	// set prescaler to 1024 and start the timer
	sei();								//enable global interrupts
}

void init_adc(void)
{
	ADMUX |= (1<<REFS0);						/* Select Vref=AVcc */
	ADCSRA |= (1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0);	/* set prescaller to 128  */
	ADCSRA |= _BV(ADEN);						/* Enable ADC */
}




uint16_t read_adc(uint8_t adcx)
{
	/* adcx is the analog pin we want to use. ADMUX's first few bits are
	* the binary representations of the numbers of the pins so we can
	* just 'OR' the pin's number with ADMUX to select that pin.
	* We first zero the four bits by setting ADMUX equal to its higher
	* four bits. */
	//select ADC channel with safety mask
	ADMUX = (ADMUX & 0xF0) | (adcx & 0x0F);
	
	/* This starts the conversion. */
	//single conversion mode
	ADCSRA |= (1<<ADSC);
	
	/* This is an idle loop that just wait around until the conversion
	* is finished. It constantly checks ADCSRA's ADSC bit, which we just
	* set above, to see if it is still set. This bit is automatically
	* reset (zeroed) when the conversion is ready so if we do this in
	* a loop the loop will just go until the conversion is ready. */
	while( ADCSRA & (1<<ADSC) );
	
	/* Finally, we return the converted value to the calling function. */
	return ADC;
}

/*
double read_compass(void)
{
	float reading = 0;
	
	hmc5883l_getrawdata(&mxraw, &myraw, &mzraw);
	reading = atan2((double)myraw,(double)mxraw);
	
	//hmc5883l_getdata(&mx, &my, &mz);
	//reading = atan2((double)my,(double)mx);
	
	//get magnetic heading (in degrees)
	
	//north_deg = atan2((double)myraw,(double)mxraw)*57.29578;
	
	//add magnetic declination (optional)
	//get magnetic declination (in degrees) http://magnetic-declination.com
	//if you have an EAST declination use +, if you have a WEST declination use -
	//es, my declination is 1.73 positive
	//float declination = 1.6;
	//reading += declination;
	
	if(reading < 0) reading += 2*PI;
	if(reading > 2*PI) reading -= 2*PI;
	
	//check 360degree heading
	//if(north_deg < 0)
	//north_deg = 360 + north_deg;

	//itoa(mxraw, itmp, 10); uart_puts(itmp); uart_putc(' ');
	//itoa(myraw, itmp, 10); uart_puts(itmp); uart_putc(' ');
	//itoa(mzraw, itmp, 10); uart_puts(itmp); uart_putc(' ');
	//dtostrf(mx, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
	//dtostrf(my, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
	//dtostrf(mz, 3, 5, itmp); uart_puts(itmp); uart_putc(' ');
	reading = rtod(reading);
	
	
	if ((reading+PCB_COMPASS_CORR)>=360)
	{
	//	reading = (reading+PCB_COMPASS_CORR)-360;	// PCB mounting correction 180 deg
	}
	else
	{
		//reading += PCB_COMPASS_CORR;	// PCB mounting correction 180 deg
	}
	

	return (reading);
}*/
/*
void read_axis(void)
{
	hmc5883l_getrawdata(&mxraw, &myraw, &mzraw);
	hmc5883l_getdata(&mx, &my, &mz);

	uart_puts("X RAW = "); print_float(mxraw, 0);
	uart_puts(" Y RAW= "); print_float(myraw, 0);
	uart_puts(" Z RAW = "); print_float(mzraw, 0);

	uart_puts(" X = "); print_float(mx, 0);
	uart_puts(" Y = "); print_float(my, 0);
	uart_puts(" Z = "); print_float(mz, 1);
}*/

void do_button_action(void)
{
	switch (device.buttonaction)
	{
		case SHORT_PRESS:	// Minus
			if (UI.displaymode>0)
			{
				UI.displaymode--;
			}
			
			if (debug_on)
			{uart_puts("Short press action executed\n");}
		break;
		
		case MIDDLE_PRESS:	// Plus
			if (UI.displaymode<7)
			{
				UI.displaymode++;
			}
			
			if (debug_on)
			{uart_puts("Middle press action executed\n");}
		break;
		
		case LONG_PRESS:
			uart_puts("next\n");			// Found coordinate, go to new one in App			
			if (debug_on)
			{uart_puts("Long press action executed\n");}	
		break;
		
		default:
			uart_puts("Error in button press! (fix!)\n");
		break;
	}
	device.buttonaction = FALSE;
}

void print_status(void)
{
	uart_puts("Device status = ");
	switch (read_device_status())
	{
		case NO_CONNECTION:
			uart_puts("NO CONNECTION\n");
		break;
		
		case NAVIGATING:
			uart_puts("NAVIGATING\n");
		break;
		
		case BUTTON_PRESSED:
			uart_puts("BUTTON PRESSED\n");
		break;
		
		case BUTTON_NOT_PRESSED:
			uart_puts("BUTTON NOT PRESSED\n");
		break;
		
		case AT_LOCATION:
		uart_puts("AT_LOCATION\n");
		break;
		
		default:
			uart_puts("ERROR!\n");
		break;
	}
}

uint16_t filtered_compass_reading(uint16_t heading_current)
{
	int16_t heading_new = get_heading_avg(100);
	uint16_t difference = 0;
	static uint16_t error_counter = 0;
	int error = 0;
	
	if (heading_current>heading_new)
	{
		difference = heading_current - heading_new;
	}
	else
	{
		difference = heading_new - heading_current;
	}
	if (difference>180)
	{
		difference -= 360;	// for angles > 180 correct in the opposite direction
	}
	
	error = difference;
	
	
	// Calculate error
	//error = heading_current - heading_new;
	//if (error>180)
	//{
	//	error -= 360;	// for angles > 180 correct in the opposite direction
	//}
	
	/*if (heading_current>=heading_new)
	{
		if ((heading_current-heading_new)<=((heading_new+360)-heading_current))
		{
			difference = heading_current-heading_new;	// smalles result
		}	
		else
		{
			difference = (heading_new+360)-heading_current;
		}
	}
	else
	{
		if ((heading_new-heading_current)<=((heading_current+360)-heading_new))
		{
			difference = heading_new - heading_current;	// smalles result
		}
		else
		{
			difference = (heading_current+360)-heading_new;
		}
	}*/
	
	if (debug_on)
	{
		uart_puts("Current: ");
		print_int(heading_current,FALSE);
		uart_puts(" New: ");
		print_int(heading_new,FALSE);
		uart_puts(" Difference: ");
		print_int(difference,FALSE);
		uart_puts(" Error: ");
		print_int(error,FALSE);
	}
	
	//print_int(heading_current,TRUE);
	
	if ((error>SENSOR_THRESHOLD)&&(error_counter<ERROR_THRESHOLD_NR))
	{
		if (debug_on)
		{uart_puts(" Outlier!\n");}
		error_counter++;
		return (heading_current);	// data was an outlier	
	}
	else
	{
		if (debug_on)
		{uart_puts(" OK\n");}
		error_counter=0;
		return (heading_new);
	}
}

int rollingAverage(int newValue)
{
	static int values[8];		// Buffer nr
	static int nextPosition;	// Index
	static int sum;				// Sum of samples
	
	if (nextPosition>7)
	{
		nextPosition = 0;	// Reset index
	}

	sum -= values[nextPosition];	// Subtract oldest value
	sum += (newValue+360);				// Add new value
	values[nextPosition] = newValue+360;// Add new value to the sum
	nextPosition++;					// Set index to oldest value, one position forward from the value just inserted
	return (sum/8)-360;
}
/*
{
	//1/2 averaging kicks out on a noise spike.
	// Do whatever needs to be done for averaging.  Let's try a 2 sample average
	heading_total += raw_heading;	// Add the raw data to the stream
	heading_total /= 2;
	return heading_total;
}*/

void load_eeprom_settings(void)
{
	UI.brightness = eeprom_read_word(&eeprom_brightness);
	UI.displaymode = eeprom_read_word(&eeprom_displaymode);
	UI.navigationcolor = eeprom_read_dword(&eeprom_navigationcolor);
	UI.showdistance = eeprom_read_word(&eeprom_showdistance);
	UI.shownorth = eeprom_read_word(&eeprom_shownorth);
}

void reset_factory_settings(void)
{
	setBrightness(45);				/* Set brightness of leds */
	setUIMode(NORMAL);				/* Set display modus */
	UI.navigationcolor = YELLOW;	/* Set color of navigation */
	UI.showdistance = FALSE;
	UI.shownorth = FALSE;
	
	/* Save values in EEPROM */
	eeprom_update_word(&eeprom_brightness, getBrightness());
	eeprom_update_word(&eeprom_displaymode, getUIMode());
	eeprom_update_dword(&eeprom_navigationcolor, YELLOW);	/* 32 bits! */
	eeprom_update_word(&eeprom_showdistance, FALSE);
	eeprom_update_word(&eeprom_shownorth, FALSE);
}

void save_eeprom_parameter(uint16_t param, uint16_t value)
{
	eeprom_update_word((uint16_t *)&param, value);	// Write to eeprom
}

uint16_t load_eeprom_parameter(uint16_t param)
{
	return eeprom_read_word(&param);
}

void set_initial_device_status(void)
{
	device.status = NO_CONNECTION;
	device.previousstatus = NO_CONNECTION;
}

uint8_t set_device_status(uint8_t status)
{
	device.previousstatus = device.status;	// Save previous state
	device.status = status;					// Write new state
	return device.status;
}

uint8_t set_previous_device_status(void)
{
	device.status = device.previousstatus;	// Go to previous state
	return device.previousstatus;
}

uint8_t read_device_status(void)
{
	return device.status;
}

/*
float codeValue(char *line) {
	return (strtof(line + 1, NULL)); //Pointer is to gcode char. Add 1 for value
}

long codeValueLong(char *line) {
	return (strtol(line + 1, NULL, 0)); //Pointer is to gcode char. Add 1 for value
}

uint8_t codeSeen(char code, char *line, char **str_ptr) {
	*str_ptr = strchr(line, code);
	return (*str_ptr != NULL); //Return True if a character was found
}

*/










