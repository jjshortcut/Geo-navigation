/*
* Uart handler, handles the incoming and outgoing stream
*/

#include <avr/io.h>
#include "defines.h"
#include "uart_handler.h"
#include "LSM303_simple.h"
#include <util/atomic.h>
#include <string.h>
#include <avr/interrupt.h>


uint8_t debug_on = 0;	// Debug msg's

void process_serial(void)
{
	uint16_t val = 0;
	
	unsigned int c = uart_getc();
	if (!(c & UART_NO_DATA)	// If data is correct
	&&	!(c & UART_FRAME_ERROR)
	&&	!(c & UART_OVERRUN_ERROR)
	&&	!(c & UART_BUFFER_OVERFLOW)
	)
	{
		switch (c)
		{
			case 'A':	// Set led
				TEST_ON;
				if (debug_on)	uart_puts("Set led\n");
			break;
			
			case 'a':	// reset led
				TEST_OFF;
				if (debug_on)	uart_puts("Reset led\n");
			break;
			
			case 'b':	/* Begin of message from phone */
				if (receiveCoordinates())
				{
					if (debug_on)
					{
						uart_puts("Received correct coordinates:\n");
						uart_puts("Location:");
						print_float(lat_current,0);
						uart_puts(",");
						print_float(lon_current,0);
						uart_puts(" Destination: ");
						print_float(lat_dest,0);
						uart_puts(",");
						print_float(lon_dest,1);
					}
				
					device.distance = calculate_distance(lat_current,lon_current,lat_dest,lon_dest);
					device.heading = calculate_bearing(lat_current,lon_current,lat_dest,lon_dest);
					if (debug_on)
					{
						uart_puts("Distance = ");
						print_float(device.distance,0);
						uart_puts("KM\n");
						uart_puts("Bearing = ");
						print_float(device.heading,0);
						uart_puts("deg\n");
					}
					uart_puts("OK\n");
					device.newdata=TRUE;
				}
				else
				{
					if (debug_on)
					{
						uart_puts("Received wrong coordinate format\n");	
					}
				}	
			break;
			
			case 'B':	// Check battery adc value
				device.battery = read_battery();
				setLedPercentage(device.battery,MULTIPLE,GREEN,125);	/* Show battery percentage on display */
				_delay_ms(750);		/* Wait for the user to be able to see the battery percentage */
				clearLeds();
				//read_battery();
			break;
			
			case 'c':	// Give calibration values
				print_calibration_lsm303();	/* Print calibration values of this LSM303 sensor (min/max data) */
			break;
			
			case 'C':	// Check charge status
				device.chargingstate = read_charge_status();
			break;
			
			case 'D':	/* Turn on/off distance on display */
				UI.showdistance = (UI.showdistance) ? FALSE : TRUE;
				eeprom_update_word(&eeprom_showdistance, UI.showdistance);	/* Save in EEPROM */
				if (debug_on)
				{
					uart_puts_p(PSTR("Show UI distance = "));
					print_int(UI.showdistance,1);
				}
			break;
			
			case '?':	// Debug msg on
				if (debug_on)
				{
					debug_on = 0;
					uart_puts_p(PSTR("Debug msg's off\n"));
				}
				else
				{
					debug_on = 1;
					uart_puts_p(PSTR("Debug msg's on\n"));
				}
			break;
			
			case 'e':	/* direction test */
				val = readCommand();
				if (val>=0 && val<=100)
				{
					device.distance = val;
					device.newdata=TRUE;
				}
			
				if (debug_on)
				{
					uart_puts_p(PSTR("Compass value set to"));
					print_int(device.compass,1);
				}
			break;
			
			case 'F':	/* Flashlight mode */
				if (debug_on)
				{
					uart_puts("Flashlight mode = TODO! ");
				}
				device.newdata=TRUE;
			break;
			
			case 's':	/* Set navigation color */
				uart_puts_p(PSTR("TODO, implement set navigation color\n"));
			break;
			
			case 'I':	/* Intensity of display */
				setBrightness(readCommand());
				eeprom_update_word(&eeprom_brightness, getBrightness());	/* Save in EEPROM */
				device.newdata=TRUE;
				if (debug_on)
				{
					uart_puts_p(PSTR("Brightness set to:"));
					print_int(getBrightness(),1);
				}
			break;
			
			case 'N':	/* Show north point on display */
				UI.shownorth = (UI.shownorth) ? FALSE : TRUE;
				eeprom_update_word(&eeprom_shownorth, UI.shownorth);	/* Save in EEPROM */
				if (debug_on)
				{
					uart_puts_p(PSTR("Show UI North = "));
					print_int(UI.shownorth,1);
				}
			break;
			
			case 'P':	// Power off
				if (debug_on)
				{
					uart_puts_p(PSTR("Powering off..\n"));
					control_power(OFF);
				}
			break;
			
			case 'r':	// Reset device to factory settings
				if (debug_on)
				{
					uart_puts_p(PSTR("Resetting Bluetooth settings!\n"));
				}
				init_bluetooth();
			break;
			
			case 'R':	// Reset device to factory settings
				if (debug_on)
				{	
					uart_puts_p(PSTR("Device reseted to factory settings!\n"));
				}
				reset_factory_settings();
			break;
			
			case 'S':	// Check switch
				device.buttonstate = read_button();
			break;
			
			case 'T':	// get temp of LSM303 sensor
				uart_puts("Temperature = ");
				print_int(get_temp(),FALSE);
				uart_puts(" deg. C.\n");
			break;
			
			case 'u':	/* Change UI mode */
				if (debug_on)
				{
					uart_puts("UI mode++");
				}
				setUIModeNext();
				eeprom_update_word(&eeprom_displaymode, getUIMode());	/* Save in EEPROM */
				device.newdata=TRUE;
			break;
			
			case 'U':	/* Change UI mode */
				//setUIMode();
				setUIMode(readCommand());
				if (debug_on)
				{
					uart_puts("UI mode = ");
					print_int(getUIMode(),1);
				}
				eeprom_update_word(&eeprom_displaymode, getUIMode());	/* Save in EEPROM */
				device.newdata=TRUE;
			break;
			
			case 'K':	/* Change navigation color */			
				if (debug_on)
				{
					uart_puts("Navigation color set\n");
					//print_int(getUIMode(),1);
				}
				setNavigationColorNext();
					
				//eeprom_update_word(&eeprom_displaymode, getUIMode());	/* Save in EEPROM */
				device.newdata=TRUE;
				break;
			
				default:
				uart_puts("No valid command:");
				uart_putc(c);
				uart_puts(" (");
				print_int(c, 0);
				uart_puts(")\n");
				uart_puts_p(PSTR("'?' = debug messages\n"));
			break;
		}
		while((uart_getc()!=UART_NO_DATA)) {uart_getc();}	/* Empty buffer */
	}
}

uint8_t receiveCoordinates(void)
{	// Format: |b|lat_current|,|lon_current|;|lat_dest|,|lon_dest|e|
	// ex. b123.456,456.789;222.333,444.555e 
	uint8_t data_count = 0;
		
	//c = uart_getc();	// get new byte
	//c = getchar();
	unsigned int c = uart_getc();
	/*while (!(c & UART_NO_DATA)	// If data is correct
	&&	!(c & UART_FRAME_ERROR)
	&&	!(c & UART_OVERRUN_ERROR)
	&&	!(c & UART_BUFFER_OVERFLOW)
	&&	!(c == '\n')*/
	//)
	{
		data_in[data_count] = c;
		//uart_putc(c);
		//uart_puts("-data-\n");
		
		// End of line!
		//print_int(data_in[data_count],1);
		//if (data_in[data_count] == '\n') {
			//command_ready = 1;
			//uart_puts("seen \\n\n");
			// Reset to 0, ready to go again
			//data_count = 0;
		//	} else {
			data_count++;
		//}
		
		c = uart_getc();	// get new byte
	}
	return read_int_value();	// fill vars
}
/*
uint8_t read_int_value (void)
{
	char *pch;
	//double val=0;
	// Find the position the equals sign is
	// in the string, keep a pointer to it
	//pch = strchr(command_in, ';');
	
	uart_puts("data in =<");
	for(uint8_t i=0; i<sizeof(data_in); i++) {
		uart_putc(data_in[i]);
	}
	uart_puts(">\n");
	
	pch = strtok (data_in,",");	// return pointer to next token 
	lat_current = atof(pch);
	
	pch = strtok (NULL, ";");
	lon_current = atof(pch);
	
	pch = strtok (NULL, ",");
	lat_dest = atof(pch);
	
	pch = strtok (NULL, "e");
	lon_dest = atof(pch);
	
	uart_puts("floattest:\n");
	char temp[20];
	sprintf(temp,"\ndata_f=%f",lat_current);
	uart_puts(temp);
	
//b123.456,456.789;222.333,444.555e
//b111.222,333.444;555.666,777.888e
	
	//uart_puts("printvalue\n");
	//print_value("T", lat_current);
	
	//pch = strtok (NULL, ";");
	
	
	//pch = strtok (NULL, ";");//pch = strtok (data_in,";");	// return pointer to next token 
	//strcpy(cmdValue, pch);
	
	//while (pch != NULL)
//	{
	//	uart_puts(pch);
		//uart_puts("\n");
		//pch = strtok (NULL, ";");
	//}
	
	// Copy everything after that point into
	// the buffer variable
	
	// Now turn this value into an integer and
	// return it to the caller.
	memset(data_in, 0, sizeof(data_in));	
	
	if ((lat_current>0) && (lon_current>0) && (lat_dest>0) && (lon_current>0))
	{
		return 1;
	}
	
	return 0;
}*/

/*
{
	char str[] ="- This, a sample string.";
	char * pch;
	printf ("Splitting string \"%s\" into tokens:\n",str);
	pch = strtok (str," ,.-");
	while (pch != NULL)
	{
		printf ("%s\n",pch);
		pch = strtok (NULL, " ,.-");
	}
	return 0;
}
*/

uint16_t readCommand(void)
{
	char *pch;
	char cmdValue[16];
	char data[10];
	uint8_t counter = 0;
	unsigned int c;
	
	c = uart_getc();	// get new byte
	
	//unsigned int c = uart_getc();
	while (!(c & UART_NO_DATA)	// If data is correct
	&&	!(c & UART_FRAME_ERROR)
	&&	!(c & UART_OVERRUN_ERROR)
	&&	!(c & UART_BUFFER_OVERFLOW)
	)
	{
		data[counter] = c;
		if (data[counter] == '\n') {
			} else {
			counter++;
		}
		
		c = uart_getc();	// get new byte
	}
	
	/*uart_puts("data in-");
	for(uint8_t i=0; i<sizeof(data); i++) {
		uart_putc(data[i]);
	}
	uart_puts("-\n");*/
	
	if (data[0] == '=') {
			// Find the position the equals sign is
			// in the string, keep a pointer to it
			pch = strchr(data, '=');
			// Copy everything after that point into
			// the buffer variable
			strcpy(cmdValue, pch+1);
			// Now turn this value into an integer and
			// return it to the caller.
			
			return atoi(cmdValue);
	}
	else
	{
		return 0;
	}
}

void print_int(int c, uint8_t ln)
{
	char buffer[8];
	itoa( c, buffer, 10);
	uart_puts(buffer);
	
	if (ln)	// if add line end
	{
		uart_puts("\n");
	}
}

void print_float(double c, uint8_t ln)
{
	char buffer[20];
	dtostrf(c, 10, 7, buffer);
	uart_puts(buffer);
	
	if (ln)	// if add line end
	{
		uart_puts("\n");
	}
}
/*
void print_value (char *id, int *value)
{
	char buffer[8];
	//itoa(value, buffer, 10);
	//uart_putc(id);
	uart_putc(':');
	uart_puts(buffer);
	uart_puts("\n");
}*/

void print_value (char id, int value)
{
	char buffer[8];
	itoa(value, buffer, 10);
	uart_putc(id);
	uart_putc('=');
	uart_puts(buffer);
	uart_putc('\n');
}

void get_serial(void)
{
	unsigned int c = uart_getc();
	if (!(c & UART_NO_DATA)	// If data is correct
	&&	!(c & UART_FRAME_ERROR)
	&&	!(c & UART_OVERRUN_ERROR)
	&&	!(c & UART_BUFFER_OVERFLOW)
	)
	{
		command_in[data_count] = c;
		if (command_in[data_count] == '\n')
		{
			uart_puts("Received =<");
			for(uint8_t i=0; i<data_count; i++) {
				uart_putc(command_in[i]);
			}
			uart_puts(">\n");
			
			command_ready = TRUE;
			process_command();
			data_count = 0;	// reset
			memset(command_in, 0, sizeof(command_in));	// set command to 0
		}
		else
		{
			data_count++;
		}
	}	
}

unsigned long read_int_value ()
{
	char *pch;
	char cmdValue[16];
	// Find the position the equals sign is
	// in the string, keep a pointer to it
	pch = strchr(command_in, '=');
	// Copy everything after that point into
	// the buffer variable
	strcpy(cmdValue, pch+1);
	// Now turn this value into an integer and
	// return it to the caller.
	return atoi(cmdValue);
}

void process_command()
{
	uint16_t val = 0;
	static unsigned int test;
	switch (command_in[0]) {
		case 'O':
		if (command_in[1] == '?') {
			// Do the query action for S
			uart_puts("Asked S value\n");
			print_value('S',test);
			} else if (command_in[1] == '=') {
			test = read_int_value();
			uart_puts("Received S value\n");
			print_value('S',test);
		}
		break;
		
		case 'M':
		if (command_in[1] == '?') {
			// Do the query action for M
			} else if (command_in[1] == '=') {
			// Do the set action for M
		}
		break;
		
		case 'V':	// BUZZER
		if (command_in[1] == '1') 
		{
			PWR_ON;
		}
		
		if (command_in[1] == '0')
		{
			PWR_OFF;
		}
		
		if (command_in[1] == 'S')
		{
			uart_puts("BUZZER!\n");
			buzzer(BUZZER_SHORT);	
		}
		
		if (command_in[1] == 'L')
		{
			uart_puts("BUZZER!\n");
			buzzer(BUZZER_LONG);
		}
		
		break;
		
		case 'A':	// Set led
		TEST_ON;
		if (debug_on)	uart_puts("Set led\n");
		break;
			
		case 'a':	// reset led
		TEST_OFF;
		if (debug_on)	uart_puts("Reset led\n");
		break;
		
			
		case 'b':	/* Begin of message from phone */
		if (receive_gps())
		{
			if (debug_on)
			{
				uart_puts("Received correct coordinates:\n");
				uart_puts("Location:");
				print_float(lat_current,0);
				uart_puts(",");
				print_float(lon_current,0);
				uart_puts(" Destination: ");
				print_float(lat_dest,0);
				uart_puts(",");
				print_float(lon_dest,1);
			}
				
			device.distance = calculate_distance(lat_current,lon_current,lat_dest,lon_dest);
			device.heading = calculate_bearing(lat_current,lon_current,lat_dest,lon_dest);
			if (debug_on)
			{
				uart_puts("Distance = ");
				print_float(device.distance,0);
				uart_puts("KM\n");
				uart_puts("Bearing = ");
				print_float(device.heading,0);
				uart_puts("deg\n");
			}
			uart_puts("OK\n");
			device.newdata=TRUE;
		}
		else
		{
			if (debug_on)
			{
				uart_puts("Received wrong coordinate format\n");
			}
		}
		break;
			
		case 'B':	// Check battery adc value
		device.battery = read_battery();
		setLedPercentage(device.battery,MULTIPLE,GREEN,125);	/* Show battery percentage on display */
		_delay_ms(750);		/* Wait for the user to be able to see the battery percentage */
		clearLeds();
		//read_battery();
		break;
			
		case 'c':	// Give calibration values
		print_calibration_lsm303();	/* Print calibration values of this LSM303 sensor (min/max data) */
		break;
			
		case 'C':	// Check charge status
		device.chargingstate = read_charge_status();
		break;
			
		case 'D':	/* Turn on/off distance on display */
		UI.showdistance = (UI.showdistance) ? FALSE : TRUE;
		eeprom_update_word(&eeprom_showdistance, UI.showdistance);	/* Save in EEPROM */
		if (debug_on)
		{
			uart_puts_p(PSTR("Show UI distance = "));
			print_int(UI.showdistance,1);
		}
		break;
			
		case '?':	// Debug msg on
		if (debug_on)
		{
			debug_on = 0;
			uart_puts_p(PSTR("Debug msg's off\n"));
		}
		else
		{
			debug_on = 1;
			uart_puts_p(PSTR("Debug msg's on\n"));
		}
		break;
			
		case 'e':	/* direction test */
		val = readCommand();
		if (val>=0 && val<=100)
		{
			device.distance = val;
			device.newdata=TRUE;
		}
			
		if (debug_on)
		{
			uart_puts_p(PSTR("Compass value set to"));
			print_int(device.compass,1);
		}
		break;
			
		case 'F':	/* Flashlight mode */
		if (debug_on)
		{
			uart_puts("Flashlight mode = TODO! ");
		}
		device.newdata=TRUE;
		break;
			
		case 's':	/* Set navigation color */
		uart_puts_p(PSTR("TODO, implement set navigation color\n"));
		break;
			
		case 'I':	/* Intensity of display */
		setBrightness(readCommand());
		eeprom_update_word(&eeprom_brightness, getBrightness());	/* Save in EEPROM */
		device.newdata=TRUE;
		if (debug_on)
		{
			uart_puts_p(PSTR("Brightness set to:"));
			print_int(getBrightness(),1);
		}
		break;
			
		case 'N':	/* Show north point on display */
		UI.shownorth = (UI.shownorth) ? FALSE : TRUE;
		eeprom_update_word(&eeprom_shownorth, UI.shownorth);	/* Save in EEPROM */
		if (debug_on)
		{
			uart_puts_p(PSTR("Show UI North = "));
			print_int(UI.shownorth,1);
		}
		break;
			
		case 'P':	// Power off
		if (debug_on)
		{
			uart_puts_p(PSTR("Powering off..\n"));
			control_power(OFF);
		}
		break;
			
		case 'r':	// Reset device to factory settings
		if (debug_on)
		{
			uart_puts_p(PSTR("Resetting Bluetooth settings!\n"));
		}
		init_bluetooth();
		break;
			
		case 'R':	// Reset device to factory settings
		if (debug_on)
		{
			uart_puts_p(PSTR("Device reseted to factory settings!\n"));
		}
		reset_factory_settings();
		break;
			
		case 'S':	// Check switch
		device.buttonstate = read_button();
		break;
			
		case 'T':	// get temp of LSM303 sensor
		uart_puts("Temperature = ");
		print_int(get_temp(),FALSE);
		uart_puts(" deg. C.\n");
		break;
			
		case 'u':	/* Change UI mode */
		if (debug_on)
		{
			uart_puts("UI mode++");
		}
		setUIModeNext();
		eeprom_update_word(&eeprom_displaymode, getUIMode());	/* Save in EEPROM */
		device.newdata=TRUE;
		break;
			
		case 'U':	/* Change UI mode */
		//setUIMode();
		setUIMode(readCommand());
		if (debug_on)
		{
			uart_puts("UI mode = ");
			print_int(getUIMode(),1);
		}
		eeprom_update_word(&eeprom_displaymode, getUIMode());	/* Save in EEPROM */
		device.newdata=TRUE;
		break;
			
		case 'K':	/* Change navigation color */
		if (debug_on)
		{
			uart_puts("Navigation color set\n");
			//print_int(getUIMode(),1);
		}
		setNavigationColorNext();
			
		//eeprom_update_word(&eeprom_displaymode, getUIMode());	/* Save in EEPROM */
		device.newdata=TRUE;
		break;
			
		default:
		uart_puts("No valid command:");
		uart_putc(command_in[0]);
		uart_puts(" (");
		print_int(command_in[0], 0);
		uart_puts(")\n");
		uart_puts_p(PSTR("'?' = debug messages\n"));
		break;
		
	}
	command_ready = FALSE;
}

uint8_t receive_gps(void)
{
	// Format: |b|lat_current|,|lon_current|;|lat_dest|,|lon_dest|e|
	// ex. b123.456,234.567;222.333,444.555e
	double lat_current_temp, lon_current_temp, lat_dest_temp, lon_dest_temp;
	char value[50];
	char buffer[50];
	uint8_t i = 1;
	char *pch;
	
	memset(value, 0, sizeof(value));	// set command to 0
	
	for(uint8_t j=0; j<data_count-3; j++) {
		value[j] = command_in[j+1];
	}

	uart_puts("in b =<");
	for( i=0; i<50; i++) {
		//value[i] = command_in[i+1];	// copy
		uart_putc(value[i]);
	}
	uart_puts(">\n");
	
	if ((command_in[strlen(command_in)-3]) != 'e')	// Check end of string 'e'
	{
		uart_puts("Error in format\n");
		return FALSE;
	}
	
	pch = strtok (value,",");	// return pointer to next token
	lat_current_temp = atof(pch);
	
	pch = strtok (NULL,";");	// return pointer to next token
	lon_current_temp = atof(pch);
	
	pch = strtok (NULL,",");	// return pointer to next token
	lat_dest_temp = atof(pch);
	
	pch = strtok (NULL,",");	// return pointer to next token
	lon_dest_temp = atof(pch);
	
	//sprintf(lat_current_temp, "value = %f");
	if (debug_on)
	{
		sprintf(buffer, "Lat curr = %0.5f", lat_current_temp);
		uart_puts(buffer);
		uart_puts("\n");
		sprintf(buffer, "Lon curr = %0.5f", lon_current_temp);
		uart_puts(buffer);
		uart_puts("\n");
		sprintf(buffer, "Lat dest = %0.5f", lat_dest_temp);
		uart_puts(buffer);
		uart_puts("\n");
		sprintf(buffer, "Lon dest =  %0.5f", lon_dest_temp);
		uart_puts(buffer);
		uart_puts("\n");
	}
	
	/* coordinates are feasible */
	if ((lat_current_temp!=0)&&
		(lon_current_temp!=0)&&
		(lat_dest_temp!=0)&&
		(lon_dest_temp!=0))
	{
		lat_current = lat_current_temp;
		lon_current = lon_current_temp;
		lat_dest = lat_dest_temp;
		lon_dest = lon_dest_temp;
		return TRUE;
	}
	else
	{
		uart_puts("No correct coordinates\n");
		return FALSE;
	}
}