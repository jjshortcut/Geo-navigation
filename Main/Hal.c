/*
* Hal handles the device specifics
*/

#include <avr/io.h>
#include "defines.h"
#include "uart_handler.h"

void control_power(uint8_t value)
{
	if (value)
	{
		PWR_ON;		// Device will keep power after switch is released
	}
	else
	{
		PWR_OFF;	// Device turn off
	}
}

void check_device_status(void)
{
	device.buttonstate = read_button();				// 1=pressed
	//device.chargingstate = read_charge_status();
	//device.battery = read_battery();
	
	if ((device.buttonstate==BUTTON_PRESSED))	// set state
	{
		if ((read_device_status()!=BUTTON_PRESSED))
		{
			if (debug_on){uart_puts("Button pressed\n");}
			set_device_status(BUTTON_PRESSED);
		}
	}
	else
	{
		if ((device.buttonstate==BUTTON_NOT_PRESSED)&&(read_device_status()==BUTTON_PRESSED)) // reset state
		{
			if (debug_on){uart_puts("Button released\n");}
			device.status = BUTTON_NOT_PRESSED;	// Fix this.. stays in a loop due to loss of previous state before button..
		}
		else if (read_device_status()==BUTTON_NOT_PRESSED)
		{
			set_previous_device_status();
		}
		else
		{
			if (timeout_timer<TIMEOUT_TIME) // still ok
			{
				timeout_timer++;
			}
			else //time-out
			{
				timeout_timer = 40;
				if (read_device_status()!=NO_CONNECTION)
				{
					set_device_status(NO_CONNECTION);
					if (debug_on){uart_puts("We have NO Connection (time-out)..\n");}
				}
			}
			
			if (device.newdata)
			{
				set_device_status(NAVIGATING);
				device.newdata = FALSE;
				timeout_timer = 0;	// reset timer
			}
		}
	}
}

uint8_t read_button(void)
{
	if(debug_on)
	{
		//uart_puts("Button state = ");
		//print_int(SW_STATUS, 1);
		
		if (!SW_STATUS)	// If pressed, pulled low
		{
			//uart_puts("Pressed\n");
		}
	}
	
	return (!SW_STATUS);
}

uint8_t read_charge_status(void)
{
	if (!CHARGE_PIN_STATUS) // If 0
	{
		if(debug_on) uart_puts("Charge status: CHARGING\n");
		return CHARGING;
	}
	else
	{
		if(debug_on) uart_puts("Charge status: FULL\n");
		return FULL;
	}
}

uint16_t read_battery(void)
{
	uint16_t battery_val = 0;
	uint16_t battery_percentage = 0;
	/*	47K/47K=0.5 gain, lipo min bat=3.14V, max bat=4.2V  
	*	ADC = 0-3V3 (0-1024)
	*   Min val = 3.14*0.5 = 1.57V*1024/3V3 = 487
	*	Min val = 4.2*0.5 = 2.1V*1024/3V3 = 651
	*/
	static uint16_t MIN_BAT_ADC = 487, MAX_BAT_ADC = 660;	
	
	BATT_PWR_ON;
	_delay_ms(5);	// Stabilize power supply
	battery_val = read_adc(BATT_PIN);
	battery_percentage = (battery_val - MIN_BAT_ADC) * (100-0) / (MAX_BAT_ADC - MIN_BAT_ADC) + 0;
	BATT_PWR_OFF;
	
	if (battery_percentage>100) {battery_percentage=100;}
	
	if (debug_on)
	{
		uart_puts("Battery = ");
		print_int(battery_percentage, 0);
		uart_puts("%\n");
	}

// read the battery level from the ESP8266 analog in pin.
// analog read level is 10 bit 0-1023 (0V-1V).
// our 1M & 220K voltage divider takes the max
// lipo value of 4.2V and drops it to 0.758V max.
// this means our min analog read value should be 580 (3.14V)
// and the max analog read value should be 774 (4.2V).
//int level = analogRead(A0);

// convert battery level to percent
//level = map(level, 580, 774, 0, 100);
//Serial.print("Battery level: "); Serial.print(level); Serial.println("%");

	
	//battery_percentage = (battery_percentage>=100) ? 100: battery_percentage;	/* limit */
	//uart_puts("TODO:battery percentage should still be mapped!!\n");
	return (battery_percentage);
}

void init_bluetooth(void)
{	/*TODO, AT pin not at correct layout for this module*/
	uart_init( UART_BAUD_SELECT(9600,F_CPU) );	/* Init Uart */
	/* Maybe set KEY-pin first? */
	BT_RST_LOW;
	_delay_ms(10);
	BT_RST_HIGH;
	_delay_ms(10);
	AT_ON;	// Set module into AT mode
	
	//uart_puts("AT+ORGL\n");	/* Original settings */
	uart_puts("AT+NAMEBIKENAV\n"); /* Set name */
	uart_puts("AT+ROLE=0\n");		/* Set role */
	uart_puts("AT+BAUD4\n");	/* set baudrate */
	
	/* Clear KEY-pin before reset */
	AT_OFF;	// Reset AT mode
	uart_puts("AT+RESET\n");	/* Reset module */
	uart_init( UART_BAUD_SELECT(UART_BAUD_RATE,F_CPU) );	/* Reset baudrate */
}
