/*
* defines.h
*
* Created: 2-4-16
*  Author: J-J_LAPTOP
*/
#ifndef DEFINES_H_
#define DEFINES_H_

#include <avr/eeprom.h>
// Defines
#include <avr/io.h>
#include <util/delay.h>
#include "ui.h"
#include "Geocalc.h"
#include "Hal.h"
#define VERSION 1

#define ON 1
#define OFF 0
#define PI 3.14159265359
#define PCB_COMPASS_CORR 90		/* PCB chip placement correction (north = top of pcb 0th LED)*/


#define SENSOR_THRESHOLD DEGREE_PER_PIXEL*3		/* Max sensor change step*/
#define ERROR_THRESHOLD_NR 25	/* How many errors before the value was correct? */

/* For interrupt */
#define INTERRUPT_DISPLAY_MS 32	/* = max (250 cycles) */


#define DISPLAY_REFRESH_MS 100	/* Gets triggered by int, triggers every +/- 3 times */
#define REFRESH_LOOP_MS 1		/* delay in main loop */ 
//#define UI_DELAY_CYCLES (250/REFRESH_LOOP_MS)	// 250MS
#define TIMEOUT_TIME (5000/REFRESH_LOOP_MS) // 5 SEC

#define FALSE 0
#define TRUE 1

/* Battery state*/
#define BATT_EMPTY 130
#define BATT_FULL 500	//???

/* Device_status */
#define NAVIGATING 2
#define NO_CONNECTION 3
#define AT_LOCATION 4

/* Button state*/
#define BUTTON_PRESSED 1
#define BUTTON_NOT_PRESSED 0
#define SHORT_PRESS 1
#define MIDDLE_PRESS 2
#define LONG_PRESS 3

struct database {
	float compass;
	float heading;
	float distance;
	uint8_t battery;
	uint8_t temperature;
	uint8_t chargingstate;
	uint8_t buttonstate;
	uint8_t buttonaction;
	uint8_t newdata;
	uint8_t previousstatus;
	uint8_t status;
};

struct database device;

double lat_current, lon_current, lat_dest, lon_dest;
//float lat_current, lon_current, lat_dest, lon_dest;
void print_int(int c, uint8_t ln);
void print_float(double c, uint8_t ln);
void print_value (char id, int value);

/* EEPROM functionality */
void load_eeprom_settings(void);
uint16_t load_eeprom_parameter(uint16_t param);
void save_eeprom_parameter(uint16_t param, uint16_t value);
void reset_factory_settings(void);	/* Reset the settings of device, also in EEPROM */

/* Settings stored in EEPROM */
extern uint16_t EEMEM eeprom_first_startup, eeprom_brightness, eeprom_displaymode, eeprom_showdistance, eeprom_shownorth;
extern uint32_t EEMEM eeprom_navigationcolor;

/* Functions to keep track of status of device */
void set_initial_device_status(void);
uint8_t set_device_status(uint8_t status);
uint8_t set_previous_device_status(void);
uint8_t read_device_status(void);

//#define ADC_PIN_PORT	PORTC
//#define ADC_PIN			PC6

/* EXTRA PIN */
#define TEST_PIN		PC3
#define TEST_PIN_DDR	DDRC
#define TEST_PIN_PORT	PORTC
#define TEST_PIN_REG	PINC
#define TEST_ON			(TEST_PIN_PORT = (1 << TEST_PIN))			// debug led on
#define TEST_OFF 		(TEST_PIN_PORT &= ~(1 << TEST_PIN))			// debug led off
#define TEST_TOGGLE		(TEST_PIN_PORT ^= (1 << TEST_PIN))			// debug toggle

/* ON/OFF SWITCH */
#define SW_PIN			PD7
#define SW_PIN_REG		PIND
#define SW_PIN_DDR		DDRD
#define SW_PIN_PORT		PORTD
#define SW_STATUS		(SW_PIN_REG & (1<<SW_PIN))

/* BATTERY/ADC PIN */
#define BATT_PIN		PC0	// ADC0
#define BATT_PWR_PIN	PC1	// To turn on the Battery ADC measurement (LOW=ON)
#define BATT_PWR_DDR	DDRC
#define BATT_PWR_PORT	PORTC
#define BATT_PWR_REG	PINC
#define BATT_PWR_INIT	(BATT_PWR_DDR |= (1<<BATT_PWR_PIN))			// INIT pin
#define BATT_PWR_OFF	(BATT_PWR_PORT = (1 << BATT_PWR_PIN))		// Reversed logic, on=gnd	
#define BATT_PWR_ON 	(BATT_PWR_PORT &= ~(1 << BATT_PWR_PIN))		// Reversed logic, on=gnd	

/* CHARGE STATUS PIN */
#define CHARGING 1
#define FULL 0
#define CHARGE_ST_PIN		PC2
#define CHARGE_ST_REG		PINC
#define CHARGE_ST_DDR		DDRC
#define CHARGE_ST_PORT		PORTC
#define CHARGE_PIN_STATUS	(CHARGE_ST_REG & (1<<CHARGE_ST_PIN))

/* POWER ON PIN */
#define PWR_ON_PIN	PD6
#define PWR_ON_DDR	DDRD
#define PWR_ON_PORT	PORTD
#define PWR_ON_REG	PIND
#define PWR_ON_INIT	(PWR_ON_DDR |= (1<<PWR_ON_PIN))			
#define PWR_ON		(PWR_ON_PORT = (1 << PWR_ON_PIN))		
#define PWR_OFF		(PWR_ON_PORT &= ~(1 << PWR_ON_PIN))		

#define AT_PIN		PD3
#define AT_PIN_DDR	DDRD
#define AT_PIN_PORT	PORTD
#define AT_PIN_REG	PIND
#define AT_PIN_INIT	(AT_PIN_DDR |= (1<<AT_PIN))		
#define AT_ON		(AT_PIN_PORT = (1 << AT_PIN))			// debug led on
#define AT_OFF 		(AT_PIN_PORT &= ~(1 << AT_PIN))			// debug led off

#define BT_RST_PIN		PD2
#define BT_RST_PIN_DDR	DDRD
#define BT_RST_PIN_PORT	PORTD
#define BT_RST_PIN_REG	PIND
#define BT_RST_HIGH		(BT_RST_PIN_PORT = (1 << BT_RST_PIN))			// debug led on
#define BT_RST_LOW 		(BT_RST_PIN_PORT &= ~(1 << BT_RST_PIN))			// debug led off

/*  WS2811/NeoPixel is defined in ws2812_config */
#define PIXELS 12	// led array 
uint32_t uiBuffer[PIXELS+1];	// holds the display

#endif