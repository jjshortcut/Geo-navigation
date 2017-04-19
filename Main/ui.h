/*
 * ui.h
 *
 * Created: 11-4-2016 17:39:42
 *  Author: J-J_LAPTOP
 */ 

#ifndef UI_H_
#define UI_H_

#include <avr/io.h>

#define FALSE 0
#define TRUE 1

extern uint16_t timeout_timer;		// timeout timer for ui

/* Buffer type functions */
void writeFrame(uint16_t degrees, uint16_t wait_ms);
void writeBuffer(uint8_t n, uint32_t color);
void clearBuffer(void);

/* WS2811/LED functions */
void colorWipe(uint32_t c, uint8_t wait);
void setPixelColor(uint16_t n, uint32_t c);	// Set pixel color from 'packed' 32-bit RGB color:
uint32_t Color(uint8_t r, uint8_t g, uint8_t b);
uint8_t setBrightness(uint8_t b);	// Set brightness of all leds
uint8_t getBrightness(void);
void clearLeds();

/* UI functions */
void refreshDisplay(uint16_t north_deg, uint16_t destination_deg, uint8_t status);
uint8_t setUIMode(uint8_t mode);
uint8_t getUIMode(void);
void setUIModeNext(void);
void showNorth(uint8_t var);
void setLedPercentage(uint8_t percent,  uint8_t mode, uint32_t color, uint8_t wait);
void setLedDegrees(uint16_t degrees,  uint8_t mode, uint32_t color, uint32_t boundary_color);
void setLedValue(int val,uint8_t wait);
void setNavigationColor(uint32_t color);
void setNavigationColorNext(void);

/* Extra functions*/
void delay(uint16_t wait_ms);

/* Control the vibrator motor */
void buzzer(uint8_t status);

#define BUZZER_OFF		0
#define BUZZER_ON		1
#define BUZZER_SHORT	2
#define BUZZER_LONG	3

struct UI_struct
{
	uint8_t refresh;
	uint8_t displaymode;			
	uint8_t brightness;
	uint32_t navigationcolor;
	uint8_t shownorth;
	uint8_t showdistance;
};

struct UI_struct UI;

/* UI presentation modes */
#define NORMAL		0	// Set 1 LED
#define EXTEND		1	// Set range of 3 LEDS
#define EXTEND_P	2	// Extend precision, second led shows which side of the destination led
#define ARROW		3	// Make arrow with leds
#define ARROW_P		4	// Arrow with extended precision
#define BARR		5	// Barr with distance view
#define ANIMATION_1	6	// Pixel barr animation
#define ANIMATION_2	7	// Moving barr animation
#define MAX_UI_MODES ANIMATION_2	// max is 7 

/* Predefined colors */
#define RED Color(255,0,0)
#define GREEN Color(0,255,0)
#define BLUE Color(0,0,255)
#define WHITE Color(255,255,255)
#define ORANGE Color(255,100,0)
//#define YELLOW Color(100,255,0)
#define YELLOW Color(200,255,0)
#define LIGHT_BLUE Color(0,100,255)
#define VIOLET Color(100,0,255)

/* Modes for setLedPercentage */
#define SINGLE		0	// just 1 LED
#define MULTIPLE	1	// wipe to nr of LEDS

/* Calculation values */
#define PERCENT_PER_PIXEL (100/PIXELS)
#define DEGREE_PER_PIXEL (360/PIXELS)


#endif