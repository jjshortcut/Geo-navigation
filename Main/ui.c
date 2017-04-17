/*
 * ui.c
 *
 * Created: 11-4-2016 17:27:54
 *  Author: J-J_LAPTOP
 * This has all the code for the buffered writing to the LEDS (display)
 */ 

#include "ui.h"
#include "defines.h"
#include "light_ws2812.h"
#include <avr/interrupt.h>
#include <avr/io.h>
#include <string.h>  
#include <util/delay.h>
#include "uart.h"


struct cRGB led[PIXELS+1];
//uint8_t brightness = 255;
//uint8_t ui_mode = NORMAL;
//uint8_t show_north = TRUE;
uint8_t buffer_ready_flag = FALSE;
uint8_t togglebit = FALSE;
uint8_t button_counter = 0;
uint8_t ui_counter = 0;
uint16_t timeout_timer = 0;		// timeout timer for ui

void writeFrame(uint16_t degrees, uint16_t wait_ms)	/* Write buffer to display */
{
	uint8_t buffer_nr = 1;
	uint8_t pixel_offset = 0;
	
	if (degrees>360)	{degrees=360;}					// if degrees=360
	
	if (degrees)	// if not 0
	{
		pixel_offset = (degrees/DEGREE_PER_PIXEL);	// rotation of display
	}

	//uart_puts("pixel offset = ");
	//print_int(pixel_offset,1);
	
	for (uint8_t i = pixel_offset+1; i <= PIXELS+pixel_offset; i++) {	// do this for all pixels
			
		if (i>PIXELS)
		{
			//uart_puts("to big, rescale\n");
			setPixelColor(i-PIXELS,uiBuffer[buffer_nr]);
		}
		else
		{
			//uart_puts("printpixel\n");
			setPixelColor(i,uiBuffer[buffer_nr]);
		}
		buffer_nr++;
		delay(wait_ms);
		//print_int(i,1);
	}

	//if (buffer_ready_flag)
	//clearBuffer();	// empty the buffer
}

void writeBuffer(uint8_t n, uint32_t color)	/* Write to buffer */
{
	//uint8_t red_rec,green_rec,blue_rec;
	//uint8_t red_cur,green_cur,blue_cur;
	//uint8_t red_res,green_res,blue_res;
	/*
	if (n>0 && n<=PIXELS)	// When not a full buffer, between 1-PIXELS
	{
		if (add)
		{ 
			// Unpack 
			red_rec = (uint8_t)(color >> 16),
			green_rec = (uint8_t)(color >>  8),
			blue_rec = (uint8_t)color;
			
			red_cur = (uint8_t)(uiBuffer[n] >> 16),
			green_cur = (uint8_t)(uiBuffer[n] >>  8),
			blue_cur = (uint8_t)uiBuffer[n];
			
			// calculate
			red_res = ((red_rec+red_cur)>=255) ? 255 :  (red_rec+red_cur);
			green_res = ((green_rec+green_cur)>=255) ? 255 :  (green_rec+green_cur);
			blue_res = ((blue_rec+blue_cur)>=255) ? 255 :  (blue_rec+blue_cur);
			
			uiBuffer[n] = ((uint32_t)red_res << 16) | ((uint32_t)green_res <<  8) | blue_res;
		}
		else*/
		//{
			uiBuffer[n] = color;	
		//}
		
	//}
}

void clearBuffer(void)
{
	memset(uiBuffer, 0, sizeof uiBuffer);	
}

void colorWipe(uint32_t c, uint8_t wait)
{
	for (uint8_t i = 1; i < PIXELS+1; i++) {
		writeBuffer(i, c);
	}
	writeFrame(0, wait);
}

void delay(uint16_t wait_ms)
{
	for (uint16_t i = 0; i < wait_ms; i++) {
	_delay_ms(1);}
}

// Set pixel color from 'packed' 32-bit RGB color:
void setPixelColor(uint16_t n, uint32_t c) 
{
	uint16_t r,g,b;
	if(n < PIXELS+1) 
	{
		r = (uint8_t)(c >> 16),
		g = (uint8_t)(c >>  8),
		b = (uint8_t)c;
		
		if(UI.brightness) 
		{
			led[n-1].r = (r * UI.brightness) >> 8;
			led[n-1].g = (g * UI.brightness) >> 8;
			led[n-1].b = (b * UI.brightness) >> 8;	
			ws2812_setleds(led,n);
		}
	}
}

// Convert separate R,G,B into packed 32-bit RGB color.
// Packed format is always RGB, regardless of LED strand color order.
uint32_t Color(uint8_t r, uint8_t g, uint8_t b) {
	return ((uint32_t)r << 16) | ((uint32_t)g <<  8) | b;
}

uint8_t setBrightness(uint8_t b) 
{
	// Stored brightness value is different than what's passed.
	// This simplifies the actual scaling math later, allowing a fast
	// 8x8-bit multiply and taking the MSB.  'brightness' is a uint8_t,
	// adding 1 here may (intentionally) roll over...so 0 = max brightness
	// (color values are interpreted literally; no scaling), 1 = min
	// brightness (off), 255 = just below max brightness.
	/*uint8_t newBrightness = b + 1;
	if(newBrightness != brightness) { // Compare against prior value
		// Brightness has changed -- re-scale existing data in RAM
		uint8_t  c,
		*ptr           = pixels,
		oldBrightness = brightness - 1; // De-wrap old brightness value
		uint16_t scale;
		if(oldBrightness == 0) scale = 0; // Avoid /0
		else if(b == 255) scale = 65535 / oldBrightness;
		else scale = (((uint16_t)newBrightness << 8) - 1) / oldBrightness;
		for(uint16_t i=0; i<numBytes; i++) {
			c      = *ptr;
			*ptr++ = (c * scale) >> 8;
		}
		brightness = newBrightness;
	}*/
	if(b>=0 && b<=255)
	{
		UI.brightness = b;
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

uint8_t getBrightness(void)
{
	return UI.brightness;
}

void setNavigationColor(uint32_t color)
{
	UI.navigationcolor = color;
}

void setNavigationColorNext(void)
{
	static uint8_t color_counter = 0;

	switch (color_counter)
	{
		case 0:
		setNavigationColor(RED);
		eeprom_update_dword(&eeprom_navigationcolor, RED);
		break;
		case 1:
		setNavigationColor(GREEN);
		eeprom_update_dword(&eeprom_navigationcolor, GREEN);
		break;
		case 2:
		setNavigationColor(BLUE);
		eeprom_update_dword(&eeprom_navigationcolor, BLUE);
		break;
		case 3:
		setNavigationColor(WHITE);
		eeprom_update_dword(&eeprom_navigationcolor, WHITE);
		break;
		case 4:
		setNavigationColor(ORANGE);
		eeprom_update_dword(&eeprom_navigationcolor, ORANGE);
		break;
		case 5:
		setNavigationColor(YELLOW);
		eeprom_update_dword(&eeprom_navigationcolor, YELLOW);
		break;
		case 6:
		setNavigationColor(LIGHT_BLUE);
		eeprom_update_dword(&eeprom_navigationcolor, LIGHT_BLUE);
		break;
		case 7:
		setNavigationColor(VIOLET);
		eeprom_update_dword(&eeprom_navigationcolor, VIOLET);
		break;
		default:
		break;
	}
	color_counter = (color_counter>=7) ? 0 : color_counter+1;
}

void clearLeds() {
	clearBuffer();	// clearBuffer and writeFrame is more neat but slower..
	//writeFrame(0,0);
	//TODO:memset(pixels, 0, numBytes);
	for (uint8_t i = 0; i < PIXELS+1; i++) {
		setPixelColor(i, Color(0,0,0));
	}
}

uint8_t setUIMode(uint8_t mode)
{
	if(mode>=0 && mode<=MAX_UI_MODES)
	{
		UI.displaymode = mode;
		ui_counter = 0;			/* Clear counter for visuals */
		return TRUE;
	}
	else
	{
		return FALSE;
	}
}

void setUIModeNext(void)
{
	static uint8_t mode = 0;
	setUIMode(mode);
	eeprom_update_word(&eeprom_displaymode, getUIMode());	/* Save in EEPROM */
	mode = (mode>=MAX_UI_MODES) ? 0 : mode+1;
}

uint8_t getUIMode(void)
{
	return UI.displaymode;
}

void showNorth(uint8_t var)
{
	UI.shownorth = var;
}

void setLedPercentage(uint8_t percent,  uint8_t mode, uint32_t color, uint8_t wait)
{
	uint8_t nr_of_leds = 0;
	if (percent>100)	{percent=100;}
		
	nr_of_leds = percent/PERCENT_PER_PIXEL;
		
	if (mode==SINGLE)	// set just 1 led
	{
		if (percent==100)
		{
			writeBuffer(1, color);	
		}
		else
		{
			writeBuffer(nr_of_leds+1, color);
		}
		writeFrame(0, wait);
	}
	
	if (mode==MULTIPLE)	// do a color wipe until the last led reached
	{
		if (percent==100)
		{
			nr_of_leds--;
		}
		for (uint8_t i = 1; i < (nr_of_leds)+2; i++) {
			writeBuffer(i, color);
		}
		writeFrame(0, wait);
	}
}

void setLedValue(int val,uint8_t wait)
{
	uint8_t i = 0;
	uint8_t val_1000 = (val % 10000)/1000;
	uint8_t val_100 = (val % 1000)/100;
	uint8_t val_10 = (val % 100)/10;
	uint8_t val_1 = (val % 10);	// Divide by 10'ers
	
	// IF a too high number! cut!
	if ((val_100+val_10+val_100)>12)
	{
		uart_puts("Value too high to display!\n");
	}
	else
	{
		for (i = 0;	i < val_1000+1; i++)						{writeBuffer(i, WHITE);}	/* Write 1000's */
		//if (val_100==0)	{i++;}
		for (i=i;	i < val_100+val_1000+1; i++)				{writeBuffer(i, RED);}		/* Write 100's */
		//if (val_10==0)	{i++;}
		for (i=i;	i < val_10+val_100+val_1000+1; i++)			{writeBuffer(i, BLUE);}		/* Write 10's */
		//if (val_1==0)	{i++;}
		for (i=i;	i < val_1+val_10+val_100+val_1000+1; i++)	{writeBuffer(i, GREEN);}	/* Write 1's */
		
		writeFrame(0, wait);	// Write to display
	}
}

void refreshDisplay(uint16_t north_deg, uint16_t destination_deg, uint8_t status)
{
	//static uint8_t connection_counter;
	uint16_t rotation_deg = 0;
	uint8_t x = 0;
	/* Calculate rotation */
	//rotation_deg = (north_deg + destination_deg);	/* 45 = 45 + 45*/
	//rotation_deg = (north_deg - destination_deg);	/* 330 = 10 - 20*/
	if (destination_deg>= north_deg)	// 25 - 20
	{
		rotation_deg = destination_deg-north_deg;
	}	// 10 - 20
	else
	{
		rotation_deg = (destination_deg+360)-north_deg;
	}
	
	clearBuffer();	/* Clear buffer with colors */
	
	/* Fill frame buffer with data */	
	switch (status)
	{
		case NO_CONNECTION:
			togglebit = (togglebit) ? FALSE : TRUE;
			if (togglebit)
			{writeBuffer(1,RED);		/* NO connection */}
		break;
		
		case BUTTON_PRESSED:
		ui_counter++;
			if (device.buttonstate==BUTTON_PRESSED)
			{
				for(uint8_t i=0; i<=button_counter; i++) {
					if (i<=6)
					{
						writeBuffer(i+1,BLUE);			/*  */
					}
					else
					{
						writeBuffer(i+1,ORANGE);		/*  */
					}
				}
				button_counter++;
					
				if (button_counter>12)	/* Counter full, reset */
				{
					device.buttonaction = LONG_PRESS;
					//uart_puts("Long press action\n");
					//device.status = device.previousstatus;		/* Go to earlier state */
					button_counter=0;
				}
			}	
			ui_counter = 0;
		break;
		
		case BUTTON_NOT_PRESSED:	/* Button released */
			{
				if (button_counter<=6)
				{
					device.buttonaction = SHORT_PRESS;	/* short press action */
					//uart_puts("Short press action\n");
				}
				if (button_counter>6)
				{
					device.buttonaction = MIDDLE_PRESS;	/* Middle press action */
					//uart_puts("Middle press action\n");
				}
				//device.status = device.previousstatus;	/* Go to earlier state */
				//device.status = NAVIGATING;
				button_counter = 0;							/* Released button */
			}
		break;
		
		case NAVIGATING:
			switch (UI.displaymode)
			{
				case NORMAL:
					writeBuffer(1,UI.navigationcolor);			/* Pointer to destination */
				break;
				
				case EXTEND:
					writeBuffer(1,UI.navigationcolor);			/* Pointer to destination */
					writeBuffer(2,ORANGE);		/*  */
					writeBuffer(12,ORANGE);		/*  */
				break;
				
				case EXTEND_P:
					writeBuffer(1,UI.navigationcolor);			/* Pointer to destination */
					x = ((destination_deg+(DEGREE_PER_PIXEL/2))/DEGREE_PER_PIXEL);	// Make use of rounding numbers
					if (x%2)	// Higher led (even)
					{
						writeBuffer(12,ORANGE);		/*  */
					}
					else		// Lower led
					{
						writeBuffer(2,ORANGE);		/*  */
					}
				break;
				
				case ARROW:
					writeBuffer(1,UI.navigationcolor);			/* Pointer to destination */
					writeBuffer(5,UI.navigationcolor);		/*  */
					writeBuffer(6,UI.navigationcolor);		/*  */
					writeBuffer(7,UI.navigationcolor);		/*  */
					writeBuffer(8,UI.navigationcolor);		/*  */
					writeBuffer(9,UI.navigationcolor);		/*  */
				break;
				
				case ARROW_P:
					writeBuffer(1,UI.navigationcolor);			/* Pointer to destination */
					x = ((destination_deg+(DEGREE_PER_PIXEL/2))/DEGREE_PER_PIXEL);	// Make use of rounding numbers
					if (x%2)	// Higher led (even)
					{
						writeBuffer(12,ORANGE);		/*  */
					}
					else		// Lower led
					{
						writeBuffer(2,ORANGE);		/*  */
					}
					
					writeBuffer(5,UI.navigationcolor);		/*  */
					writeBuffer(6,UI.navigationcolor);		/*  */
					writeBuffer(7,UI.navigationcolor);		/*  */
					writeBuffer(8,UI.navigationcolor);		/*  */
					writeBuffer(9,UI.navigationcolor);		/*  */
				break;
				
				case BARR:
					writeBuffer(1,UI.navigationcolor);			/* Pointer to destination */
					if (device.distance>=1)
					{
						writeBuffer(2,UI.navigationcolor);		/*  */
						writeBuffer(12,UI.navigationcolor);		/*  */
					}
					if(device.distance>=2.5)
					{
						writeBuffer(3,UI.navigationcolor);		/*  */
						writeBuffer(11,UI.navigationcolor);		/*  */
					}
					if(device.distance>=5)
					{
						writeBuffer(4,UI.navigationcolor);		/*  */
						writeBuffer(10,UI.navigationcolor);		/*  */
					}
				break;
				
				/* UI modes which depends on animations/cycles */
				case ANIMATION_1:
					writeBuffer(1,UI.navigationcolor);			/* Pointer to destination */
					switch(ui_counter)
					{
						case 0:
						writeBuffer(4,UI.navigationcolor);		/*  */
						writeBuffer(10,UI.navigationcolor);		/*  */
						break;
						
						case 1:
						writeBuffer(3,UI.navigationcolor);		/*  */
						writeBuffer(11,UI.navigationcolor);		/*  */
						break;
						
						case 2:
						writeBuffer(2,UI.navigationcolor);		/*  */
						writeBuffer(12,UI.navigationcolor);		/*  */
						break;
						
						default:
						break;
					}
					ui_counter++;
					ui_counter = (ui_counter>=3) ? 0: ui_counter;
				break;
				
				case ANIMATION_2:
					writeBuffer(1,UI.navigationcolor);			/* Pointer to destination */
					switch(ui_counter)
					{
						case 0:
						writeBuffer(2,UI.navigationcolor);		/*  */
						writeBuffer(3,UI.navigationcolor);		/*  */
						writeBuffer(4,UI.navigationcolor);		/*  */
						writeBuffer(10,UI.navigationcolor);		/*  */
						writeBuffer(11,UI.navigationcolor);		/*  */
						writeBuffer(12,UI.navigationcolor);		/*  */
						break;
						
						case 1:
						writeBuffer(2,UI.navigationcolor);		/*  */
						writeBuffer(3,UI.navigationcolor);		/*  */
						//writeBuffer(4,UI.navigationcolor);		/*  */
						//writeBuffer(10,UI.navigationcolor);		/*  */
						writeBuffer(11,UI.navigationcolor);		/*  */
						writeBuffer(12,UI.navigationcolor);		/*  */
						break;
						
						case 2:
						writeBuffer(2,UI.navigationcolor);		/*  */
						//writeBuffer(3,UI.navigationcolor);		/*  */
						//writeBuffer(4,UI.navigationcolor);		/*  */
						//writeBuffer(10,UI.navigationcolor);		/*  */
						//writeBuffer(11,UI.navigationcolor);		/*  */
						writeBuffer(12,UI.navigationcolor);		/*  */
						break;
						
						default:
						break;
					}
					ui_counter++;
					ui_counter = (ui_counter>=4) ? 0 : ui_counter;
				break;
				
				default:
					uart_puts("default error in UI case!\n");
				break;
					
			}
					
			if (UI.shownorth)		/* Add north to display */
			{
				//????writeBuffer(1-(rotation_deg/DEGREE_PER_PIXEL), BLUE, TRUE);
			}
			
			if (UI.showdistance)	/* Add distance (KM's) to display */
			{
				/*for (uint8_t i = 2; i < (device.distance/PERCENT_PER_PIXEL)+1; i++) {
					writeBuffer(i, VIOLET, TRUE);
				}*/
				uart_puts("TODO SHOW UI DISTANCE\n");	
			}
		break;
		
		default:
		uart_puts("UNKNOWN status!\n");
		break;
	}	
		
		/*
		if (rotation_deg>360)
		{
			rotation_deg -=360;
		}
		*/
		/*
		uart_puts("northdeg = ");
		print_int(north_deg,0);
		uart_puts(" heading = ");
		print_int(destination_deg,0);
		uart_puts(" calc rotation = ");
		print_int(rotation_deg,1);*/
		// 45 = 0 + 45
		// 165 = 120 + 45
		// 400 (wanted = ) = 355 + 45
		writeFrame(rotation_deg, 0);	/* Write buffer to display */
		UI.refresh = FALSE;
}

void vibrate(uint8_t status)
{
	uint8_t i;
	
	switch (status)
	{
		case VIBRATE_ON:
			PWR_OFF;
		break;
		
		case VIBRATE_OFF:
			PWR_ON;
		break;
		
		case VIBRATE_SHORT:
			for(i=0; i<3; i++)
			{
				PWR_ON;
				delay(500);
				PWR_OFF;
				delay(500);
			}
		break;
		
		case VIBRATE_LONG:
		for(i=0; i<3; i++)
			{
				PWR_ON;
				delay(1000);
				PWR_OFF;
				delay(500);
			}
		break;
		
		default:
		break;
	}
}
