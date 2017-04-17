/*
 * light weight WS2812 lib include
 *
 * Version 2.3  - Nev 29th 2015
 * Author: Tim (cpldcpu@gmail.com) 
 *
 * Please do not change this file! All configuration is handled in "ws2812_config.h"
 *
 * License: GNU GPL v2 (see License.txt)
 +
 */ 

#ifndef UART_HANDLER_H_
#define UART_HANDLER_H_

#include <avr/io.h>
#include <stdlib.h>
#include "uart/uart.h"
#include "global.h"
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <avr/pgmspace.h>

#define UART_BAUD_RATE	9600//19200//115200	// Baudrate

extern uint8_t debug_on;	// Debug msg's

/* Uart command receive vars */
char data_in[100];
//volatile unsigned char data_count;
volatile uint8_t command_ready;

void process_serial(void);
uint8_t receiveCoordinates(void);
//uint8_t parse_assignment (void);
uint16_t readCommand(void);	/* handle normal command e.g. U=1 (UI-mode 2)*/

uint8_t receive_gps(void);
////
void get_serial(void);

// The inputted commands are never going to be
// more than 8 chars long. Volatile for the ISR.
volatile unsigned char data_in2[100];
char command_in[100];

volatile unsigned char data_count;
volatile unsigned char command_ready;
unsigned long parse_assignment ();
void process_command();

#endif
