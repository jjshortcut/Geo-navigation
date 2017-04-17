/*
    11-11-09
    Copyright Spark Fun Electronics© 2009
    Aaron Weiss
    aaron at sparkfun.com
    
    HMC5843 3-axis magnetometer
	
	ATMega328 w/ external 16MHz resonator
	High Fuse DA
    Low Fuse FF
	
	raw output in continuous mode
*/

#include <stdlib.h>
#include <stdio.h>
#include <avr/io.h>
#include <avr/pgmspace.h>
#include <avr/interrupt.h>
#include "types.h"
#include "defs.h"
#include "i2c.h"

#define FOSC 16000000
#define BAUD 9600

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

#define WRITE_sda() DDRC = DDRC | 0b00010000 //SDA must be output when writing
#define READ_sda()  DDRC = DDRC & 0b11101111 //SDA must be input when reading - don't forget the resistor on SDA!!

///============Function Prototypes=========/////////////////
void HMC5843(void);

///============I2C Prototypes=============//////////////////
void i2cSendStart(void);
void i2cSendStop(void);
void i2cWaitForComplete(void);
void i2cSendByte(unsigned char data);
void i2cInit(void);
void i2cHz(long uP_F, long scl_F);

///============Initialize Prototypes=====//////////////////
void ioinit(void);
void UART_Init(unsigned int ubrr);
static int uart_putchar(char c, FILE *stream);
void put_char(unsigned char byte);
static FILE mystdout = FDEV_SETUP_STREAM(uart_putchar, NULL, _FDEV_SETUP_WRITE);
void delay_ms(uint16_t x);

/////===================================////////////////////

int main(void)
{
	ioinit();
	i2cInit();
	delay_ms(100);
	
	while(1)
	{
		HMC5843();
		delay_ms(400); //at least 100ms interval between measurements
	}
}

void HMC5843(void)
{		
	uint8_t xh, xl, yh, yl, zh, zl;
	long xo, yo, zo;
	
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(0x3C);    //write to HMC
	i2cWaitForComplete();
	i2cSendByte(0x02);    //mode register
	i2cWaitForComplete();
	i2cSendByte(0x00);    //continuous measurement mode
	i2cWaitForComplete();
	i2cSendStop();
	
	//must read all six registers plus one to move the pointer back to 0x03
	i2cSendStart();
	i2cWaitForComplete();
	i2cSendByte(0x3D);          //read from HMC
	i2cWaitForComplete();
	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	xh = i2cGetReceivedByte();	//x high byte
	i2cWaitForComplete();
	//printf(" %d", xh);
	
	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	xl = i2cGetReceivedByte();	//x low byte
	i2cWaitForComplete();
	xo = xl|(xh << 8);
	printf("x=%4ld, ", xo);
	
	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	yh = i2cGetReceivedByte();	//y high byte
	i2cWaitForComplete();
	//printf(" %d ", yh);
	
	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	yl = i2cGetReceivedByte();	//y low byte
	i2cWaitForComplete();
	yo = yl|(yh << 8);
	printf("y=%4ld, ", yo);
	
	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	zh = i2cGetReceivedByte();	
	i2cWaitForComplete();      //z high byte
	//printf(" %d ", zh);
	
	i2cReceiveByte(TRUE);
	i2cWaitForComplete();
	zl = i2cGetReceivedByte();	//z low byte
	i2cWaitForComplete();
	zo = zl|(zh << 8);
	printf("z=%ld \r\n", zo);
	
	i2cSendByte(0x3D);         //must reach 0x09 to go back to 0x03
	i2cWaitForComplete();
	
	i2cSendStop();	
}

/*********************
 ****Initialize****
 *********************/
 
void ioinit (void)
{
    //1 = output, 0 = input
	DDRB = 0b01100000; //PORTB4, B5 output
    DDRC = 0b00010000; //PORTC4 (SDA), PORTC5 (SCL), PORTC all others are inputs
    DDRD = 0b11111110; //PORTD (RX on PD0), PD2 is status output
	PORTC = 0b00110000; //pullups on the I2C bus
	
	UART_Init((unsigned int)(FOSC/16/BAUD-1));		// ocillator fq/16/baud rate -1	
}

void UART_Init( unsigned int ubrr)
{
	// Set baud rate 
	UBRR0H = ubrr>>8;
	UBRR0L = ubrr;
	
	// Enable receiver and transmitter 
	UCSR0A = (0<<U2X0);
	UCSR0B = (1<<RXEN0)|(1<<TXEN0);
	
	// Set frame format: 8 bit, no parity, 1 stop bit,   
	UCSR0C = (1<<UCSZ00)|(1<<UCSZ01);
	
	stdout = &mystdout; //Required for printf init
}

static int uart_putchar(char c, FILE *stream)
{
    if (c == '\n') uart_putchar('\r', stream);
  
    loop_until_bit_is_set(UCSR0A, UDRE0);
    UDR0 = c;
    
    return 0;
}

void put_char(unsigned char byte)
{
	/* Wait for empty transmit buffer */
	while ( !( UCSR0A & (1<<UDRE0)) );
	/* Put data into buffer, sends the data */
	UDR0 = byte;
}

void delay_ms(uint16_t x)
{
  uint8_t y, z;
  for ( ; x > 0 ; x--){
    for ( y = 0 ; y < 90 ; y++){
      for ( z = 0 ; z < 6 ; z++){
        asm volatile ("nop");
      }
    }
  }
}

/*********************
 ****I2C Functions****
 *********************/

void i2cInit(void)
{
	// set i2c bit rate to 40KHz
	i2cSetBitrate(100);
	// enable TWI (two-wire interface)
	sbi(TWCR, TWEN);
}

void i2cSetBitrate(unsigned short bitrateKHz)
{
	unsigned char bitrate_div;
	// set i2c bitrate
	// SCL freq = F_CPU/(16+2*TWBR))
	//#ifdef TWPS0
		// for processors with additional bitrate division (mega128)
		// SCL freq = F_CPU/(16+2*TWBR*4^TWPS)
		// set TWPS to zero
		cbi(TWSR, TWPS0);
		cbi(TWSR, TWPS1);
	//#endif
	// calculate bitrate division	
	bitrate_div = ((F_CPU/4000l)/bitrateKHz);
	if(bitrate_div >= 16)
		bitrate_div = (bitrate_div-16)/2;
	outb(TWBR, bitrate_div);
}

void i2cSendStart(void)
{
	WRITE_sda();
	// send start condition
	TWCR = (1<<TWINT)|(1<<TWSTA)|(1<<TWEN);
}

void i2cSendStop(void)
{
	// transmit stop condition
        TWCR = (1<<TWINT)|(1<<TWEN)|(1<<TWSTO);
}

void i2cWaitForComplete(void)
{
	int i = 0;		//time out variable
	
	// wait for i2c interface to complete operation
        while ((!(TWCR & (1<<TWINT))) && (i < 90))
			i++;
}

void i2cSendByte(unsigned char data)
{

	WRITE_sda();
	// save data to the TWDR
	TWDR = data;
	// begin send
	TWCR = (1<<TWINT)|(1<<TWEN);
}

void i2cReceiveByte(unsigned char ackFlag)
{
	// begin receive over i2c
	if( ackFlag )
	{
		// ackFlag = TRUE: ACK the recevied data
		outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT)|BV(TWEA));
	}
	else
	{
		// ackFlag = FALSE: NACK the recevied data
		outb(TWCR, (inb(TWCR)&TWCR_CMD_MASK)|BV(TWINT));
	}
}

unsigned char i2cGetReceivedByte(void)
{
	// retieve received data byte from i2c TWDR
	return( inb(TWDR) );
}

unsigned char i2cGetStatus(void)
{
	// retieve current i2c status from i2c TWSR
	return( inb(TWSR) );
}