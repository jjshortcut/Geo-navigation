/*
 * LSM303_simple.h for LSM303DLHC
 *
 * Created: 24-6-2016 14:28:55
 *  Author: J-J_LAPTOP
 */ 
#ifndef __LSM303_SIMPLE_H__
#define __LSM303_SIMPLE_H__

#include <avr/io.h>
#include "vector.h"
/* I2C clock in Hz */
#define SCL_CLOCK  10000L

#define X_AXIS 1
#define Y_AXIS 2
#define Z_AXIS 3
#define MAX 1
#define MIN 2

/* Adresses */
 #define CTRL_REG1_A	0x20 // DLH, DLM, DLHC
 #define CTRL_REG2_A    0x21 // DLH, DLM, DLHC
 #define CTRL_REG3_A    0x22 // DLH, DLM, DLHC
 #define CTRL_REG4_A    0x23 // DLH, DLM, DLHC
 #define CTRL_REG5_A    0x24 // DLH, DLM, DLHC
 #define CTRL_REG6_A    0x25 // DLHC
 #define REFERENCE_A	0x26 // DLH, DLM, DLHC
 #define STATUS_REG_A	0x27 // DLH, DLM, DLHC
 
 #define CRA_REG_M      0x00 // DLH, DLM, DLHC
 #define CRB_REG_M      0x01 // DLH, DLM, DLHC
 #define MR_REG_M       0x02 // DLH, DLM, DLHC
 #define SR_REG_M       0x09 // DLH, DLM, DLHC
 #define IRA_REG_M      0x0A // DLH, DLM, DLHC
 #define IRB_REG_M      0x0B // DLH, DLM, DLHC
 #define IRC_REG_M		0x0C // DLH, DLM, DLHC		
 
 #define TEMP_OUT_H_M	0x31 
 #define TEMP_OUT_L_M	0x32
	
void init_lsm303(void);
void read_data(vector *a, vector *m);
int get_heading(const vector *a, const vector *m, const vector *p);
int get_heading_avg(uint8_t number_of_avg);
double get_heading_simple(void);
int calibrate_lsm303(uint8_t axis, uint8_t minmax);
void print_calibration_lsm303(void);	// Print calibration (min/max values of sensor)

void writeAccReg(uint8_t reg, uint8_t value);
void writeMagReg(uint8_t reg, uint8_t value);

int get_temp(void);
#endif


