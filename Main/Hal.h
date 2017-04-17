/*
 * Calculates geo stuff
 */ 

#ifndef HAL_H_
#define HAL_H_

#include <avr/io.h>

void control_power(uint8_t value);
uint16_t read_adc(uint8_t adcx);
void check_device_status(void);
uint8_t read_button(void);
uint8_t read_charge_status(void);
uint16_t read_battery(void);
void init_bluetooth(void);

#endif
