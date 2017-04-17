/*
 * Calculates geo stuff
 */ 

#ifndef GEOCALC_H_
#define GEOCALC_H_

#include <avr/io.h>
#include "uart/uart.h"

uint16_t calculate_bearing(double lat1, double lon1, double lat2, double lon2);
long calculate_distance(double lat1, double lon1, double lat2, double lon2);
double dtor(double fdegrees);
double rtod(double fradians);


#endif
