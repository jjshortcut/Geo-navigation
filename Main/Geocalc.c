/*
* Uart handler, handles the incoming and outgoing stream
*/

#include <avr/io.h>
#include "defines.h"
#include "Geocalc.h"

uint16_t calculate_bearing(double lat1, double lon1, double lat2, double lon2)
{
	//Calculate bearing from lat1/lon1 to lat2/lon2
	//Note lat1/lon1/lat2/lon2 must be in radians
	//Returns bearing in degrees
	lat1 = dtor(lat1);
	lon1 = dtor(lon1);
	lat2 = dtor(lat2);
	lon2 = dtor(lon2);

	//determine angle
	double bearing = atan2(sin(lon2 - lon1) * cos(lat2), (cos(lat1) * sin(lat2)) - (sin(lat1) * cos(lat2) * cos(lon2 - lon1)));
	//convert to degrees
	bearing = rtod(bearing);
	//use mod to turn -90 = 270
	bearing = fmod((bearing + 360.0), 360);
	return (int) bearing + 0.5;
}

long calculate_distance(double lat1, double lon1, double lat2, double lon2)
{
	//Calculate distance form lat1/lon1 to lat2/lon2 using haversine formula
	//Note lat1/lon1/lat2/lon2 must be in radians
	//Returns distance in KM's
	double dlon, dlat, a, c;
	double dist = 0.0;
	dlon = dtor(lon2 - lon1);
	dlat = dtor(lat2 - lat1);
	a = pow(sin(dlat / 2), 2) + cos(dtor(lat1)) * cos(dtor(lat2)) * pow(sin(dlon / 2), 2);
	c = 2 * atan2(sqrt(a), sqrt(1 - a));

	dist = 20925656.2 * c;  //radius of the earth (6378140 meters) in feet 20925656.2
	return (( (long) dist + 0.5) * 0.0003048);
}

//convert degrees to radians
double dtor(double fdegrees)
{
	return (fdegrees * PI / 180);
}

//Convert radians to degrees
double rtod(double fradians)
{
	return (fradians * 180.0 / PI);
}