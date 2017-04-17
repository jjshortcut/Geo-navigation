/*
 * LSM303_simple.c
 *
 * Created: 24-6-2016 14:28:30
 *  Author: J-J_LAPTOP
 */ 
/* 
 * This program assumes that the LSM303DLH carrier is oriented with X pointing
 * to the right, Y pointing backward, and Z pointing down (toward the ground).
 * The code compensates for tilts of up to 90 degrees away from horizontal.
 * Vector p should be defined as pointing forward, parallel to the ground,
 * with coordinates {X, Y, Z}.
 */
/*
 * m_max and m_min are calibration values for the maximum and minimum
 * measurements recorded on each magnetic axis, which can vary for each
 * LSM303DLH. You should replace the values below with max and min readings from
 * your particular device.
 *
 * To obtain the max and min values, you can use this program's
 * calibration mode, which is enabled by pressing one of the pushbuttons. While
 * calibration mode is active, point each of the axes of the LSM303DLH toward
 * and away from the earth's North Magnetic Pole. Due to space constraints on an
 * 8x2 LCD, only one axis is displayed at a time; each button selects an axis to
 * display (top = X, middle = Y, bottom = Z), and pressing any button a second
 * time exits calibration mode and returns to normal operation.
 */

#include "LSM303_simple.h"
#include "i2chw/i2cmaster.h"
#include <math.h>
#include "defines.h"
//#include "vector.h"

// First pcb Xmin:	-578 Xmax: 548 Ymin: -520 Ymax: 465 Zmin: -637 Zmax: 442
//			Xmin:	-677 Xmax: 650 Ymin: -661 Ymax: 529 Zmin: -555 Zmax: 591

vector cal_m_max = {0, 0, 0};
vector cal_m_min = {0, 0, 0};
//vector p = {0, 0, 0};
vector p = {0, -1, 0};
//vector m_min = {-578, -520, -637};	// calibration
//vector m_max = {548, 465, 422};		// calibration
vector m_min = {-667, -661, -555};	// calibration
vector m_max = {650, 529, 591};		// calibration
vector a, m;

void init_lsm303(void)
{
	i2c_init();
	
	/* TEST */
	
	// 0x08 = 0b00001000
	// FS = 00 (+/- 2 g full scale); HR = 1 (high resolution enable)
	writeAccReg(CTRL_REG4_A, 0x08);
	/*i2c_start(0x32);		// ACC address
	i2c_write(CTRL_REG4_A);	// CTRL_REG4_A
	i2c_write(0x08);		// Set to high resolution mode
	i2c_stop();*/

	
	// lsm303dlhc sensor
	//enable accelerometer
	// 0x47 = 0b01000111
	// ODR = 0100 (50 Hz ODR); LPen = 0 (normal mode); Zen = Yen = Xen = 1 (all axes enabled)
	writeAccReg(CTRL_REG1_A, 0x47);
	/*i2c_start(0x32);		// ACC address
	i2c_write(CTRL_REG1_A);	// CTRL_REG4_A
	i2c_write(0x47);		// Normal / low-power mode (50 Hz)
	i2c_stop();*/
	/* END TEST */

	//enable magnetometer

	// 0x0C = 0b00001100
	// DO = 011 (7.5 Hz ODR)
	/* Enable temp sensor in LSM303 */
	writeMagReg(CRA_REG_M, 0x8C);	// Enable temp sensor, 7.5Hz
	
	//writeMagReg(CRA_REG_M, 0x0C);
	/*i2c_start(0x3C); // MAG address
	i2c_write(CRA_REG_M); // 
	i2c_write(0x0C); // hertz output 7.5
	i2c_stop();*/

	// 0x20 = 0b00100000
	// GN = 001 (+/- 1.3 gauss full scale)
	writeMagReg(CRB_REG_M, 0x20);
	/*i2c_start(0x3C); // MAG address
	i2c_write(CRB_REG_M); // CRB_REG_M
	i2c_write(0x20); // continuous conversion mode
	i2c_stop();*/

	// 0x00 = 0b00000000
	// MD = 00 (continuous-conversion mode)
	writeMagReg(MR_REG_M, 0x00);
	/*i2c_start(0x3C); // MAG address
	i2c_write(MR_REG_M); // MR_REG_M
	i2c_write(0x00); // continuous conversion mode
	i2c_stop();*/
}

// Returns a set of acceleration and raw magnetic readings from the cmp01a.
void read_data_raw(vector *a, vector *m)
{
	//i2c_start((0x32 >> 1));	//LSM303_ADDRESS_ACCEL
	//i2c_write(0x28 | 0x80);	//LSM303_REGISTER_ACCEL_OUT_X_L_A
	//i2c_stop();
	//i2c_start(0x33);		  // repeated start
	
	//i2c_write(0x30); // write acc
	
	//i2c_write(0xa8); // OUT_X_L_A, MSB set to enable auto-increment
	//i2c_start(0x33);		  // repeated start
	//i2c_write(0x31); // read acc
	
	//i2c_start(0x32);
	//i2c_write(0x28 | 1<<7);
	//i2c_start(0x33);
	
	// read accelerometer values
	i2c_start(0x32);
	i2c_write(0xA8);
	i2c_start(0x33);
	
	uint8_t axl = i2c_readAck();
	uint8_t axh = i2c_readAck();
	uint8_t ayl = i2c_readAck();
	uint8_t ayh = i2c_readAck();
	uint8_t azl = i2c_readAck();
	uint8_t azh = i2c_readNak();
	i2c_stop();
	
	//i2c_start(0x3C);
	//i2c_write(0x3C); // write mag
	//i2c_write(0x03); // OUTXH_M
	//i2c_start(0x3D);		  // repeated start
	//i2c_write(0x3D); // read mag
	
	// read magnetometer values
	i2c_start(0x3C);
	i2c_write(0x03);
	i2c_start(0x3D);
	
	uint8_t mxh = i2c_readAck();	/* LSM303DLHC A data = XYZ */
	uint8_t mxl = i2c_readAck();
	uint8_t mzh = i2c_readAck();	
	uint8_t mzl = i2c_readAck();	/* LSM303DLHC M data = XZY */
	uint8_t myh = i2c_readAck();
	uint8_t myl = i2c_readNak();
	i2c_stop();

	//a->x = (int16_t)((axh << 8) | axl);
	//a->y = (int16_t)((ayh << 8) | ayl);
	//a->z = (int16_t)((azh << 8) | azl);
	a->x = axh << 8 | axl;
	a->y = ayh << 8 | ayl;
	a->z = azh << 8 | azl;
	m->x = mxh << 8 | mxl;
	m->y = myh << 8 | myl;
	m->z = mzh << 8 | mzl;
	//m->x = (mxl | (mxh << 8));
	//m->y = (myl | (myh << 8));
	//m->z = (mzl | (mzh << 8));
}

// Returns a set of acceleration and adjusted magnetic readings from the cmp01a.
void read_data(vector *a, vector *m)
{
	read_data_raw(a, m);

	// shift and scale
	m->x = (m->x - m_min.x) / (m_max.x - m_min.x) * 2 - 1.0;
	m->y = (m->y - m_min.y) / (m_max.y - m_min.y) * 2 - 1.0;
	m->z = (m->z - m_min.z) / (m_max.z - m_min.z) * 2 - 1.0;
}

// Returns a heading (in degrees) given an acceleration vector a due to gravity, a magnetic vector m, and a facing vector p.
int get_heading(const vector *a, const vector *m, const vector *p)
{
	vector E;
	vector N;

	// cross magnetic vector (magnetic north + inclination) with "down" (acceleration vector) to produce "east"
	vector_cross(m, a, &E);
	vector_normalize(&E);

	// cross "down" with "east" to produce "north" (parallel to the ground)
	vector_cross(a, &E, &N);
	vector_normalize(&N);

	// compute heading
	int heading = (round(atan2(vector_dot(&E, p), vector_dot(&N, p)) * 180 / M_PI))-PCB_COMPASS_CORR;
	if (heading < 0) heading += 360;
	return heading;
}

/**/
int get_heading_avg(uint8_t number_of_avg)
{
	int heading;
	vector a_avg = {0,0,0}, m_avg = {0,0,0};
	// take 5 acceleration and magnetic readings and average them
	for(int i = 0; i < number_of_avg; i++)
	{
		read_data(&a, &m);
		
		a_avg.x += a.x;
		a_avg.y += a.y;
		a_avg.z += a.z;
		m_avg.x += m.x;
		m_avg.y += m.y;
		m_avg.z += m.z;
	}
	a_avg.x /= number_of_avg;
	a_avg.y /= number_of_avg;
	a_avg.z /= number_of_avg;
	m_avg.x /= number_of_avg;
	m_avg.y /= number_of_avg;
	m_avg.z /= number_of_avg;
	
	heading = get_heading(&a_avg, &m_avg, &p); 
	return heading; 
}

int calibrate_lsm303(uint8_t axis, uint8_t minmax)
{
	read_data_raw(&a, &m);
	
	switch (axis)
	{
		case X_AXIS:	// Calibrate X Axis
			if (m.x < cal_m_min.x) cal_m_min.x = m.x;
			if (m.x > cal_m_max.x) cal_m_max.x = m.x;
			if (minmax==MIN)
			{
				return cal_m_min.x;
			}
			if (minmax==MAX)
			{
				return cal_m_max.x;
			}
		break;
		
		case Y_AXIS:
			if (m.y < cal_m_min.y) cal_m_min.y = m.y;
			if (m.y > cal_m_max.y) cal_m_max.y = m.y;
			if (minmax==MIN)
			{
				return cal_m_min.y;
			}
			if (minmax==MAX)
			{
				return cal_m_max.y;
			}
		break;
		
		case Z_AXIS:
			if (m.z < cal_m_min.z) cal_m_min.z = m.z;
			if (m.z > cal_m_max.z) cal_m_max.z = m.z;
			if (minmax==MIN)
			{
				return cal_m_min.z;
			}
			if (minmax==MAX)
			{
				return cal_m_max.z;
			}
		break;
		
		default:
			return 0;
		break;
	}
	return 0;
}

double get_heading_simple(void)
{
	float heading;
	
	i2c_start(0x3C);
	i2c_write(0x03);
	i2c_start(0x3D);
	
	uint8_t mxh = i2c_readAck();
	uint8_t mxl = i2c_readAck();
	uint8_t mzh = i2c_readAck();
	uint8_t mzl = i2c_readAck();
	uint8_t myh = i2c_readAck();
	uint8_t myl = i2c_readNak();
	i2c_stop();
	
	m.x = mxh << 8 | mxl;
	m.y = myh << 8 | myl;
	m.z = mzh << 8 | mzl;
	
	m.x = (m.x - m_min.x) / (m_max.x - m_min.x) * 2 - 1.0;
	m.y = (m.y - m_min.y) / (m_max.y - m_min.y) * 2 - 1.0;
	m.z = (m.z - m_min.z) / (m_max.z - m_min.z) * 2 - 1.0;
	// Calculate the angle of the vector y,x

	heading = (atan2(m.y,m.x) * 180) / PI;
	
	// Normalize to 0-360
	if (heading < 0)
	{
		heading = 360 + heading;
	}
	return heading;
}

void print_calibration_lsm303(void)
{
	//uint8_t x_min, x_max, y_min, y_max, z_min, z_max;
	
	/*x_min = calibrate_lsm303(X_AXIS,MIN);
	x_max = calibrate_lsm303(X_AXIS,MAX);
	y_min = calibrate_lsm303(Y_AXIS,MIN);
	y_max = calibrate_lsm303(Y_AXIS,MAX);
	z_min = calibrate_lsm303(Z_AXIS,MIN);
	z_max = calibrate_lsm303(Z_AXIS,MAX);
	
	while (x_min>calibrate_lsm303(X_AXIS,MIN))
	{
	}*/
	uart_puts("Xmin:");
	print_int(calibrate_lsm303(X_AXIS,MIN),FALSE);
	uart_puts(" Xmax:");
	print_int(calibrate_lsm303(X_AXIS,MAX),FALSE);
	uart_puts(" Ymin:");
	print_int(calibrate_lsm303(Y_AXIS,MIN),FALSE);
	uart_puts(" Ymax:");
	print_int(calibrate_lsm303(Y_AXIS,MAX),FALSE);
	uart_puts(" Zmin:");
	print_int(calibrate_lsm303(Z_AXIS,MIN),FALSE);
	uart_puts(" Zmax:");
	print_int(calibrate_lsm303(Z_AXIS,MAX),FALSE);
	uart_puts(" (Rotate device until all axis are at their min/max)\n");
}

void writeAccReg(uint8_t reg, uint8_t value)
{
	i2c_start(0x32);	// ACC address
	i2c_write(reg);	
	i2c_write(value);
	i2c_stop();
}

void writeMagReg(uint8_t reg, uint8_t value)
{
	i2c_start(0x3C);	// Mag address
	i2c_write(reg);	
	i2c_write(value);
	i2c_stop();
}

int get_temp(void)
{
	// Enable sensor first (in init)!
	//Temperature data (8LSB/deg - 12-bit resolution).
	i2c_start(0x3C);			// Its in the Mag address
	i2c_write(TEMP_OUT_H_M);	// Set pointer
	i2c_start(0x3D);			// Start reading

	uint8_t mth = i2c_readAck();
	uint8_t mtl = i2c_readNak();
	
	i2c_stop();
	return (((mth << 8 | mtl)>>4)/4);
}
