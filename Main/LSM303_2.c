/***************************************************************************
  This is a library for the LSM303 Accelerometer and magnentometer/compass

  Designed specifically to work with the Adafruit LSM303DLHC Breakout

  These displays use I2C to communicate, 2 pins are required to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <stdlib.h>
#include <avr/io.h>
#include <util/delay.h>
//path to i2c fleury lib
#include "i2chw/i2cmaster.h"
#include "uart.h"
#include "LSM303_2.h"
/***************************************************************************
 CONSTRUCTOR
 ***************************************************************************/

uint8_t init_LSM303()
{
	i2c_init();
	
	_delay_us(10);
  //Wire.begin();
uart_puts("Here1\n");
  // Enable the accelerometer
  //write8(LSM303_ADDRESS_ACCEL | I2C_WRITE, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x27);
  //write8(LSM303_ADDRESS_ACCEL | I2C_WRITE, LSM303_REGISTER_ACCEL_CTRL_REG1_A, 0x27);	// ???? I2C_WRITE OR NOT? 
  
  //regValue = regValue << 5;
  //i2c_start(LSM303_ADDRESS_ACCEL);
  //i2c_write(LSM303_REGISTER_ACCEL_CTRL_REG1_A);
  //i2c_write(0x27);
 // i2c_stop();

  //set measurement mode
  i2c_start_wait(HMC5883L_ADDR | I2C_WRITE);
  i2c_start((0x19<<1) | I2C_WRITE);	// CORRECT -> probably something with W/R and bitshifts...
  i2c_start((LSM303_ADDRESS_ACCEL) | I2C_WRITE);
  
  
  //i2c_write(HMC5883L_MODEREG);
  //i2c_write(HMC5883L_MEASUREMODE);
  //i2c_stop();
  
  /*
  i2c_start_wait(LSM303_ADDRESS_ACCEL);
  i2c_write(LSM303_REGISTER_ACCEL_CTRL_REG1_A);
  i2c_write(0x27);
  i2c_stop();
  */
  uart_puts("Here2\n");
  
  // Enable the magnetometer
  //write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_MR_REG_M, 0x00);
uart_puts("Here3\n");
  return 1;
}

/***************************************************************************
 PUBLIC FUNCTIONS
 ***************************************************************************/
void read_LSM303(void)
{
	uint8_t buff[6];
  // Read the accelerometer
  i2c_start_wait((uint8_t)LSM303_ADDRESS_ACCEL | I2C_WRITE);
  i2c_write(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
  i2c_stop();
  i2c_start_wait(LSM303_ADDRESS_ACCEL | I2C_READ);
  for(uint8_t i=0; i<6; i++) {
	  if(i==6-1)
	  buff[i] = i2c_readNak();
	  else
	  buff[i] = i2c_readAck();
  }
  i2c_stop();
  
  accelData.x = (int16_t)((buff[0] << 8) | buff[1]);	/* cast to 16 int????*/
  accelData.y = (int16_t)((buff[2] << 8) | buff[3]);
  accelData.z = (int16_t)((buff[4] << 8) | buff[5]);
  
  //Wire.requestFrom((uint8_t)LSM303_ADDRESS_ACCEL, (uint8_t)6);

  // Wait around until enough data is available
  /*while (Wire.available() < 6);

  uint8_t xlo = Wire.read();
  uint8_t xhi = Wire.read();
  uint8_t ylo = Wire.read();
  uint8_t yhi = Wire.read();
  uint8_t zlo = Wire.read();
  uint8_t zhi = Wire.read();*/

  // Shift values to create properly formed integer (low byte first)
  // KTOWN: 12-bit values are left-aligned, no shift needed
  // accelData.x = (xlo | (xhi << 8)) >> 4;
  // accelData.y = (ylo | (yhi << 8)) >> 4;
  // accelData.z = (zlo | (zhi << 8)) >> 4;
  /*accelData.x = (int16_t)((xhi << 8) | xlo);
  accelData.y = (int16_t)((yhi << 8) | ylo);
  accelData.z = (int16_t)((zhi << 8) | zlo);*/
  
  i2c_start_wait((uint8_t)LSM303_ADDRESS_MAG | I2C_WRITE);
  i2c_write(LSM303_REGISTER_MAG_OUT_X_H_M);
  i2c_stop();
  i2c_start_wait(LSM303_ADDRESS_MAG | I2C_READ);
  for(uint8_t i=0; i<6; i++) {
	  if(i==6-1)
	  buff[i] = i2c_readNak();
	  else
	  buff[i] = i2c_readAck();
  }
  i2c_stop();
  
  magData.x = (int16_t)((buff[0] << 8) | buff[1]);	/* cast to 16 int????*/
  magData.y = (int16_t)((buff[2] << 8) | buff[3]);
  magData.z = (int16_t)((buff[4] << 8) | buff[5]);
  
  
  
  // Read the magnetometer
  /*Wire.beginTransmission((uint8_t)LSM303_ADDRESS_MAG);
  Wire.write(LSM303_REGISTER_MAG_OUT_X_H_M);
  Wire.endTransmission();
  Wire.requestFrom((uint8_t)LSM303_ADDRESS_MAG, (uint8_t)6);
  */
  // Wait around until enough data is available
  /*while (Wire.available() < 6);

  // Note high before low (different than accel)  
  xhi = Wire.read();
  xlo = Wire.read();
  zhi = Wire.read();
  zlo = Wire.read();
  yhi = Wire.read();
  ylo = Wire.read();
  
  // Shift values to create properly formed integer (low byte first)
  magData.x = (xlo | (xhi << 8));
  magData.y = (ylo | (yhi << 8));
  magData.z = (zlo | (zhi << 8));  */
  
  // ToDo: Calculate orientation
  magData.orientation = 0.0;
}

void setMagGain_LSM303(lsm303MagGain gain)
{
  write8(LSM303_ADDRESS_MAG, LSM303_REGISTER_MAG_CRB_REG_M, (uint8_t)gain);
}

/***************************************************************************
 PRIVATE FUNCTIONS
 ***************************************************************************/
void write8(uint8_t address, uint8_t reg, uint8_t value)
{
	i2c_start_wait(address);
	i2c_write(reg);
	i2c_write(value);
	i2c_stop();
	
  /*Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();*/
}

uint8_t read8(uint8_t address, uint8_t reg)
{
	uint8_t value;

	i2c_start_wait((uint8_t)LSM303_ADDRESS_ACCEL | I2C_WRITE);
	i2c_write(LSM303_REGISTER_ACCEL_OUT_X_L_A | 0x80);
	i2c_stop();
	i2c_start_wait(LSM303_ADDRESS_ACCEL | I2C_READ);


	i2c_start_wait(address);
	i2c_write(reg);
	i2c_stop();
	i2c_start_wait(LSM303_ADDRESS_ACCEL | I2C_READ);
  
	value =	i2c_readNak(); // READ NACK OR ACK? value = Wire.read();
	i2c_stop();

	return value;
}
