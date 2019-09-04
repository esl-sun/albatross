
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include "hmc5883.h"


void realMag(int16_t* rawValues, double* mag, double* bias){

  double ratio = 1.3/65535.0; // for a full scale ratio of 1.3Ga

  double mag_x = ((double)rawValues[0])*ratio;
  double mag_y = ((double)rawValues[1])*ratio;
  double mag_z = ((double)rawValues[2])*ratio;

  mag[0] = mag_x;
  mag[1] = mag_y;
  mag[2] = mag_z;

  // printf("mag_x: %f \nmag_y: %f \nmag_z: %f\n",mag_x, mag_y, mag_z);
}
void setupHMC5883(){
	int file_i2c;


//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";

	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
	}

	uint8_t addr = 0x1e;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
	}

  uint8_t config_reg_a_setting = 0x30;
  uint8_t config_reg_a_addr = 0x00;

  uint8_t config_reg_b_setting = 0x20;
  uint8_t config_reg_b_addr = 0x01;


  uint8_t mode_setting = 0x00;
  uint8_t mode_addr = 0x02;

	i2c_smbus_write_word_data(file_i2c, config_reg_a_addr, config_reg_a_setting);
	i2c_smbus_write_word_data(file_i2c, config_reg_b_addr, config_reg_b_setting);
	i2c_smbus_write_word_data(file_i2c, mode_addr, mode_setting);


	close(file_i2c);

}

void getMagnetometer(int16_t* results){

	int file_i2c;


//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";

	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
	}

	uint8_t addr = 0x1e;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
	}

	uint8_t reg = 0x03;
	uint8_t reg1 = reg + 1;
	uint16_t res = 0;
	uint8_t res1 = 0;


	res = i2c_smbus_read_word_data(file_i2c, reg);
	res1 = i2c_smbus_read_word_data(file_i2c, reg1);

  results[0] = (int16_t)((res<<8) + res1);

	reg = 0x05;
	reg1 = reg + 1;
	res = 0;
	res1 = 0;

	res = i2c_smbus_read_word_data(file_i2c, reg);
	res1 = i2c_smbus_read_word_data(file_i2c, reg1);
  results[1] = (int16_t)((res<<8) + res1);


	reg = 0x07;
	reg1 = reg + 1;
	res = 0;
	res1 = 0;


	res = i2c_smbus_read_word_data(file_i2c, reg);
	res1 = i2c_smbus_read_word_data(file_i2c, reg1);
  results[2] = (int16_t)((res<<8) + res1);

	close(file_i2c);



}
