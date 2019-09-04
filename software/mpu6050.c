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
#include "mpu6050.h"

void realGyro(int16_t* rawValues, double* gyro, double* bias){

  double ratio = 1000/65535.0; // for a full scale ratio of 250

  double gyro_x = ((double)rawValues[0])*ratio;
  double gyro_y = ((double)rawValues[1])*ratio;
  double gyro_z = ((double)rawValues[2])*ratio;

  gyro[0] = gyro_x - bias[3];
  gyro[1] = gyro_y - bias[4];
  gyro[2] = gyro_z - bias[5];
}

void realAcc(int16_t* rawValues,double* acc, double* bias){

  double ratio = 8/65535.0; // for a full scale ratio of 2

  double acc_x = ((double)rawValues[0])*ratio;
  double acc_y = ((double)rawValues[1])*ratio;
  double acc_z = ((double)rawValues[2])*ratio;

  acc[0] = acc_x - bias[0];
  acc[1] = acc_y - bias[1];
  acc[2] = acc_z - bias[2];

}
void getGyroscope(int16_t* results){
	int file_i2c;
//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";
	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
	}
	uint8_t addr = 0x68;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
	}

	i2c_smbus_write_word_data(file_i2c, 0x6b, 0);
	uint8_t reg = 0x43;
	uint8_t reg1 = reg + 1;
	uint16_t res = 0;
	uint8_t res1 = 0;


	res = i2c_smbus_read_word_data(file_i2c, reg);
	res1 = i2c_smbus_read_word_data(file_i2c, reg1);

  results[0] = (int16_t)((res<<8) + res1);

	reg = 0x45;
	reg1 = reg + 1;
	res = 0;
	res1 = 0;

	res = i2c_smbus_read_word_data(file_i2c, reg);
	res1 = i2c_smbus_read_word_data(file_i2c, reg1);
  results[1] = (int16_t)((res<<8) + res1);


	reg = 0x47;
	reg1 = reg + 1;
	res = 0;
	res1 = 0;


	res = i2c_smbus_read_word_data(file_i2c, reg);
	res1 = i2c_smbus_read_word_data(file_i2c, reg1);
  results[2] = (int16_t)((res<<8) + res1);
	close(file_i2c);
}

void getAcceleration(int16_t* results){
	int file_i2c;
//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";

	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
	}

	uint8_t addr = 0x68;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
	}

	i2c_smbus_write_word_data(file_i2c, 0x6b, 0);

	uint8_t reg = 0x3b;
	uint8_t reg1 = reg + 1;
	uint8_t res = 0;
	uint8_t res1 = 0;


	res = i2c_smbus_read_word_data(file_i2c, reg);
	res1 = i2c_smbus_read_word_data(file_i2c, reg1);
	// val = (res<<8) + res1;
  results[0] = (int16_t)((res<<8)+res1);

	reg = 0x3d;
	reg1 = reg + 1;
	res = 0;
	res1 = 0;


	res = i2c_smbus_read_word_data(file_i2c, reg);
	res1 = i2c_smbus_read_word_data(file_i2c, reg1);
  results[1] = (int16_t)((res<<8)+res1);


	reg = 0x3f;
	reg1 = reg + 1;
	res = 0;
	res1 = 0;


	res = i2c_smbus_read_word_data(file_i2c, reg);
	res1 = i2c_smbus_read_word_data(file_i2c, reg1);
	// val = (res<<8) + res1;
  results[2] = (int16_t)((res<<8)+res1);

	close(file_i2c);

}

void setupMPU6050(){

	int file_i2c;

//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";

	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
	}

	uint8_t addr = 0x68;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
	}

  uint8_t config_setting = 0x30; // default value
  uint8_t config_addr = 0x1a;

  // uint8_t gyro_config = 0x00; // Full scale range 250
  // uint8_t gyro_config = 0x08; // Full scale range 500
  uint8_t gyro_config = 0x10; // Full scale range 1000
  // uint8_t gyro_config = 0x18; // Full scale range 2000
  uint8_t config_gyro_addr = 0x1b;


  // uint8_t acc_config = 0x00; // Full scale range 2g
  // uint8_t acc_config = 0x08; // Full scale range 4g
  uint8_t acc_config = 0x10; // Full scale range 8g
  // uint8_t acc_config = 0x18; // Full scale range 16g
  uint8_t config_acc_addr = 0x1c;

	i2c_smbus_write_word_data(file_i2c, config_addr, config_setting);
	i2c_smbus_write_word_data(file_i2c, config_gyro_addr, gyro_config);
	i2c_smbus_write_word_data(file_i2c, config_acc_addr, acc_config);


	close(file_i2c);
}
