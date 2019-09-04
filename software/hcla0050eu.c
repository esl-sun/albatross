
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/i2c-dev.h>
#include "hcla0050eu.h"

void getAirspeed(double* airspeed){

	int file_i2c;
//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";

	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
	}

	uint8_t addr = 0x78;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
	}


	uint8_t res = 0;
	uint8_t res1 = 0;
	int16_t raw_airspeed = 0;

	res = i2c_smbus_read_byte(file_i2c);
	res1 = i2c_smbus_read_byte(file_i2c);

  raw_airspeed = (int16_t)((res<<8)+res1) -1542;
	*airspeed = raw_airspeed*(50.0/32767);

	close(file_i2c);

}
