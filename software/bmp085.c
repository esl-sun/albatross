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
#include <wiringPi.h>
#include "bmp085.h"


void getBarometer(int16_t* results, int16_t* coeff){

	int file_i2c;

//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";

	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
	}

	uint8_t addr = 0x77;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
	}

  uint8_t reg1 = 0xf4;
  uint8_t reg2 = 0xf6;
  uint8_t reg3 = 0xf7;
  uint8_t reg4 =  0xf8;

  uint8_t val = 0x2e;

	uint8_t res = 0;
	uint8_t res1 = 0;
  uint8_t res2 = 0;
  int16_t UT = 0;

	i2c_smbus_write_word_data(file_i2c, reg1, val);

  // usleep(5000);
  delayMicroseconds(4500);

  res = i2c_smbus_read_word_data(file_i2c, reg2);
  res1 = i2c_smbus_read_word_data(file_i2c, reg3);

  UT = (res<<8)+res1;


  delayMicroseconds(7500);

  val = 0x34;
  uint8_t over_sample = 3;
  val = val + (over_sample<<6);


	i2c_smbus_write_word_data(file_i2c, reg1, val);

  // usleep(7500);
  delayMicroseconds(25500);


  res = i2c_smbus_read_word_data(file_i2c, reg2);
  res1 = i2c_smbus_read_word_data(file_i2c, reg3);
  res2 = i2c_smbus_read_word_data(file_i2c, reg4);

  int32_t UP = 0;

  UP = ((int32_t)(res<<16)+(int32_t)(res1<<8)+(int32_t)res2)>>(8-over_sample);
  // UP = (res<<8)+res1;


/////////////////////////////////////////////////////////////////////////////////////
// Calculate True Temperature
  int32_t X1 = ( ( UT-(int32_t)((uint16_t)coeff[5]) )* ( (int32_t)((uint16_t)coeff[4]) ) )>>15;

  int32_t X2 = ( (int32_t)coeff[9]<<11 )/( X1 + (int32_t)coeff[10] );

  int32_t B5 = X1 +X2;

  double T = ((B5+8)>>4)/10;
/////////////////////////////////////////////////////////////////////////////////////
// Calculate True Pressure

  int32_t B6 = B5 - 4000;

  X1 = ((int32_t)coeff[7] * ((B6*B6)>>12))>>11;

  X2 = ((int32_t)coeff[1]*B6)>>11;

  int32_t X3 = X1+X2;

  int32_t B3 = ( (((int32_t)coeff[0]*4+X3)<<over_sample)+2)/4;

  X1 =  ((int32_t)coeff[2]*B6)>>13;

  X2 = ((int32_t)coeff[6]*((B6*B6)>>12))>>16;

  X3 = ((X1+X2)+2)>>2;

  uint32_t B4 = ( (uint32_t)((uint16_t)coeff[3]) )*(uint32_t)(X3 + 32768)>>15;

  uint32_t B7 = ((int32_t)UP-B3)*(uint32_t)(50000UL>>over_sample);

  int32_t p = 0;

  if(B7 < 0x80000000){
    p = (B7*2)/B4;
  }
  else{
    p = (B7/B4)*2;
  }



  X1 = (p>>8)*(p>>8);

  X1 = (X1*(int32_t)3038)>>16;

  X2 = ((int32_t)(-7357)*p)>>16;

  p = p + ((X1+X2+(int32_t)3791)>>4);

  double exponent = 1/5.255;
  double sea_lvl_p = 101325;
  double ratio = ((double)p)/sea_lvl_p;

  double altitude = 44307.69*( 1.0 - pow(ratio, exponent));


  printf("temp: %f \npressure: %d \naltitude: %f\n", T, p, altitude);

}


void setupBMP085(int16_t* coeff){

	int file_i2c;

//----- OPEN THE I2C BUS -----
	char *filename = (char*)"/dev/i2c-1";

	if ((file_i2c = open(filename, O_RDWR)) < 0)
	{
		//ERROR HANDLING: you can check errno to see what went wrong
		printf("Failed to open the i2c bus");
	}

	uint8_t addr = 0x77;          //<<<<<The I2C address of the slave
	if (ioctl(file_i2c, I2C_SLAVE, addr) < 0)
	{
		printf("Failed to acquire bus access and/or talk to slave.\n");
	}


  uint8_t counter =0;

	uint8_t reg = 0xAA;
	uint8_t reg1 = reg + 1;
	uint16_t res = 0;
	uint8_t res1 = 0;

  for(counter =0; counter < 11; counter++){

  	res = i2c_smbus_read_word_data(file_i2c, reg);
  	res1 = i2c_smbus_read_word_data(file_i2c, reg1);
    coeff[counter] = (int16_t)((res<<8)+res1);
    reg += 2;
    reg1 += 2;
  }

	close(file_i2c);

}
