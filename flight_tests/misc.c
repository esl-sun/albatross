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
#include "misc.h"
#include "hmc5883.h"
#include "bmp085.h"
#include "mpu6050.h"
void calibrateSensors(double* means){

  int16_t* raw_gyro = malloc(4*sizeof(int16_t));
  int16_t* raw_acc = malloc(4*sizeof(int16_t));
  int16_t* raw_mag = malloc(4*sizeof(int16_t));

  double* gyro = malloc(4*sizeof(double));
  double* acc = malloc(4*sizeof(double));
  double* mag = malloc(4*sizeof(double));

  double mean_gyro_x = 0;
  double mean_gyro_y = 0;
  double mean_gyro_z = 0;
  double mean_acc_x = 0;
  double mean_acc_y = 0;
  double mean_acc_z = 0;
  double mean_mag_x = 0;
  double mean_mag_y = 0;
  double mean_mag_z = 0;

  long double sumGyro_x = 0;
  long double sumGyro_y = 0;
  long double sumGyro_z = 0;

  long double sumAcc_x = 0;
  long double sumAcc_y = 0;
  long double sumAcc_z = 0;

  long double sumMag_x = 0;
  long double sumMag_y = 0;
  long double sumMag_z = 0;

  for(int i = 0; i <200; i++){

    getAcceleration(raw_acc);
    getGyroscope(raw_gyro);
    getMagnetometer(raw_mag);
    realAcc(raw_acc,acc,means);
    realGyro(raw_gyro,gyro,means);
    realMag(raw_mag,mag,means);

    sumGyro_x += gyro[0];
    sumGyro_y += gyro[1];
    sumGyro_z += gyro[2];

    sumAcc_x += acc[0];
    sumAcc_y += acc[1];
    sumAcc_z += acc[2];

    sumMag_x += mag[0];
    sumMag_y += mag[1];
    sumMag_z += mag[2];
  }

  mean_acc_x = sumAcc_x/200.0;
  mean_acc_y = sumAcc_y/200.0;
  mean_acc_z = sumAcc_z/200.0;

  mean_gyro_x = sumGyro_x/200.0;
  mean_gyro_y = sumGyro_y/200.0;
  mean_gyro_z = sumGyro_z/200.0;


  mean_mag_x = sumMag_x/200.0;
  mean_mag_y = sumMag_y/200.0;
  mean_mag_z = sumMag_z/200.0;

  printf("mean_gyro_x: %f\n", mean_gyro_x);
  printf("mean_gyro_y: %f\n", mean_gyro_y);
  printf("mean_gyro_z: %f\n", mean_gyro_z);

  printf("mean_acc_x: %f\n", mean_acc_x);
  printf("mean_acc_y: %f\n", mean_acc_y);
  printf("mean_acc_z: %f\n", mean_acc_z);

  printf("mean_acc_x: %f\n", mean_mag_x);
  printf("mean_acc_y: %f\n", mean_mag_y);
  printf("mean_acc_z: %f\n", mean_mag_z);

  means[0] = mean_acc_x;
  means[1] = mean_acc_y;
  means[2] = mean_acc_z;

  means[3] = mean_gyro_x;
  means[4] = mean_gyro_y;
  means[5] = mean_gyro_z;

  means[6] = mean_mag_x;
  means[7] = mean_mag_y;
  means[8] = mean_mag_z;


  free(raw_gyro);
  free(raw_acc);
  free(raw_mag);

  free(gyro);
  free(acc);
  free(mag);



}

 char* createNewLog(){
		FILE *fp1;
		fp1 = fopen("/home/pi/flight_tests/flightnum.txt","r");
		char ch;
		int value;
		ch = fgetc(fp1);
		value = atoi(&ch);
		value = value+1;
		fclose(fp1);
		fp1 = fopen("/home/pi/flight_tests/flightnum.txt","w+");
		fprintf(fp1,"%d",value);
		fclose(fp1);

		char filename[40];
		sprintf(filename,"/home/pi/flight_tests/Flight%d.txt",value);
		FILE *fp;
		fp = fopen(filename,"a");
		fprintf(fp,"New Log");
		fclose(fp);
		char *f_name;
		f_name = malloc(sizeof(char)*40);
		strcpy(f_name,filename);
		return f_name;
	}
