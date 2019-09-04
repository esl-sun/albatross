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
#include <gps.h>
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

int gps_lock_status(struct gps_data_t* gps_data){
	int rc;
  if (gps_waiting (gps_data,100)) {
      	/* read data */
          if ((rc = gps_read(gps_data)) == -1) {
            printf("error occured reading gps data. code: %d, reason: %s\n", rc, gps_errstr(rc));
            return 0;
        	} else {
            /* Display data from the GPS receiver. */
            			if ((gps_data->status == STATUS_FIX) &&
               			(gps_data->fix.mode == MODE_2D || gps_data->fix.mode == MODE_3D) &&
                		!isnan(gps_data->fix.latitude) &&
                		!isnan(gps_data->fix.longitude) && !isnan(gps_data->fix.altitude)) {
              return 1;
            } else {
              return 0;
            }
          }
    }
		return 0;

}

void get_gps_coord(struct gps_data_t* gps_data, int* error_code){

	if (gps_waiting (gps_data,10000000)) {
		/* read data */
		if ((*error_code = gps_read(gps_data)) == -1) {
	  	printf("error occured reading gps data. code: %d, reason: %s\n", *error_code, gps_errstr(*error_code));
		}
	}
}

void decode_spektrum(uint16_t rx_length,uint16_t channel_id,uint32_t* control,uint8_t* message,uint16_t step){

  uint16_t position = 0;
  uint16_t last = 0;

  for(int i =0; i < rx_length; i++){

    if(i%2){

      channel_id = last & 0b11111100;
      channel_id = channel_id >> 2;
      last = last << 8;
      position = message[i] | last;
      control[step] = (int32_t)(position);
      step += 1;
    }
    last = message[i];
  }
}

void get_heading_distance(double* curr_coord, double* dest_coord, double* distance, double* bearing){

  double psi[2]; //Both latitude coordinates in radians  N_curr;N_dest
  double lambda[2]; //both longitude coordinates in radians E_curr;E_dest

  double R = 6371000; //earths radius in meters

  double delta_psi = 0;
  double delta_lambda = 0;

  double a = 0;
  double c = 0;
  double d = 0;

  double theta = 0; //bearing to current destination

  psi[0] = curr_coord[0]*(M_PI/180);
  psi[1] = dest_coord[0]*(M_PI/180);

  lambda[0] = curr_coord[1]*(M_PI/180);
  lambda[1] = dest_coord[1]*(M_PI/180);

  delta_psi = (psi[1] - psi[0]);
  delta_lambda = (lambda[1] - lambda[0]);

  a = sin(delta_psi/2)*sin(delta_psi/2) + cos(psi[0])*cos(psi[1])*sin(delta_lambda/2)*sin(delta_lambda/2);
  c = 2*atan2(sqrt(a),sqrt(1-a));
  d = R*c;

  theta = atan2(sin(delta_lambda)*cos(psi[1]),cos(psi[0])*sin(psi[1])-sin(psi[0])*cos(psi[1])*cos(delta_lambda))*(180/M_PI);

  theta = (theta+360) % 360;

  *bearing = theta;
  *distance = d;

}
