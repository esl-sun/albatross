#include <unistd.h>				//Needed for I2C port
#include <fcntl.h>				//Needed for I2C port
#include <sys/ioctl.h>			//Needed for I2C port
#include <linux/i2c-dev.h>		//Needed for I2C port
#include <wiringPi.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <termios.h>		//Used for UART
#include <softPwm.h>
#include <gps.h>
#include "misc.h"
#include "bmp085.h"
#include "hmc5883.h"
#include "mpu6050.h"
#include "hcla0050eu.h"


// void calibrateSensors(double* means);
// char* createNewLog();


int main(void){

  int servo1_pin = 0; //pin 1 workVs in this configuration
  int servo2_pin = 2;
  int16_t* baro_coeff  = malloc(12*sizeof(int16_t));
  int16_t* values = malloc(12*sizeof(int16_t));
  // double* states = malloc(12*sizeof(double));

  int16_t* raw_gyro = malloc(4*sizeof(int16_t));
  int16_t* raw_acc = malloc(4*sizeof(int16_t));
  int16_t* raw_mag = malloc(4*sizeof(int16_t));

  double* gyro = malloc(4*sizeof(double));
  double* acc = malloc(4*sizeof(double));
  double* mag = malloc(4*sizeof(double));
  double* mean_values = malloc(10*sizeof(double));

  double* raw_airspeed  = malloc(1*sizeof(double));



  int calibrate_flag = 1;
  int log_flag = 0;
  memset(mean_values,0.0,10);

  setupMPU6050(); // Acceleramoter and Gyroscope
  setupHMC5883(); // Magnetometer
  setupBMP085(baro_coeff); // Barometer
  getBarometer(values,baro_coeff);
  // calibrateSensors(mean_values);
  // printf("Done calibration\n");

	wiringPiSetup();
	softPwmCreate(servo2_pin,0,100);
	softPwmCreate(servo1_pin,0,100);


	int uart0_filestream = -1;
	uart0_filestream = open("/dev/ttyAMA0", O_RDONLY | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode

	if (uart0_filestream == -1){
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}

	struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);

	uint32_t control[16] = {0};
	uint8_t message[16] = {0};
  uint16_t rx_length = 0;
  uint16_t last = 0;
  uint16_t position = 0;
  uint16_t channel_id = 0;
  uint16_t step = 0;

	char *log_file_name;
  FILE *fpNew;
	//fp = fopen("/home/pi/flight_tests/log.txt","a");
	//fprintf(fp,"New Log \n");
	//fclose(fp); //move to end of programme

	struct gps_data_t* gps_data = malloc(sizeof(struct gps_data_t));
	int* ec = malloc(sizeof(int));
  int gps_lock = 0;
  int gps_read = 0;

	if ((*ec = gps_open("localhost", "2947", gps_data)) == -1) {
    		printf("code: %d, reason: %s\n", *ec, gps_errstr(*ec));
    		return EXIT_FAILURE;
	}

	gps_stream(gps_data, WATCH_ENABLE | WATCH_JSON, NULL);

	while(gps_lock == 0){
		gps_lock = gps_lock_status(gps_data);
	}


  while(1){

		get_gps_coord(gps_data,ec);

    if(step == 8){
      step = 0;
      control[4] = (control[4]-2218)/22.52;
      control[7] = (2719.75-control[7])*-0.017366;
      control[3] = (control[3]-4260);
      control[5] = control[5]-6310; // Calibration
      control[2] = control[2]-5280; // log

      if(control[5]>20 && calibrate_flag == 0 && control[2]<600){
        calibrate_flag = 1;
        printf("Calibrating\n");
        calibrateSensors(mean_values);
        printf("Calibration Completed\n");
        printf("Creating New Log File\n");
	      log_file_name = createNewLog();
        log_flag = 1;
      }
      else if(log_flag==1 && control[2]>600 && control[3] > 20){

        printf("Test Start\n");
        softPwmWrite(servo1_pin,control[4]);
        softPwmWrite(servo2_pin,control[7]);
        getAcceleration(raw_acc);
        getGyroscope(raw_gyro);
        getMagnetometer(raw_mag);

        realAcc(raw_acc,acc,mean_values);
        realGyro(raw_gyro,gyro,mean_values);
        realMag(raw_mag,mag,mean_values);

	      getAirspeed(raw_airspeed);
        // printf("gyro_x: %f \ngyro_y: %f \ngyro_z: %f\n",gyro[0], gyro[1], gyro[2]);
    	   fpNew = fopen(log_file_name,"a");
      	fprintf(fpNew,"%d,%d,%f,%f,%f,%f,%f,%f,%f,%f,%f\n",control[4],control[7],acc[0],acc[1],acc[2],gyro[0],gyro[1],gyro[2],mag[0],mag[1],mag[2]);
      	fclose(fpNew); //move to end of programme
      }
      else if(control[3]<20){
          printf("Motors Off\n");
	        softPwmWrite(servo1_pin,0);
	        softPwmWrite(servo2_pin,0);
      }
      else if(control[3]>20){
        printf("Motors Enables\n");
        softPwmWrite(servo1_pin,control[4]);
        softPwmWrite(servo2_pin,control[7]);

      }

      if(control[5]<20){
        calibrate_flag = 0;
      }

      // printf("%d,%d,%d,%d,%d,%d,%d,%d\n", control[0],control[1],control[2],control[3],control[4],control[5],control[6],control[7]);

    }

	  rx_length = read(uart0_filestream, (void*)message, 8);

    if(rx_length > 7 && rx_length < 17){

      decode_spektrum(rx_length,channel_id,control,message,step);

      // for(int i =0; i < rx_length; i++){
      //   if(i%2){
      //
      //     channel_id = last & 0b11111100;
      //     channel_id = channel_id >> 2;
      //     last = last << 8;
      //     position = message[i] | last;
      //     control[step] = (int32_t)(position);
      //     step += 1;
      //   }
      //   last = message[i];
      // }
    }
  }

  free(baro_coeff);
  free(values);
  free(raw_gyro);
  free(raw_acc);
  free(raw_mag);
  free(mean_values);
  free(raw_airspeed);
  free(gps_data);

	return 0;


}
