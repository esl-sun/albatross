# Albatross
Albatross V1 is an autonomous aerial platform capable of taking measurements and photos of the area it is flying above.  It is built with the following:

## Hardware
* Raspberry PI 2B
* PiCamera
* 9DOF IMU (MPU6050)
* Barometer (BMP085)
* Magnetometer (HMC5883)
* Airspeed Sensor (HCLA0050EU)
* Styrofoam Glider Aircraft
* GSM Module (Mini GSM: SIM800L)
* GPS (WaveShare Neo-6M/7M)
* RC Receiver (Spektrum)

### BMP085.c
Retrieving the calibration coefficients, computing the temperature and pressure.

### HCLA0050EU.c
Retrieving data from registers and converting to sensible data

### HMC5883.c
Configuring the magnetometer, retrieving the data from registers and converting to sensible data.


### MPU6050.c
Configuring the IMU, retrieving the data from registers and converting to sensible data.

### MISC.c
Calibration and creating new logging file.

### FLIGHT_TEST.c
flight_test.c is the RC version of Albatross. Currently it is capable of controlling the servo's and logging the IMU data. When the specific commands are given by the test-pilot the system will calibrate and or start a new test. 

### How to Compile
```
gcc -o flight *.c -lm -lwiringPi -Wall -lpthread -std=c11 -lgps
```
