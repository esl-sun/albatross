
#ifndef HMC5883
#define HMC5883



void getMagnetometer(int16_t* results);
void realMag(int16_t* rawValues, double* mag, double* bias);
void setupHMC5883(); // Magnetometer


#endif
