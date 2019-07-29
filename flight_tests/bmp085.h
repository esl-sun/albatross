#ifndef BMP085
#define BMP085


void setupBMP085(int16_t* coeff); // Barometer
void getBarometer(int16_t* results, int16_t* coeff);

#endif
