#ifndef MPU6050
#define MPU6050

void setupMPU6050(); // Acceleramoter and Gyroscope
void getAcceleration(int16_t* results);
void getGyroscope(int16_t* results);
void realAcc(int16_t* rawValues,double* acc, double* bias);
void realGyro(int16_t* rawValues,double* gyro, double* bias);


#endif
