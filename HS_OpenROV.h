

#ifndef HS_OpenROV_h
#define HS_OpenROV_h


// LSM303 accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
#define M_X_MIN -421
#define M_Y_MIN -639
#define M_Z_MIN -238
#define M_X_MAX 424
#define M_Y_MAX 295
#define M_Z_MAX 472


typedef struct mpu9150MEMSStruct{
  int   AN[6]; //array that stores the gyro and accelerometer data
  int   AN_OFFSET[6]; //Array that stores the Offset of the sensors
  int   gyro_x;
  int   gyro_y;
  int   gyro_z;
  int   accel_x;
  int   accel_y;
  int   accel_z;
  int   magnetom_x;
  int   magnetom_y;
  int   magnetom_z;
  float MAG_Heading;
  float Accel_Vector[3]; //Store the acceleration in a vector
  float Gyro_Vector[3];//Store the gyros turn rate in a vector
  float Omega_Vector[3]; //Corrected Gyro_Vector data
  float Omega_P[3];//Omega Proportional correction
  float Omega_I[3];//Omega Integrator
  float Omega[3];
  // Euler angles
  float roll;
  float pitch;
  float yaw;
  float errorRollPitch[3];
  float errorYaw[3];
  char  sample_time[7]; // HHmmss\0
  char  comm_error;
};


typedef struct ms5803PTStruct{
  int DevAddress;  // 7-bit I2C address of the MS5803
  // Here are the commands that can be sent to the 5803
  byte D1_256 ;
  byte D1_512 ;
  byte D1_1024;
  byte D1_2048;
  byte D1_4096;
  byte D2_256 ;
  byte D2_512 ;
  byte D2_1024;
  byte D2_2048;
  byte D2_4096;
  byte AdcRead;
  byte PromBaseAddress;
  byte Reset; 
  long AdcTemperature;// Holds raw ADC data for temperature
  long AdcPressure;  // Holds raw ADC data for pressure
  float Temperature;
  float Pressure;
  float TempDifference;
  float Offset;
  float Sensitivity;
  // Offsets for second-order temperature computation
  float T2;
  float Off2;
  float Sens2;
  unsigned int CalConstant[8];  // Matrix for holding calibration constants
  char  sample_time[7]; // HHmmss\0
  char  comm_error;
};


#endif