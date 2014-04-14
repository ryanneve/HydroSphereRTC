

#ifndef HS_OpenROV_h
#define HS_OpenROV_h


// LSM303 accelerometer: 8 g sensitivity
// 3.9 mg/digit; 1 g = 256
#define GRAVITY 256  //this equivalent to 1G in the raw data coming from the accelerometer 
#define ToRad(x) ((x)*0.01745329252)  // *pi/180
#define ToDeg(x) ((x)*57.2957795131)  // *180/pi

// L3G4200D gyro: 2000 dps full scale
// 70 mdps/digit; 1 dps = 0.07
#define Gyro_Gain_X 0.07 //X axis Gyro gain
#define Gyro_Gain_Y 0.07 //Y axis Gyro gain
#define Gyro_Gain_Z 0.07 //Z axis Gyro gain
#define Gyro_Scaled_X(x) ((x)*ToRad(Gyro_Gain_X)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Y(x) ((x)*ToRad(Gyro_Gain_Y)) //Return the scaled ADC raw data of the gyro in radians for second
#define Gyro_Scaled_Z(x) ((x)*ToRad(Gyro_Gain_Z)) //Return the scaled ADC raw data of the gyro in radians for second

// LSM303 magnetometer calibration constants; use the Calibrate example from
// the Pololu LSM303 library to find the right values for your board
#define M_X_MIN -421
#define M_Y_MIN -639
#define M_Z_MIN -238
#define M_X_MAX 424
#define M_Y_MAX 295
#define M_Z_MAX 472

#define Kp_ROLLPITCH 0.02
#define Ki_ROLLPITCH 0.00002
#define Kp_YAW 1.2
#define Ki_YAW 0.00002

// Log levels
#define DEBUG 0
#define INFO 1
#define WARN 2
#define ERROR 3
#define CRITICAL 4

/*----------( Forward Declarations (?) )----------*/
//bool recentSample(char time[7], int16_t fresh_time);

typedef struct GyroStruct{
  int16_t   gyro_x;
  int16_t   gyro_y;
  int16_t   gyro_z;
  int16_t   accel_x;
  int16_t   accel_y;
  int16_t   accel_z;
  int16_t   magnetom_x;
  int16_t   magnetom_y;
  int16_t   magnetom_z;
  float heading;
};

/*
typedef struct PTStruct{
  int16_t DevAddress;  // 7-bit I2C address of the MS5803
  // Here are the commands that can be sent to the 5803
  uint8_t D1_256 ;
  uint8_t D1_512 ;
  uint8_t D1_1024;
  uint8_t D1_2048;
  uint8_t D1_4096;
  uint8_t D2_256 ;
  uint8_t D2_512 ;
  uint8_t D2_1024;
  uint8_t D2_2048;
  uint8_t D2_4096;
  uint8_t AdcRead;
  uint8_t PromBaseAddress;
  uint8_t Reset; 
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
  uint16_t CalConstant[8];  // Matrix for holding calibration constants
  char  sample_time[7]; // HHmmss\0
  char  comm_error;
};
*/
/*-----( Declare function protoTypes )-----*/

void setupAccelGyro();
void _initAccelGyro();
void getAccelGyro(struct GyroStruct *aMEMS);
void reportAccel(struct GyroStruct *aMEMS);
void reportGyro(struct GyroStruct *aMEMS);

void _initMagnetometer();
void getMagnetometer(struct GyroStruct *aMEMS);
void reportMagnetometer(struct GyroStruct *aMEMS);

void setupPT();
int32_t getTemperature();
int32_t getPressure();
void debugTemperature();

#endif