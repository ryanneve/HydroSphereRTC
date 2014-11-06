/*
Sketch to read a MS5803-14BA pressure sensor, written from scratch.
Will output data to the serial console.

Written by Walt Holm 
Initial revision 10 Oct 2013
Rev 1 12 Oct 2013 -- Implements 2nd order temperature compensation
*/

/* See
http://openrov.com/profiles/blogs/introduction-to-the-openrov-imu-depth-sensor
*/


#include <arduino.h>
#include <Wire.h>
#include <Logger_SD.h>
//#include <L3G.h>  // For accelerometer
//#include <LSM303.h> // For accelerometer

#include <I2Cdev.h>
#include <AK8975.h>
#include <MPU6050.h>
//#include <MS5803_I2C.h>
#include "HS_OpenROV.h"

#define MAGNETOMETER_ADDRESS 0x0C

const int16_t SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// Define some Objects
//L3G gyro = L3G();
//LSM303 compass;

AK8975 mag(MAGNETOMETER_ADDRESS);
MPU6050 accelgyro; // address = 0x68, the default, on MPU6050 EVB
//MS5803 presstemp(0x76);

// Program initialization starts here

void setupAccelGyro(){   
  Logger_SD::Instance()->msgL(DEBUG,"setupAccelGyro");
  _initAccelGyro();
  _initMagnetometer();
  Serial.println("Testing Accel/Gyro device connections...");
  bool result = accelgyro.testConnection();
  if ( result ) Logger_SD::Instance()->msgL(INFO,F("AccelGyro connection successful"));
  else {
    uint8_t dev_id = accelgyro.getDeviceID();
    Logger_SD::Instance()->msgL(WARN,F("AccelGyro connection failed with DeviceID %#Xh"),dev_id);
  }
  if ( mag.testConnection() ) {
    Logger_SD::Instance()->msgL(INFO,F("Magnetometer connection successful"));
  }
  else {
    uint8_t dev_id = accelgyro.getDeviceID();
    Logger_SD::Instance()->msgL(WARN,F("Magnetometer connection failed with DeviceID %#Xh"),dev_id);
  }
  accelgyro.setFullScaleAccelRange(0); // Set out maximum acceleration to +/- 2g
  uint8_t range = pow(2,(accelgyro.getFullScaleAccelRange() + 1));
  Logger_SD::Instance()->msgL(INFO,"Accelerometer range is +/- %dg",range);
  }

void _initAccelGyro(){
  accelgyro.initialize();
  accelgyro.setI2CBypassEnabled(true); //the host application processor will be able to directly access the * auxiliary I2C bus
}

void getAccelGyro(struct GyroStruct *aMEMS){
  Logger_SD::Instance()->msgL(DEBUG,"getAccelGyro");
  accelgyro.getMotion6(&aMEMS->accel_x,&aMEMS->accel_y,&aMEMS->accel_z,&aMEMS->gyro_x,&aMEMS->gyro_y,&aMEMS->gyro_z);
  // May need to correct for orientation
}

void reportAccel(struct GyroStruct *aMEMS){
  Logger_SD::Instance()->msgL(DEBUG,F("Acceleration readings: %d{x} %d(y) %d(z)"),aMEMS->accel_x,aMEMS->accel_y,aMEMS->accel_z);
}
void reportGyro(struct GyroStruct *aMEMS){
  Logger_SD::Instance()->msgL(DEBUG,F("Gyro readings: %d{x} %d(y) %d(z)"),aMEMS->gyro_x,aMEMS->gyro_y,aMEMS->gyro_z);
}

void _initMagnetometer(){
  mag.initialize();
}

void getMagnetometer(struct GyroStruct *aMEMS){
  // read raw heading measurements from device
  mag.getHeading(&aMEMS->magnetom_x, &aMEMS->magnetom_y, &aMEMS->magnetom_z);
  aMEMS->heading = atan2((double)aMEMS->magnetom_y, (double)aMEMS->magnetom_x) * 180.0/3.14159265 + 180;
  while (aMEMS->heading < 0)   aMEMS->heading += 360;
  while (aMEMS->heading > 360) aMEMS->heading -= 360;
  // This was to correct for orientation. Still may be useful
  //aMEMS->magnetom_x = SENSOR_SIGN[6] * aMEMS->magnetom_x;
  //aMEMS->magnetom_y = SENSOR_SIGN[7] * aMEMS->magnetom_y;
  //aMEMS->magnetom_z = SENSOR_SIGN[8] * aMEMS->magnetom_z;
}

void reportMagnetometer(struct GyroStruct *aMEMS) {
  char headstr[10];
  dtostrf(aMEMS->heading,5,1,headstr);
  Logger_SD::Instance()->msgL(INFO,"magnetometer heading is %s based on readings: %d{x} %d(y) %d(z)",headstr,aMEMS->magnetom_x,aMEMS->magnetom_y,aMEMS->magnetom_z);
}