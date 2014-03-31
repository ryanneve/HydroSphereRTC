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



#include <Wire.h>

#include "L3G.h"  // For accelerometer
#include "LSM303.h" // For accelerometer
#include "HS_OpenROV.h"



int SENSOR_SIGN[9] = {1,1,1,-1,-1,-1,1,1,1}; //Correct directions x,y,z - gyro, accelerometer, magnetometer

// Define some Objects
L3G gyro;
LSM303 compass;

// Program initialization starts here

void I2C_Init(){ Wire.begin();}

void Pressure_Init(struct ms5803PTStruct *aPT){
   // Here are the pressure related commands that can be sent to the 5803
  aPT->D1_256 = 0x40; 
  aPT->D1_512 = 0x42;
  aPT->D1_1024 = 0x44;
  aPT->D1_2048 = 0x46;
  aPT->D1_4096 = 0x48;
}
void Temperature_Init(struct ms5803PTStruct *aPT){
   // Here are the temperature related commands that can be sent to the 5803
  aPT->D2_256 = 0x50;
  aPT->D2_512 = 0x52;
  aPT->D2_1024 = 0x54;
  aPT->D2_2048 = 0x56;
  aPT->D2_4096 = 0x58;
}
void ms5803_Setup(struct ms5803PTStruct *aPT){
  I2C_Init();
  Serial.println("initialized I2C");
  delay(10);
  delay(1000);
  byte ByteHigh, ByteLow;  // Variables for I2C reads
  
  aPT->DevAddress = 0x76;  // 7-bit I2C address of the MS5803
   // Here are the commands that can be sent to the 5803
  aPT->AdcRead = 0x00;
  aPT->PromBaseAddress = 0xA0;
  aPT->Reset = 0x1E;
  Pressure_Init(aPT);
  Temperature_Init(aPT);
  
  // Reset the device and check for device presence
  sendWireCommand(aPT->Reset,aPT->DevAddress);
  delay(1000);
  Serial.println("Device is reset");
   
  // Get the calibration constants and store in array
  Serial.println("Calibration constants are:");
  for (byte i = 0; i < 8; i++) {
    sendWireCommand(aPT->PromBaseAddress + (2*i),aPT->DevAddress);
    Wire.requestFrom(aPT->DevAddress, 2);
    while(Wire.available()){
      ByteHigh = Wire.read();
      ByteLow = Wire.read();
    }
    aPT->CalConstant[i] = (((unsigned int)ByteHigh << 8) + ByteLow);
    Serial.println(aPT->CalConstant[i]);
  }
}

void mpu9150_Setup(struct mpu9150MEMSStruct *aMEMS){   
  I2C_Init();

  Serial.println("Pololu MinIMU-9 + Arduino AHRS");
  Accel_Init();
  Compass_Init();
  Gyro_Init();
  
  /*a
  MEMS->AN_OFFSET[6] = {0,0,0,0,0,0}; //Array that stores the Offset of the sensors
  aMEMS->Accel_Vector = {0,0,0}; //Store the acceleration in a vector
  aMEMS->Gyro_Vector = {0,0,0};//Store the gyros turn rate in a vector
  aMEMS->Omega_Vector = {0,0,0}; //Corrected Gyro_Vector data
  aMEMS->Omega_P = {0,0,0};//Omega Proportional correction
  aMEMS->Omega_I = {0,0,0};//Omega Integrator
  aMEMS->Omega = {0,0,0};
  aMEMS->errorRollPitch = {0,0,0}; 
  aMEMS->errorYaw = {0,0,0};
  */
  
  delay(20);
  
  for(int i=0;i<32;i++)    // We take some readings...
    {
    Read_Gyro(aMEMS);
    Read_Accel(aMEMS);
    for(int y=0; y<6; y++)   // Cumulate values
      aMEMS->AN_OFFSET[y] += aMEMS->AN[y];
    delay(20);
    }
    
  for(int y=0; y<6; y++)
    aMEMS->AN_OFFSET[y] = aMEMS->AN_OFFSET[y]/32;
    
  aMEMS->AN_OFFSET[5]-=GRAVITY*SENSOR_SIGN[5];
  
  //Serial.println("Offset:");
  for(int y=0; y<6; y++)
    Serial.println(aMEMS->AN_OFFSET[y]);
}

void Gyro_Init(){
  gyro.init();
  gyro.writeReg(L3G_CTRL_REG4, 0x20); // 2000 dps full scale
  gyro.writeReg(L3G_CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
}

void Read_Gyro(struct mpu9150MEMSStruct *aMEMS){
  gyro.read();
  aMEMS->AN[0] = gyro.g.x;
  aMEMS->AN[1] = gyro.g.y;
  aMEMS->AN[2] = gyro.g.z;
  aMEMS->gyro_x = SENSOR_SIGN[0] * (aMEMS->AN[0] - aMEMS->AN_OFFSET[0]);
  aMEMS->gyro_y = SENSOR_SIGN[1] * (aMEMS->AN[1] - aMEMS->AN_OFFSET[1]);
  aMEMS->gyro_z = SENSOR_SIGN[2] * (aMEMS->AN[2] - aMEMS->AN_OFFSET[2]);
}

void Accel_Init(){
  compass.init();
  compass.enableDefault();
  switch (compass.getDeviceType())
  {
    case LSM303::device_D:
      compass.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
      break;
    case LSM303::device_DLHC:
      compass.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
      break;
    default: // DLM, DLH
      compass.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
  }
}

// Reads x,y and z accelerometer registers
void Read_Accel(struct mpu9150MEMSStruct *aMEMS){
  compass.readAcc();
  
  aMEMS->AN[3] = compass.a.x >> 4; // shift left 4 bits to use 12-bit representation (1 g = 256)
  aMEMS->AN[4] = compass.a.y >> 4;
  aMEMS->AN[5] = compass.a.z >> 4;
  aMEMS->accel_x = SENSOR_SIGN[3] * (aMEMS->AN[3] - aMEMS->AN_OFFSET[3]);
  aMEMS->accel_y = SENSOR_SIGN[4] * (aMEMS->AN[4] - aMEMS->AN_OFFSET[4]);
  aMEMS->accel_z = SENSOR_SIGN[5] * (aMEMS->AN[5] - aMEMS->AN_OFFSET[5]);
}

void Compass_Init(){
  // doesn't need to do anything because Accel_Init() should have already called compass.enableDefault()
}

void Read_Compass(struct mpu9150MEMSStruct *aMEMS){
  compass.readMag();
  
  aMEMS->magnetom_x = SENSOR_SIGN[6] * compass.m.x;
  aMEMS->magnetom_y = SENSOR_SIGN[7] * compass.m.y;
  aMEMS->magnetom_z = SENSOR_SIGN[8] * compass.m.z;
}


void get_Temperature(struct ms5803PTStruct *aPT){
   byte ByteHigh, ByteMiddle, ByteLow;  // Variables for I2C reads
  // Read the Device for the ADC Temperature and Pressure values
  
  sendWireCommand(aPT->D2_512,aPT->DevAddress);
  delay(10);
  sendWireCommand(aPT->AdcRead,aPT->DevAddress);
  Wire.requestFrom(aPT->DevAddress, 3);
  while(Wire.available())
  {
    ByteHigh = Wire.read();
    ByteMiddle = Wire.read();
    ByteLow = Wire.read();
  }
  aPT->AdcTemperature = ((long)ByteHigh << 16) + ((long)ByteMiddle << 8) + (long)ByteLow;
 // Serial.print("D2 is: ");
//  Serial.println(aPT->AdcTemperature);
  
    
  // Calculate the Temperature (first-order computation)
  
  aPT->TempDifference = (float)(aPT->AdcTemperature - ((long)aPT->CalConstant[5] << 8));
  aPT->Temperature = (aPT->TempDifference * (float)aPT->CalConstant[6])/ pow(2, 23);
  aPT->Temperature = aPT->Temperature + 2000;  // This is the temperature in hundredths of a degree C
  
  // Calculate the second-order offsets
  
  if (aPT->Temperature < 2000.0)  // Is temperature below or above 20.00 deg C ?
  {
    aPT->T2 = 3 * pow(aPT->TempDifference, 2) / pow(2, 33);
    aPT->Off2 = 1.5 * pow((aPT->Temperature - 2000.0), 2);
    aPT->Sens2 = 0.625 * pow((aPT->Temperature - 2000.0), 2);
  }
  else
  {
    aPT->T2 = (aPT->TempDifference * aPT->TempDifference) * 7 / pow(2, 37);
    aPT->Off2 = 0.0625 * pow((aPT->Temperature - 2000.0), 2); 
    aPT->Sens2 = 0.0;
  }
  
  // Check print the offsets
  
  Serial.println("Second-order offsets are:");
  Serial.println(aPT->T2);
  Serial.println(aPT->Off2);
  Serial.println(aPT->Sens2);
  
  
  // Print the temperature results
  
  aPT->Temperature = aPT->Temperature / 100;  // Convert to degrees C
  Serial.print("First-Order Temperature in Degrees C is ");
  Serial.println(aPT->Temperature);
  Serial.print("Second-Order Temperature in Degrees C is ");
  Serial.println(aPT->Temperature - (aPT->T2 / 100));
}

void get_Pressure(struct ms5803PTStruct *aPT){
  /* Assumes that get_Temperature() has been called recently
  */
   byte ByteHigh, ByteMiddle, ByteLow;  // Variables for I2C reads
  // Read the Device for the ADC Temperature and Pressure values
  sendWireCommand(aPT->D1_512,aPT->DevAddress);
  delay(10);
  sendWireCommand(aPT->AdcRead,aPT->DevAddress);
  Wire.requestFrom(aPT->DevAddress, 3);
  while(Wire.available())
  {
    ByteHigh = Wire.read();
    ByteMiddle = Wire.read();
    ByteLow = Wire.read();
  }
  aPT->AdcPressure = ((long)ByteHigh << 16) + ((long)ByteMiddle << 8) + (long)ByteLow;
//  Serial.print("D1 is: ");
//  Serial.println(aPT->AdcPressure);
  
  // Calculate the pressure parameters
  aPT->Offset = (float)aPT->CalConstant[2] * pow(2,16);
  aPT->Offset = aPT->Offset + ((float)aPT->CalConstant[4] * aPT->TempDifference / pow(2, 7));

  aPT->Sensitivity = (float)aPT->CalConstant[1] * pow(2, 15);
  aPT->Sensitivity = aPT->Sensitivity + ((float)aPT->CalConstant[3] * aPT->TempDifference / pow(2, 8));
  
  // Add second-order corrections
  aPT->Offset = aPT->Offset - aPT->Off2;
  aPT->Sensitivity = aPT->Sensitivity - aPT->Sens2;
  
  // Calculate absolute pressure in bars
  aPT->Pressure = (float)aPT->AdcPressure * aPT->Sensitivity / pow(2, 21);
  aPT->Pressure = aPT->Pressure - aPT->Offset;
  aPT->Pressure = aPT->Pressure / pow(2, 15);
  aPT->Pressure = aPT->Pressure / 10000;  // Set output to bars;
  
  // Convert to psig and display
  aPT->Pressure = aPT->Pressure - 1.015;  // Convert to gauge pressure (subtract atmospheric pressure)
  aPT->Pressure = aPT->Pressure * 14.50377;  // Convert bars to psi
  Serial.print("Pressure in psi is: ");
  Serial.println(aPT->Pressure);
  Serial.println();
}

void sendWireCommand(byte command, int DevAddress){
  Wire.beginTransmission(DevAddress);
  Wire.write(command);
  Wire.endTransmission();
}

void Compass_Heading(struct mpu9150MEMSStruct *aMEMS){
  float cos_roll  = cos(aMEMS->roll);
  float sin_roll  = sin(aMEMS->roll);
  float cos_pitch = cos(aMEMS->pitch);
  float sin_pitch = sin(aMEMS->pitch);
  
  // adjust for LSM303 compass axis offsets/sensitivity differences by scaling to +/-0.5 range
  //                    int                 int                   int 
  float c_magnetom_x = (aMEMS->magnetom_x - SENSOR_SIGN[6]*M_X_MIN) / (M_X_MAX - M_X_MIN) - SENSOR_SIGN[6]*0.5;
  float c_magnetom_y = (aMEMS->magnetom_y - SENSOR_SIGN[7]*M_Y_MIN) / (M_Y_MAX - M_Y_MIN) - SENSOR_SIGN[7]*0.5;
  float c_magnetom_z = (aMEMS->magnetom_z - SENSOR_SIGN[8]*M_Z_MIN) / (M_Z_MAX - M_Z_MIN) - SENSOR_SIGN[8]*0.5;
  
  // Tilt compensated Magnetic filed X:
  float MAG_X = (c_magnetom_x*cos_pitch) + (c_magnetom_y*sin_roll*sin_pitch) + (c_magnetom_z*cos_roll*sin_pitch);
  // Tilt compensated Magnetic filed Y:
  float MAG_Y = c_magnetom_y*cos_roll-c_magnetom_z*sin_roll;
  // Magnetic Heading
  aMEMS->MAG_Heading = atan2(-MAG_Y,MAG_X);
}

String gen_OpenROV_log(int fresh_time,struct ms5803PTStruct *aPT, struct mpu9150MEMSStruct *aMEMS){
  /* Returns a comma delimited string
     If the instrments' sample_time is within fresh_time of now, values are inserted in the log_string
  */
  String log_string;
  if ( recentSample(aPT->sample_time,fresh_time)) {
    log_string += aPT->Pressure;            log_string += ","; 
    log_string += aPT->Temperature;         log_string += ","; 
  }
  else log_string += " , ,";
  if ( recentSample(aMEMS->sample_time,fresh_time)) { //gyro_x,gyro_y,gyro_z,MAG_Heading,
    log_string += aMEMS->gyro_x;            log_string += ","; 
    log_string += aMEMS->gyro_y;            log_string += ","; 
    log_string += aMEMS->gyro_z;            log_string += ","; 
    log_string += aMEMS->MAG_Heading;            log_string += ","; 
  }
  else log_string += " , , , ,";
  return log_string;
}