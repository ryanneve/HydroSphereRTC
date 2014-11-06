/*
Main script, includes scheduler
Desired schedule:
1)  10  seconds: Compass, acceleration, orientation
2)  60  seconds: Light and depth
3)  300 seconds: DO, temperature, conductivity.
*/
#include "HydroSphereRTC.h" // DO and Conductivity sensors
#include <SPI.h>         // For SD card and clock
#include <SdFat.h>          // SPI SD card
#include <RTClib.h>      // RTC Clock
#include <Logger_SD.h>   // Logging
#include <Wire.h>        // for I2C
#include <I2Cdev.h>      // For HS_OpenROV libraries
#include <AK8975.h>      // For HS_OpenROV magnetometer
#include <MPU6050.h>     // For HS_OpenROV accel and gyro
#include <MS5803_I2C.h>  // fot HS_OpenRov pressure and temperature
#include <DS3234.h>      // SPI RTC Clock
#include "HS_Atlas.h"    // DO & COND
#include <Atlas_RGB.h>   // RGB sensor
#include "HS_OpenROV.h"  // Pressure & accelerometer
//#include <MemoryFree.h>  // Tool for monitoring SRAM usage

// Solenoid will trigger at or below this voltage.
// For 9.9v LiFe use 8.0
// For 8x1.5v (12v) alkaline use 9.2v
// For 2x4x1.5v (6v) alkaline use 4.6v
#define TRIGGER_VOLTAGE  4.6 
#define MIN_TRIGGER_VOLTAGE 4.4 // Don't trigger below this. This prevents triggering when on 5v USB
uint8_t SCHED_RATE1 = 15; // Must be lowest
uint8_t SCHED_RATE2 = 30; // Must be multiple of SCHED_RATE1
uint8_t SCHED_RATE3 = 60;// Must be multiple of SCHED_RATE1
uint8_t EXT_MS5803_MAX_BAR = 5;  // OpenROV uses MS5803-14. We will be using -05 or -01
uint8_t EXT_MS5803_ADDR = 0x76;  // i2c address
uint8_t INT_MS5803_MAX_BAR = 5;  // 
uint8_t INT_MS5803_ADDR = 0x77;  // i2c address

// Trigger date/time. This should come from EEPROM eventually
uint8_t TRIG_DAY = 11;
uint8_t TRIG_HOUR = 16;
uint8_t TRIG_MINUTE = 30;
uint16_t SOL_DURATION = 1000; // One second
uint16_t DEPLOY_DELAY = 0;  // Delay before logging/triggering in minutes
bool HAS_PT_SENSOR = 1;
uint16_t MPU_VERSION = 6050;  // Either 0, 6050 or 9050

//----------------------NOTHING TO CONFIGURE BELOW HERE----------------------------------------


/*----------( Forward Declarations)----------*/
//HydroSphereUtil.cpp
float getDensity(float temp_C, float salinity);
float getDepth(float pressure_kpa, float density_kgm3);
uint16_t freeRam();
void lowPowerDelay(uint16_t seconds);
void deepSleep(uint16_t seconds);
float get_analog_temperature(uint8_t analog_pin);

/*----------( Define Global Objects)----------*/
RTC_DS3234 RTClock(CLOCK_SS_PIN);
struct DO_Struct   sensor_DO;
struct CondStruct  sensor_COND;
struct GyroStruct  sensor_GYRO;
AtlasRGB sensor_RGB(&SERIAL_RGB);
MS5803 presstemp(EXT_MS5803_ADDR);
DateTime t; // Structure to hold a DateTime

/*----------( Define Global Variables)----------*/
char g_battery_voltage[7] = "??????";
const float VOLTAGE_MULTIPLIER = 2.0; // multiply by this to get true voltage. 
bool on_batt = false; // Keep track if we've been on battery for solenoid trigger.
bool do_sched1,do_sched2,do_sched3 = true;
const uint8_t g_LOG_SIZE = 160;
uint32_t g_Sched1, g_Sched2, g_Sched3, g_SchedNext,g_now,g_delay_start;
uint32_t g_millisDelta = millis();
bool g_sol_triggered = false;
const bool LOG_DEBUG = false;
const char LOG_FILE[] = "HS_LOG.CSV";
const char SCHED1_FILE[] = "SCHED1.CSV";
const char SCHED2_FILE[] = "SCHED2.CSV";
const char SCHED3_FILE[] = "SCHED3.CSV";
const char CONFIG_FILE[] = "config.txt";
const char SCHED1_HEADER[] = "Sample Time,GyX,GyY,GyZ,AccX,AccY,AccZ,MagX,MagY,MagZ,Head,VBatt\r\n";
const char SCHED2_HEADER[] = "Sample Time,ExtTemp,D1temp,Press_mBar,D2press,red,grn,blue,lux_r,lux_g,lux_b,lux_tot,lux_beyond\r\n";
const char SCHED3_HEADER[] = "Sample Time,DO%,DO_mgl,EC,TDS,Sal,SG\r\n";

void setup() {
  Serial.begin(9600);
  for (uint8_t i = 0; i < 80 ; i++) Serial.print('-');
  Serial.println();
  pinMode(SOLENOID_PIN,OUTPUT);
  digitalWrite(SOLENOID_PIN,LOW);
  pinMode(OFF_PIN,OUTPUT);
  digitalWrite(OFF_PIN,LOW); // may be high?
  pinMode(LEVEL_SHIFT_POWER_PIN,OUTPUT);
  digitalWrite(LEVEL_SHIFT_POWER_PIN,HIGH); // Turn on level shifter. Needed for PT & gyro
  Wire.begin();
  bool has_SD = Logger_SD::Instance()->initializeSD(SD_CHIP_SELECT,CLOCK_SS_PIN);
  Logger_SD::Instance()->initializeLog(LOG_FILE,LOG_DEBUG);
  Logger_SD::Instance()->setSampleFile(SCHED1_FILE);
  Logger_SD::Instance()->initializeSample(SCHED1_HEADER);
  Logger_SD::Instance()->setSampleFile(SCHED2_FILE);
  Logger_SD::Instance()->initializeSample(SCHED2_HEADER);
  Logger_SD::Instance()->setSampleFile(SCHED3_FILE);
  Logger_SD::Instance()->initializeSample(SCHED3_HEADER);
  Logger_SD::Instance()->initializeConfig(CONFIG_FILE);
  if ( has_SD ) loadConfigValues(); // Loads values from SD
  initRTC();
  t = RTClock.now();
  g_delay_start = t.unixtime() + (DEPLOY_DELAY * 60);
  g_Sched1 = t.unixtime();
  g_Sched2 = t.unixtime();
  g_Sched3 = t.unixtime();
  g_SchedNext = g_Sched1;
  RTClock.setA2Time(TRIG_DAY, TRIG_HOUR, TRIG_MINUTE,0, 0, 0, 0); // Set Solenoid Alarm
  RTClock.turnOnAlarm(2);
  Logger_SD::Instance()->setLogDT(&t);
  initDO();
  initCond(&sensor_COND);
  sensor_RGB.initialize(3,true); // mode 2, quiet = false (continuous mode)
  presstemp.setAddress(EXT_MS5803_ADDR);
  presstemp.initialize(EXT_MS5803_MAX_BAR);
  bool result = presstemp.testConnection();
  if ( result ) Logger_SD::Instance()->msgL(INFO,F("MS5803 connection successful"));
  else {
	  uint8_t dev_addr = presstemp.getAddress();
	  Logger_SD::Instance()->msgL(WARN,F("MS5803 connection failed with address %#Xh"),dev_addr);
  }
  presstemp.debugCalConstants();
  // Record pressure and temperature calibration constants
  for ( uint8_t c = 1 ; c <= 6 ; c++ ) {
    Logger_SD::Instance()->msgL(INFO,F("MS5803-%02i Cal Constant %u = %li"),EXT_MS5803_MAX_BAR, c,presstemp.getCalConstant(c));
  }
  if (MPU_VERSION != 0) setupAccelGyro(); // gyrocompass  
  // Some debug info
  Logger_SD::Instance()->msgL(INFO,F("Schedule rates set to %d, %d & %d."),SCHED_RATE1,SCHED_RATE2,SCHED_RATE3);
  Logger_SD::Instance()->msgL(INFO,F("Trigger date set to day %d at %d:%02d"),TRIG_DAY,TRIG_HOUR,TRIG_MINUTE);
  Logger_SD::Instance()->msgL(INFO,F("Trigger duration set to %d ms"),SOL_DURATION);
  // First check if is's time to do schedule
  t = RTClock.now();
  if ( g_delay_start > t.unixtime() ) {
    Logger_SD::Instance()->msgL(INFO,F("Delaying instrument start %d minutes"), DEPLOY_DELAY);
    Serial.flush();
    delay(100);
    lowPowerDelay((uint16_t)(g_delay_start - t.unixtime()));
  }
}

void loop() {
  /**/
  t = RTClock.now();
  g_now = t.unixtime();
  Serial.print(g_now ); Serial.print(" <-> "); Serial.println(g_SchedNext);
  if ( !g_sol_triggered ) checkSolenoid(); // triggered only once;
  if ( g_now >= g_Sched1 ) do_sched1 = true;
  if ( g_now >= g_Sched2 ) do_sched2 = true;
  if ( g_now >= g_Sched3 ) do_sched3 = true;
  // Now see if we need to run a schedule
  if ( do_sched1 || do_sched2 || do_sched3 ) {
    // Set time for next occurance.
    if ( do_sched1 ) g_Sched1 = t.unixtime() + SCHED_RATE1;
    if ( do_sched2 ) g_Sched2 = t.unixtime() + SCHED_RATE2;
    if ( do_sched3 ) g_Sched3 = t.unixtime() + SCHED_RATE3;
    if ( do_sched1 ) {
      do_sched1 = false;
      sched1();
    }
    if ( do_sched2 ) {
      do_sched2 = false;
      sched2();
    }
    if ( do_sched3 ) {
      do_sched3 = false;
      sched3();
    }
    t = RTClock.now();
    Logger_SD::Instance()->msgL(DEBUG,F("Done with schedules."));
    g_SchedNext = min(g_Sched1,min(g_Sched2,g_Sched3));
    int32_t lp_delay = (int32_t)g_SchedNext - (int32_t)t.unixtime() - 1; // Sleep for one sec less than we need to
    if ( lp_delay > 0 ) {
      DateTime wake (g_SchedNext); uint8_t buf_len = 20 ; char buf[buf_len];
      Logger_SD::Instance()->msgL(DEBUG,F("Sleeping %d seconds until %s."),lp_delay,wake.toString(buf,buf_len));
      Serial.flush();
      delay(100);
      lowPowerDelay((uint16_t)lp_delay);
    }
    else Logger_SD::Instance()->msgL(DEBUG,F("NOT Sleeping %d seconds."),lp_delay);
  }
  else delay(900);
}

void sched1() {
  t = RTClock.now(); // Update for logger
  Logger_SD::Instance()->msgL(DEBUG,F("---------- Sched1 entered"));
  char head[10];
  if (MPU_VERSION != 0)
  {
	getAccelGyro(&sensor_GYRO);  // update data from accelerometer and gyro
	if (MPU_VERSION == 9050) getMagnetometer(&sensor_GYRO); //update data from magnetometer
  }

  // Now log
  Logger_SD::Instance()->setSampleFile(SCHED1_FILE);
  if (MPU_VERSION != 0) {
	reportGyro(&sensor_GYRO);
	reportAccel(&sensor_GYRO);
	if (MPU_VERSION == 9050) {
		reportMagnetometer(&sensor_GYRO);
		dtostrf(sensor_GYRO.heading,5,1,head); head[7] = '\0';
	}
  }
  char log_output[200]; uint8_t log_idx = 0;
  uint8_t buf_len = 20;
  char buf[buf_len];
  t = RTClock.now();
  log_idx +=sprintf(log_output + log_idx,"%s,",t.toYMDString(buf,buf_len));
  if (MPU_VERSION != 0) {
	  log_idx +=sprintf(log_output + log_idx,"%d,",sensor_GYRO.gyro_x);
	  log_idx +=sprintf(log_output + log_idx,"%d,",sensor_GYRO.gyro_y);
	  log_idx +=sprintf(log_output + log_idx,"%d,",sensor_GYRO.gyro_z);
	  log_idx +=sprintf(log_output + log_idx,"%d,",sensor_GYRO.accel_x);
	  log_idx +=sprintf(log_output + log_idx,"%d,",sensor_GYRO.accel_y);
	  log_idx +=sprintf(log_output + log_idx,"%d,",sensor_GYRO.accel_z);
  }
  else {
	  log_idx +=sprintf(log_output + log_idx,"0000,"); // gyro x is zero
	  log_idx +=sprintf(log_output + log_idx,"0000,"); // gyro y is zero
	  log_idx +=sprintf(log_output + log_idx,"0000,"); // gyro z is zero
	  log_idx +=sprintf(log_output + log_idx,"0000,"); // sensor x is zero
	  log_idx +=sprintf(log_output + log_idx,"0000,"); // sensor y is zero
	  log_idx +=sprintf(log_output + log_idx,"0000,"); // sensor z is zero
  }
  if (MPU_VERSION == 9050) {
	  log_idx +=sprintf(log_output + log_idx,"%d,",sensor_GYRO.magnetom_x);
	  log_idx +=sprintf(log_output + log_idx,"%d,",sensor_GYRO.magnetom_y);
	  log_idx +=sprintf(log_output + log_idx,"%d,",sensor_GYRO.magnetom_z);
	  log_idx +=sprintf(log_output + log_idx,"%s,",head);
  }
  else {
	  log_idx +=sprintf(log_output + log_idx,"0000,"); // magnetom_x is zero
	  log_idx +=sprintf(log_output + log_idx,"0000,"); // magnetom_y is zero
	  log_idx +=sprintf(log_output + log_idx,"0000,"); // magnetom_z is zero
	  log_idx +=sprintf(log_output + log_idx,"0000,"); // head is zero
  }
  log_idx +=sprintf(log_output + log_idx,"%s" ,g_battery_voltage);
  Logger_SD::Instance()->msgL(DEBUG,"sched1 Logging %s of size %d",log_output,log_idx);
  Logger_SD::Instance()->saveSample(log_output,log_idx);
}

void sched2() {
  t = RTClock.now();// Update for logger
  Logger_SD::Instance()->msgL(DEBUG,F("---------- Sched2 entered"));
  if ( not sensor_RGB.getContinuous() ) sensor_RGB.getRGB(); // Get some new values
  if ( HAS_PT_SENSOR ) {
      presstemp.getPressure(true);
      presstemp.getTemperature(true);
  }
  // Log readings
  Logger_SD::Instance()->setSampleFile(SCHED2_FILE);
  char log_output[200]; uint8_t log_idx = 0;
  uint8_t buf_len = 20;
  char buf[buf_len];
  t = RTClock.now();// Update for logger
  log_idx +=sprintf(log_output + log_idx,"%s,",t.toYMDString(buf,buf_len));
  if ( HAS_PT_SENSOR ) {
      dtostrf(presstemp.temp_C,6,3,buf);
      log_idx +=sprintf(log_output + log_idx,"%s,",buf);
      itoa(presstemp.getD1Pressure(),buf,10);
      log_idx +=sprintf(log_output + log_idx,"%s,",buf);
      dtostrf(presstemp.press_mBar,6,3,buf);
      log_idx +=sprintf(log_output + log_idx,"%s,",buf);
      itoa(presstemp.getD2Temperature(),buf,10);
      log_idx +=sprintf(log_output + log_idx,"%s,",buf);
  }
  else {
	  dtostrf(get_analog_temperature(TEMPERATURE_PIN),6,3,buf);
	  log_idx +=sprintf(log_output + log_idx,"%s,",buf);
	  log_idx +=sprintf(log_output + log_idx,"0000,"); // D1temp is zero
	  log_idx +=sprintf(log_output + log_idx,"0000,"); // pressure is zero
	  log_idx +=sprintf(log_output + log_idx,"0000,"); // D2press is zero
  }
  log_idx +=sprintf(log_output + log_idx,"%d,",sensor_RGB.getRed());
  log_idx +=sprintf(log_output + log_idx,"%d,",sensor_RGB.getGreen());
  log_idx +=sprintf(log_output + log_idx,"%d,",sensor_RGB.getBlue());
  log_idx +=sprintf(log_output + log_idx,"%d,",sensor_RGB.getLxRed());
  log_idx +=sprintf(log_output + log_idx,"%d,",sensor_RGB.getLxGreen());
  log_idx +=sprintf(log_output + log_idx,"%d,",sensor_RGB.getLxBlue());
  log_idx +=sprintf(log_output + log_idx,"%d,",sensor_RGB.getLxTotal());
  log_idx +=sprintf(log_output + log_idx,"%d",sensor_RGB.getLxBeyond());
  Logger_SD::Instance()->msgL(DEBUG,F("sched2 Logging %s of size %d"),log_output,log_idx);
  Logger_SD::Instance()->saveSample(log_output,log_idx);
}

void sched3() {
  t = RTClock.now();
  Logger_SD::Instance()->msgL(DEBUG,F("---------- Sched3 entered"));
  float ext_temperature;
  if ( HAS_PT_SENSOR) {
    ext_temperature = presstemp.getTemperature();
    Serial.print("Digital temperature is "); Serial.println(ext_temperature/100);
  }
  else {
    ext_temperature = get_analog_temperature(TEMPERATURE_PIN);
    Serial.print("Analog temperature is "); Serial.println(ext_temperature);
  }
  //setCondTemp(ext_temperature); // Causing problems
  getCond(&sensor_COND);// conductivity.
  getCompDO(&sensor_DO,&sensor_COND,ext_temperature);
  Logger_SD::Instance()->setSampleFile(SCHED3_FILE);
  char log_output[200]; uint8_t log_idx = 0;
  uint8_t buf_len = 20;
  char buf[buf_len];
  t = RTClock.now();
  log_idx +=sprintf(log_output + log_idx,"%s,",t.toYMDString(buf,buf_len));
  log_idx +=sprintf(log_output + log_idx,"%d,",sensor_DO._sat);
  dtostrf(sensor_DO._dox,6,2,buf);
  log_idx +=sprintf(log_output + log_idx,"%s,",buf);
  dtostrf(sensor_COND._ec,7,2,buf);
  log_idx +=sprintf(log_output + log_idx,"%s,",buf);
  dtostrf(sensor_COND._tds,7,2,buf);
  log_idx +=sprintf(log_output + log_idx,"%s,",buf);
  dtostrf(sensor_COND._sal,7,2,buf);
  log_idx +=sprintf(log_output + log_idx,"%s,",buf);
  dtostrf(sensor_COND._sg,7,2,buf);
  log_idx +=sprintf(log_output + log_idx,"%s",buf);
  Logger_SD::Instance()->msgL(DEBUG,F("sched3 Logging %s of size %u"),log_output,log_idx);
  Logger_SD::Instance()->saveSample(log_output,log_idx);
  //float press_mBar = presstemp.press_mBar;
  //float density_kgm3 = getDensity(presstemp.temp_C,atof(sensor_COND.sal));
  //float depth_m = getDepth(presstemp.press_kPa, density_kgm3);
  //Serial.print("density is "); Serial.println(depth_m);
  //Serial.print("depth is "); Serial.println(depth_m);
}

bool checkSolenoid(){
  /* Check solenoid trigger conditions */
  bool trigger = false;
  int16_t volt_pin_val = analogRead(BATT_VOLTAGE_PIN);
  float battery_voltage = ((float)volt_pin_val/1024) * 5.00 * VOLTAGE_MULTIPLIER; 
  if ( battery_voltage > 5.0 ) on_batt = true;
  dtostrf(battery_voltage,5,3,g_battery_voltage); g_battery_voltage[6] = 0;
  char batt_min[7]; dtostrf(TRIGGER_VOLTAGE,5,3,batt_min); batt_min[6] = 0;
  if (battery_voltage < MIN_TRIGGER_VOLTAGE ) Logger_SD::Instance()->msgL(DEBUG,F("Battery voltage %s. on USB power"),g_battery_voltage); 
  else Logger_SD::Instance()->msgL(DEBUG,F("Battery voltage %s. Min is %s (%d)."),g_battery_voltage,batt_min,volt_pin_val); 
  if ( g_sol_triggered == false ) { //&& on_batt == true ) { // only once and must have seen a battery
    if ( battery_voltage <= TRIGGER_VOLTAGE && battery_voltage > MIN_TRIGGER_VOLTAGE) {
      Logger_SD::Instance()->msgL(CRITICAL,F("Low voltage %s <= %s triggering Solenoid"),g_battery_voltage,batt_min);
      trigger = true;
    }
    // Now check time.
    t = RTClock.now();
    if ( TRIG_DAY == t.day() ) { // Date matches
      Serial.println("DAY matches");
      Logger_SD::Instance()->msgL(DEBUG,F("Looking for %d:%d >= %d:%d"),t.hour(),t.minute(), TRIG_HOUR,TRIG_MINUTE);
      if ( (TRIG_HOUR < t.hour()) || ( TRIG_HOUR == t.hour() && TRIG_MINUTE <= t.minute() )) { // Hour matches or has passed. Minute is now or has passed.
        uint8_t time_str_len = 20;
        char time_str[time_str_len];
        t = RTClock.now();
        Logger_SD::Instance()->msgL(WARN,F("Current time %s >= trigger time %d-%d:%d"),t.toYMDString(time_str,time_str_len),TRIG_DAY,TRIG_HOUR,TRIG_MINUTE);
        trigger = true;
      }
      Serial.println("Didn't find it");
    }
    // Check RTC
    if ( RTClock.checkIfAlarm(2) ) {
        t = RTClock.now();
        Logger_SD::Instance()->msgL(WARN,F("RTC Alarm 2 indicated"));
        trigger = true;
    }
  }
  if ( trigger ) trigSolenoid(SOL_DURATION);
  return trigger;
}

void trigSolenoid(int16_t open_millis) {
  /* Trigger solenoid for open_millis ms */
  unsigned long close_time = millis() + open_millis;
  Logger_SD::Instance()->msgL(WARN,F("Opening ballast solenoid")); // at %02d:%02d:%02d",Rtc.getHours24(),Rtc.getMinutes(),Rtc.getSeconds());
  digitalWrite(SOLENOID_PIN,HIGH);
  g_sol_triggered = true; // So we only do this once.
  while ( millis() < close_time ) {} // wait
  digitalWrite(SOLENOID_PIN,LOW);
  Logger_SD::Instance()->msgL(WARN,F("Closing ballast solenoid")); // at %02d:%02d:%02d",Rtc.getHours24(),Rtc.getMinutes(),Rtc.getSeconds());
}


// Functions related to date and time

void initRTC(){
  Logger_SD::Instance()->msgL(INFO,F("initRTC(), testing RTC connection..."));
  RTClock.begin();
  // verify connection
  t = RTClock.now();
  if ( (t.year() > 2050) || (t.year() < 2014) ) {
    Logger_SD::Instance()->msgL(ERROR,F("Date %d/%d/%d out of range"),
               t.year(), t.month(), t.day());
  }
  Logger_SD::Instance()->msgL(INFO,F("Clock reports %d/%02d/%02d at %d:%02d:%02d"),
               t.year(), t.month(), t.day(),t.hour(), t.minute(), t.second());
}

void setupAlarm(){
}
void loadConfigValues(){
	// Load config defaults
	int32_t config_value;
	Logger_SD::Instance()->setDebug(false);
	// Scheduling
	config_value = Logger_SD::Instance()->getConfig((char *)"SCHED_RATE1");
	if ( config_value != -1 ) SCHED_RATE1 = (uint8_t) config_value;
	config_value  = Logger_SD::Instance()->getConfig((char *)"SCHED_RATE2");
	if ( config_value != -1 ) SCHED_RATE2 = (uint8_t) config_value;
	config_value  = Logger_SD::Instance()->getConfig((char *)"SCHED_RATE3");
	if ( config_value != -1 ) SCHED_RATE3 = (uint8_t) config_value;
	config_value     = Logger_SD::Instance()->getConfig((char *)"TRIG_DAY");
	if ( config_value != -1 ) TRIG_DAY = (uint8_t) config_value;
	config_value    = Logger_SD::Instance()->getConfig((char *)"TRIG_HOUR");
	if ( config_value != -1 ) TRIG_HOUR = (uint8_t) config_value;
	config_value  = Logger_SD::Instance()->getConfig((char *)"TRIG_MINUTE");
	if ( config_value != -1 ) TRIG_MINUTE = (uint8_t) config_value;
	config_value = Logger_SD::Instance()->getConfig((char *)"SOL_DURATION");
	if ( config_value != -1 ) SOL_DURATION = (uint16_t) config_value;
	config_value = Logger_SD::Instance()->getConfig((char *)"DEPLOY_DELAY");
	if ( config_value != -1 ) DEPLOY_DELAY = (uint16_t) config_value;
	// Pressure/Temperature
	config_value = Logger_SD::Instance()->getConfig((char *)"HAS_PT_SENSOR");
	if ( config_value != -1 ) HAS_PT_SENSOR = (bool) config_value;
	config_value = Logger_SD::Instance()->getConfig((char *)"EXT_MS5803_MAX_BAR");
	if ( config_value != -1 ) EXT_MS5803_MAX_BAR = (uint8_t) config_value;
	config_value = Logger_SD::Instance()->getConfig((char *)"EXT_MS5803_ADDR");
	if ( config_value != -1 ) EXT_MS5803_ADDR = (uint8_t) config_value;
	// Accelerometer
	config_value = Logger_SD::Instance()->getConfig((char *)"MPU_VERSION");
	if ( config_value != -1 ) MPU_VERSION = (uint16_t) config_value;
	Logger_SD::Instance()->setDebug(false);
}

