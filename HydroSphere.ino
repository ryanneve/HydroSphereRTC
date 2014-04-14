/*

Main script, includes scheduler
Desired schedule:
1)  10  seconds: Compass, acceleration, orientation
2)  60  seconds: Light and depth
3)  300 seconds: DO, temperature, conductivity.
*/
#include "HydroSphere.h" // DO and Conductivity sensors
#include <SPI.h> // For SD card
#include <SD.h>
#include <Logger_SD.h>
//#include <L3G.h>  // For HS_OpenROV
//#include <LSM303.h> // For HS_OpenROV
#include <Wire.h> // for I2C
#include <I2Cdev.h>      // For HS_OpenROV libraries
#include <AK8975.h>      // For HS_OpenROV magnetometer
#include <MPU6050.h>     // For HS_OpenROV accel and gyro
#include <MS5803_I2C.h>  // fot HS_OpenRov pressure and temperature
//#include <DS1307.h>      // for RTC
//#include "RTClib.h" // Clock
#include "HS_Atlas.h"
#include <Atlas_RGB.h>
#include "HS_OpenROV.h"


/*----------( Define Global Objects)----------*/
struct DO_Struct   sensor_DO;
struct CondStruct  sensor_COND;
//struct PTStruct    sensor_PT;
struct GyroStruct  sensor_GYRO;
AtlasRGB sensor_RGB(&SERIAL_RGB);
/* 
DS1307 Rtc;   // Instantiate Real Time Clock
DateTime now;
DateTime g_Sched1;
DateTime g_Sched2;
DateTime g_Sched3;
*/
unsigned long g_Sched1;
unsigned long g_Sched2;
unsigned long g_Sched3;

/*----------( Define Global Variables)----------*/
bool do_sched1,do_sched2,do_sched3 = true;
const uint8_t g_LOG_SIZE = 160;

const uint16_t SCHED_RATE1 = 10; // Must be lowest
const uint16_t SCHED_RATE2 = 20; // Must be multiple of SCHED_RATE1
const uint16_t SCHED_RATE3 = 30;// Must be multiple of SCHED_RATE1
unsigned long next_check = millis(); // Used for sleeping between checks.
// Trigger date/time. THis should come from EEPROM eventually

//const uint8_t TRIG_DAY = 11;
//const uint8_t TRIG_HOUR = 16;
//const uint8_t TRIG_MINUTE = 30;
#define HOUR_DELAY 6
#define MINUTE_DELAY 30
const unsigned long SOL_DELAY_MS = (( HOUR_DELAY * 36000) + (MINUTE_DELAY * 60 ))  * 1000;//
const uint16_t SOL_DURATION = 1000; // One second
bool g_sol_triggered = false;
const char LOG_FILE[] = "HYDROSPH.CSV";

void setup() {
  Serial.begin(9600);
  pinMode(SOLENOID_PIN,OUTPUT);
  digitalWrite(SOLENOID_PIN,LOW);
  pinMode(OFF_PIN,OUTPUT);
  digitalWrite(OFF_PIN,LOW); // may be high?
  pinMode(LEVEL_SHIFT_POWER_PIN,OUTPUT);
  digitalWrite(LEVEL_SHIFT_POWER_PIN,HIGH); // Turn on level shifter. Needed for PT & gyro
  Wire.begin();
  //initRTC();
  Logger_SD::Instance()->initializeSD(SD_CHIP_SELECT);
  Logger_SD::Instance()->initializeLog(LOG_FILE);
  //initDO();
  initCond(&sensor_COND);
  sensor_RGB.initialize();
  setupPT(); // Pressure, Temperature 
  setupAccelGyro(); // gyrocompass
  /*
  if ( true ) {
    Logger_SD::Instance()->msgL(DEBUG,"Synchronizing with clock.");
    while ( Rtc.getSeconds() != 0 ) {
      if ( millis() % 1000 == 0 ) { // check once per second
        now = Rtc.getDateTime();
        Serial.print(now.second(),DEC);Serial.print(' ');
      } // Sync with clock.
    }
  }
  */
  g_Sched1 = millis(); //Rtc.getDateTime();
  g_Sched2 = millis(); //Rtc.getDateTime();
  g_Sched3 = millis(); //Rtc.getDateTime();
}

void loop() {
  /**/
  //now = Rtc.getDateTime();
  // First check if is's time to do schedule
  if ( millis() >= g_Sched1 ) do_sched1 = true;
  if ( millis() >= g_Sched2 ) do_sched2 = true;
  if ( millis() >= g_Sched3 ) do_sched3 = true;
  //if ( checkSolenoidTime(TRIG_DAY,TRIG_HOUR,TRIG_MINUTE) && !g_sol_triggered) trigSolenoid(SOL_DURATION); // only once;
  if ( millis() > SOL_DELAY_MS && !g_sol_triggered) trigSolenoid(SOL_DURATION); // only once;
  // Now see if we need to update our values
  if ( do_sched1 || do_sched2 || do_sched3 ) {
    next_check = millis() + (1000 * SCHED_RATE1);
    if ( do_sched1 ) {
      //  UPDATE SAMPLES
      sched1();
      g_Sched1 += (SCHED_RATE1 * 1000); do_sched1 = false;
    }
    if ( do_sched2 ) {
      //  UPDATE SAMPLES
      sched2();
      g_Sched2 += (SCHED_RATE2 * 1000); do_sched2 = false;
    }
    if ( do_sched3 ) {
      //  UPDATE SAMPLES
      sched3();
      g_Sched3 += (SCHED_RATE3 * 1000); do_sched3 = false;
    }
    // All updated, now log
    saveLog(); // May want to pass structs later rather than use globals.
  }
  while  ( millis() < next_check ) {
     if ( millis() % 1000 == 0 ) {
       //Serial.print(next_check - millis()); Serial.print(" "); 
       delay(1); // So we don't get 20 '.'
     }
    // save power here.
  }
}
/*
DateTime addSeconds(DateTime dt,uint8_t sec) {
  long sec_dt = dt.unixtime();
  sec_dt += sec;
  return DateTime(sec_dt);
}

bool checkSolenoidTime(const uint8_t trig_day,const uint8_t trig_hour,const uint8_t trig_minute){
  now = Rtc.getDateTime();
  if ( (now.day() == trig_day) && (now.hour() == trig_hour) && (now.minute() >= trig_minute) ) return true;
  else return false;
}
*/

void saveLog() { // Pass all the structs?
  char g_log_arr[g_LOG_SIZE]; // NEED TO GET RID OF BUGGY STRING OBJECT.
  uint8_t g_log_index = 0;
}

void sched1() {
  Logger_SD::Instance()->msgL(DEBUG,"sched1");
  getAccelGyro(&sensor_GYRO);  // update data from accelerometer and gyro
  getMagnetometer(&sensor_GYRO); //update data from magnetometer
  // Now log
  reportAccel(&sensor_GYRO);
  reportGyro(&sensor_GYRO);
  reportMagnetometer(&sensor_GYRO);
}

void sched2() {
  Logger_SD::Instance()->msgL(DEBUG,"sched2");
  sensor_RGB.getRGB(); // Light
  sensor_RGB.printRGB();
  
  int32_t temp = getTemperature();
  Serial.print("Temperature: ");
  Serial.println(temp);// depth
  //  debugTemperature();
  // Log readings
}

void sched3() {
  Logger_SD::Instance()->msgL(DEBUG,"sched3");
  getCompCond(&sensor_COND);// conductivity.
  Serial.print("TDS = ");
  Serial.println(sensor_COND.tds);
  Serial.print("COND = ");
  Serial.println(sensor_COND.ec);
  Serial.print("SAL = ");
  Serial.println(sensor_COND.sal);
  //Serial.print("Pressure: ");
  //Serial.println(getPressure());// depth
  //getCompDO(&sensor_DO,&sensor_COND);
  //Serial.print("DO = ");
  //Serial.println(sensor_DO.dox); // DO
}

// Functions related to date and time
/*
void initRTC(){
  Logger_SD::Instance()->msgL(DEBUG,"initRTC");
  Rtc.initialize();
  Rtc.setClockRunning(1); // Shouldn't be necessary once initialize() is fixed, but doesn't hurt.
  // verify connection
  Logger_SD::Instance()->msgL(DEBUG,"Testing device connections...");
  Serial.println(Rtc.testConnection() ? "DS1307 connection successful" : "DS1307 connection failed");
  setDate();
  now = Rtc.getDateTime();
  if (now.year() > 2050) {
    Logger_SD::Instance()->msgL(ERROR,F("Date %d/%d/%d out of range"),
               now.year(), now.month(), now.day());
    setDate();
  }
  Logger_SD::Instance()->msgL(DEBUG,F("Clock reports %d/%02d/%02d at %d:%02d:%02d"),
               now.year(), now.month(), now.day(),now.hour(), now.minute(), now.second());
}

// Time related functions
void getDate(char* date_str) { // assumes date_str is char[9] CCYYMMDD\0
  char temp_str[4] = "000";
  now = Rtc.getDateTime();
  sprintf(temp_str,"%u",now.year());
  memcpy(date_str,temp_str,4);
  if ( now.month() < 10 ) sprintf(temp_str,"0%u\0",now.month());
  else           sprintf(temp_str,"%u\0", now.month());
  memcpy(date_str + 4,temp_str,2);
  if ( now.day() < 10 ) sprintf(temp_str,"0%u\0",now.day());
  else           sprintf(temp_str,"%u\0", now.day());
  memcpy(date_str + 6,temp_str,2);
  date_str[8] = '\0';
}

void getTime(char* time_str)  { // Assumes time_str is char[7] HHMMSS\0
  char char_time[3];
  now = Rtc.getDateTime();
  uint8_t hr, mi, se;
  hr = now.hour();
  if ( hr < 10 ) sprintf(char_time,"0%u\0",hr);
  else           sprintf(char_time,"%u\0", hr);
  memcpy(time_str,char_time,2);
  mi = now.minute();
  if ( mi < 10 ) sprintf(char_time,"0%u\0",mi);
  else           sprintf(char_time,"%u\0", mi);
  memcpy(time_str + 2,char_time,2);
  se = now.second();
  if ( se < 10 ) sprintf(char_time,"0%u\0",se);
  else           sprintf(char_time,"%u\0", se);
  memcpy(time_str + 4,char_time,3);
}

void setDate() {
  Logger_SD::Instance()->msgL(DEBUG,F("Setting date and time to %s  %s"),__DATE__, __TIME__);
  Rtc.setDateTime(DateTime(__DATE__, __TIME__)); // Set RTC time
}


bool recentSample(char time[7], int16_t fresh_time){
  // Returns a 1 if now - time < fresh_time (in minutes)
  //
  // First convert time to an integer representing hours and minutes.
  now = Rtc.getDateTime();
  int16_t spl_time = atoi(time) / 100;
  int16_t now_time = now.minute() + (now.hour() * 100);
  if (now_time < spl_time ) now_time += 2400;
  int16_t delta = now_time - spl_time;
  if ( delta <= fresh_time && delta >= 0 ) return 1; 
  else return 0;
}
*/
uint16_t freeRam() {
  extern int16_t __heap_start, *__brkval; 
  int16_t v; 
  return (int16_t) &v - (__brkval == 0 ? (int16_t) &__heap_start : (int16_t) __brkval); 
}

void trigSolenoid(int16_t open_millis) {
  unsigned long close_time = millis() + open_millis;
  Logger_SD::Instance()->msgL(WARN,"Opening ballast solenoid"); // at %02d:%02d:%02d",Rtc.getHours24(),Rtc.getMinutes(),Rtc.getSeconds());
  digitalWrite(SOLENOID_PIN,HIGH);
  g_sol_triggered = true;
  while ( millis() < close_time ) {} // wait
  digitalWrite(SOLENOID_PIN,LOW);
  Logger_SD::Instance()->msgL(WARN,"Closing ballast solenoid");// at %02d:%02d:%02d",Rtc.getHours24(),Rtc.getMinutes(),Rtc.getSeconds());
}

/*
lpDelay(quarterSeconds) - Low Power Delay.  Drops the system clock
                          to its lowest setting and sleeps for 256*quarterSeconds milliseconds.
*/
uint16_t lpDelay(uint16_t quarterSeconds) {
  int16_t oldClkPr = CLKPR;  // save old system clock prescale
  CLKPR = 0x80;    // Tell the AtMega we want to change the system clock
  CLKPR = 0x08;    // 1/256 prescaler = 60KHz for a 16MHz crystal
  delay(quarterSeconds);  // since the clock is slowed way down, delay(n) now acts like delay(n*256)
  CLKPR = 0x80;    // Tell the AtMega we want to change the system clock
  CLKPR = oldClkPr;    // Restore old system clock prescale
}

void deepSleep(uint16_t quarterSeconds){
  /* See 
     https://www.sparkfun.com/tutorials/309 
     for much more
   */
  // Turn off all peripherals
  lpDelay(quarterSeconds);
  // Turn on Peripherals
}