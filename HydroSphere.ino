/*

Main script, includes scheduler
Desired schedule:
  10  seconds: Compass, acceleration, orientation
  60  seconds: Light and depth
  300 seconds: DO, temperature, conductivity.
*/
#include "HydroSphere.h" // DO and Conductivity sensors
#include "HS_SD.h"
#include "RTClib.h" // Clock

/*----------( Define Global Objects)----------*/
struct DO_Struct sensor_DO;
struct Cond_Struct sensor_cond;
struct Light_Struct sensor_light;
struct ms5803PTStruct ms5803PT;
struct mpu9150MEMSStruct mpu9150MEMS;
RTC_DS1307 Rtc;   // Clock
DateTime now;
DateTime Sched1;
DateTime Sched2;
DateTime Sched3;
bool do_sched1,do_sched2,do_sched3 = true;
String log_string;
const unsigned int DELAY_SECONDS = 8;
unsigned int sched_rate1 = 10; // Must be lowest
unsigned int sched_rate2 = 20; // Must be multiple of sched_rate1
unsigned int sched_rate3 = 30;// Must be multiple of sched_rate1

void setup() {
  Serial.begin(57600);
  rtc_setup();
  SD_setup();
  initialize_sample(); // After SD
  DO_setup();
  COND_setup();
  LIGHT_setup(&sensor_light);
  ms5803_Setup(&ms5803PT); // Pressure, Temperature and gyrocompass
}


void loop() {
  /**/
  now = Rtc.now();
  log_string = "";
  // First check if is's time to do schedule
  if ( now.unixtime() - Sched1.unixtime() >= sched_rate1 ) do_sched1 = true;
  if ( now.unixtime() - Sched2.unixtime() >= sched_rate2 ) do_sched2 = true;
  if ( now.unixtime() - Sched3.unixtime() >= sched_rate3 ) do_sched3 = true;
  // Now see if we need to update our values
  if ( do_sched1 || do_sched2 || do_sched3 ) {
    if ( do_sched1 ) {
      //  UPDATE SAMPLES
      Serial.print("Action 1 at "); Serial.println(now.unixtime());
      Sched1 = Rtc.now(); do_sched1 = false;
    }
    if ( do_sched2 ) {
      //  UPDATE SAMPLES
      Serial.print("Action 2 at "); Serial.println(now.unixtime());
      Sched2 = Rtc.now(); do_sched2 = false;
    }
    if ( do_sched3 ) {
      //  UPDATE SAMPLES
      Serial.print("Action 3 at "); Serial.println(now.unixtime());
      Sched3 = Rtc.now(); do_sched3 = false;
    }
    // All updated, now log
    log_string = "CCYYMMDD,HHMMSS,";
    // Need to build log file based on what was updated in the last sched_rate1 minutes.
    log_string += gen_OpenROV_log(sched_rate1,&ms5803PT,&mpu9150MEMS);
    log_string += gen_Atlas_log(  sched_rate1,&sensor_DO,&sensor_cond,&sensor_light);
    log_string += "\r\n";
    save_sample(log_string);
  }
  else {
  Serial.print(now.unixtime()) ; Serial.print(" "); Serial.println(Sched1.unixtime());
    Serial.println(now.unixtime() - Sched1.unixtime());
    // HERE IS WHERE WE CAN MAKE POWER SAVINGS
    for ( int i = DELAY_SECONDS; i > 0 ; i-- ){
      Serial.print(i); Serial.print(" ");
      delay(1000);
    }
    Serial.println();
  }
}


// Functions related to date and time
void rtc_setup(){
  Rtc.begin();  
  if ( !Rtc.isrunning() ) {
    log_msgL(1,F("RTC is NOT running! Setting RTC"));
    setDate();
  }
  DateTime now = Rtc.now();
  if ( now.year() > 2050) {
    log_msgL(3,F("Date %d/%d/%d out of range"),
               now.year(), now.month(), now.day());
    setDate();
  }
  log_msgL(0,F("Clock reports %d/%d/%d at %d:%d:%d"),
               now.year(), now.month(), now.day(),
               now.hour(), now.minute(), now.second());
               
}

// Time related functions
void get_date(char* date_str) { // assumes date_str is char[9] CCYYMMDD\0
  DateTime now = Rtc.now();
  char temp_str[4] = "000";
  unsigned int yr = int(now.year());
  unsigned int mo = int(now.month());
  byte dy = now.day();
  sprintf(temp_str,"%u",yr);
  memcpy(date_str,temp_str,4);
  if ( mo < 10 ) sprintf(temp_str,"0%u\0",mo);
  else           sprintf(temp_str,"%u\0", mo);
  memcpy(date_str + 4,temp_str,2);
  if ( dy < 10 ) sprintf(temp_str,"0%u\0",dy);
  else           sprintf(temp_str,"%u\0", dy);
  memcpy(date_str + 6,temp_str,2);
  date_str[8] = '\0';
}

void get_time(char* time_str)  { // Assumes time_str is char[7] HHMMSS\0
  DateTime now = Rtc.now();
  char char_time[3];
  byte hr, mi, se;
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

boolean recentSample(char time[7], int fresh_time){
  /* Returns a 1 if now - time < fresh_time (in minutes)
  */
  // First convert time to an integer representing hours and minutes.
  int spl_time = atoi(time) / 100;
  now = Rtc.now();
  int now_time = now.minute() + (now.hour() * 100);
  if (now_time < spl_time ) now_time += 2400;
  int delta = now_time - spl_time;
  if ( delta <= fresh_time && delta >= 0 ) return 1; 
  else return 0;
}
int freeRam() {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

/*
lpDelay(quarterSeconds) - Low Power Delay.  Drops the system clock
                          to its lowest setting and sleeps for 256*quarterSeconds milliseconds.
*/
int lpDelay(unsigned int quarterSeconds) {
  int oldClkPr = CLKPR;  // save old system clock prescale
  CLKPR = 0x80;    // Tell the AtMega we want to change the system clock
  CLKPR = 0x08;    // 1/256 prescaler = 60KHz for a 16MHz crystal
  delay(quarterSeconds);  // since the clock is slowed way down, delay(n) now acts like delay(n*256)
  CLKPR = 0x80;    // Tell the AtMega we want to change the system clock
  CLKPR = oldClkPr;    // Restore old system clock prescale
}

void deepSleep(unsigned int quarterSeconds){
  /* See 
     https://www.sparkfun.com/tutorials/309 
     for much more
   */
  // Turn off all peripherals
  lpDelay(quarterSeconds);
  // Turn on Peripherals
}