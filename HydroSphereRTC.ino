/*
Main script, includes scheduler
Desired schedule:
1)  10  seconds: Compass, acceleration, orientation
2)  60  seconds: Light and depth
3)  300 seconds: DO, temperature, conductivity.
*/
#include "HydroSphereRTC.h" // DO and Conductivity sensors
#include <avr/wdt.h>     // Watchdog timer
#include <SPI.h>         // For SD card and clock
#include <SdFat.h>          // SPI SD card
#include <RTClib.h>      // RTC Clock
#include <Logger_SD.h>   // Logging
#include <Wire.h>        // for I2C
#include <I2Cdev.h>      // For HS_OpenROV libraries
#include <AK8975.h>      // For HS_OpenROV magnetometer
#include <MPU6050.h>     // For HS_OpenROV accel and gyro
#include <MS5803_I2C.h>  // for HS_OpenRov pressure and temperature
//#include <DS3234.h>      // SPI RTC Clock
#include <Atlas_RGB.h>   // RGB sensor
#include "HS_Atlas.h"    // DO & COND
#include "HS_OpenROV.h"  // Pressure & accelerometer

// Solenoid will trigger at or below this voltage.
// For 9.9v LiFe use 8.0
// For 8x1.5v (12v) alkaline use 9.2v
// For 2x4x1.5v (6v) alkaline use 4.6v
#define TRIGGER_VOLTAGE  4.6
bool ON_BATTERY = false;
uint8_t SCHED_RATE1 = 15; // Must be lowest
uint8_t SCHED_RATE2 = 30; // Must be multiple of SCHED_RATE1
uint8_t SCHED_RATE3 = 60;// Must be multiple of SCHED_RATE1
uint8_t EXT_MS5803_MAX_BAR = 5;  // OpenROV uses MS5803-14. We will be using -05 or -01
uint8_t EXT_MS5803_ADDR = 0x76;  // i2c address
//uint8_t INT_MS5803_MAX_BAR = 5;  //
//uint8_t INT_MS5803_ADDR = 0x77;  // i2c address

// Trigger date/time. These come from config.txt
uint8_t TRIG_DAY = 1;
uint8_t TRIG_HOUR = 12;
uint8_t TRIG_MINUTE = 00;
uint16_t SOL_DURATION = 5000; // Five seconds
uint16_t DEPLOY_DELAY = 0;  // Delay before logging/triggering in minutes
bool HAS_PT_SENSOR = 1;
uint16_t MPU_VERSION = 6050;  // Either 0, 6050 or 9050
char DO_VERSION = '5'; // Atlas Scientific DO chip version. '5','6' or 'E'
float EC_K = 0; // Either 0.1, 1.0 or 10.0
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
DateTime g_time; // Structure to hold a DateTime. USed for Messaging
DateTime g_sched_time; // Next scheduled action

/*----------( Define Global Variables)----------*/
char g_battery_voltage[7] = "??????";
const float VOLTAGE_MULTIPLIER = 2.0; // multiply by this to get true voltage.
bool do_sched1 = false,do_sched2 = false,do_sched3 = false;
const uint8_t g_LOG_SIZE = 160;
uint32_t g_Sched1;
uint32_t g_Sched2;
uint32_t g_Sched3;
uint32_t g_SchedNext;
uint32_t g_now_unixtime;
uint32_t g_delay_start;
bool g_sol_triggered = false;
const bool LOG_DEBUG = true; // Send DEBUG messages to log file?
const char SCHED1_HEADER[] = "Sample Time,GyX,GyY,GyZ,AccX,AccY,AccZ,MagX,MagY,MagZ,Head,VBatt\r\n";
const char SCHED2_HEADER[] = "Sample Time,External Temperature,Press_mBar,D1press,D2temp,red,green,blue,lux_r,lux_g,lux_b,lux_tot,lux_beyond,DO%,DO_mg/L,EC,TDS,Sal,SG\r\n";
const char SCHED3_HEADER[] = "Sample Time,\r\n";

void setup() {
	wdt_disable(); // Disable watchdog
	Serial.begin(57600);
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
	g_time = RTClock.now();
	g_sched_time = RTClock.now();
	g_now_unixtime = g_time.unixtime();
	g_delay_start  = g_now_unixtime + (DEPLOY_DELAY * 60);
	g_Sched1       = 0;
	g_Sched2       = 0;
	g_Sched3       = 0;
	g_SchedNext    = g_now_unixtime;
	RTClock.setA2Time(TRIG_DAY, TRIG_HOUR, TRIG_MINUTE,0, 0, 0, 0); // Set Solenoid Alarm
	RTClock.turnOnAlarm(2);
	Logger_SD::Instance()->setLogDT(&g_time); // We will use this global as the current time for logging.
	initDO();
	setDO_version(&sensor_DO,DO_VERSION);
	initCond(&sensor_COND);
	sensor_RGB.initialize(3,true); // mode 2, quiet = false (continuous mode)
	presstemp.setAddress(EXT_MS5803_ADDR);
	if ( presstemp.initialize(EXT_MS5803_MAX_BAR,true)) Logger_SD::Instance()->msgL(INFO,F("MS5803 initialization successful"));
	else Logger_SD::Instance()->msgL(WARN,F("MS5803 initialization failed."));
	if ( presstemp.testConnection() ) Logger_SD::Instance()->msgL(INFO,F("MS5803 connection successful"));
	else Logger_SD::Instance()->msgL(WARN,F("MS5803 connection failed with address %#Xh"),presstemp.getAddress());
	// Record pressure and temperature calibration constants
	/*for ( uint8_t c = 1 ; c <= 6 ; c++ ) {
		Logger_SD::Instance()->msgL(INFO,F("MS5803-%02i Cal Constant %u = %li"),EXT_MS5803_MAX_BAR, c,presstemp.getCalConstant(c));
	}*/
	if (MPU_VERSION != 0) setupAccelGyro(); // gyrocompass
	// Some debug info
	Logger_SD::Instance()->msgL(INFO,F("Schedule rates set to %d, %d & %d."),SCHED_RATE1,SCHED_RATE2,SCHED_RATE3);
	Logger_SD::Instance()->msgL(INFO,F("Trigger date set to day %d at %d:%02d"),TRIG_DAY,TRIG_HOUR,TRIG_MINUTE);
	Logger_SD::Instance()->msgL(INFO,F("Trigger duration set to %d ms"),SOL_DURATION);
	while ( Serial.available() ) Serial.read(); // clear buffer
	
	// Go into console mode?
	g_time = RTClock.now();
	char in_byte = 0;
	g_time = RTClock.now();
	const double CONSOLE_START = g_time.unixtime();
	Serial.println("Press c to go into console mode");
	while (!in_byte){
		if ( Serial.available() ) in_byte = Serial.read();
		// Now we need a timeout
		g_time = RTClock.now();
		if ( g_time.unixtime() > CONSOLE_START + HS_CONSOLE_DELAY ) break;
		else delay(100);
	}
	g_time = RTClock.now();
	switch ( in_byte ) {
		case 'c':
		case 'C':
			hs_console();
			break;
		case 'x':
			g_delay_start = g_time.unixtime(); // skip initial delay
			break;
		case 0:
			break;
		default:
			Logger_SD::Instance()->msgL(INFO,F("Console prompt got %d"), in_byte);
	}
	
	// First check if it's time to do schedule
	g_time = RTClock.now();
	if ( g_delay_start > g_time.unixtime() ) {
		Logger_SD::Instance()->msgL(INFO,F("Delaying instrument start %d minutes"), DEPLOY_DELAY);
		Serial.flush();
		delay(100);
		lowPowerDelay((uint16_t)(g_delay_start - g_time.unixtime()));
	}
	//watchdogSetup(); // Need to do wdt_reset() at least every 8 seconds.
}

void loop() {
	/**/
	wdt_reset(); // Reset watchdog timer
	g_time = RTClock.now();
	g_now_unixtime = g_time.unixtime(); // This is the time of the beginning of the loop. Not updated until next loop.
	/*
	Serial.print(g_now_unixtime); Serial.print(" >--> "); Serial.println(g_sched_time.unixtime());
	Serial.print('('); Serial.print(g_Sched1);
	Serial.print(','); Serial.print(g_Sched2);
	Serial.print(','); Serial.print(g_Sched3);
	Serial.println(')');
	*/
	if ( !g_sol_triggered ) checkSolenoid(); // triggered only once;
	if ( g_now_unixtime >= g_Sched1 && SCHED_RATE1 > 0 ) do_sched1 = true;
	if ( g_now_unixtime >= g_Sched2 && SCHED_RATE2 > 0 ) do_sched2 = true;
	if ( g_now_unixtime >= g_Sched3 && SCHED_RATE3 > 0 ) do_sched3 = true;
	// Now see if we need to run a schedule
	if ( do_sched1 || do_sched2 || do_sched3 ) {
		// Set time for next schedule, then do it.
		if ( do_sched1 ) {
			g_Sched1 = g_now_unixtime + SCHED_RATE1;
			sched1();
			do_sched1 = false;
		}
		if ( do_sched2 ) {
			g_Sched2 = g_now_unixtime + SCHED_RATE2;
			sched2();
			do_sched2 = false;
		}
		if ( do_sched3 ) {
			g_Sched3 = g_now_unixtime + SCHED_RATE3;
			sched3();
			do_sched3 = false;
		}
		g_time = RTClock.now();
		Logger_SD::Instance()->msgL(DEBUG,F("Done with schedules."));
		// Get the next occurrence
		g_SchedNext = min(g_Sched1,min(g_Sched2,g_Sched3));
		int32_t lp_delay = (int32_t)g_SchedNext - (int32_t)g_time.unixtime() - 1; // Sleep for one sec less than we need to
		if ( lp_delay > 0 ) {
			uint8_t buf_len = 30;
			char buf[buf_len];
			g_sched_time.setFromUNIX(g_SchedNext);
			g_sched_time.toString(buf,buf_len);
			Serial.println(buf);
			Logger_SD::Instance()->msgL(DEBUG,F("Next action in %d seconds at %s"),lp_delay,buf);
			Serial.flush();
			delay(100);
			lowPowerDelay((uint16_t)lp_delay);
		}
		else Logger_SD::Instance()->msgL(DEBUG,F("NOT Sleeping %d seconds."),lp_delay);
	}
	else delay(900);
}

void sched1() {
	/* Log GyX,GyY,GyZ,AccX,AccY,AccZ,MagX,MagY,MagZ,Head,VBatt
	*/
	g_time = RTClock.now(); // Update for logger
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
	uint16_t log_line_max = 200;
	uint8_t buf_len = 20;
	char log_output[log_line_max]; uint8_t log_idx = 0;
	char buf[buf_len];
	g_time = RTClock.now();
	log_idx +=sprintf(log_output + log_idx,"%s,",g_time.toYMDString(buf,buf_len));
	if (MPU_VERSION != 0) {// Both 6050 and 9050 have these.
		log_idx +=sprintf(log_output + log_idx,"%d,",sensor_GYRO.gyro_x);
		log_idx +=sprintf(log_output + log_idx,"%d,",sensor_GYRO.gyro_y);
		log_idx +=sprintf(log_output + log_idx,"%d,",sensor_GYRO.gyro_z);
		log_idx +=sprintf(log_output + log_idx,"%d,",sensor_GYRO.accel_x);
		log_idx +=sprintf(log_output + log_idx,"%d,",sensor_GYRO.accel_y);
		log_idx +=sprintf(log_output + log_idx,"%d,",sensor_GYRO.accel_z);
	}
	else { // Blanks
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
	else { // More blanks
		log_idx +=sprintf(log_output + log_idx,"0000,"); // magnetom_x is zero
		log_idx +=sprintf(log_output + log_idx,"0000,"); // magnetom_y is zero
		log_idx +=sprintf(log_output + log_idx,"0000,"); // magnetom_z is zero
		log_idx +=sprintf(log_output + log_idx,"0000,"); // head is zero
	}
	log_idx +=sprintf(log_output + log_idx,"%s" ,g_battery_voltage);
	Logger_SD::Instance()->msgL(DEBUG,"sched1 Logging %s of size %d",log_output,log_idx);
	if ( log_idx >= log_line_max ){
		Logger_SD::Instance()->msgL(ERROR,F("sched2 log line exceeds maximum of %d"),log_line_max);
		log_idx = log_line_max;
		log_output[log_idx - 1] = 0;
	}
	Logger_SD::Instance()->saveSample(log_output,log_idx);
}

void sched2() {
	/* "Sample Time,ExtTemp,D1press,Press_mBar,D2temp,red,grn,blue,lux_r,lux_g,lux_b,lux_tot,lux_beyond,DOsat,DO%,EC,TDS,Sal,SG\r\n";
	*/
	float ext_temperature;
	uint16_t log_line_max = 250;
	char log_output[log_line_max];
	uint8_t log_idx = 0;
	uint8_t buf_len = 20;
	char buf[buf_len];
	g_time = RTClock.now();// Update for logger
	Logger_SD::Instance()->msgL(DEBUG,F("---------- Sched2 entered"));
	if ( not sensor_RGB.getContinuous() ) sensor_RGB.getRGB(); // Get some new values
	if ( HAS_PT_SENSOR) {
		presstemp.getMeasurements(ADC_512,true);
		ext_temperature = presstemp.temp_C; // returns float
		//ext_temperature = presstemp.getTemperature()/100; // returns float
		//presstemp.getPressure(true);
		//Serial.print(F("Digital temperature is "); Serial.println(ext_temperature);
		dtostrf(ext_temperature,6,3,buf);
		Logger_SD::Instance()->msgL(INFO,F("Digital temperature is %s"),buf);
	}
	else {
		ext_temperature = get_analog_temperature(TEMPERATURE_PIN);
		//Serial.print(F("Analog temperature is ")); Serial.println(ext_temperature);
		dtostrf(ext_temperature,6,3,buf);
		Logger_SD::Instance()->msgL(INFO,F("Analog temperature is %s"),buf);
	}
	setCondTemp(ext_temperature); // Causing problems
	getCond(&sensor_COND);// conductivity.
	
	setDOtemp_cond(ext_temperature,sensor_COND._ec);
	getDO(&sensor_DO);
	
	// Log readings
	Logger_SD::Instance()->setSampleFile(SCHED2_FILE);
	g_time = RTClock.now();// Update for logger
	log_idx +=sprintf(log_output + log_idx,"%s,",g_time.toYMDString(buf,buf_len));
	dtostrf(ext_temperature,6,3,buf);	// External Temperature
	log_idx +=sprintf(log_output + log_idx,"%s,",buf);
	
	// Pressure and diagnostics
	if ( HAS_PT_SENSOR ) {
		dtostrf(presstemp.press_mBar,6,3,buf);
		log_idx +=sprintf(log_output + log_idx,"%s,",buf);
		itoa(presstemp.getD1Pressure(),buf,10);
		log_idx +=sprintf(log_output + log_idx,"%s,",buf);
		itoa(presstemp.getD2Temperature(),buf,10);
		log_idx +=sprintf(log_output + log_idx,"%s,",buf);
	}
	else {
		log_idx +=sprintf(log_output + log_idx,"0000,"); // D1temp is zero
		log_idx +=sprintf(log_output + log_idx,"0000,"); // pressure is zero
		log_idx +=sprintf(log_output + log_idx,"0000,"); // D2press is zero
	}
	// Light sensor readings
	log_idx +=sprintf(log_output + log_idx,"%d,",sensor_RGB.getRed());
	log_idx +=sprintf(log_output + log_idx,"%d,",sensor_RGB.getGreen());
	log_idx +=sprintf(log_output + log_idx,"%d,",sensor_RGB.getBlue());
	log_idx +=sprintf(log_output + log_idx,"%d,",sensor_RGB.getLxRed());
	log_idx +=sprintf(log_output + log_idx,"%d,",sensor_RGB.getLxGreen());
	log_idx +=sprintf(log_output + log_idx,"%d,",sensor_RGB.getLxBlue());
	log_idx +=sprintf(log_output + log_idx,"%d,",sensor_RGB.getLxTotal());
	log_idx +=sprintf(log_output + log_idx,"%d," ,sensor_RGB.getLxBeyond());
	// Now DO saturation, %oxy
	dtostrf(sensor_DO._sat,8,2,buf);
	log_idx +=sprintf(log_output + log_idx,"%s,",buf);
	dtostrf(sensor_DO._dox,8,2,buf);
	log_idx +=sprintf(log_output + log_idx,"%s,",buf);
	//Now conductivity: EC, TDS, SAL, SG
	dtostrf(sensor_COND._ec,7,2,buf);
	log_idx +=sprintf(log_output + log_idx,"%s,",buf);
	dtostrf(sensor_COND._tds,7,2,buf);
	log_idx +=sprintf(log_output + log_idx,"%s,",buf);
	dtostrf(sensor_COND._sal,7,2,buf);
	log_idx +=sprintf(log_output + log_idx,"%s,",buf);
	dtostrf(sensor_COND._sg,7,2,buf);
	log_idx +=sprintf(log_output + log_idx,"%s",buf);
	Logger_SD::Instance()->msgL(DEBUG,F("sched2 Logging %s of size %d"),log_output,log_idx);
	if ( log_idx >= log_line_max ) Logger_SD::Instance()->msgL(ERROR,F("sched2 log line exceeds maximum of %d"),log_line_max);
	Logger_SD::Instance()->saveSample(log_output,log_idx);
}

void sched3() {
	/* Log 
	*/
	Logger_SD::Instance()->setSampleFile(SCHED3_FILE);
	uint16_t log_line_max = 200;
	uint8_t buf_len = 20;
	char log_output[log_line_max]; uint8_t log_idx = 0;
	char buf[buf_len];
	g_time = RTClock.now();
	Logger_SD::Instance()->msgL(DEBUG,F("---------- Sched3 entered"));
	log_idx +=sprintf(log_output + log_idx,"%s,",g_time.toYMDString(buf,buf_len));
	Logger_SD::Instance()->msgL(DEBUG,F("sched3 Logging %s of size %u"),log_output,log_idx);
	if ( log_idx >= log_line_max ) Logger_SD::Instance()->msgL(ERROR,F("sched3 log line exceeds maximum of %d"),log_line_max);
	Logger_SD::Instance()->saveSample(log_output,log_idx);
	//delay(10000); // should trip wathcdog
	//delay(10000); // should trip wathcdog 
}

bool checkSolenoid(){
	/* Check solenoid trigger conditions */
	bool trigger = false;
	int16_t volt_pin_val = analogRead(BATT_VOLTAGE_PIN);
	float battery_voltage = ((float)volt_pin_val/1024) * 5.00 * VOLTAGE_MULTIPLIER;
	if ( battery_voltage > 5.0  && ON_BATTERY == false ) {
		ON_BATTERY = true; //ON_BATTERY
		Logger_SD::Instance()->msgL(INFO,F("Battery detected, enabling solenoid"));
	}
	dtostrf(battery_voltage,5,3,g_battery_voltage); g_battery_voltage[6] = 0;
	char batt_min[7]; dtostrf(TRIGGER_VOLTAGE,5,3,batt_min); batt_min[6] = 0;
	Logger_SD::Instance()->msgL(DEBUG,F("Battery voltage %s. Min is %s (%d)."),g_battery_voltage,batt_min,volt_pin_val);
	if ( g_sol_triggered == false && ON_BATTERY) { // only once and must have seen a battery
		if ( battery_voltage <= TRIGGER_VOLTAGE ) {
			Logger_SD::Instance()->msgL(CRITICAL,F("Low voltage %s <= %s triggering Solenoid"),g_battery_voltage,batt_min);
			trigger = true;
		}
		// Now check time.
		g_time = RTClock.now();
		if ( TRIG_DAY == g_time.day() ) { // Date matches
			//Serial.println("DAY matches");
			Logger_SD::Instance()->msgL(DEBUG,F("Looking for %d:%d >= %d:%d"),g_time.hour(),g_time.minute(), TRIG_HOUR,TRIG_MINUTE);
			if ( (TRIG_HOUR < g_time.hour()) || ( TRIG_HOUR == g_time.hour() && TRIG_MINUTE <= g_time.minute() )) { // Hour matches or has passed. Minute is now or has passed.
				uint8_t time_str_len = 20;
				char time_str[time_str_len];
				g_time = RTClock.now();
				Logger_SD::Instance()->msgL(WARN,F("Current time %s >= trigger time %d-%d:%d"),g_time.toYMDString(time_str,time_str_len),TRIG_DAY,TRIG_HOUR,TRIG_MINUTE);
				trigger = true;
			}
			Logger_SD::Instance()->msgL(INFO,F("Didn't find trigger match"));
		}
		// Check RTC
		if ( RTClock.checkIfAlarm(2) ) {
			g_time = RTClock.now();
			Logger_SD::Instance()->msgL(WARN,F("RTC Alarm 2 indicated"));
			trigger = true;
		}
	}
	if ( trigger ) trigSolenoid(SOL_DURATION);
	return trigger;
}

void trigSolenoid(uint16_t open_millis) {
	/* Trigger solenoid for open_millis ms */
	uint32_t close_time = millis() + open_millis;
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
	g_time = RTClock.now();
	if ( (g_time.year() > 2050) || (g_time.year() < 2014) ) {
		Logger_SD::Instance()->msgL(ERROR,F("Date %d/%d/%d out of range"),
		g_time.year(), g_time.month(), g_time.day());
	}
	Logger_SD::Instance()->msgL(INFO,F("Clock reports %d/%02d/%02d at %d:%02d:%02d"),
	g_time.year(), g_time.month(), g_time.day(),g_time.hour(), g_time.minute(), g_time.second());
}

void setupAlarm(){
}
void loadConfigValues(){
	// Load config defaults. All values in file are integers.
	int32_t config_value;
	Logger_SD::Instance()->setDebug(false);
	// Scheduling
	config_value = Logger_SD::Instance()->getConfig((char *)"SCHED_RATE1");	        if ( config_value != -1 ) SCHED_RATE1 =        (uint8_t) config_value;
	config_value = Logger_SD::Instance()->getConfig((char *)"SCHED_RATE2");	        if ( config_value != -1 ) SCHED_RATE2 =        (uint8_t) config_value;
	config_value = Logger_SD::Instance()->getConfig((char *)"SCHED_RATE3");	        if ( config_value != -1 ) SCHED_RATE3 =        (uint8_t) config_value;
	config_value = Logger_SD::Instance()->getConfig((char *)"TRIG_DAY");	        if ( config_value != -1 ) TRIG_DAY =           (uint8_t) config_value;
	config_value = Logger_SD::Instance()->getConfig((char *)"TRIG_HOUR");	        if ( config_value != -1 ) TRIG_HOUR =          (uint8_t) config_value;
	config_value = Logger_SD::Instance()->getConfig((char *)"TRIG_MINUTE");	        if ( config_value != -1 ) TRIG_MINUTE =        (uint8_t) config_value;
	config_value = Logger_SD::Instance()->getConfig((char *)"SOL_DURATION");	    if ( config_value != -1 ) SOL_DURATION =       (uint16_t) config_value;
	config_value = Logger_SD::Instance()->getConfig((char *)"DEPLOY_DELAY");        if ( config_value != -1 ) DEPLOY_DELAY =       (uint16_t) config_value;
	// Pressure/Temperature
	config_value = Logger_SD::Instance()->getConfig((char *)"HAS_PT_SENSOR");	    if ( config_value != -1 ) HAS_PT_SENSOR =      (bool) config_value;
	config_value = Logger_SD::Instance()->getConfig((char *)"EXT_MS5803_MAX_BAR");	if ( config_value != -1 ) EXT_MS5803_MAX_BAR = (uint8_t) config_value;
	config_value = Logger_SD::Instance()->getConfig((char *)"EXT_MS5803_ADDR");	    if ( config_value != -1 ) EXT_MS5803_ADDR =    (uint8_t) config_value;
	// Accelerometer
	config_value = Logger_SD::Instance()->getConfig((char *)"MPU_VERSION");        	if ( config_value != -1 ) MPU_VERSION =        (uint16_t) config_value;
	// DO
	config_value = Logger_SD::Instance()->getConfig((char *)"DO_VERSION");	        if ( config_value != -1 ) DO_VERSION =         (char) config_value;
	// EC
	config_value = Logger_SD::Instance()->getConfig((char *)"EC_K_10");	            if ( config_value != -1 ) EC_K =               (float) config_value / 10.0; 
	
	Logger_SD::Instance()->setDebug(false);
}