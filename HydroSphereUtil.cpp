/*
 * HydroSphereUtil.cpp
 *
 * Created: 9/19/2014 10:02:11 AM
 *  Author: Neve
 */ 


#include <avr/wdt.h>     // Watchdog timer
#include "HydroSphereRTC.h" // DO and Conductivity sensors
#include <RTClib.h>      // RTC Clock

RTC_DS3234 HSU_RTClock(CLOCK_SS_PIN);
DateTime HSU_time; // Structure to hold a DateTime


float getDensity(float temp_C, float salinity) {
	// salinity is in kg/m^3
	// Density is on kg/m^3
	float density_kgm3 = (  (  999.842594 +
	(  0.06793952        * temp_C ) +
	(  0.009095290       * pow(temp_C,2) ) +
	(  0.0001001685      * pow(temp_C,3) ) -
	(  0.000001120083    * pow(temp_C,4) ) +
	(  0.000000006536332 + pow(temp_C,5) ) ) +
	(  (  0.824493 -
	(  0.0040899       * temp_C ) +
	(  0.000076438     * pow(temp_C,2) ) -
	(  0.00000082467   * pow(temp_C,3) ) +
	(  0.0000000053875 * pow(temp_C,4) ) ) *  salinity ) +
	(  (  -0.00572466 +
	(  0.00010227 * temp_C) -
	(  0.0000016546 * pow(temp_C,2) ) *
	pow(salinity,1.5) ) ) +
	(  0.00048314 * pow(salinity,2) ) );
	return density_kgm3;
}

float getDepth(float pressure_kpa, float density_kgm3) {
	// returns depth in meters
	// Standard atmospheric pressure 101.325 kPa
	float density_lbft = density_kgm3 * 0.0624279606;
	float pressure_psi = pressure_kpa * 0.1450377;
	float pressure_lbft = pressure_psi * 144.0;
	float depth_m = (0.3048 * pressure_lbft ) / ( density_lbft );
	return depth_m;
}


uint16_t freeRam() {
	extern int16_t __heap_start, *__brkval;
	int16_t v;
	return (int16_t) &v - (__brkval == 0 ? (int16_t) &__heap_start : (int16_t) __brkval);
}


/*
// lowPowerDelay(seconds) - Low Power Delay.  Drops the system clock
// to its lowest setting and sleeps for (256*quarterSeconds) milliseconds.
*/
void lowPowerDelay(uint16_t delay_secs) {
	const uint16_t MAX_DELAY = 4; // Longest we will delay without resetting watchdog
	uint16_t delay_portion;
	wdt_reset(); // Reset watchdog timer, needs to be reset at least every 8 seconds.
	Serial.print("delaying "); Serial.print(delay_secs);
	if ( delay_secs <= 2 ) return ;
	delay_secs -= 2;
	uint16_t oldClkPr = CLKPR;  // save old system clock prescale
	//Logger_SD::Instance()->msgL(DEBUG,F("Delaying about %u seconds. CLKPR = %X"),delay_secs,oldClkPr);
	delay(1000); // Allow time for things to flush and write?
	while ( delay_secs >= 1){
		wdt_reset(); // Reset watchdog timer, needs to be reset at least every 8 seconds.
		CLKPR = 0x80;    // Tell the AtMega we want to change the system clock
		CLKPR = 0x08;    // 1/256 prescaler = 60KHz for a 16MHz crystal
		if ( delay_secs > MAX_DELAY ) {
			delay_portion = MAX_DELAY;
			delay_secs -= MAX_DELAY;
		}
		else {
			delay_portion = delay_secs;
			delay_secs = 0;
		}
		delay((delay_portion)* 4);  // since the clock is slowed way down, delay(n) now acts like delay(n*256)
		CLKPR = 0x80;    // Tell the AtMega we want to change the system clock
		CLKPR = oldClkPr;    // Restore old system clock prescale
	}
	delay(1000);
}

void deepSleep(uint16_t delay_secs){
  /* See 
     https://www.sparkfun.com/tutorials/309 
     for much more
   */
  // Turn off all peripherals?
  Serial.flush();
  lowPowerDelay(delay_secs);
  // Turn on Peripherals
}

float get_analog_temperature(uint8_t analog_pin) {
    float adc_val = analogRead(analog_pin);
    adc_val *= .0048;  //volts
    adc_val *= 1000; //millivolts
    float temp_C = ( 0.0512 * adc_val ) - 20.5128;
    return temp_C;
}

void hs_console(){
	/* HydroSphere console.
		Functions include:
			Download data files
			clear data files
			exit
	*/
	char read_byte;
	do {
		// Prompt
		Serial.println(F("HydroSphere Console"));
		Serial.println(F("1 - Download Data Files"));
		Serial.println(F("2 - Delete Data Files"));
		Serial.println(F("3 - Card Utilities"));
		Serial.println(F("0 - Exit console mode"));
		// Look for and process input
		read_byte = getByte0(true);
		switch (read_byte){
		case '1':
		    HSconsoleDownloadData();
		    break;
	    case '2':
		    HSconsoleDeleteData();
		    break;
	    case '3':
		    HSconsoleUtilities();
		    break;
	    case '0':
		    Serial.println(F("Exiting console mode"));
		    return;
		    break;
		    default:
		    continue;
		}
	} while(1);
}

void HSconsoleDownloadData(){
	char read_byte;
	do 
	{
		// Prompt for which file to download
		Serial.println(F("Select file to download:"));
		Serial.print("1 - "); Serial.println(SCHED1_FILE);
		Serial.print("2 - "); Serial.println(SCHED2_FILE);
		Serial.print("3 - "); Serial.println(SCHED3_FILE);
		Serial.print("9 - "); Serial.println(LOG_FILE);
		Serial.println(F("0 - Exit download mode"));
		read_byte = getByte0(true);
		switch (read_byte) {
			case '1':
				Logger_SD::Instance()->setSampleFile(SCHED1_FILE);
				downloadSampleFile();
				break;
			case '2':
				Logger_SD::Instance()->setSampleFile(SCHED2_FILE);
				downloadSampleFile();
				break;
			case '3':
				Logger_SD::Instance()->setSampleFile(SCHED3_FILE);
				downloadSampleFile();
				break;
			case '9':
				Logger_SD::Instance()->dumpLogFile();
				break;
			case '0':
				return;
			default:
				continue;
		}
	} while (true);	
}
void downloadSampleFile(){
	const uint16_t DUMP_DELAY = 5;
	Serial.print(F("Prepare to Capture file: "));
	for ( uint16_t i = DUMP_DELAY; i > 0 ; i-- ) {
		delay(1000);
		Serial.print(i); Serial.print(' ');
	}
	Serial.println();
	Logger_SD::Instance()->dumpSampleFile();
	Serial.println("DONE");
	delay(5000); // Time to end capture
}

void HSconsoleDeleteData(){
	char read_byte;
	do	{
		// Prompt for which file to download
		Serial.println(F("Select file to delete:"));
		Serial.print("1 - "); Serial.println(SCHED1_FILE);
		Serial.print("2 - "); Serial.println(SCHED2_FILE);
		Serial.print("3 - "); Serial.println(SCHED3_FILE);
		Serial.print("9 - "); Serial.println(LOG_FILE);
		Serial.println(F("0 - Exit delete mode"));
		read_byte = getByte0(true);
		switch (read_byte) {
			case '1':
			Logger_SD::Instance()->setSampleFile(SCHED1_FILE);
			Logger_SD::Instance()->deleteSampleFile();
			break;
			case '2':
			Logger_SD::Instance()->setSampleFile(SCHED2_FILE);
			Logger_SD::Instance()->deleteSampleFile();
			break;
			case '3':
			Logger_SD::Instance()->setSampleFile(SCHED3_FILE);
			Logger_SD::Instance()->deleteSampleFile();
			//break;
			case '9':
			Logger_SD::Instance()->deleteLogFile();
			break;
			case '0':
			return;
			default:
			continue;
		}
	} while (true);	
}


void HSconsoleUtilities(){
	char read_byte;
	do	{
		// Prompt for which file to download
		Serial.println(F("Select function:"));
		Serial.println("1 - Card Info");
		Serial.println("2 - Upload ini file");
		Serial.println("3 - View ini file");
		Serial.println("9 - FORMAT CARD");
		Serial.println(F("0 - Exit utility mode"));
		read_byte = getByte0(true);
		switch (read_byte) {
			case '1':
				Logger_SD::Instance()->cardInfo();
				break;
			case '2':
				Serial.println("Not Implemented");
				break;
			case '3':
				Serial.println("Not Implemented");
				break;
			case '9':
				Serial.println("Not Implemented");
				break;
			case '0':
				return;
			default:
				continue;
		}
	} while (true);
}



uint16_t clearSerial0() {
	uint16_t chars = 0;
	while ( Serial.available() ) {
		Serial.read();
		chars++;
	}
	return chars;
}

char getByte0(bool ignore_EOL){
	char read_byte = 0;
	clearSerial0();
	do {
		if ( Serial.available() ) {
			read_byte = Serial.read();
			if  ( ignore_EOL && ( read_byte == 13 || read_byte == 10 )) read_byte = 0;
		}
		else delay(100);
	} while ( !read_byte );
	clearSerial0();
	return read_byte;
}

void watchdogSetup(){
	cli(); // Disable all interrupts
	wdt_reset();
	// Enter Watchdog configuration mode WatchDogTimerControlRegister
	WDTCSR |= (1<<WDCE) | (1<<WDE);
	// Set Watchdog settings
	// WDIE is interrupt on timeout
	// WDE is reset on timeout
	// WDPx: 1001 = 8 seconds
	WDTCSR = (1<<WDIE) | (1<<WDE) | (1<<WDP3) | (0<<WDP2) | (0<<WDP1) | (1<<WDP0);
	sei(); // enable interrupts
}

ISR(WDT_vect){
	// Watchdog timer interrupt
}