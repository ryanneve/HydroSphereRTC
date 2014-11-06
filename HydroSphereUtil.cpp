/*
 * HydroSphereUtil.cpp
 *
 * Created: 9/19/2014 10:02:11 AM
 *  Author: Neve
 */ 


#include "HydroSphereRTC.h" // DO and Conductivity sensors
#include <RTClib.h>      // RTC Clock




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
// Also keeps track of clock drift in g_millisDelta
*/
void lowPowerDelay(uint16_t seconds) {
  //Logger_SD::Instance()->msgL(DEBUG,"Delaying about %u seconds from %lu",slow_millis);// at %02d:%02d:%02d",Rtc.getHours24(),Rtc.getMinutes(),Rtc.getSeconds());
  int16_t oldClkPr = CLKPR;  // save old system clock prescale
  CLKPR = 0x80;    // Tell the AtMega we want to change the system clock
  CLKPR = 0x08;    // 1/256 prescaler = 60KHz for a 16MHz crystal
  delay(seconds * 4);  // since the clock is slowed way down, delay(n) now acts like delay(n*256)
  CLKPR = 0x80;    // Tell the AtMega we want to change the system clock
  CLKPR = oldClkPr;    // Restore old system clock prescale
}

void deepSleep(uint16_t seconds){
  /* See 
     https://www.sparkfun.com/tutorials/309 
     for much more
   */
  // Turn off all peripherals?
  Serial.flush();
  lowPowerDelay(seconds);
  // Turn on Peripherals
}

float get_analog_temperature(uint8_t analog_pin) {
    float adc_val = analogRead(analog_pin);
    adc_val *= .0048;  //volts
    adc_val *= 1000; //millivolts
    float temp_C = ( 0.0512 * adc_val ) - 20.5128;
    return temp_C;
}