
#ifndef HydroSphere_h
#define HydroSphere_h
#include <stdint.h>

#include <Logger_SD.h>
// Uncomment this to re-implement 

#define OFF_PIN               10
#define SOLENOID_PIN          11
#define LEVEL_SHIFT_POWER_PIN  2
#define BATT_VOLTAGE_PIN       0
#define TEMPERATURE_PIN        1  // Alternate temperature sensor when there is no pressure sensor
const uint8_t CLOCK_SS_PIN = 49;

#define SERIAL_BUFFER_SIZE 128
#define HS_CONSOLE_DELAY 5
// Log levels
#define DEBUG 0
#define INFO 1
#define WARN 2
#define ERROR 3
#define CRITICAL 4


/*----------( Structures )----------*/

/*----------( Function Prototypes )----------*/
void		saveLog();
void		sched1();
void		sched2();
void		sched3();
uint16_t	freeRam();
void		trigSolenoid(int16_t open_millis);
void		lowPowerDelay(uint16_t seconds);
void		deepSleep(uint16_t quarterSeconds);
void		hs_console();
char		getByte0(bool ignore_EOL);
void		HSconsoleDownloadData();
void		HSconsoleDeleteData();
void		HSconsoleUtilities();
void		downloadSampleFile();
void		watchdogSetup();

extern Logger_SD logger(uint8_t);
#endif //HydroSphere_h


/*----------( Global constants )----------*/
const char LOG_FILE[]    = "HS_LOG.CSV";
const char SCHED1_FILE[] = "SCHED1.CSV";
const char SCHED2_FILE[] = "SCHED2.CSV";
const char SCHED3_FILE[] = "SCHED3.CSV";
const char CONFIG_FILE[] = "config.txt";