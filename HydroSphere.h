
#ifndef HydroSphere_h
#define HydroSphere_h
#include <stdint.h>

#include <Logger_SD.h>
// Uncomment this to re-implement 
#define NO_RTC

#define OFF_PIN               10
#define SOLENOID_PIN          11
#define LEVEL_SHIFT_POWER_PIN  2
#define SERIAL_BUFFER_SIZE 128
// Log levels
#define DEBUG 0
#define INFO 1
#define WARN 2
#define ERROR 3
#define CRITICAL 4


/*----------( Structures )----------*/

/*----------( Function Prototypes )----------*/
void saveLog();
void sched1();
void sched2();
void sched3();
#ifdef NO_RTC
void            initRTC();
void            getDate(char* date_str);
void            getTime(char* time_str);
void            setDate();
//bool            recentSample(uint32_t sample_time, uint16_t fresh_time);
#endif // NO_RTC
uint16_t         freeRam();
void            trigSolenoid(int16_t open_millis);
uint16_t         lpDelay(uint16_t quarterSeconds);
void            deepSleep(uint16_t quarterSeconds);


extern Logger_SD logger(uint8_t);
#endif //HydroSphere_h