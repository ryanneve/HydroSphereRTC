/* Manage LimnoTerra SD card files.
  Created by Ryan Neve October 2, 2013
  PUT LICENSE HERE
*/

#ifndef HS_SD_h
#define HS_SD_h
#include <arduino.h>



/*-----( Defenitions )-----*/  
#define LOG_FILE "HYDROSPH.CSV"
#define HS_ERROR_LOG_FILE "ERR_LOG.CSV"

/*-----( Declare CONSTANTS )-----*/  
#if (defined(__AVR_ATmega1280__)||defined(__AVR_ATmega2560__)) // SD card chip select. 10 on small boards, 53 on MEGA
const byte SD_CHIP_SELECT = 53;
#else
const byte SD_CHIP_SELECT = 10;
#endif
const char SAMPLE_HEADER[] = "Date,Time,                    // Always included\
                              pressure, temperature,                          \
                              gyro_x,gyro_y,gyro_z,MAG_Heading,               \
                              dox,dosat                                       \
                              tds,ec,sal,                                     \
                              red,green,blue,lx_red,lx_green,lx_blue,lx_total,\
                              lx_beyond,light_sat,\r\n\0";                     
/*-----( Declare function protoTypes )-----*/ 
// SD related functions
boolean          setup_SD();
void             _sd_log_msg(boolean is_lcd, byte log_level,char* logMsg);
unsigned int     upload_file(  const char* file_name);
unsigned int     download_file(const char* file_name);
void diagnostics_HS_SD();

// Logging related functions
void _log_msg(byte logLevel,boolean nl, boolean serial_out,const __FlashStringHelper *format, va_list args);
void log_msg(byte logLevel, const char *format, ... );
void log_msg(byte logLevel,const __FlashStringHelper *format, ... );
void log_msgL(byte logLevel, const char *format, ... );
void log_msgL(byte logLevel,const __FlashStringHelper *format, ... );

void setDate();

#endif