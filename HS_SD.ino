/*
Functions relating to SD and logging
*/


#include <SD.h>
#include <SPI.h>
#include "HS_SD.h"


boolean SD_setup() {
  log_msgL(0,F("Initializing SD card with CS %u"),SD_CHIP_SELECT);
  pinMode(SD_CHIP_SELECT,OUTPUT); 
  pinMode(10,OUTPUT);  
  if (!SD.begin(SD_CHIP_SELECT)) {
    log_msgL(5,F("Card failed, or not present"));
    return 0;
  }
  log_msgL(0,F("--------------SD SETUP DONE-----------------"));
  return 1;
}
void initialize_sample(){
  File sample_file = SD.open(LOG_FILE,FILE_WRITE); // Open file for WRITING
  log_msgL(0,F("Opened new collection file %s of size %u"),sample_file,sample_file.size());
  sample_file.print(SAMPLE_HEADER);
  sample_file.close();
}

void save_sample(String log_string){
  File sample_file = SD.open(LOG_FILE,FILE_WRITE); // Open file for WRITING
  log_msgL(0,F("Opened new collection file %s of size %u"),sample_file,sample_file.size());
  // Perhaps we should check the String to see if it has the right number of commas?
  sample_file.print(log_string);
  sample_file.close();
}
void _sd_log_msg(byte log_level,char* logMsg) {
  /* Save message to HS_ERROR_LOG_FILE with other data.
  */
  char log_date[9]; get_date(log_date); 
  char log_time[7]; get_time(log_time);
  char log_char = log_level + '0';
  File log_file = SD.open(HS_ERROR_LOG_FILE,FILE_WRITE); // Open file for WRITING  
  log_file.write(log_char);   log_file.write(",");
  log_file.write(log_date);   log_file.write(",");
  log_file.write(log_time);   log_file.write(",");
  log_file.write(logMsg);     log_file.write("\r\n");
  log_file.close();
}

int xSendFile(const char* xSendFileName){
  /* Send file via xmodem
  */
  //return XSend(&SD,&Serial,xSendFileName);
}
short xRecFile(const char* xRecFileName){
  /* Receive file via xmodem
  */
  //return XReceive(&SD,&Serial,xRecFileName);
}


/*-----( Message Logging  )-----*/
/*  There are two logging functions log_msg() and log_msgL() with the latter appending a LF
    The functions must be called with at least three arguments
      log_msg[l](<log_level>,<message_string as in printf>,<printf arg[,<printf arg>[,...]]>)
      The message_string argument can be surrounded by the F() macro to save RAM.
    Log Levels:
      4 - critical
      3 - error
      2 - warning
      1 - info
      0 - debug
*/
void _log_msg(byte logLevel,boolean nl, boolean serial_out,const char *format, va_list args){
  char buf[128]; // resulting string limited to 128 chars
  vsnprintf(buf, sizeof(buf), format, args);
  if ( serial_out  ) { //&& !g_quiet_mode ){
    if (nl) Serial.println(buf);
    else Serial.print(buf);
  }
  _sd_log_msg(logLevel,buf);
}
void _log_msg(byte logLevel,boolean nl, boolean serial_out,const __FlashStringHelper *format, va_list args){
  char buf[128]; // resulting string limited to 128 chars
#ifdef __AVR__
  vsnprintf_P(buf, sizeof(buf), (const char *)format, args); // progmem for AVR
#else
  vsnprintf(buf, sizeof(buf), (const char *)format, args); // for the rest of the world
#endif
  if ( serial_out ) { //&& !g_quiet_mode) {
    if (nl) Serial.println(buf);
    else Serial.print(buf);
  }
  _sd_log_msg(logLevel,buf);
}
void log_msg(byte logLevel, const char *format, ... ){ // no F()
  va_list args;
  va_start(args, format );
  _log_msg(logLevel,0,1,format,args);
  va_end(args);}
void log_msg(byte logLevel,const __FlashStringHelper *format, ... ){ // with F() 
  va_list args;
  va_start(args, format );
  _log_msg(logLevel,0,1,format,args);
  va_end(args);}
void log_msgL(byte logLevel, const char *format, ... ){ // no F()
  va_list args;
  va_start (args, format );
  _log_msg(logLevel,1,1,format,args);
  va_end(args);}
void log_msgL(byte logLevel,const __FlashStringHelper *format, ... ){ // with F()
  va_list args;
  va_start(args, format );
  _log_msg(logLevel,1,1,format,args);
  va_end(args);}

void setDate() {
  log_msgL(0,F("Setting date and time to %s  %s"),__DATE__, __TIME__);
  Rtc.adjust(DateTime(__DATE__, __TIME__)); // Set RTC time
}