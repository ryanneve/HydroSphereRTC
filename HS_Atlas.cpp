/*Functionality  for Atlas Scientific's DO and COnductivity sensors*/
#include <arduino.h>
#include "HS_Atlas.h"

#include "HS_OpenROV.h" // for structure with Temperature
#include "logger_SD.h" // for Logger_SD::Instance()->msgL()

/*----------( Define Global Objects)----------*/

//logger Logger_SD(SD_CHIP_SELECT);
  
  
/*----------( Functions )----------*/
void initDO(){
  Logger_SD::Instance()->msgL(DEBUG,"initDO");
  pinMode(DO_POWER_PIN,OUTPUT);
  digitalWrite(DO_POWER_PIN,HIGH);
  SerialDO.begin(38400);
  quietDO();
}
uint8_t _getDO(struct DO_Struct *aDO){
  /* Prompts for a value. Might be faster to put in continuous mode and just parse.
  */
  uint8_t idx = 0; char readChar;
  uint8_t arraySize = 20;
  char tempArray[arraySize];
  //while (!SerialDO.available()) {}; // wait for data COULD GET STCUK HERE!
  delay(1000);
  aDO->return_sat = 0;
  //Serial.print("Data from DO sensor: ");
  while (SerialDO.available()){
    readChar = SerialDO.read();
    //Serial.print(readChar);
    if ( idx >= (arraySize - 1)) return 1; // Array to big
    if ( readChar == ' ')        return 2; // value includes space
    if ( readChar == 13)        break; // CR
    if ( readChar == 10)        break; // LF
    if ( readChar == ',' ) { // We're getting saturation
      tempArray[idx] = '\0';
      memcpy(aDO->saturation,tempArray,idx+1);
      aDO->return_sat = 1;
      idx = 0;
    }
    else {
      tempArray[idx] = readChar;
      idx++;
    }
  }
  //Serial.println();
  if (idx > 0 ) {
    tempArray[idx] = '\0';
    memcpy(aDO->dox,tempArray,idx+1);
    return 0;
  }
  else return 3;  // Saw no characters
}
uint8_t getCompDO(struct DO_Struct *aDO, struct CondStruct *aCond){
  int32_t temp = getTemperature();
  char buf[10];
  dtostrf((int64_t)temp / 100,5,2,buf); buf[5] = '\0';
  //Serial.print("sending "); Serial.print(buf);
  SerialDO.print(buf);
  SerialDO.print(','); //Serial.print(',');
  SerialDO.print(aCond->ec);// Serial.println(aCond->ec);
  SerialDO.print(13);
  return _getDO(aDO);
}
uint8_t getBasicDO(struct DO_Struct *aDO){
  SerialDO.print("R");SerialDO.print(13);
  return _getDO(aDO);
}
void toggleDOSat() {SerialDO.print('%');SerialDO.print(13);}
void quietDO() {
  Logger_SD::Instance()->msgL(DEBUG,"quietDO");
  SerialDO.print('E');SerialDO.print(13);
}
void setDOContinuous() {SerialDO.print('C');SerialDO.print(13);}

void initCond(struct CondStruct *aCond){
  pinMode(COND_POWER_PIN,OUTPUT);
  digitalWrite(COND_POWER_PIN,HIGH);
  SerialCond.begin(38400);
  quietCond();
  strncpy(aCond->ec,"0\0",2); // initialize conductivity to 0
}
uint8_t _getCond(struct CondStruct *aCond){
  /* Prompts for a value. Might be faster to put in continuous mode and just parse.
  */
  uint8_t field = 1; uint8_t idx = 0; char readChar;
  uint8_t arraySize = 20;
  char tempArray[arraySize];
  //while (!SerialCond.available()) {}; // wait for data COULD GET STCUK HERE!
  delay(1000);
  //Serial.print("COND data:");
  while (SerialCond.available()){
    readChar = SerialCond.read();
    //Serial.write(readChar);
    if ( idx >= (arraySize - 1)) return 1; // Array to big
    if ( readChar == ' ')        return 2; // value includes space
    //if ( readChar == 13)        break; // CR
    //if ( readChar == 10)        break; // LF
    if ( readChar == ',' || readChar == 13 || readChar == 10) { // We're getting saturation
      tempArray[idx] = '\0';
      if (field == 1 ) {
        memcpy(aCond->ec,tempArray,idx+1);
      }
      else if ( field == 2 ) {
        memcpy(aCond->tds,tempArray,idx+1);
      }
      else if (field == 3 ){
        memcpy(aCond->sal,tempArray,idx+1);
        return 0; // Only correct way out.
      }
      field++; idx = 0;
      if (readChar == 13 || readChar == 10) break;
    }
    else {
      tempArray[idx] = readChar;
      idx++;
    }
  }
  //Serial.println();
  return 3;  // Saw no characters
}
uint8_t getCompCond(struct CondStruct *aCond){
  int32_t temp = getTemperature();
  char buf[10];
  dtostrf((int64_t)temp / 100,4,1,buf); buf[4] = ','; buf[5] = '\0';
  SerialCond.print(buf);SerialCond.print(13);
  return _getCond(aCond);
}
uint8_t getBasicCond(struct CondStruct *aCond){
  SerialCond.print('R');SerialCond.print(13);
  return _getCond(aCond);
}
void quietCond() {   SerialCond.print('E');SerialCond.print(13);}
void setCondContinuous() {   SerialCond.print('C');SerialCond.print(13);}

void gen_Atlas_log(char * log_array, uint8_t log_len, int16_t fresh_time, struct DO_Struct *aDO,struct CondStruct *aCond){
  if ( recentSample(aDO->sample_time,fresh_time )) {
    //log_string += aDO->dox;            log_string += ","; 
    //log_string += aDO->saturation;     log_string += ","; 
  }
  //else log_string += " , ,";
  if ( recentSample(aCond->sample_time,fresh_time )) { //tds,ec,sal
    //log_string += aCond->tds;            log_string += ","; 
    //log_string += aCond->ec;             log_string += ","; 
    //log_string += aCond->sal;            log_string += ","; 
  }
  //else log_string += " , , ,";
  //if ( recentSample(aLight->sample_time,fresh_time )) { //red,green,blue,lx_red,lx_green,lx_blue,lx_total,lx_beyond,light_sat  
    //log_string += aLight->red;       log_string += ",";
    //log_string += aLight->green;     log_string += ",";
    //log_string += aLight->blue;      log_string += ",";
    //log_string += aLight->lx_red;    log_string += ",";
    //log_string += aLight->lx_green;  log_string += ",";
    //log_string += aLight->lx_blue;   log_string += ",";
    //log_string += aLight->lx_total;  log_string += ",";
    //log_string += aLight->lx_beyond; log_string += ",";
    //log_string += aLight->light_sat; log_string += ",";
  //}
  //else log_string += " , , , , , , , , ,";
  //return log_string;
}

void DO_testing(struct DO_Struct *aDO, CondStruct *aCond){  
  // put your main code here, to run repeatedly:
  Serial.println("DO----------------");
  delay(500);
  aDO->comm_error = getCompDO(aDO,aCond);
  if (!aDO->comm_error) {
    Serial.print("DO: "); Serial.println(aDO->dox);
    if ( aDO->return_sat ) {
      Serial.print("DO saturation: "); Serial.println(aDO->saturation);
    }
  }
  else Serial.print(F("DO Communications Error ")); Serial.println(aDO->comm_error,DEC);

  if ( !aDO->return_sat) toggleDOSat();
  Serial.println("COND----------------");
    delay(500);
  aCond->comm_error = getBasicCond(aCond);
  //sensor_COND.comm_error = getCompCond(&sensor_temp,&sensor_COND);
  if (!aCond->comm_error) {
    Serial.print("EC: "); Serial.print(aCond->ec);
    Serial.print("uS/cm\r\nTDS: "); Serial.println(aCond->tds);
    Serial.print("SAL: "); Serial.println(aCond->sal);
  }
  else Serial.print(F("Conductivity Communications Error:")); Serial.println(aCond->comm_error,DEC);
}