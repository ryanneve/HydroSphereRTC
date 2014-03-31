/*Functionality  for Atlas Scientific's DO and COnductivity sensors*/
#include "HS_Atlas.h"

/*----------( Define Global Objects)----------*/

  
  
/*----------( Functions )----------*/
void DO_setup(){
  SerialDO.begin(38400);
  DO_quiet();
}
byte _DO_getReading(struct DO_Struct *aDO){
  /* Prompts for a value. Might be faster to put in continuous mode and just parse.
  */
  byte idx = 0; char readChar;
  byte arraySize = 20;
  char tempArray[arraySize];
  //while (!SerialDO.available()) {}; // wait for data COULD GET STCUK HERE!
  delay(1000);
  aDO->return_sat = 0;
  while (SerialDO.available()){
    readChar = SerialDO.read();
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
  if (idx > 0 ) {
    tempArray[idx] = '\0';
    memcpy(aDO->dox,tempArray,idx+1);
    return 0;
  }
  else return 3;  // Saw no characters
}
byte DO_getCompReading(struct DO_Struct *aDO,struct ms5803PTStruct *aTemp, struct Cond_Struct *aCond){
  SerialDO.print(aTemp->Temperature);
  SerialDO.print(',');
  SerialDO.print(aCond->ec);
  SerialDO.print(13);
  return _DO_getReading(aDO);
}
byte DO_getBasicReading(struct DO_Struct *aDO){
  SerialDO.print("R");SerialDO.print(13);
  return _DO_getReading(aDO);
}
void DO_toggle_sat() {SerialDO.print('%');SerialDO.print(13);}
void DO_quiet() {     SerialDO.print('E');SerialDO.print(13);}
void DO_continuous() {SerialDO.print('C');SerialDO.print(13);}

void COND_setup(){
  SerialCond.begin(38400);
  COND_quiet();
}
byte _COND_getReading(struct Cond_Struct *aCond){
  /* Prompts for a value. Might be faster to put in continuous mode and just parse.
  */
  byte field = 1; byte idx = 0; char readChar;
  byte arraySize = 20;
  char tempArray[arraySize];
  //while (!SerialCond.available()) {}; // wait for data COULD GET STCUK HERE!
  delay(1000);
  while (SerialCond.available()){
    readChar = SerialCond.read();
    Serial.println(readChar);
    if ( idx >= (arraySize - 1)) return 1; // Array to big
    if ( readChar == ' ')        return 2; // value includes space
    if ( readChar == 13)        break; // CR
    if ( readChar == 10)        break; // LF
    if ( readChar == ',' ) { // We're getting saturation
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
    }
    else {
      tempArray[idx] = readChar;
      idx++;
    }
  }
  return 3;  // Saw no characters
}
byte COND_getCompReading(struct ms5803PTStruct *aTemp, struct Cond_Struct *aCond){
  SerialCond.print(aTemp->Temperature);SerialCond.print(13);
  return _COND_getReading(aCond);
}
byte COND_getBasicReading(struct Cond_Struct *aCond){
  SerialCond.print('R');SerialCond.print(13);
  return _COND_getReading(aCond);
}
void COND_quiet() {   SerialCond.print('E');SerialCond.print(13);}
void COND_continuous() {   SerialCond.print('C');SerialCond.print(13);}

void LIGHT_setup(struct Light_Struct *aLight) {
  SerialLight.begin(38400);
  LIGHT_quiet();
  LIGHT_setMode(3,aLight);
}
byte LIGHT_getReading(struct Light_Struct *aLight){
  /* Prompts for a value. 
  */
  if ( aLight->mode < 1 || aLight->mode > 3 ) return 99;
  byte field = 1; byte idx = 0; char readChar;
  byte arraySize = 20;
  char tempArray[arraySize];
  delay(1200);
  aLight->light_sat = 0; // Assume not light_sat
  while (SerialCond.available()){
    readChar = SerialCond.read();
    Serial.println(readChar);
    if ( idx >= (arraySize - 1)) return 1; // Array to big
    else if ( readChar == ' ')        return 2; // value includes space
    else if ( readChar == 13)        break; // CR
    else if ( readChar == 10)        break; // LF
    else if ( readChar == '*' ) aLight->light_sat = 1; // We're getting saturation
    else if ( readChar == ',' ) {
      tempArray[idx] = '\0';
      if (field == 1 ) {
        if ( aLight->mode == 1 || aLight->mode == 3 ) memcpy(aLight->red,tempArray,idx+1);
        else memcpy(aLight->lx_red,tempArray,idx+1);
      }
      else if ( field == 2 ) {
        if ( aLight->mode == 1 || aLight->mode == 3 ) memcpy(aLight->green,tempArray,idx+1);
        else memcpy(aLight->lx_green,tempArray,idx+1);
      }
      else if (field == 3)  {
        if ( aLight->mode == 1 || aLight->mode == 3 ) memcpy(aLight->blue,tempArray,idx+1);
        else memcpy(aLight->lx_blue,tempArray,idx+1);
      }
      else if (field == 4)  {
        if ( aLight->mode == 1 ) return 7;
        else if ( aLight->mode == 2 ) memcpy(aLight->lx_total,tempArray,idx+1);
        else memcpy(aLight->lx_red,tempArray,idx+1);
      }
      else if (field == 5)  {
        if ( aLight->mode == 1 ) return 7;
        else if ( aLight->mode == 2 ) memcpy(aLight->lx_beyond,tempArray,idx+1);
        else  memcpy(aLight->lx_green,tempArray,idx+1);
      }
      else if (field == 6)  {
        if ( aLight->mode <= 2 ) return 7;
        memcpy(aLight->lx_blue,tempArray,idx+1);
      }
      else if (field == 7)  {
        if ( aLight->mode <= 2 ) return 7;
        memcpy(aLight->lx_total,tempArray,idx+1);
      }
      else if (field == 8)  {
        if ( aLight->mode <= 2 ) return 7;
        memcpy(aLight->lx_beyond,tempArray,idx+1);
      }
      else return 7;
      field++; idx = 0;
    }
    else {
      tempArray[idx] = readChar;
      idx++;
    }
  }
  return 3;  // Saw no characters

}
void LIGHT_setMode(unsigned char desired_mode,struct Light_Struct *aLight){
  char tempStr[4];
  tempStr[0] = 'M';
  tempStr[1] = desired_mode + '0';
  tempStr[2] = 13;
  tempStr[3] = '\0'; // May not want or need this.
  SerialLight.print(tempStr);
  // The ENV-RGB will respond: RGB<CR> or lx<CR> or RGB+lx<CR>
  aLight->mode = desired_mode;
}
void LIGHT_quiet() {     SerialLight.print('E');SerialLight.print(13);}
void LIGHT_continuous() {SerialLight.print('C');SerialLight.print(13);}

String gen_Atlas_log(int fresh_time, struct DO_Struct *aDO,struct Cond_Struct *aCond,struct Light_Struct *aLight){
  String log_string;
  if ( recentSample(aDO->sample_time,fresh_time )) {
    log_string += aDO->dox;            log_string += ","; 
    log_string += aDO->saturation;     log_string += ","; 
  }
  else log_string += " , ,";
  if ( recentSample(aCond->sample_time,fresh_time )) { //tds,ec,sal
    log_string += aCond->tds;            log_string += ","; 
    log_string += aCond->ec;             log_string += ","; 
    log_string += aCond->sal;            log_string += ","; 
  }
  else log_string += " , , ,";
  if ( recentSample(aLight->sample_time,fresh_time )) { //red,green,blue,lx_red,lx_green,lx_blue,lx_total,lx_beyond,light_sat  
    log_string += aLight->red;       log_string += ",";
    log_string += aLight->green;     log_string += ",";
    log_string += aLight->blue;      log_string += ",";
    log_string += aLight->lx_red;    log_string += ",";
    log_string += aLight->lx_green;  log_string += ",";
    log_string += aLight->lx_blue;   log_string += ",";
    log_string += aLight->lx_total;  log_string += ",";
    log_string += aLight->lx_beyond; log_string += ",";
    log_string += aLight->light_sat; log_string += ",";
  }
  else log_string += " , , , , , , , , ,";
  return log_string;
}

void DO_testing(struct DO_Struct *aDO,struct ms5803PTStruct *aTemp,struct Cond_Struct *aCond){  
  // put your main code here, to run repeatedly:
  Serial.println("DO----------------");
  delay(500);
  aDO->comm_error = DO_getCompReading(aDO,aTemp,aCond);
  if (!aDO->comm_error) {
    Serial.print("DO: "); Serial.println(aDO->dox);
    if ( aDO->return_sat ) {
      Serial.print("DO saturation: "); Serial.println(aDO->saturation);
    }
  }
  else Serial.print(F("DO Communications Error ")); Serial.println(aDO->comm_error,DEC);

  if ( !aDO->return_sat) DO_toggle_sat();
  Serial.println("COND----------------");
    delay(500);
  aCond->comm_error = COND_getBasicReading(aCond);
  //sensor_cond.comm_error = COND_getCompReading(&sensor_temp,&sensor_cond);
  if (!aCond->comm_error) {
    Serial.print("EC: "); Serial.print(aCond->ec);
    Serial.print("uS/cm\r\nTDS: "); Serial.println(aCond->tds);
    Serial.print("SAL: "); Serial.println(aCond->sal);
  }
  else Serial.print(F("Conductivity Communications Error:")); Serial.println(aCond->comm_error,DEC);
}