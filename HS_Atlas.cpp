/*Functionality  for Atlas Scientific's DO and COnductivity sensors*/
#include <arduino.h>
#include "HS_Atlas.h"

#include "HS_OpenROV.h" // for structure with Temperature
#include <Logger_SD.h> // for Logger_SD::Instance()->msgL()

/*----------( Define Global Objects)----------*/

//logger Logger_SD(SD_CHIP_SELECT);
bool g_atlas_debug = 1;
  
/*----------( Functions )----------*/
void initDO(){
  Logger_SD::Instance()->msgL(INFO,F("Initializing DO system."));
  pinMode(DO_POWER_PIN,OUTPUT);
  digitalWrite(DO_POWER_PIN,HIGH);
  SerialDO.begin(38400);
  quietDO();
}

uint8_t  _getDO(struct DO_Struct *aDO){
  /* Prompts for a value. Might be faster to put in continuous mode and just parse.
  */
 
  //while (!SerialDO.available()) {}; // wait for data COULD GET STCUK HERE!
  delay(1000);
  aDO->return_sat = 0;
  //Serial.print("Data from DO sensor: ");
  
  
  uint32_t timeout_ms = millis() + 1000;
  while (!SerialDO.available()) { // wait for data up to timeout_ms
      delay(10);
      if ( millis() > timeout_ms ) break;
  }
  aDO->_sat = SerialDO.parseInt();
  aDO->_dox = SerialDO.parseFloat();
  return 0; // Doesn't really mean anything
  /*
  uint8_t idx = 0; char readChar;
  uint8_t arraySize = 20;
  char tempArray[arraySize];
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
  */
}
uint8_t getCompDO(struct DO_Struct *aDO, struct CondStruct *aCond, float temp_C){
  //int32_t temp = getTemperature();
  char buf[10];
  uint8_t sig_fig = 3;
  dtostrf(temp_C / 100,sig_fig,0,buf);
  if ( g_atlas_debug ) {
	  Serial.write(buf,sig_fig);
	  Serial.write(','); 
	  //Serial.print(aCond->ec);
	  Serial.write('0'); // Fresh water
	  Serial.println("<CR>");
  }
  SerialDO.write(buf,sig_fig);
  SerialDO.write(','); //Serial.print(',');
  //SerialDO.print(aCond->ec);// Serial.println(aCond->ec);
  SerialDO.write('0');// Fresh WATER
  SerialDO.write(13);
  SerialDO.write('R');
  SerialDO.write(13);
  return _getDO(aDO);
}
uint8_t getBasicDO(struct DO_Struct *aDO){
  SerialDO.print("R");SerialDO.print(13);
  return _getDO(aDO);
}
void toggleDOSat() {SerialDO.print('%');SerialDO.print(13);}
void quietDO() {
  Logger_SD::Instance()->msgL(DEBUG,"Setting DO mode to quiet");
  SerialDO.print('E');SerialDO.print(13);
}
void setDOContinuous() {SerialDO.print('C');SerialDO.print(13);}

void initCond(struct CondStruct *aCond){
  Logger_SD::Instance()->msgL(INFO,"Initializing Conductivity system.");
  pinMode(COND_POWER_PIN,OUTPUT);
  digitalWrite(COND_POWER_PIN,HIGH);
  SerialCond.begin(38400);
  // Clear out buffer.
  SerialCond.write(13);
  delay(300);
  while ( SerialCond.available() ) {
      SerialCond.read();
  }
  quietCond();
  //strncpy(aCond->ec,"0\0",2); // initialize conductivity to 0
}

uint8_t _getCond(struct CondStruct *aCond){
  /* Prompts for a value. Might be faster to put in continuous mode and just parse.
  */
  uint32_t timeout_ms = millis() + 1000;
  aCond->_ec = 0;
  aCond->_tds = 0;
  aCond->_sal = 0;
  aCond->_sg = 0;
  while (!SerialCond.available()) { // wait for data up to timeout_ms
    delay(10);
    if ( millis() > timeout_ms ) break;
  }
  /*
    Serial.println("COND R response:");
    while ( SerialCond.available() ) {
        charRead = SerialCond.read();
        Serial.write(charRead);
        if ( charRead == 13 ) Serial.write(10);
    }
  Serial.println("]"); */
  aCond->_ec = SerialCond.parseInt();
  aCond->_tds = SerialCond.parseInt();
  aCond->_sal = SerialCond.parseFloat();
  aCond->_sg = SerialCond.parseFloat();
    Serial.println("Conductivity values:");
    Serial.println(aCond->_ec);
    Serial.println(aCond->_tds);
    Serial.println(aCond->_sal);
    Serial.println(aCond->_sg);
	return 0; // Doesn't mean anything
  /*
  uint8_t field = 1; uint8_t idx = 0; char readChar;
  uint8_t arraySize = 20;
  char charRead;
  char tempArray[arraySize];
  while (SerialCond.available()){
    readChar = SerialCond.read();
    //Serial.write(readChar);
    if ( idx >= (arraySize - 1)) return 1; // Array to big
    if ( readChar == ' ')        return 2; // value includes space
    if ( readChar == ',' || readChar == 13 || readChar == 10) { 
      tempArray[idx] = '\0';
      if (field == 1 ) {
        memcpy(aCond->ec,tempArray,idx+1);
      }
      else if ( field == 2 ) {
        memcpy(aCond->tds,tempArray,idx+1);
      }
      else if (field == 3 ){
        memcpy(aCond->sal,tempArray,idx+1);
      }
      else if (field == 4 ){
	      memcpy(aCond->sg,tempArray,idx+1);
	      return 0; // Only correct way out.
      }
      field++; idx = 0; tempArray[idx] = 0;
      Serial.print("--->"); Serial.println(tempArray);
      if (readChar == 13 || readChar == 10) break;
    }
    else {
      tempArray[idx] = readChar;
      idx++;
      tempArray[idx] = 0;
    }
    if ( !SerialCond.available() ) delay(100); // give it 100 ms to see if another character is coming.
  }
  */
  //Serial.println();
  //return 3;  // Saw no characters
}

void setCondTemp(float temp_C){
	char buf[10];
    char charRead;
	uint8_t sig_fig = 4;
	dtostrf(temp_C / 100,sig_fig,1,buf);
    Serial.print("sending: T,"); Serial.write(buf,sig_fig); Serial.println("<CR>");
	SerialCond.write('T');
	SerialCond.write(',');
	Serial.write(buf,sig_fig);
	SerialCond.write(13);
    delay(1500);
    Serial.println("COND Temp Set response:");
    while ( SerialCond.available() ) {
        charRead = SerialCond.read();
        Serial.write(charRead);
        if ( charRead == 13 ) Serial.write(10);
    }
    Serial.println("]");
}

uint8_t getCond(struct CondStruct *aCond){
    char charRead;
    SerialCond.write('R');
    SerialCond.write(13);
    Serial.println("Get COND clearing:");
    while ( SerialCond.available() ) {
        charRead = SerialCond.read();
        Serial.write(charRead);
        if ( charRead == 13 ) Serial.write(10);
    }
    Serial.println("]");
    delay(1500);
    return _getCond(aCond);
}

void quietCond() {
    SerialCond.write('C');
    SerialCond.write(',');
    SerialCond.write('0');
    SerialCond.write(13);
    
}

void setCondContinuous() {
	SerialCond.write('C');
	SerialCond.write(',');
	SerialCond.write('1');
	SerialCond.write(13);
}

void DO_testing(struct DO_Struct *aDO, CondStruct *aCond,float temp_C){  
  // put your main code here, to run repeatedly:
  Serial.println("DO----------------");
  delay(500);
  aDO->comm_error = getCompDO(aDO,aCond,temp_C);
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
  aCond->comm_error = getCond(aCond);
  if (!aCond->comm_error) {
    Serial.print("EC: "); Serial.print(aCond->ec);
    Serial.print("uS/cm\r\nTDS: "); Serial.println(aCond->tds);
    Serial.print("SAL: "); Serial.println(aCond->sal);
  }
  else Serial.print(F("Conductivity Communications Error:")); Serial.println(aCond->comm_error,DEC);
}
