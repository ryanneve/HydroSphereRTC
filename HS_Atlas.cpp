/*Functionality  for Atlas Scientific's DO 6.0 and EZO EC Conductivity sensors.
Will need to support EZO pH and EZO DO
*/
#include <arduino.h>
#include <Logger_SD.h> // for Logger_SD::Instance()->msgL()
#include "HS_Atlas.h"
#include "HS_OpenROV.h" // for structure with Temperature

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


void setDO_version(struct DO_Struct *aDO,char ver) {
	aDO->_version = ver;
} // Either '5', '6', or 'E' for EZO


bool _getDO(struct DO_Struct *aDO){
	/* Prompts for a value. Might be faster to put in continuous mode and just parse.
	*/
	delay(1000);
	aDO->return_sat = 0;
	//Serial.print("Data from DO sensor: ");
	
	
	uint32_t timeout_ms = millis() + 1000;
	while (!SerialDO.available()) { // wait for data up to timeout_ms
		delay(10);
		if ( millis() > timeout_ms ) break;
	}
	aDO->_sat = SerialDO.parseFloat();
	aDO->_dox = SerialDO.parseFloat();
	if ( g_atlas_debug ) {
		Serial.println("DO sensor %sat, DOx ");
		Serial.print(aDO->_sat);
		Serial.print("%,");
		Serial.println(aDO->_dox);
	}
	return 1; // 1 = success. Need to check this better.
}
bool getCompDO(struct DO_Struct *aDO, struct CondStruct *aCond, float temp_C){
	char buf[10];
	//uint8_t sig_fig = 3;
	dtostrf(temp_C,6,2,buf); // Convert temperature from float to char array with two decimal places.
	if ( g_atlas_debug ) {
		Serial.println("getCompDO Setting Temperature:");
		Serial.print(buf);
		Serial.write(',');
		Serial.write('0'); // Fresh water otherwise Serial.print(aCond->ec);
		Serial.println("<CR>");
		Serial.write('R');
		Serial.println("<CR>");
	}
	// Send command to DO sensor. Format is <temperature>
	SerialDO.write(buf,6);
	SerialDO.write(',');
	SerialDO.write('0');// Fresh WATER
	SerialDO.write(13);
	SerialDO.write('R');
	SerialDO.write(13);
	return _getDO(aDO);
}
bool getBasicDO(struct DO_Struct *aDO){
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

bool _getCond(struct CondStruct *aCond){
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
	aCond->_ec = SerialCond.parseFloat();
	aCond->_tds = SerialCond.parseFloat();
	aCond->_sal = SerialCond.parseFloat();
	aCond->_sg = SerialCond.parseFloat();
	Serial.println("Conductivity values: EC, TDS, SAL, SG");
	Serial.print(aCond->_ec);  Serial.write(',');
	Serial.print(aCond->_tds); Serial.write(',');
	Serial.print(aCond->_sal); Serial.write(',');
	Serial.print(aCond->_sg);  Serial.println();
	return 1; // Doesn't mean anything
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

bool getCond(struct CondStruct *aCond){
	char charRead;
	Serial.print("getCond clearing buffer: [");
	while ( SerialCond.available() ) {
		charRead = SerialCond.read();
		if ( charRead == 13 ) Serial.print("<CR>");
		else Serial.write(charRead);
	}
	Serial.println("]\r\nRequesting data");
	SerialCond.write('R');
	SerialCond.write(13);
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
