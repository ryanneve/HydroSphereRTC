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
	uint32_t start_millis = millis();
	uint16_t timeout = 1000;
	while (!SerialDO.available()) { // wait for data up to timeout_ms
		delay(10);
		if ((millis() - start_millis) >= timeout ) break;
	}
	float parsed_value = SerialDO.parseFloat();
	char terminator = SerialDO.peek();
	if ( terminator == ',' ) aDO->_sat = parsed_value;
	else if ( terminator == 13 )	aDO->_dox = SerialDO.parseFloat();
	if ( g_atlas_debug ) {
		Logger_SD::Instance()->msgL(DEBUG,F("DO sensor \%sat, DOx %f\%,%f"),aDO->_sat,aDO->_dox);
		Serial.println("DO sensor %sat, DOx ");
		Serial.print(aDO->_sat);
		Serial.print("%,");
		Serial.println(aDO->_dox);
	}
	// Now check to see if values are reasonable
	if ( aDO->_sat > 100 ) return 0; // Can't have < 100%
	return 1; // 1 = success. Need to check this better.
}
bool getDO(struct DO_Struct *aDO) {
	char terminator;
	bool result = 0;
	uint8_t bytes_returned;
	char buf[20];
	// clear previous values
	aDO->_sat = -1.0;
	aDO->_dox = -1.0;
	// clear buffer.
	clearDObuffer(true);
	// Send DO requests
	for (uint8_t i = 0 ; i < 4 ; i++){
		SerialDO.print(DO_SINGLE_SAMPLE);
		delay(200);
	}
	// read response
	delay(3000);
	terminator = ',';
	bytes_returned = SerialDO.readBytesUntil(terminator,buf,19);
	if ( bytes_returned ) {
		result = 1;
		buf[bytes_returned] = 0;
		aDO->_sat = atof(buf);
	}
	terminator = '\n';
	bytes_returned = SerialDO.readBytesUntil(terminator,buf,19);
	if ( bytes_returned ) {
		result = 1;
		buf[bytes_returned] = 0;
		aDO->_dox = atof(buf);
	}
	Logger_SD::Instance()->msgL(DEBUG,F("DO sensor \%sat, DOx %f\%,%f"),aDO->_sat,aDO->_dox);
	/*
	Serial.println("DO sensor %sat, DOx ");
	Serial.print(aDO->_sat);
	Serial.print("%,");
	Serial.println(aDO->_dox);
	Serial.println(aDO->_sat);
	Serial.println(aDO->_dox);
	*/
	return result;
}

void setDOtemp_cond(float temp_C,float cond){
	char buf1[10];
	char buf2[10];
	dtostrf(temp_C,6,2,buf1); // Convert temperature from float to char array with two decimal places.
	dtostrf(cond,6,2,buf2); // Convert conductivity from float to char array with two decimal places.
	if ( g_atlas_debug ) {
		Logger_SD::Instance()->msgL(DEBUG,F("setDOtemp_cond Setting Temperature and Conductivity: %s,%s"),buf1,buf2);
	}
	// Send command to DO sensor. Format is <temperature>
	SerialDO.write(buf1,6);
	SerialDO.write(',');
	SerialDO.write(buf2,6);
	SerialDO.write(13);	
}
bool getContDO(struct DO_Struct *aDO){
	// Get DO while in continuous mode
	// Clear out buffer.
	clearDObuffer(false);
	char terminator[] = "\n";
	// Wait for next <CR>
	SerialDO.setTimeout(3000);
	SerialDO.find(terminator);
	SerialDO.setTimeout(1000);
	// Wait for next data
	delay(1000);
	// Parse
	return _getDO(aDO);
}
uint16_t clearDObuffer(bool verbose){
	uint16_t cleared = 0;
	char byte_read;
	if ( verbose ) Logger_SD::Instance()->msgL(DEBUG,F("Clearing DO buffer."));
	while (SerialDO.available()) { // wait for data up to timeout_ms
		byte_read = SerialDO.read();
		if ( verbose ) Serial.print(byte_read);
		cleared++;
	}
	if ( verbose ) Serial.println();
	return cleared;
}
bool getCompDO(struct DO_Struct *aDO, struct CondStruct *aCond, float temp_C){
	char buf[10];
	//uint8_t sig_fig = 3;
	dtostrf(temp_C,6,2,buf); // Convert temperature from float to char array with two decimal places.
	/*
	if ( g_atlas_debug ) {
		Serial.println("getCompDO Setting Temperature:");
		Serial.print(buf);
		Serial.write(',');
		Serial.write('0'); // Fresh water otherwise Serial.print(aCond->ec);
		Serial.println("<CR>");
		Serial.write('R');
		Serial.println("<CR>");
	} */
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
	uint32_t start_millis = millis();
	uint16_t timeout = 1000;
	aCond->_ec = 0;
	aCond->_tds = 0;
	aCond->_sal = 0;
	aCond->_sg = 0;
	while (!SerialCond.available()) { // wait for data up to timeout ms
		delay(10);
		if ( (millis() - start_millis) >= timeout ) break;
	}
	aCond->_ec = SerialCond.parseFloat();
	aCond->_tds = SerialCond.parseFloat();
	aCond->_sal = SerialCond.parseFloat();
	aCond->_sg = SerialCond.parseFloat();
	// Save as strings
	dtostrf(aCond->_ec,8,2,aCond->ec);
	dtostrf(aCond->_tds,8,2,aCond->tds);
	dtostrf(aCond->_sal,8,2,aCond->sal);
	dtostrf(aCond->_sg,8,2,aCond->sg);
	// Log values
	Logger_SD::Instance()->msgL(INFO,F("Conductivity values: EC, TDS, SAL, SG %s,%s,%s,%s"),
		aCond->ec,aCond->tds,aCond->sal,aCond->sg);
}
void setCondK(char *ec_k) {
	// Set probe K value. Can be 0.1, 1.0, 10.0
	if ( ec_k[0] != 0 ) {// There's something
		SerialCond.print("k,");
		SerialCond.println(ec_k);
	}
}
void setCondTemp(float temp_C){
	char buf[50];
	char charRead;
	uint8_t sig_fig = 5;
	dtostrf(temp_C,sig_fig,2,buf);
	Logger_SD::Instance()->msgL(INFO,F("Setting EC temperature to %s."),buf);
	SerialCond.print("T,");
	SerialCond.print(buf);
	SerialCond.write(13); // CR
	delay(1500);
	// Response:
	int16_t i = 1;
	while ( SerialCond.available() ) {
		charRead = SerialCond.read();
		buf[i] = charRead;
		i++;
		if ( i >= 48 ) break;
		//Serial.write(charRead);
		if ( charRead == 13 )  {
		  buf[i] = 10;
		  i++;
		}
		delay(50); // Time for another character...
	}
	buf[i] = 0; // null terminate
	Logger_SD::Instance()->msgL(INFO,F("HS_Atlas.setCondTemp response: [%s]."),buf);
}

bool getCond(struct CondStruct *aCond){
	char charRead;
	Serial.print("f(getCond clearing buffer: [");
	while ( SerialCond.available() ) {
		charRead = SerialCond.read();
		if ( charRead == 13 ) Serial.print("<CR>");
		else Serial.write(charRead);
	}
	Serial.println(F("]\r\nRequesting Conductivity data"));
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
