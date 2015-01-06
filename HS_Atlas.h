#ifndef HS_Atlas_h
#define HS_Atlas_h

#include <logger_SD.h>

#define SerialDO    Serial2
#define SerialCond  Serial3

#define DO_POWER_PIN    48
#define COND_POWER_PIN  46
// Log levels
#define DEBUG 0
#define INFO 1
#define WARN 2
#define ERROR 3
#define CRITICAL 4

/*----------( Forward Declarations (?) )----------*/
bool recentSample(char time[7], int16_t fresh_time);
/*----------( Structures )----------*/
//typedef struct DO_Struct {
struct DO_Struct {
	char _version;
	bool    return_sat;
	float   _sat;
	char    saturation[10]; // Ends in \0
	float   _dox;
	char    dox[10]; // Ends in \0
	char    sample_time[7]; // HHmmss\0
	char    comm_error;
};

//typedef struct CondStruct {
struct CondStruct {
	float   _ec;
	float   _tds;
	float   _sal;
	float   _sg;
	char    tds[10];
	char    ec[10]; // Ends in \0
	char    sal[10]; // Ends in \0
	char    sg[10]; // Ends in \0
	char    sample_time[7]; // HHmmss\0
	char    comm_error;
};

/*----------( Function Prototypes )----------*/
//logger Logger_SD;
void initDO();
void setDO_version(struct DO_Struct *aDO,char ver);// Either '5', '6', or 'E' for EZO
bool _getDO(struct DO_Struct *aDO);
bool getDO(struct DO_Struct *aDO);
void setDOtemp_cond(float temp_C,float cond);
bool getContDO(struct DO_Struct *aDO);
uint16_t clearDObuffer(bool verbose);
bool getCompDO(struct DO_Struct *aDO,struct CondStruct *aCond,float temp_C);
bool getBasicDO(struct DO_Struct *aDO);
void toggleDOSat();
void quietDO();
void setDOContinuous();

void initCond(struct CondStruct *aCond);
bool _getCond(struct CondStruct *aCond);
void   setCondTemp(float temp_C);
bool getCond(struct CondStruct *aCond);
void quietCond();
void setCondContinuous();

#endif


/*----------( Constants )----------*/
const char DO_SINGLE_SAMPLE[] = "R\n";
const char DO_QUIET[]         = "E\n";
const char DO_CONTINUOUS[]    = "C\n";