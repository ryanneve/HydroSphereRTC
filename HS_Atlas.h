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
  bool    return_sat;
  float   _dox;
  uint16_t _sat; 
  char    dox[10]; // Ends in \0
  char    sample_time[7]; // HHmmss\0
  char    saturation[10]; // Ends in \0MEMS
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
uint8_t _getDO(struct DO_Struct *aDO);
uint8_t getCompDO(struct DO_Struct *aDO,struct CondStruct *aCond,float temp_C);
uint8_t getBasicDO(struct DO_Struct *aDO);
void toggleDOSat();
void quietDO();
void setDOContinuous();

void initCond(struct CondStruct *aCond);
uint8_t _getCond(struct CondStruct *aCond);
void   setCondTemp(float temp_C);
uint8_t getCond(struct CondStruct *aCond);
void quietCond();
void setCondContinuous();

void DO_testing(struct DO_Struct *aDO,struct CondStruct *aCond,float temp_C);

#endif