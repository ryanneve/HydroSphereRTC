#ifndef HS_Atlas_h
#define HS_Atlas_h



#define SerialLight Serial1
#define SerialCond  Serial2
#define SerialDO    Serial3

/*----------( Structures )----------*/
typedef struct DO_Struct {
  bool    return_sat;
  char    dox[10]; // Ends in \0
  char    sample_time[7]; // HHmmss\0
  char    saturation[10]; // Ends in \0MEMS
  char    comm_error;
};

typedef struct Cond_Struct {
  char    tds[10];
  char    ec[10]; // Ends in \0
  char    sal[10]; // Ends in \0
  char    sample_time[7]; // HHmmss\0
  char    comm_error;
};

typedef struct Light_Struct {
  unsigned char mode; // 1 (RGB), 2 (lx) or 3 (RGB+lx)
  char          red[4];
  char          green[4];
  char          blue[4];
  char          lx_red[4];
  char          lx_green[4];
  char          lx_blue[4];
  char          lx_total[4]; // Should be = lx_red + lx_green + lx_blue + lx_non_vis.
  char          lx_beyond[4]; // lx beyond visible light spectrum
  bool          light_sat; // More than 3235 lx detected.
  char          sample_time[7]; // HHmmss\0
  char          comm_error;
};
/*----------( Function Prototypes )----------*/
void DO_setup();
unsigned char _DO_getReading(struct DO_Struct);
unsigned char DO_getCompReading(struct DO_Struct,struct ms5803PTStruct, struct Cond_Struct);
unsigned char DO_getBasicReading(struct DO_Struct);
void DO_toggle_sat();
void DO_quiet();
void DO_continuous();

void COND_setup();
unsigned char _COND_getReading(struct Cond_Struct);
unsigned char COND_getCompReading(struct ms5803PTStruct, struct Cond_Struct);
unsigned char COND_getBasicReading(struct Cond_Struct);
void COND_quiet();
void COND_continuous();

void LIGHT_setup(struct Light_Struct);
unsigned char LIGHT_getReading(struct Light_Struct);
void LIGHT_setMode(unsigned char,struct Light_Struct);
void LIGHT_quiet();
void LIGHT_continuous();

String gen_Atlas_log(bool DO, bool COND, bool LIGHT, struct DO_Struct *aDO,struct Cond_Struct *aCond,struct Light_Struct *aLight);

void DO_testing(struct DO_Struct *aDO,struct ms5803PTStruct *aTemp,struct Cond_Struct *aCond);

#endif