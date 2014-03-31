
#ifndef HydroSphere_h
#define HydroSphere_h
#include <arduino.h>
#include "HS_Atlas.h"
#include "HS_OpenROV.h"


#define SERIAL_BUFFER_SIZE 128


/*----------( Structures )----------*/
typedef struct Temperature_Struct { // THis will move to HS_OpenROV.h eventually
  char    temp_C[5]; // XX.XX"
  char    sample_time[7]; // HHmmss\0
};

/*----------( Function Prototypes )----------*/

#endif