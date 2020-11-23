// Header file for accelerometer
//   Function prototypes

#ifndef ACCEL_DEFS_H
#define ACCEL_DEFS_H

#include <MKL25Z4.h>

int initAccel(void) ;
void readXYZ(int16_t* accel) ;

#endif
