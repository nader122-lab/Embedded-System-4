// accelerometer interface 

#include <MKL25Z4.h>
#include <stdbool.h>
#include "i2c.h"
#include "accel.h"


#define MMA8451_I2C_A_BASE (0x1d)  // MMA8451Q base address
#define MMA8451_I2C_ADDRESS (0x3a) // id << 1: 0001 1101 -> 0011 1010 0x3A
#define MMA8451_ID (0x1a)      // MMA8451Q ID

#define REG_WHO_AM_I      (0x0D)
#define REG_CTRL_REG_1    (0x2A)
#define REG_OUT_X_MSB     (0x01)
#define REG_OUT_Y_MSB     (0x03)
#define REG_OUT_Z_MSB     (0x05)

// horrible delay routine
void Delay(volatile unsigned int d) {
    while (d--) ;
}

/* -------------------------------
   ------------------------------- */
int initAccel() {
    uint8_t whoami = i2c_read_byte(MMA8451_I2C_ADDRESS, REG_WHO_AM_I) ;
    // should be 26
    
    if (whoami == MMA8451_ID) {
        Delay(40) ;  // no idea why we need this 
        
        // set active mode 
        i2c_write_byte(MMA8451_I2C_ADDRESS, REG_CTRL_REG_1, 0x1) ;
        return 1 ;
    }
    return 0 ;
}

/* -------------------------------
    readXYX: read the X, Y and Z acelerations

   Default mode is +/- 2g, using 14 bit signed integer
   Range is 8191 to -8192
   1g is 4096

   ------------------------------- */
#define TWO14 (16384)     // 2^14 
#define INT14_MAX (8191)  // 2^13 - 1

void readXYZ(int16_t* accel) {
    int i ;
    uint8_t data[6] ;
    
    i2c_read_bytes(MMA8451_I2C_ADDRESS, REG_OUT_X_MSB, data, 6) ;
    
    for (i=0; i<3; i++) {
        // combine data bytes
        accel[i] = (data[2*i]<<6) | (data[2*i+1] >> 2) ;
        // allow for negative
        if (accel[i] > INT14_MAX) accel[i] -= TWO14 ;
    }
}
