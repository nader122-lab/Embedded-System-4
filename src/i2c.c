
/* ======================================================
      I2C 0 interface for Accelerometer
      
     Based on textbook 
   ====================================================== */

//#include "cmsis_os2.h"
#include <MKL25Z4.h>
#include <stdbool.h>
#include "i2c.h"

#define SDApin (25) // port E
#define SCLpin (24) // port E


void i2c_init(void) {
    // clock the i2c peripheral
    SIM->SCGC4 |= SIM_SCGC4_I2C0_MASK ;
    
    // clock port E
    SIM->SCGC5 |= SIM_SCGC5_PORTE_MASK ;

    // set pins to I2C function
    PORTE->PCR[SCLpin] |= PORT_PCR_MUX(5) ;
    PORTE->PCR[SDApin] |= PORT_PCR_MUX(5) ;
    
    //set 400K baud
    // baud = bus_freq / (scl_div * mul)
    // bus is 10.48 MHz
    // SCL of 28 give 375 KHz
    I2C0->F = I2C_F_ICR(0x4) | I2C_F_MULT(0) ;
    
    // enable and set to master mode
    I2C0->C1 |= I2C_C1_IICEN_MASK ;
    
    // select high drive mode 
    I2C0->C2 |= I2C_C2_HDRS_MASK ;
}

static inline void startI2C() { I2C0->C1 |= I2C_C1_MST_MASK ; }
static inline void stopI2C() { I2C0->C1 &= ~I2C_C1_MST_MASK ; } 
static inline void rstartI2C() { I2C0->C1 |= I2C_C1_RSTA_MASK ; }
static inline void transI2C() { I2C0->C1 |= I2C_C1_TX_MASK ; }
static inline void rcvI2C() { I2C0->C1 &= ~I2C_C1_TX_MASK ; }
static inline void waitI2C() { 
    while ((I2C0->S & I2C_S_IICIF_MASK) == 0) ;
    I2C0->S |= I2C_S_IICIF_MASK ;
}
static inline void nackI2C() { I2C0->C1 |= I2C_C1_TXAK_MASK ; }
static inline void ackI2C() { I2C0->C1 &= ~I2C_C1_TXAK_MASK ; }

void i2c_write_byte(uint8_t dev, uint8_t reg, uint8_t data) {
    
    // transmit mode and start
    transI2C() ;
    startI2C() ;
    
    // device address
    I2C0->D = dev ;
    waitI2C() ;

    // register address    
    I2C0->D = reg ;
    waitI2C() ;
    
    // send data
    I2C0->D = data ;
    waitI2C() ;
    
    // stop
    stopI2C() ;
   
}

uint8_t i2c_read_byte(uint8_t dev, uint8_t reg) {
    uint8_t data ;
    
    // transmit mode and start
    transI2C() ;
    startI2C() ;
    
    // device address
    I2C0->D = dev ;
    waitI2C() ;

    // register address    
    I2C0->D = reg ;
    waitI2C() ;
    
    // repeated start 
    rstartI2C() ;
    I2C0->D = (dev | 0x1) ;
    waitI2C() ;
    
    // receive mode
    rcvI2C() ;
    nackI2C() ;
    
    // dummy read
    data = I2C0->D ;
    waitI2C() ;
    
    // read and stop
    stopI2C() ;
    data = I2C0->D ;
    
    return data ;
}

/* -----------------------------------
     Read multiple bytes
   
   From sequential registers in I2C device

   dev  device address
   reg  start register adddress
   data array for data read
   count number of bytes
   ----------------------------------- */
void i2c_read_bytes(uint8_t dev, uint8_t reg, uint8_t* data, int8_t count) {
    uint8_t dummy = 0;
    
    // transmit mode and start
    transI2C() ;
    startI2C() ;
    
    // device address
    I2C0->D = dev ;
    waitI2C() ;

    // register address    
    I2C0->D = reg ;
    waitI2C() ;
    
    // repeated start 
    rstartI2C() ;
    I2C0->D = (dev | 0x1) ;
    waitI2C() ;
    
    // receive mode with ACK
    rcvI2C() ;
    ackI2C() ; 
    
    // dummy read 
    dummy = I2C0->D ;
    waitI2C() ;
    
    // read bytes except last
    int num = 0 ;
    while (num < count-1) {
        ackI2C() ;
        data[num++] = I2C0->D ;
        waitI2C() ;
    }
    
    // read last
    nackI2C() ;
    data[num] = I2C0->D ;
    waitI2C() ;
    stopI2C() ;
}



