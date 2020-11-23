// Header file for i2c
//   Function prototypes

#ifndef I2C_DEFS_H
#define I2C_DEFS_H

#include <MKL25Z4.h>
//#include <stdbool.h>


void i2c_init(void) ;
void i2c_write_byte(uint8_t dev, uint8_t reg, uint8_t data) ;
uint8_t i2c_read_byte(uint8_t dev, uint8_t reg) ;
void i2c_read_bytes(uint8_t dev, uint8_t reg, uint8_t* data, int8_t count) ;


#endif
