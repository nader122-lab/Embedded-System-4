
/*----------------------------------------------------------------------------
    Given code for Embedded Systems Lab 5 
    
    Poll the accelerometer and print x, y, and z axis values to to terminal

    There is one threads
       t_accel: polls the accelerometer every 2 seconds.
    
 *---------------------------------------------------------------------------*/
 
#include "cmsis_os2.h"
//#include "string.h"

#include <MKL25Z4.h>
#include <stdbool.h>
#include "serialPort.h"
#include "i2c.h"
#include "accel.h"



/*--------------------------------------------------------------
 *   Thread t_accel
 *      Read accelarations periodically  
 *--------------------------------------------------------------*/
osThreadId_t t_accel;      /* id of thread to poll accelerometer */

// convert signed integer +/- 999 to +/-X.XX
//   SX.XX
//   01234
void aToString(int16_t a, char* s) {  
    bool negative = false ;
    if (a < 0) {
        a = -a ;
        negative = true ; 
    }

    // get digits
    s[4] = '0' + (a % 10) ;
    a = a / 10 ;
    s[3] = '0' + (a % 10) ;
    a = a / 10 ;
    s[1] = '0' + (a % 10) ;
        
    // set sign
    s[0] = '+' ;
    if (negative) {
        s[0] = '-' ;
    }
}

// buffer for message
char xyzStr[] = "X=SX.XX Y=SX.XX Z=SX.XX" ;
//               01234567890123456789012

void accelThread(void *arg) {
    int16_t xyz[3] ; // array of values from accelerometer
        // signed integers in range +8191 (+2g) to -8192 (-2g)
    
    // initialise accelerometer
    int aOk = initAccel() ;
    if (aOk) {
        sendMsg("Accel init ok", CRLF) ;
    } else {
        sendMsg("Accel init failed", CRLF) ;
    }
    while(1) {
        osDelay(2000) ;
        readXYZ(xyz) ; // read X, Y Z values
        
        // write X scaled integer
        aToString((xyz[0] * 100) / 4096, &xyzStr[2]) ;
        
        // write Y scaled integer
        aToString((xyz[1] * 100) / 4096, &xyzStr[10]) ;
        
        // write Z scaled integer
        aToString((xyz[2] * 100) / 4096, &xyzStr[18]) ;
        sendMsg(xyzStr, CRLF) ;        
    }
}
    

/*----------------------------------------------------------------------------
 * Application main
 *   Initialise I/O
 *   Initialise kernel
 *   Create threads
 *   Start kernel
 *---------------------------------------------------------------------------*/

int main (void) { 
    
    // System Initialization
    SystemCoreClockUpdate();

    //configureGPIOinput();
    init_UART0(115200) ;

    // Initialize CMSIS-RTOS
    osKernelInitialize();
    
    // initialise serial port 
    initSerialPort() ;

    // Initialise I2C0 for accelerometer 
    i2c_init() ;
    
    // Create threads
    t_accel = osThreadNew(accelThread, NULL, NULL); 
 
    osKernelStart();    // Start thread execution - DOES NOT RETURN
    for (;;) {}         // Only executed when an error occurs
}
