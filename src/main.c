/*----------------------------------------------------------------------------
    Given code for Embedded Systems Lab 5 
    
    Poll the accelerometer and print x, y, and z axis values to the terminal

    There is one thread
       t_accel: polls the accelerometer every 2 seconds.
    
 *---------------------------------------------------------------------------*/

#include "cmsis_os2.h"

#include <MKL25Z4.h>

#include <stdbool.h>

#include "gpio.h"

#include "serialPort.h"

#include "i2c.h"

#include "accel.h"

/*--------------------------------------------------------------
 *   Thread t_accel
 *      Read accelarations periodically  
 *      Write results to terminal
 *      Toggle green LED on each poll
 *--------------------------------------------------------------*/
osThreadId_t t_accel, t_green; /* id of thread to poll accelerometer */
osMessageQueueId_t controlIQ;
// convert signed integer +/- 999 to +/-X.XX
//   SX.XX
//   01234
//               01234567890123456789012

enum controlMsg_t {
  flat,
  right,
  left,
  down,
  up,
  over,
  error
}; // type for the messages

void accelThread(void * arg) {
  enum controlMsg_t msg;
  int16_t xyz[3]; // array of values from accelerometer
  // signed integers in range +8191 (+2g) to -8192 (-2g)
  int x, y, z;
  // initialise green LED
//  greenLEDOnOff(LED_ON);
  int state = LED_ON;

  // initialise accelerometer
  int aOk = initAccel();
  if (aOk) {
    sendMsg("Accel init ok", CRLF);
  } else {
    sendMsg("Accel init failed", CRLF);
  }
  while (1) {
    osDelay(200);
    readXYZ(xyz); // read X, Y Z values
    x = (xyz[0] * 100) / 4096;
    y = (xyz[1] * 100) / 4096;
    z = (xyz[2] * 100) / 4096;

    switch (state) {
    case INTERMEDIATE:
      if (z > 90) {
        sendMsg("FLAT", CRLF) ;
        state = FLAT;
        msg = flat;
        osMessageQueuePut(controlIQ, & msg, 0, NULL);
      } else if (y < -90) {
        sendMsg("RIGHT", CRLF) ;
        state = RIGHT;
        msg = right;
        osMessageQueuePut(controlIQ, & msg, 0, NULL);
      } else if (y > 90) {
        sendMsg("LEFT", CRLF) ;
        state = LEFT;
        msg = error;
        osMessageQueuePut(controlIQ, & msg, 0, NULL);
      } else if (x > 90) {
        sendMsg("DOWN", CRLF) ;
        state = DOWN;
        msg = error;
        osMessageQueuePut(controlIQ, & msg, 0, NULL);
      } else if (x < -90) {
        sendMsg("UP", CRLF) ;
        state = UP;
        msg = up;
        osMessageQueuePut(controlIQ, & msg, 0, NULL);
      } else if (z < -90) {
        sendMsg("OVER", CRLF) ;
        state = OVER;
        msg = error;
        osMessageQueuePut(controlIQ, & msg, 0, NULL);
      } else {}

      break;

    case FLAT:
      if (z < 80) {
        state = INTERMEDIATE;        
      }
      break;

    case RIGHT:
      if (y > -80) {
        state = INTERMEDIATE;
      }
      break;

    case LEFT:
      if (y < 80) {
        state = INTERMEDIATE;
      }
      break;

    case DOWN:
      if (x < 80) {
        state = INTERMEDIATE;
      }
      break;

    case UP:
      if (x > -80) {
        state = INTERMEDIATE;
      }
      break;

    case OVER:
      if (z > -80) {
        state = INTERMEDIATE;
      }
      break;

    }
  }
}

#define FIRST (1)
#define SECOND (2)
#define THIRD (3)
#define ERROR (6)
#define GREENON (5)
#define FOURTH (4)
void greenThread(void * arg) {
  int greenState = FIRST;
  enum controlMsg_t msg;
  osStatus_t status;
  uint32_t counter;
  uint32_t timer = osWaitForever;
  while (1) {
      counter = osKernelGetTickCount();
      status = osMessageQueueGet(controlIQ, & msg, NULL, timer);
      if (status == osOK) {
        
        if (msg == error) {
          greenState = ERROR;
        }
        counter = osKernelGetTickCount() - counter;

        switch (greenState) {

        case FIRST:
          if (msg == flat) {
            greenState = SECOND;
            timer = osWaitForever;
          } else {
            redLEDOnOff(LED_ON);
            greenState = ERROR;
          }
          break;

        case SECOND:
          if (counter > 10000) {
            if (msg == right) { 
            greenState = THIRD;
            timer = 6000;
            }
            else {
            redLEDOnOff(LED_ON);
            greenState = ERROR;
            }             
          } 
        else {
            redLEDOnOff(LED_ON);
            greenState = ERROR;
          }
          break;

        case THIRD:
          if (counter < 2000) {
           redLEDOnOff(LED_ON);
           greenState = ERROR;
          } else {
            
           if (msg == up) {
              greenState = FOURTH;
              timer = 8000;
            } 
           else {
              redLEDOnOff(LED_ON);
              greenState = ERROR;
            }
           }
          break;

          
        case FOURTH:
          if (counter < 4000) {
            redLEDOnOff(LED_ON);
            greenState = ERROR;
          } else {            
            if (msg == flat) {
              timer = osWaitForever;
              greenState = GREENON;
              greenLEDOnOff(LED_ON);
            } else {
              redLEDOnOff(LED_ON);
              greenState = ERROR;
            }
          }

          break;

        case GREENON:
          greenLEDOnOff(LED_ON);
          osDelay(osWaitForever);
          break;

        case ERROR:
          redLEDOnOff(LED_ON);
          break;

        }

      } else if (status == osErrorTimeout) {
        redLEDOnOff(LED_ON);
        greenState = ERROR;
      }


    }

  }



/*----------------------------------------------------------------------------
 * Application main
 *   Initialise I/O
 *   Initialise kernel
 *   Create threads
 *   Start kernel
 *---------------------------------------------------------------------------*/

int main(void) {

  // System Initialization
  SystemCoreClockUpdate();

  //configureGPIOinput();
  init_UART0(115200);

  // Initialize CMSIS-RTOS
  osKernelInitialize();

  // initialise serial port 
  initSerialPort();

  // Initialise I2C0 for accelerometer 
  i2c_init();

  // Initialise GPIo for on-board LEDs
  configureGPIOoutput();

  // Create message queue flags
 controlIQ = osMessageQueueNew(2, sizeof(enum controlMsg_t), NULL) ;
  // Create threads
  t_accel = osThreadNew(accelThread, NULL, NULL);
  t_green = osThreadNew(greenThread, NULL, NULL);
  osKernelStart(); // Start thread execution - DOES NOT RETURN
  for (;;) {} // Only executed when an error occurs
}