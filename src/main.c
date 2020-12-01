

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
osThreadId_t t_accel, t_green; /* id of thread to poll accelerometer and light the green LED */
osMessageQueueId_t controlIQ; // id for message queue 
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
  int16_t xyz[3]; // array of values from accelerometer, signed integers in range +8191 (+2g) to -8192 (-2g)
  int x, y, z; 
  int state = INTERMEDIATE; // initial state

  // initialise accelerometer
  int aOk = initAccel();
  if (aOk) {
    sendMsg("Accel init ok", CRLF);
  } else {
    sendMsg("Accel init failed", CRLF);
  }
  while (1) {
    osDelay(150); // accelerometer polling delay
    readXYZ(xyz); // read X, Y Z values
    x = (xyz[0] * 100) / 4096; // holds acceleration value in the X axis
    y = (xyz[1] * 100) / 4096; // holds acceleration value in the Y axis
    z = (xyz[2] * 100) / 4096; // holds acceleration value in the Z axis 

    switch (state) { // determines the orientation of the development board
    case INTERMEDIATE:
      if (z > 90) { 
        sendMsg("FLAT", CRLF);
        state = FLAT;
        msg = flat;
        osMessageQueuePut(controlIQ, & msg, 0, NULL); // send message if flat
      } else if (y < -90) {
        sendMsg("RIGHT", CRLF);
        state = RIGHT;
        msg = right;
        osMessageQueuePut(controlIQ, & msg, 0, NULL); // send message if right
      } else if (y > 90) {
        sendMsg("LEFT", CRLF);
        state = LEFT;
        msg = error;
        osMessageQueuePut(controlIQ, & msg, 0, NULL); // send message if left
      } else if (x > 90) {
        sendMsg("DOWN", CRLF);
        state = DOWN;
        msg = error;
        osMessageQueuePut(controlIQ, & msg, 0, NULL); // send message if down
      } else if (x < -90) {
        sendMsg("UP", CRLF);
        state = UP;
        msg = up;
        osMessageQueuePut(controlIQ, & msg, 0, NULL); // send message if up
      } else if (z < -90) {
        sendMsg("OVER", CRLF);
        state = OVER;
        msg = error;
        osMessageQueuePut(controlIQ, & msg, 0, NULL); // send message if over
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

// states
#define FIRST (1)
#define SECOND (2)
#define THIRD (3)
#define ERROR (6)
#define GREENON (5)
#define FOURTH (4)
void greenThread(void * arg) {
  int greenState = FIRST;
  enum controlMsg_t msg;
  osStatus_t status; // returned by Message Queue Get
  uint32_t counter;
  uint32_t timer = osWaitForever; // initialises MessageQueueGet wait time
  while (1) {
    counter = osKernelGetTickCount(); // current value of Kernal count
    status = osMessageQueueGet(controlIQ, & msg, NULL, timer); // waits for message from queue
    if (status == osOK) { // message received 

      if (msg == error) { // wrong message received
        greenState = ERROR;
      }
      counter = osKernelGetTickCount() - counter; // determines time passed before message received

      switch (greenState) {

      case FIRST:
        if (msg == flat) {
          greenState = SECOND;   // next state
          timer = osWaitForever; // new timer value
        } else {                 // error occured   
          redLEDOnOff(LED_ON);   // turn red LED on
          greenState = ERROR;    // next state
        }
        break;

      case SECOND:
        if (counter > 10000) {
          if (msg == right) {
            greenState = THIRD;  // next state
            timer = 6000;        // new timer value
          } else {               // error occured
            redLEDOnOff(LED_ON); // turn red LED on
            greenState = ERROR;  // next state
          }
        } else {                 // error occured
          redLEDOnOff(LED_ON);   // turn red LED on 
          greenState = ERROR;    // next state 
        }
        break;

      case THIRD:
        if (counter < 2000) {    // error occured
          redLEDOnOff(LED_ON);   // turn red LED on
          greenState = ERROR;    // next state
        } else {                 // counter value correct
          if (msg == up) {       // message correct
            greenState = FOURTH; // next state
            timer = 8000;        // new timer value
          } else {               // error occured
            redLEDOnOff(LED_ON); // turn red LED on
            greenState = ERROR;  // next state
          }
        }
        break;

      case FOURTH:
        if (counter < 4000) {      // error occured
          redLEDOnOff(LED_ON);     // turn red LED on 
          greenState = ERROR;      // next state
        } else {                   // counter value correct 
          if (msg == flat) {       // message correct
            timer = osWaitForever; // new timer value
            greenState = GREENON;  // next state
            greenLEDOnOff(LED_ON); // turn green 
          } else {                 // error occured 
            redLEDOnOff(LED_ON);   // turn red LED on
            greenState = ERROR;    // next state
          }
        }

        break;

      case GREENON:
        osDelay(osWaitForever);            // stay in this state forever 
        break;

      case ERROR:
        osDelay(osWaitForever);            // stay in this state forever
        break;

      }

    } else if (status == osErrorTimeout) { // timer finished
      redLEDOnOff(LED_ON);                 // turn red lED on
      greenState = ERROR;                  // next state
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
  controlIQ = osMessageQueueNew(2, sizeof(enum controlMsg_t), NULL);
  // Create threads
  t_accel = osThreadNew(accelThread, NULL, NULL);
  t_green = osThreadNew(greenThread, NULL, NULL);
  osKernelStart(); // Start thread execution - DOES NOT RETURN
  for (;;) {} // Only executed when an error occurs
}