# Given code for lab 5

The project uses a serial data link (over USB) to a terminal emulator running on the
connected PC / laptop.

  * The accelerometer is polled every 150 milliseconds
  * The red LED is turned on if an error is detected
  * The green LED is turned on after the implementation of the state transition diagram

The project uses two threads.

The project includes the code for the serial interface. This API has two functions:
   1. `sendMsg` which write a message to the terminal
   2. `readLine` which reads a line from the terminal (not used here)

The project includes code for the accelerometer. This API has two functions:
   1. `initAccel` initialises the accelerometer
   2. `readXYZ` reads the three accelerations

The lab requirements include an implementation of the following state transition diagram:

![state transition models of the tasks in lab 2](stm.png)

