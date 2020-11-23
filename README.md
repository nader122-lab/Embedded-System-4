# Sample Answer for lab 4

The project uses a serial data link (over USB) to/from a terminal emulator running on the
connected PC / laptop.
  * The red and green LEDs alternate, with equal on times for both LEDs
  * The on time varies in 8 steps from 0.5s to 4s
  * A command (faster / slower) is entered at the series terminal on the laptop/PC terminal to control the on time
  * When the faster (slower) command is given on the fastest (slowest) time, the new time is the slowest (fastest).
  * The new time is acted on immediately: if the new delay has already expired when the command is entered, the LED changes immediately.
    Otherwise, the change occurs after the remainder of the new delay.

This behaviour is described by the following state-transition diagram:
    ![state transition model for lab 4 answer](stm-final.png)

The project uses:
 * Two threads
 * A message queue with a delay

In addition, the project includes the code for the serial interface. The API has two functions:
   1. `sendMsg` which write a message to the terminal
   2. `readLine` which reads a line from the terminal
