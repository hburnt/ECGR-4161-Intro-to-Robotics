/*
    Date: 04/02/2024
    Authors: Aidan Cowan, Hunter Burnett
    Group: 18

    Lab 7 Info/Description:


    Requirements:
    1. Robot will be placed into the maze facing the wall
    2. Robot's program should begin two seconds after a button press,signified by a flash RED LED.
    3. Robot must have an ultrasonic sensor (attached to servo in the middle of the robot)
    4. Ultrasonc Trigger and Echo should be attached to pin 32 and pin 33 on the MSP432
    5. Robot shall only use an ultrasonic sensor, LEDs, and wheel encoders
    6. Robot must drive in a straight line and turn in place
    7. Robot must scan its environment and navigate to the larger room
    8. Robot must not touch any walls, and localize to the center of the larger room
    9. Robot must flash a green LED when its program has finished
   10. Robot use present an algorithm that scans for obstalcles in all directions to determine best route
   11. Robot must only use three movement functions: forward, backward, and turn-in-place
   12. Demo should not exceed 90 seconds
*/

#include "SimpleRSLK.h"

// RSLK specified mechanics (do not change)
#define wheelDiameter       6.999    // in centimeters
#define cntPerRevolution    360      // Number of encoder (rising) pulses every time the wheel turns completely

void setup() {
  // put your setup code here, to run once:

}

void loop() {
  // put your main code here, to run repeatedly: 
  
}
