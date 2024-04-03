/*
    Date: 03/18/2024
    Authors: Aidan Cowan, Brian Primo
    Group: 18

    Lab Info/Description:
  The purpose of this lab is to create a drive in a circle function
  that takes in four inputs; left speed, right speed, left wheel distance
  right wheel distance. 

  The main loop should drive forward 18 inches, then rotate 45 degrees CCW
  drive forward 18 inches, rotate 90 degrees CCW, and then drive in a circle
  for 270 degrees, rotate 90 degrees CCW, drive forward 18 inches,
  rotate 45 degrees CCW, and finally drive forward 18 inches and end up
  at the starting position

  The robot should trace out a 'peace sign' shape
*/

#include "SimpleRSLK.h"

#define DELAY_MS            2000   // delay in milliseconds

// Default pwm signals (percentage-% of power 0-100) for both RSLK motor.
// Change these values as needed
#define LEFT_MOTOR_SPEED     11   // Speed percentage
#define RIGHT_MOTOR_SPEED    12   // Speed percentage
#define LEFT_TURN_SPEED      6    // Speed percentage
#define RIGHT_TURN_SPEED     7    // Speed percentage

// Value for turning directions (do not change)
#define CCW                  1     // rotate robot counter clockwise
#define CW                   2     // rotate robot clockwise

// RSLK specified mechanics (do not change)
#define wheelDiameter       6.999         // in centimeters
#define cntPerRevolution    360           // Number of encoder (rising) pulses every time the wheel turns completely

#define radius              45.7200       // in centimeters
#define leftWheelDistance   180      // in centimeters
#define rightWheelDistance  240      // in centimeters

/* Place code here to only run once ***********************************************/
void setup() {
  //Do not edit this setup function
  Serial.begin(9600);         // Start serial com at 9600 baud rate
  setupRSLK();                // Initialize RSLK functions and classes
  setupWaitBtn(LP_LEFT_BTN);  // Setup left button on Launchpad
  setupLed(RED_LED);          // Red led in rgb led
  setupLed(GREEN_LED);        // Green led in rgb led
  setupLed(BLUE_LED);         // Blue led in rgb led
}

/* Place code here to run forever/loop/repeat *************************************/
void loop() {
  startProgram();   // wait until left MSP 432 button is pressed to start program
//
//  forward(45);
//  rotate(CCW, 45);
//  forward(45);
//  rotate(CCW, 90);
  driveInCircle(15, 16, leftWheelDistance, rightWheelDistance);
//  rotate(CCW, 90);
//  forward(45);
//  rotate(CCW, 45);
//  forward(45);
//  rotate(CCW, 180);
  
  blinkBlueLED(2000);
  blinkBlueLED(2000);
  blinkBlueLED(2000);
  
}
/* Function Name: driveInCircle
   Input: 4 Input Variables, left wheel speed, right wheel speed,
          left wheel travel distance, right wheel travel distance
   Details: Function called to drive in a circle given a specified 
            left and right wheel speed and distance 
 */
 void driveInCircle(int leftSpeed, int rightSpeed, float leftDistance, float rightDistance){
  
  uint32_t leftTargetPulse = countForDistance(wheelDiameter, cntPerRevolution, leftDistance);   // Total amount of left encoder pulses received
  uint32_t rightTargetPulse = countForDistance(wheelDiameter, cntPerRevolution, rightDistance); // Total amount of right encoder pulses received

  resetEncoderCnts(); // Reset both encoder counts

  float motorWheelRatio = (radius + wheelDiameter) / (radius - wheelDiameter);

  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  
  setMotorSpeed(LEFT_MOTOR, leftSpeed);
  setMotorSpeed(RIGHT_MOTOR, rightSpeed);

  uint16_t leftPulse = getEncoderLeftCnt();
  uint16_t rightPulse = getEncoderRightCnt();

  while(leftPulse < leftTargetPulse || rightPulse < rightTargetPulse) {
    leftPulse = getEncoderLeftCnt();
    rightPulse = getEncoderRightCnt();

    if(rightPulse  > (leftPulse*motorWheelRatio)){
      setMotorSpeed(RIGHT_MOTOR, rightSpeed-1);
    }
    if(leftPulse > (rightPulse/motorWheelRatio)){
      setMotorSpeed(LEFT_MOTOR, leftSpeed-1);
    }
    
  }
  disableMotor(BOTH_MOTORS);
  delay(DELAY_MS);
 }

/* Function Name: rotate
   Input: 2 Input Variables, rotational direction and rotational degree as integers
   Details: Function called to rotate in place given a specified direction (CW or CCW)
            and degree to rotate to. 
*/
void rotate(int rotate_dir, int rotate_deg) {

  uint16_t leftPulse = 0; // Total amount of left encoder pulses received
  uint16_t rightPulse = 0;  // Total amount of right encoder pulses received

  float distance = (float(rotate_deg) / 360) * (14 * PI); // Convert degrees to distance
  uint32_t totalPulses = countForDistance(wheelDiameter, cntPerRevolution, distance);

  resetEncoderCnts(); // Reset both encoder counts

  /* Set motor direction based upon the rot_dir parameter */
  if (rotate_dir == CW) {
    enableMotor(BOTH_MOTORS);                          // "Turn on" the motor
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);  // Set Left Motor to drive forward
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD); // Set Right Motor to drive backward
  } else {
    enableMotor(BOTH_MOTORS);                          // "Turn on" the motor
    setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD); // Set Right Motor to drive forward
    setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD); // Set Left Motor to drive backward
  }

  setMotorSpeed(LEFT_MOTOR, LEFT_TURN_SPEED);
  setMotorSpeed(RIGHT_MOTOR, RIGHT_TURN_SPEED);

  while (leftPulse < totalPulses && rightPulse < totalPulses) {    // Check Encoders against target pulse count
    leftPulse = getEncoderLeftCnt();                               // Get Left Encoder Count
    rightPulse = getEncoderRightCnt();                             // Get Right Encoder Count

    /* If the left encoder count is less than the right, increase
       the left motor speed */
    if (leftPulse < rightPulse) {
      setMotorSpeed(LEFT_MOTOR, LEFT_TURN_SPEED + 1);
      setMotorSpeed(RIGHT_MOTOR, RIGHT_TURN_SPEED);
    }

    /* If the right encoder count is less than the left, increase
       the right motor speed */
    if (rightPulse < leftPulse) {
      setMotorSpeed(LEFT_MOTOR, LEFT_TURN_SPEED);
      setMotorSpeed(RIGHT_MOTOR, RIGHT_TURN_SPEED + 1);
    }
  }
  disableMotor(BOTH_MOTORS);
  delay(DELAY_MS);
}

/* Function Name: forward
   Input: 1 Input variable, travel distance as a floating point.
   Details: Function called to drive forward at a constant speed
            and at a specified distance in centimeters on a 
            straight linear line.
*/
void forward(float travel_dist) {

  resetEncoderCnts(); // Reset both encoder counts
  uint16_t leftPulse = 0;   // Total amount of left encoder pulses received
  uint16_t rightPulse = 0;  // Total amount of right encoder pulses received
  uint32_t totalPulses = countForDistance(wheelDiameter, cntPerRevolution, travel_dist);

  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);

  setMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED);
  setMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED);
  
  while (leftPulse < totalPulses && rightPulse < totalPulses) {    // Check Encoders against target pulse count
    leftPulse = getEncoderLeftCnt();                               // Get Left Encoder Count
    rightPulse = getEncoderRightCnt();                             // Get Right Encoder Count

    /* If the left encoder count is less than the right, increase
       the left motor speed by 1 */
    if (leftPulse < rightPulse) {
      setMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED + 1);
      setMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED);
    }
    
    /* If the right encoder count is less than the left, increase
       the right motor speed by 2 */
    if (rightPulse < leftPulse) {
      setMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED);
      setMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED + 1);
    }
  }
  disableMotor(BOTH_MOTORS);
  delay(DELAY_MS);
}
/* Function Name: resetEncoderCnts
   Input:  void
   Details: Function called to reset both left
            and right motor encoder counts.
*/
void resetEncoderCnts() {
  resetLeftEncoderCnt();  // Reset Left Encoder Count
  resetRightEncoderCnt(); // Reset Right Encoder Count
}
/* Function Name: countForDistance
   Input: 3 input variables
   Return: int value
   Details: Function called to calculate the number of pulses need to travel a specified
            distance by the user input variable "distance."
*/
uint32_t countForDistance(float wheel_diam, uint16_t cnt_per_rev, float distance) {
  float temp = (wheel_diam * PI) / cnt_per_rev;
  temp = distance / temp;
  return int(temp);
}
/* Function Name: startProgram
   Input: void
   Details: Function called to wait for a button to be pressed
            in order to start the robot program
*/
void startProgram() {
  /* Setup message to print to serial montor */
  String btnMsg = "Push left button on Launchpad to start lab program.\n";
  waitBtnPressed(LP_LEFT_BTN, btnMsg, RED_LED); //Setup button, msg, LED
  /* Using an LED as a DELAY */
  blinkLED(1000); //Cause LED to blink for a period of one second
  blinkLED(1000); //Cause LED to blink for a period of one second
}

/* Function Name: blinkLED
   Input: integer (period) in milliseconds
   Details: Function call that will blink a colored LED for a period specified.
*/
int blinkLED(int period) {
  int pause = period / 2;         // Determine on/off time
  digitalWrite(GREEN_LED, HIGH);  // Turn LED on
  delay(pause);                   // Time the LED is on
  digitalWrite(GREEN_LED, LOW);   // Turn LED off
  delay(pause);                   // Time LED is off
}

/* Function Name: blinkBlueLED
   Input: integer (period) in milliseconds
   Details: Function call that will blink the Blue LED for a period specified.
*/
int blinkBlueLED(int period) {
  int pause = period / 2;         // Determine on/off time
  digitalWrite(BLUE_LED, HIGH);  // Turn LED on
  delay(pause);                   // Time the LED is on
  digitalWrite(BLUE_LED, LOW);   // Turn LED off
  delay(pause);                   // Time LED is off
}