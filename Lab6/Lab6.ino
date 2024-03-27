/*
    Date: 03/23/2024
    Authors: Aidan Cowan, Hunter Burnett
    Group: 18

    Lab Info/Description:
    A tech company has reached out and asked for a robot with the capabilities 
    to localize itself with few external sensors. The robot must be able to
    travel forward and backwards, rotate in-place to the left or right. Localize
    itself based on which front bump sensors are contacted and set itself 
    perpendicular to the object. The robot must be able to localize itself to
    the center of the arena and stop.

    Requirements:
    1. Robot shall be placed into the arena at a random location and angle
    2. Robot's program shall begin two seconds after a button press, signified by a flashing blue LED.
    3. After mapping the room, the robot should be able to figure out the size of the room
    4. Robot should then localize itself in the center of the room and stop
    5. To show the robot has finished, a red LED should light up
    6. All units of measurement should be in centimeters
    7. The programs code should only consist of driving straight, backing up, and rotating in-place
    8. Three 'optional' functions may be used blining a LED, stopping the robot, and calculating pulses to travel
    9. Main loop should only consist of movement functions, for easy readability for the company
    10. The demonstration shouldn't last longer than 90 seconds
    11. A localization measurement will be taken by the company for robot performance. Best performance wins
*/

#include "SimpleRSLK.h"

#define DELAY_MS            2000   // delay in milliseconds

// Default pwm signals (percentage-% of power 0-100) for both RSLK motor.
// Change these values as needed
#define LEFT_MOTOR_SPEED     12   // Speed percentage (Originally 11)
#define RIGHT_MOTOR_SPEED    12   // Speed percentage (Originally 12)
#define LEFT_TURN_SPEED      8    // Speed percentage (Originally 6)
#define RIGHT_TURN_SPEED     8    // Speed percentage (Originally 7)

// Value for turning directions (do not change)
#define CCW                  1     // rotate robot counter clockwise
#define CW                   2     // rotate robot clockwise

// RSLK specified mechanics (do not change)
#define wheelDiameter       6.999    // in centimeters
#define cntPerRevolution    360      // Number of encoder (rising) pulses every time the wheel turns completely

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);         // Start serial com at 9600 baud rate
  setupRSLK();                // Initialize RSLK functions and classes
  setupWaitBtn(LP_LEFT_BTN);  // Setup left button on Launchpad
  setupLed(RED_LED);          // Red led in rgb led
  setupLed(GREEN_LED);        // Green led in rgb led
  setupLed(BLUE_LED);         // Blue led in rgb led
}

void loop() {
  startProgram();
  driveStraight(); // Drive straight till a wall is hit
  rotate(CW, 180);
  reverse(5.08);   // Back up 2 inches (5.08cm)
  
}


// -------------------------------------------------------------------------------------------- //


/* Function Name: reverse
   Input: 1 Input variable, travel distance as a floating point.
   Details: Function called to reverse backwards at a constant speed
            and at a specified distance in centimeters on a 
            straight linear line.
*/
void reverse(float travel_dist){
  resetLeftEncoderCnt();  // Reset Left Encoder Count
  resetRightEncoderCnt(); // Reset Right Encoder Count
 
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);

  setMotorSpeed(LEFT_MOTOR, LEFT_TURN_SPEED);
  setMotorSpeed(RIGHT_MOTOR, RIGHT_TURN_SPEED);

  uint32_t totalPulses = countForDistance(wheelDiameter, cntPerRevolution, travel_dist);

  uint16_t leftPulse = 0;   // Total amount of left encoder pulses received
  uint16_t rightPulse = 0;  // Total amount of right encoder pulses received
  while (leftPulse < totalPulses && rightPulse < totalPulses) {    // Check Encoders against target pulse count
    leftPulse = getEncoderLeftCnt();                               // Get Left Encoder Count
    rightPulse = getEncoderRightCnt();                             // Get Right Encoder Count

    /* If the left encoder count is less than the right, increase
       the left motor speed by 1 */
    if (leftPulse < rightPulse) {
      setMotorSpeed(LEFT_MOTOR, LEFT_TURN_SPEED + 1);
      setMotorSpeed(RIGHT_MOTOR, RIGHT_TURN_SPEED);
    }
    
    /* If the right encoder count is less than the left, increase
       the right motor speed by 2 */
    if (rightPulse < leftPulse) {
      setMotorSpeed(LEFT_MOTOR, LEFT_TURN_SPEED);
      setMotorSpeed(RIGHT_MOTOR, RIGHT_TURN_SPEED + 1);
    }
  }
  disableMotor(BOTH_MOTORS);
  delay(DELAY_MS);
}


/* Function Name: driveStraight
   Input: 1 Input variable, travel distance as a floating point.
   Details: Function called to drive forward at a constant speed
            until it bumps into an object
*/
void driveStraight() {
  resetLeftEncoderCnt();  // Reset Left Encoder Count
  resetRightEncoderCnt(); // Reset Right Encoder Count

  bool frontLeft = false;
  bool frontRight = false;

  uint16_t leftPulse = 0;   // Total amount of left encoder pulses received
  uint16_t rightPulse = 0;  // Total amount of right encoder pulses received
  
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  
  setMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED);
  setMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED);
  
  while (true) {
    leftPulse = getEncoderLeftCnt();                               // Get Left Encoder Count
    rightPulse = getEncoderRightCnt();                             // Get Right Encoder Count
    
    /* If the left encoder count is less than the right, increase
       the left motor speed by 1 */
    if (leftPulse < rightPulse) {
      setMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED + 1);
      setMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED);
    }
    
    /* If the right encoder count is less than the left, increase
       the right motor speed by 1 */
    if (rightPulse < leftPulse) {
      setMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED);
      setMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED + 1);
    }
   
   if(digitalRead(BP_SW_PIN_0) == 0){
      disableMotor(BOTH_MOTORS);
      delay(DELAY_MS);
      reverse(5.08);
      rotate(CW, 52);

      enableMotor(BOTH_MOTORS);
      setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
      
      setMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED);
      setMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED);
      
   }if(digitalRead(BP_SW_PIN_1) == 0){
      disableMotor(BOTH_MOTORS);
      delay(DELAY_MS);
      reverse(5.08);
      rotate(CW, 34);

      enableMotor(BOTH_MOTORS);
      setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
      
      setMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED);
      setMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED);

   }if(digitalRead(BP_SW_PIN_2) == 0 && digitalRead(BP_SW_PIN_3) == 1){
      
      frontRight = true;
      
      disableMotor(BOTH_MOTORS);
      delay(DELAY_MS);
      reverse(5.08);
      rotate(CW, 5);

      enableMotor(BOTH_MOTORS);
      setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
      
      setMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED);
      setMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED);

   }if(digitalRead(BP_SW_PIN_3) == 0 && digitalRead(BP_SW_PIN_2) == 1){

      frontLeft = true;
    
      disableMotor(BOTH_MOTORS);
      delay(DELAY_MS);
      reverse(5.08);
      rotate(CCW, 5);

      enableMotor(BOTH_MOTORS);
      setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
      
      setMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED);
      setMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED);
      
   }if(digitalRead(BP_SW_PIN_4) == 0){
      disableMotor(BOTH_MOTORS);
      delay(DELAY_MS);
      reverse(5.08);
      rotate(CCW, 34);

      enableMotor(BOTH_MOTORS);
      setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
      
      setMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED);
      setMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED);
      
  }if(digitalRead(BP_SW_PIN_5) == 0){
      disableMotor(BOTH_MOTORS);
      delay(DELAY_MS);
      reverse(5.08);
      rotate(CCW, 52);

      enableMotor(BOTH_MOTORS);
      setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
      
      setMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED);
      setMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED);
      
    }

    if(frontLeft && frontRight){
      disableMotor(BOTH_MOTORS);
      delay(DELAY_MS);
      break;
    }
  }
}

/* Function Name: rotate
   Input: 2 Input Variables, rotational direction and rotational degree as integers
   Details: Function called to rotate in place given a specified direction (CW or CCW)
            and degree to rotate to. 
*/
void rotate(int rotate_dir, int rotate_deg) {

  resetLeftEncoderCnt();  // Reset Left Encoder Count
  resetRightEncoderCnt(); // Reset Right Encoder Count
  
  uint16_t leftPulse = 0; // Total amount of left encoder pulses received
  uint16_t rightPulse = 0;  // Total amount of right encoder pulses received

  float distance = (float(rotate_deg) / 360) * (14 * PI); // Convert degrees to distance
  uint32_t totalPulses = countForDistance(wheelDiameter, cntPerRevolution, distance);

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


// -------------------------------------------------------------------------------------------- //

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
  blinkBlueLED(500); // Blink for a period of half a second
  blinkBlueLED(500); // Blink for a period of half a second
  blinkBlueLED(500); // Blink for a period of half a second
  blinkBlueLED(500); // Blink for a period of half a second
  
}
/* Function Name: blinkBlueLED
   Input: integer (period) in milliseconds
   Details: Function call that will blink the Blue LED for a period specified.
*/
int blinkBlueLED(int period) {
  int pause = period / 2;        // Determine on/off time
  digitalWrite(BLUE_LED, HIGH);  // Turn LED on
  delay(pause);                  // Time the LED is on
  digitalWrite(BLUE_LED, LOW);   // Turn LED off
  delay(pause);                  // Time LED is off
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
