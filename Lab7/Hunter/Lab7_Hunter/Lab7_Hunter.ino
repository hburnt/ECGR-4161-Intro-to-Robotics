/*************************************************************************
 * Written by Dr. Rhoades 4-9-19 | Edited by Joey Phillips 6-20-22

 * Be sure that the following Pins are connected correctly.
      TRIG Pin is connectged to P3.5 of the TI-Board (Pin 32)
      ECHO Pin is connected to P5.1 of the TI-Board (Pin 33) 
      Also ensure that VCC goes to +5V and Gnd goes to a Ground Pin 
*************************************************************************/
#include "SimpleRSLK.h"

#define DELAY_MS            0.01   // delay in milliseconds

// Default pwm signals (percentage-% of power 0-100) for both RSLK motor.
// Change these values as needed
#define LEFT_MOTOR_SPEED     11   // Speed percentage
#define RIGHT_MOTOR_SPEED    12   // Speed percentage
#define LEFT_TURN_SPEED      15   // 6%/100 Speed percentage
#define RIGHT_TURN_SPEED     17   // 6%/100 Speed percentage

// Value for turning directions (do not change)
#define CCW                  1     // rotate robot counter clockwise
#define CW                   2     // rotate robot clockwise

// RSLK specified mechanics (do not change)
#define wheelDiameter       6.999         // in centimeters
#define cntPerRevolution    360           // Number of encoder (rising) pulses every time the wheel turns completely

#define turnDegree            5 // 3
#define fovDegree             90
#define distanceArrayLength   fovDegree/turnDegree

#define numPings            5

const int trigPin = 32;           //connects to the trigger pin on the distance sensor       
const int echoPin = 33;           //connects to the echo pin on the distance sensor
float distance = 1;               //stores the distance measured by the distance sensor

float distances[distanceArrayLength];
float pings[numPings];

void setup() {
  //Do not edit this setup function
  Serial.begin(9600);         // Start serial com at 9600 baud rate
  setupRSLK();                // Initialize RSLK functions and classes
  setupWaitBtn(LP_LEFT_BTN);  // Setup left button on Launchpad
  setupLed(RED_LED);          // Red led in rgb led
  setupLed(GREEN_LED);        // Green led in rgb led
  setupLed(BLUE_LED);         // Blue led in rgb led
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  startProgram();
  //storeDistances();
  //alignNCenter();
  turnNCheck();

  

}

//------------------FUNCTIONS-------------------------------

//RETURNS THE DISTANCE MEASURED BY THE HC-SR04 DISTANCE SENSOR
float getDistance() {
  float echoTime;                   //variable to store the time it takes for a ping to bounce off an object
  float calculatedDistanceInches;         //variable to store the distance calculated from the echo time
  //float calculatedDistanceCentimeters;         //variable to store the distance calculated from the echo time


  //send out an ultrasonic pulse that's 10us long
  digitalWrite(trigPin, LOW); //ensures a clean pulse beforehand
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH);      //use the pulsein command to see how long it takes for the
                                          //pulse to bounce back to the sensor in microseconds

  calculatedDistanceInches = echoTime / 148.0;  //calculate the distance in inches of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)
  //calculatedDistanceCentimeters = echoTime / 58.0;  //calculate the distance in centimeters of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)


  return calculatedDistanceInches;              //send back the distance that was calculated
}
/* Function Name: storeDistance
   Input: void
   Details: Function called to store ultrasonic distances at each angles.
            depending on turnDegree
*/
void storeDistances(){
  for(int i = 0; i < distanceArrayLength; i++){
    for(int j = 0; j < numPings; j++){
      pings[j] = getDistance();
      delay(65);
    }
    for(int i = 0; i < numPings; i++){
      for(int j = 1; j < numPings; j++){
        if(pings[i] < pings[j]){
          float temp = pings[i];
          pings[i] = pings[j];
          pings[j] = temp;
        }
      }
    }
    distances[i] = pings[(numPings-1)/2];
    if(i < distanceArrayLength-1){
      rotate(CCW, turnDegree);
    }

  }
}

void alignNCenter(){
  float temp = 999;
  int idx = 0;
  
  for(int i = 0; i < distanceArrayLength; i++){
    if(distances[i] < temp){
      temp = distances[i];
      idx = i;
    }
  }

  Serial.println(idx);
  
  delayMicroseconds(10);
  rotate(CW, fovDegree - (turnDegree * idx));
}

void turnNCheck(){
  float cwDist = 0;
  float ccwDist = 0;

  // Check Left Distance
  rotate(CCW, 90);
  ccwDist = getDistance();
  delay(100);

  // Check Right Distance
  rotate(CW, 180);
  cwDist = getDistance();
  delay(100);

  rotate(CCW, 90);
  delay(100);
  
  if(cwDist > ccwDist){
    rotate(CW, 90);
    forward();
  }else{
    rotate(CCW, 90);
    forward();
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

  setRawMotorSpeed(LEFT_MOTOR, LEFT_TURN_SPEED);
  setRawMotorSpeed(RIGHT_MOTOR, RIGHT_TURN_SPEED);

  while (leftPulse < totalPulses && rightPulse < totalPulses) {    // Check Encoders against target pulse count
    leftPulse = getEncoderLeftCnt();                               // Get Left Encoder Count
    rightPulse = getEncoderRightCnt();                             // Get Right Encoder Count

    /* If the left encoder count is less than the right, increase
       the left motor speed */
    if (leftPulse < rightPulse) {
      setRawMotorSpeed(LEFT_MOTOR, LEFT_TURN_SPEED + 1);
      setRawMotorSpeed(RIGHT_MOTOR, RIGHT_TURN_SPEED);
    }

    /* If the right encoder count is less than the left, increase
       the right motor speed */
    if (rightPulse < leftPulse) {
      setRawMotorSpeed(LEFT_MOTOR, LEFT_TURN_SPEED);
      setRawMotorSpeed(RIGHT_MOTOR, RIGHT_TURN_SPEED + 1);
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
void forward() {

  resetLeftEncoderCnt();
  resetRightEncoderCnt();  // Reset both encoder counts
  uint16_t leftPulse = 0;   // Total amount of left encoder pulses received
  uint16_t rightPulse = 0;  // Total amount of right encoder pulses received
  
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);

  setMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED);
  setMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED);

  float measured_dist = getDistance();
  float too_close_dist = 6.0;
  
  while (measured_dist > too_close_dist) {
    leftPulse = getEncoderLeftCnt();                               // Get Left Encoder Count
    rightPulse = getEncoderRightCnt();                             // Get Right Encoder Count
    measured_dist = getDistance();
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
    delay(50);

    measured_dist = getDistance();   //variable to store the distance measured by the sensor
  
    Serial.print(measured_dist);     //print the distance that was measured
    Serial.println(" inches");      //print units (inches) after the distance
    //Serial.println(" cm");      //print units (centimeters) after the distance
    
    delay(65);      //delay 65ms between each reading
  }
  disableMotor(BOTH_MOTORS);
  delay(DELAY_MS);
}

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

  setRawMotorSpeed(LEFT_MOTOR, LEFT_TURN_SPEED);
  setRawMotorSpeed(RIGHT_MOTOR, RIGHT_TURN_SPEED);

  uint32_t totalPulses = countForDistance(wheelDiameter, cntPerRevolution, travel_dist);

  uint16_t leftPulse = 0;   // Total amount of left encoder pulses received
  uint16_t rightPulse = 0;  // Total amount of right encoder pulses received
  while (leftPulse < totalPulses && rightPulse < totalPulses) {    // Check Encoders against target pulse count
    leftPulse = getEncoderLeftCnt();                               // Get Left Encoder Count
    rightPulse = getEncoderRightCnt();                             // Get Right Encoder Count

    /* If the left encoder count is less than the right, increase
       the left motor speed by 1 */
    if (leftPulse < rightPulse) {
      setRawMotorSpeed(LEFT_MOTOR, LEFT_TURN_SPEED + 1);
      setRawMotorSpeed(RIGHT_MOTOR, RIGHT_TURN_SPEED);
    }
    
    /* If the right encoder count is less than the left, increase
       the right motor speed by 2 */
    if (rightPulse < leftPulse) {
      setRawMotorSpeed(LEFT_MOTOR, LEFT_TURN_SPEED);
      setRawMotorSpeed(RIGHT_MOTOR, RIGHT_TURN_SPEED + 1);
    }
  }
  disableMotor(BOTH_MOTORS);
  delay(DELAY_MS);
}

















/* Function Name: countForDistance
   Input: 3 input variables
   Return: int value
   Details: Function called to calculate the number of pulses need to travel a specified
            distance by the user input variable "distance."
*/
uint32_t countForDistance(float wheelDiam, uint16_t cntPerRev, float distance) {
  float temp = (wheelDiam * PI) / cntPerRev;
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
  String btnMsg = ""; // "Push left button on Launchpad to start lab program.\n";
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
