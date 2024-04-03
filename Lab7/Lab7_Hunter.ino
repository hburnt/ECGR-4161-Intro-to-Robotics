/*************************************************************************
 * Written by Dr. Rhoades 4-9-19 | Edited by Joey Phillips 6-20-22

 * Be sure that the following Pins are connected correctly.
      TRIG Pin is connectged to P3.5 of the TI-Board (Pin 32)
      ECHO Pin is connected to P5.1 of the TI-Board (Pin 33) 
      Also ensure that VCC goes to +5V and Gnd goes to a Ground Pin 
*************************************************************************/
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

const int trigPin = 32;           //connects to the trigger pin on the distance sensor       
const int echoPin = 33;           //connects to the echo pin on the distance sensor      
float distance = 0;               //stores the distance measured by the distance sensor

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
  float stop_dist = 12.0;
  distance = getDistance();   //variable to store the distance measured by the sensor

  Serial.print(distance);     //print the distance that was measured
  //Serial.println(" inches");      //print units (inches) after the distance
  Serial.println(" cm");      //print units (centimeters) after the distance
//
  delay(65);      //delay 65ms between each reading
  forward(stop_dist);
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

/* Function Name: forward
   Input: 1 Input variable, travel distance as a floating point.
   Details: Function called to drive forward at a constant speed
            and at a specified distance in centimeters on a 
            straight linear line.
*/
void forward(float stop_dist) {

  resetLeftEncoderCnt();
  resetRightEncoderCnt();  // Reset both encoder counts
  uint16_t leftPulse = 0;   // Total amount of left encoder pulses received
  uint16_t rightPulse = 0;  // Total amount of right encoder pulses received
  float measured_dist = getDistance();
  
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);

  setMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED);
  setMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED);
  
  while (measured_dist > stop_dist) {    // Check Encoders against target pulse count
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
  }
  disableMotor(BOTH_MOTORS);
  delay(DELAY_MS);
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
