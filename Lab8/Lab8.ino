/* Program: Autonomous Robot Navigation
Authors: Hunter Burnett & Aidan Cowan
Date: 4/11/2024
Description: This program is designed to control an autonomous robot to navigate through a hallway. 
             The robot uses an ultrasonic sensor to detect the presence of doorways. 
             The robot first moves forward and then backward, taking measurements 
             on both sides of each doorway. 
             The collected data is then converted to a binary sequence and displayed 
             on the serial monitor, with the purple LED blinking 
             to indicate the program is finished.
Lab: Lab 8
Group: Group 18
Class: ECGR-4161 Intro to Robotics
*/

#include <Servo.h>
#include "SimpleRSLK.h"

#define DELAY_MS                  2000   // delay in milliseconds
#define LEFT_MOTOR_SPEED          46     // Speed percentage, originally 12
#define RIGHT_MOTOR_SPEED         35     // Speed percentage, originally 12
#define LEFT_MOTOR_SPEED_BACK     46     // Speed percentage, originally 12
#define RIGHT_MOTOR_SPEED_BACK    35     // Speed percentage, originally 12
#define wheelDiameter             6.999  // in centimeters
#define cntPerRevolution          360    // Number of encoder (rising) pulses every time the wheel turns completely
#define DISTANCE                  18*2.5 // Distances between doorways converted from inches to cm
#define DOORS                     4      // Number of doors on one side of the robot
#define numPings                  3      // Number of measurements the robot will take to detect a door
#define minWallDist               24.00  // Threshold distance to wall (cm)
const int trigPin =               32;    // Setting the ultrasonic trigger pin
const int echoPin =               33;    // Setting the ultrasonic echo pin

Servo myservo;  // create servo object to control a servo
                // a maximum of eight servo objects can be created
           
void setup() {    
  // put your setup code here, to run once:
  Serial.begin(9600);   // Start serial monitor  

  setupRSLK();            // must be called to setup RSLK peripherals and pins
  
  // initialize two digital pins as outputs.
  pinMode(76, OUTPUT);  //RGB Green LED possible pinMode variables -> P2.1 -> 76 -> GREEN_LED
  pinMode(77, OUTPUT);  //RGB Blue LED possible pinMode variables -> P2.2 -> 77 -> BLUE_LED

  
  pinMode(trigPin, OUTPUT); // Setting the trigger pin
  pinMode(echoPin, INPUT);  // Setting the echo pin
  myservo.attach(38);       // attaches the servo on Port 6.1 (P6.1 or pin 23)to the servo object
  myservo.write(0);         // Send it to the default position
  setupWaitBtn(LP_LEFT_BTN);// Setup left button on Launchpad
}

void loop() {
  runProgram();
}

void runProgram(){
  
  // Initializing the door arrays
  int leftSequence[4];
  int rightSequence[4];
  int rightSequenceBack[4];
  int leftSequenceBack[4];
  int result[16]; 
  
 startProgram();

 blinkGreenLED(1000);
 blinkGreenLED(1000);
 
 for(int i = 0; i < DOORS; i++){
        forward(DISTANCE);
        delay(1000);
        myservo.write(180);
        delay(1500);
        leftSequence[i] = detectDoor();
        myservo.write(0);
        delay(1500);
        rightSequence[i] = detectDoor();
        delay(1000);
  }

  forward(DISTANCE);
  
  for(int i = 0; i < DOORS; i++){
        backward(DISTANCE);
        delay(1000);
        myservo.write(180);
        delay(1500);
        rightSequenceBack[i] = detectDoor();
        myservo.write(0);
        delay(1500);
        leftSequenceBack[i] = detectDoor();
        delay(1000);
  }
      
  backward(DISTANCE);
  
  concatArray(leftSequence, rightSequence, leftSequenceBack, rightSequenceBack, result);

  while(true){
    Serial.println(binaryToDecimal(result, 16));
    Serial.println(" ");
    blinkPurpLED(1000);
    }
}

/* Function Name: concatArray
   Input: 4 int arrays, 1 int array for output
   Return: void
   Details: Function concatenates the values from the 4 input int arrays into the output array.
*/
void concatArray(int arr1[], int arr2[], int arr3[], int arr4[], int result[]){
  int i, j = 0;
  for (i = 0; i < 4; i++){
    result[j++] = arr1[i];
  }
  for (i = 0; i < 4; i++){
    result[j++] = arr2[i];
  }
  for (i = 0; i < 4; i++){
    result[j++] = arr3[i];
  }
  for (i = 0; i < 4; i++){
    result[j++] = arr4[i];
  }
}


/* Function Name: forward
   Input: 1 Input variable, travel distance as a floating point.
   Details: Function called to drive forward at a constant speed
            and at a specified distance in centimeters on a 
            straight linear line.
*/
void forward(float travel_dist) {

  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  
  uint16_t leftPulse = 0;   // Total amount of left encoder pulses received
  uint16_t rightPulse = 0;  // Total amount of right encoder pulses received
  uint32_t totalPulses = countForDistance(wheelDiameter, cntPerRevolution, travel_dist);

  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);

  setRawMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED);
  setRawMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED);
  
  while (leftPulse < totalPulses && rightPulse < totalPulses) { // Check Encoders against target pulse count
    leftPulse = getEncoderLeftCnt();                            // Get Left Encoder Count
    rightPulse = getEncoderRightCnt();                          // Get Right Encoder Count

    /* If the left encoder count is less than the right, increase
       the left motor speed by 1 */
    if (leftPulse < rightPulse) {
      setRawMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED + 4);
      setRawMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED);
    }
    
    /* If the right encoder count is less than the left, increase
       the right motor speed by 2 */
    if (rightPulse < leftPulse) {
      setRawMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED);
      setRawMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED + 4);
    }
  }
  disableMotor(BOTH_MOTORS);
  delay(1000);
}

/* Function Name: backward
   Input: 1 Input variable, travel distance as a floating point.
   Details: Function called to drive backward at a constant speed
            and at a specified distance in centimeters on a 
            straight linear line.
*/
void backward(float travel_dist) {

  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  
  uint16_t leftPulse = 0;   // Total amount of left encoder pulses received
  uint16_t rightPulse = 0;  // Total amount of right encoder pulses received
  uint32_t totalPulses = countForDistance(wheelDiameter, cntPerRevolution, travel_dist);

  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);

  setRawMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED_BACK);
  setRawMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED_BACK);
  
  while (leftPulse < totalPulses && rightPulse < totalPulses) {    // Check Encoders against target pulse count
    leftPulse = getEncoderLeftCnt();                               // Get Left Encoder Count
    rightPulse = getEncoderRightCnt();                             // Get Right Encoder Count
    /* If the left encoder count is less than the right, increase
       the left motor speed by 1 */
    if (leftPulse < rightPulse) {
      setRawMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED_BACK + 4);
      setRawMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED_BACK);
    }
    
    /* If the right encoder count is less than the left, increase
       the right motor speed by 2 */
    if (rightPulse < leftPulse) {
      setRawMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED_BACK);
      setRawMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED_BACK + 4);
    }
  }
  disableMotor(BOTH_MOTORS);
  delay(DELAY_MS);
}

/* Function Name: countForDistance
   Input: 2 int input variables, 1 float input
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
   Input: None
   Return: void
   Details: Function sets up the message to print to the 
            serial monitor and waits for the left button press to start the program.
*/
void startProgram() {
  String btnMsg = "Push left button on Launchpad to start lab program.\n";
  waitBtnPressed(LP_LEFT_BTN, btnMsg, RED_LED); //Setup button, msg, LED
  delay(1000);
}

/* Function Name: detectDoor
   Input: None
   Return: int (0 or 1)
   Details: Function takes multiple distance measurements using the 
            ultrasonic sensor, sorts them, and 
            returns 1 if the median distance 
            is greater than the minimum wall distance threshold, or 0 otherwise.
*/
int detectDoor() {
  float pings[numPings];
  for(int i = 0; i < numPings; i++) {
    pings[i] = getDistance();
    delay(65);
  }
  
  // Sorting the array
  for(int i = 0; i < numPings; i++) {
    for(int j = i + 1; j < numPings; j++) {
      if(pings[i] > pings[j]) {
        float temp = pings[i];
        pings[i] = pings[j];
        pings[j] = temp;
      }
    }
  }
  
  // Finding the median distance
  float middleDistance = pings[(numPings / 2)];
  if(middleDistance > minWallDist){
    return 1;
  }else{
    return 0;
  }
}

/* Function Name: getDistance
   Input: None
   Return: float
   Details: Function uses the HC-SR04 ultrasonic sensor to measure the 
            distance to an object and returns the calculated distance in centimeters.
*/
float getDistance() {
  float echoTime;
  float calculatedDistanceCentimeters;
  
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  echoTime = pulseIn(echoPin, HIGH);
  calculatedDistanceCentimeters = echoTime / 58.0;
  
  return calculatedDistanceCentimeters;
}


/* Function Name: binaryToDecimal
   Input: int array, size of array
   Return: long
   Details: Function converts a binary number 
   represented as an array of 0s and 1s to its decimal equivalent.
*/
long binaryToDecimal(int arr[], int size) {
  long decimal = 0;
  for (int i = 0; i < size; i++) {
    decimal = decimal * 2 + arr[i];
  }
  return decimal;
}

/* Function Name: blinkPurpLED
   Input: int (period in milliseconds)
   Return: int
   Details: Function blinks the purple LED 
            (combination of red and blue LEDs) 
            for the specified period, turning it 
            on and off alternately.
*/
int blinkPurpLED(int period) {
  int pause = period / 2;
  analogWrite(RED_LED, 64);
  analogWrite(BLUE_LED, 64);
  analogWrite(GREEN_LED, 0);
  delay(pause);
  analogWrite(RED_LED, 0);
  analogWrite(BLUE_LED, 0);
  analogWrite(GREEN_LED, 0);
  delay(pause);
}

/* Function Name: blinkGreenLED
   Input: int (period in milliseconds)  
   Return: void
   Details: Function blinks the green LED for the specified 
            period, turning it on and off alternately.
*/
void blinkGreenLED(int period){
  int pause = period/2;
  digitalWrite(GREEN_LED, HIGH);
  delay(pause);
  digitalWrite(GREEN_LED, LOW);
  delay(pause);
}
