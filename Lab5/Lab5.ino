/*
 * Hunter Burnett
 * ECGR 4161 - Intro to Robotics
 * 03/19/2024
 * Lab5
 * Group Number 21
 */

#include "SimpleRSLK.h"

#define LEFT_MOTOR_SPEED_RAW    50      // Raw Speed 
#define RIGHT_MOTOR_SPEED_RAW   50      // Raw Speed 
#define RIGHT_MOTOR_SPEED       20      // Speed Percentage
#define LEFT_MOTOR_SPEED        20      // Speed Percentage
#define LEFT_TURN_SPEED         12      // Speed percentage
#define RIGHT_TURN_SPEED        12      // Speed percentage

// Value for turning directions (do not change)
#define CCW                     2       // Rotate robot counter-clockwise
#define CW                      1       // Rotate robot clockwise

// RSLK specified mechanics (do not change)
#define wheelDiameter           6.999   // Wheel diameter in centimeters
#define cntPerRevolution        360     // Number of encoder (rising) pulses every time the wheel rotates

#define BASE_R                  7       // Radius of the wheel base on the RSLK robot   
#define BASE_D                  14      // Diameter of the wheel base on the RSLK robot
#define CIRCLE_R_IN             18      // Radius of the circle in inches
#define CIRCLE_R_CM             (CIRCLE_R_IN * 2.54)    // Radius of the circle converted to cm
#define CIRCLE_D_CM             (CIRCLE_R_CM * 2)       // Diameter of the circle in cm
#define CIRCLE_PERCENT          0.75                     // Percentage of the circle to travel
#define LEFT_TRAVEL_DISTANCE    ((CIRCLE_R_CM - BASE_R) * 2 * PI * CIRCLE_PERCENT) // Distance the left wheel needs to travel
#define RIGHT_TRAVEL_DISTANCE   ((CIRCLE_R_CM + BASE_R) * 2 * PI * CIRCLE_PERCENT) // Distance the right wheel needs to travel
#define OUTSIDE_WHEEL_RATIO     ((CIRCLE_D_CM + BASE_D) / (CIRCLE_D_CM - BASE_D))  // Ratio for adjusting outside wheel speed



void setup() {
  Serial.begin(9600);         // Start serial com at 9600 baud rate
  setupRSLK();                // Initialize RSLK functions and classes
  setupWaitBtn(LP_LEFT_BTN);  // Setup left button on Launchpad
  setupLed(RED_LED);          // Red led in rgb led
  setupLed(GREEN_LED);        // Greem led in rgb led  
  setupLed(BLUE_LED);                      
}

void loop() {
    startProgram();

    forward(CIRCLE_R_CM);
    rotate(CCW, 45);
    
    forward(CIRCLE_R_CM);
    rotate(CCW, 90);
    
    driveCircle(LEFT_MOTOR_SPEED_RAW, RIGHT_MOTOR_SPEED_RAW, LEFT_TRAVEL_DISTANCE, RIGHT_TRAVEL_DISTANCE);
    
    rotate(CCW, 90);
    forward(CIRCLE_R_CM);
    
    rotate(CCW, 45);
    forward(CIRCLE_R_CM);
    
    rotate(CCW, 180);
    
    blinkBlueLED(2000);
    blinkBlueLED(2000);
    blinkBlueLED(2000);
}

void startProgram(){
  /* Setup message to print to serial montor */
  String btnMsg = "Push left button on Launchpad to start lab program.\n";
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED); // Function to setup button, msg, LED
  /* Using an LED as a DELAY */
  blinkLED(1000);                             // Cause LED to blink for a period of one second
  blinkLED(1000);                             // Cause LED to blink for a period of one second
}

int blinkLED(int period){
  int pause = period/2;           // Determine on/off time
  digitalWrite(GREEN_LED, HIGH);  // Turn LED on
  delay(pause);                   // Time the LED is on
  digitalWrite(GREEN_LED, LOW);   // Turn LED off
  delay(pause);                   // Time LED is off
}

/* Function Name: countForDistance
   Input: 1 int input variables, 2 float input
   Return: int value
   Details: Function called to calculate the number of pulses need to travel a specified
            distance by the user input variable "distance."
*/
uint32_t countForDistance(float wheel_diam, uint16_t cnt_per_rev, float distance) {
  float temp = (wheel_diam * PI) / cnt_per_rev;
  temp = distance / temp;
  return int(temp);
}

/* Function Name: driveCircle
   Input: 4 input parameters: Left_Wheel_Speed (int), Right_Wheel_Speed (int), Left_Travel_Dist (float), Right_Travel_Dist (float)
   Return: void
   Details: Drives the robot in a circular path with specified wheel speeds and travel distances for both left and right wheels.
*/
void driveCircle(int Left_Wheel_Speed, int Right_Wheel_Speed, float Left_Travel_Dist, float Right_Travel_Dist){
  
  int rightTargetSpeed = (Right_Wheel_Speed * (OUTSIDE_WHEEL_RATIO));
  int leftTargetSpeed = Left_Wheel_Speed;

  int leftTargetCnt = countForDistance(wheelDiameter, cntPerRevolution, Left_Travel_Dist)+360;
  int rightTargetCnt = countForDistance(wheelDiameter, cntPerRevolution, Right_Travel_Dist)+360;
  
  resetLeftEncoderCnt();
  resetRightEncoderCnt();

  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
  
  int leftCnt = 0;
  int rightCnt = 0;
  
  enableMotor(BOTH_MOTORS);

  while((leftCnt <= leftTargetCnt) || (rightCnt <= rightTargetCnt)){
     rightCnt = getEncoderRightCnt();
     leftCnt = getEncoderLeftCnt();

     setRawMotorSpeed(LEFT_MOTOR, leftTargetSpeed);
     setRawMotorSpeed(RIGHT_MOTOR, rightTargetSpeed);
    }
  disableMotor(BOTH_MOTORS);
  delay(2000);
}

/* Function Name: blinkBlueLED
   Input: int period
   Return: int
   Details: Blinks the blue LED with the specified period.
*/
int blinkBlueLED(int period){
  int pause = period/2;
  digitalWrite(BLUE_LED, HIGH);  // Turn LED on
  delay(pause);                   // Time the LED is on
  digitalWrite(BLUE_LED, LOW);   // Turn LED off
  delay(pause);                   // Time LED is off
}

/* Function Name: rotate
   Input: 2 input parameters: rotate_dir (int), rotate_deg (int)
   Return: void
   Details: Rotates the robot in place by the specified degrees and direction.
*/
void rotate(int rotate_dir, int rotate_deg) {
  
  float distance = (BASE_D * PI) * (rotate_deg / 360.0);
  uint32_t targetCount = countForDistance(wheelDiameter, cntPerRevolution, distance);
  resetLeftEncoderCnt();
  resetRightEncoderCnt();

  int leftCount = 0;
  int rightCount = 0;

  switch (rotate_dir) {
    
    case CW:
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_BACKWARD);
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);
      
    case CCW:  
      setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
      setMotorDirection(LEFT_MOTOR, MOTOR_DIR_BACKWARD);

  }
  enableMotor(BOTH_MOTORS);
  setMotorSpeed(LEFT_MOTOR, LEFT_TURN_SPEED);
  setMotorSpeed(RIGHT_MOTOR, RIGHT_TURN_SPEED);
  while (leftCount < targetCount || rightCount < targetCount) {
    leftCount = getEncoderLeftCnt();
    rightCount = getEncoderRightCnt();
  }
  disableMotor(BOTH_MOTORS);
  delay(2000);

}

/* Function Name: pid
   Input: 5 input parameters: setpoint (int), input (int), kp (double), ki (double), kd (double)
   Return: int
   Details: Implements a PID controller algorithm to calculate the adjustment value based on the setpoint, input, and tuning constants.
*/
int pid(int setpoint, int input, double kp, double ki, double kd) {
  static unsigned long lastTime = millis();
  static int lastErrorCalculated = input - setpoint;
  static double cumulativeError = 0;

  unsigned long currentTime = millis();
  double deltaTime = (double)(currentTime - lastTime);
  lastTime = currentTime;

  int error = input - setpoint;

  float prop = kp * error;
  cumulativeError += error * deltaTime;

  float integral = cumulativeError * ki;

  float derivative = ((error - lastErrorCalculated) / deltaTime) * kd;

  lastErrorCalculated = error;

  int adjustment = prop + integral + derivative;
  
  return adjustment;

}

/* Function Name: forward
   Input: 1 input parameter: travel_dist (float)
   Return: void
   Details: Drives the robot forward for the specified distance using a PID controller for speed control.
*/
void forward(float travel_dist) {

  int minSpeed = 5;
  int maxSpeed = 65;

  float kp = 0.396;
  float ki = 0.0005;
  float kd = 49.8;
  
  resetLeftEncoderCnt();
  resetRightEncoderCnt();

  int targetCount = countForDistance(wheelDiameter, cntPerRevolution, travel_dist);

  int leftSpeed = LEFT_MOTOR_SPEED;
  int rightSpeed = RIGHT_MOTOR_SPEED;

  setMotorDirection(RIGHT_MOTOR, MOTOR_DIR_FORWARD);
  setMotorDirection(LEFT_MOTOR, MOTOR_DIR_FORWARD);

  int rightCnt = 0;
  int leftCnt = 0;

  enableMotor(BOTH_MOTORS);

  while ((rightCnt <= targetCount || leftCnt <= targetCount)) {
    rightCnt = getEncoderRightCnt();
    leftCnt = getEncoderLeftCnt();

    int error = rightCnt - leftCnt;

    // Difference between counts should be 0, so setpoint is 0
    int adjustment = pid(0, error, kp, ki, kd);

    leftSpeed += adjustment;

    if (leftSpeed < minSpeed) {
      leftSpeed = minSpeed;
    }

    if (leftSpeed > maxSpeed) {
      leftSpeed = maxSpeed;
    }

    setMotorSpeed(LEFT_MOTOR, leftSpeed);
    setMotorSpeed(RIGHT_MOTOR, rightSpeed);

    delay(50);
  }

  disableMotor(BOTH_MOTORS);
  delay(2000);
}
