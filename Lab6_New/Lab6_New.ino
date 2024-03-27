
#include "SimpleRSLK.h"

#define DELAY_MS            2000   // delay in milliseconds

// Default pwm signals (percentage-% of power 0-100) for both RSLK motor.
// Change these values as needed
#define LEFT_MOTOR_SPEED     16 // Speed percentage (Originally 11)
#define RIGHT_MOTOR_SPEED    16 // Speed percentage (Originally 12)
#define LEFT_TURN_SPEED      8  // Speed percentage (Originally 6)
#define RIGHT_TURN_SPEED     8  // Speed percentage (Originally 7)

// Value for turning directions (do not change)
#define CCW                  1     // rotate robot counter clockwise
#define CW                   2     // rotate robot clockwise

// RSLK specified mechanics (do not change)
#define wheelDiameter       6.999    // in centimeters
#define cntPerRevolution    360      // Number of encoder (rising) pulses every time the wheel turns completely
#define BASE_R                  7
#define BASE_D                  2 * BASE_R
void setup() {
  Serial.begin(9600);         // Start serial com at 9600 baud rate
  setupRSLK();                // Initialize RSLK functions and classes
  setupWaitBtn(LP_LEFT_BTN);  // Setup left button on Launchpad
  setupLed(RED_LED);          // Red led in rgb led 
  setupLed(BLUE_LED); 
              
  pinMode(BP_SW_PIN_0,INPUT_PULLUP);
  pinMode(BP_SW_PIN_1,INPUT_PULLUP);
  pinMode(BP_SW_PIN_2,INPUT_PULLUP);
  pinMode(BP_SW_PIN_3,INPUT_PULLUP);
  pinMode(BP_SW_PIN_4,INPUT_PULLUP);
  pinMode(BP_SW_PIN_5,INPUT_PULLUP);
}

void loop() {
   startProgram();
   forward(LEFT_MOTOR_SPEED, RIGHT_MOTOR_SPEED);
   backUp(20, RIGHT_MOTOR_SPEED, LEFT_MOTOR_SPEED);
   rotate(CW, 180);
   forward(LEFT_MOTOR_SPEED, RIGHT_MOTOR_SPEED);
   rotate(CW, 180);
  }


void forward(int left_speed, int right_speed){
  int minSpeed = 5;
  int maxSpeed = 65; 

  float kp = 0.396;
  float ki = 0.0005;
  float kd = 49.8;

  int left_cnt = 0;
  int right_cnt = 0;

  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  
  bool hitObstacle = false;
  
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);
  enableMotor(BOTH_MOTORS);
   while(hitObstacle==false){

     right_cnt = getEncoderRightCnt();
     left_cnt = getEncoderLeftCnt();

     int error = right_cnt - left_cnt;

     int adjustment = pid(0, error, kp, ki, kd);

     left_speed += adjustment;
     if (left_speed < minSpeed) left_speed = minSpeed;

     if (left_speed > maxSpeed) left_speed = maxSpeed;

    setMotorSpeed(LEFT_MOTOR, left_speed);
    setMotorSpeed(RIGHT_MOTOR, right_speed);

    if(digitalRead(BP_SW_PIN_0) == 0)
      break;

    if(digitalRead(BP_SW_PIN_1) == 0)
      break;
    if(digitalRead(BP_SW_PIN_2) + digitalRead(BP_SW_PIN_3) == 0){
       hitObstacle = true;
      } 
   }
   disableMotor(BOTH_MOTORS);
   delay (2000);
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

void backUp(float distance, int right_speed, int left_speed){
  resetRightEncoderCnt();
  resetLeftEncoderCnt();

  int target_cnt = countForDistance(wheelDiameter, cntPerRevolution, distance);
  
  int left_cnt = 0;
  int right_cnt = 0;

  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_BACKWARD);
  enableMotor(BOTH_MOTORS);
  setRawMotorSpeed(LEFT_MOTOR, left_speed);
  setRawMotorSpeed(RIGHT_MOTOR, right_speed);

  while((left_cnt < target_cnt) || (right_cnt < target_cnt)){
    left_cnt = getEncoderLeftCnt();
    right_cnt = getEncoderRightCnt();
  }
  disableMotor(BOTH_MOTORS  );
  delay(2000);
}
/* Function Name: countForDistance
   Input: 1 int input variables, 2 float input
   Return: int value
   Details: Function called to calculate the number of pulses need to travel a specified
            distance by the user input variable "distance."
*/
int countForDistance(float wheel_diam, uint16_t cnt_per_rev, float distance) {
  float temp = (wheel_diam * PI) / cnt_per_rev;
  temp = distance / temp;
  return int(temp);
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

void startProgram(){
  /* Setup message to print to serial montor */
  String btnMsg = "Push left button on Launchpad to start lab program.\n";
  waitBtnPressed(LP_LEFT_BTN,btnMsg,RED_LED); // Function to setup button, msg, LED
  /* Using an LED as a DELAY */
  blinkBlueLED(1000);                             // Cause LED to blink for a period of one second
  blinkBlueLED(1000);                             // Cause LED to blink for a period of one second
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
