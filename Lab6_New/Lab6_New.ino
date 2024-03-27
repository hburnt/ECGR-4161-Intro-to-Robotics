
#include "SimpleRSLK.h"

#define DELAY_MS            2000   // delay in milliseconds

// Default pwm signals (percentage-% of power 0-100) for both RSLK motor.
// Change these values as needed
#define LEFT_MOTOR_SPEED       50 // Speed percentage (Originally 11)
#define RIGHT_MOTOR_SPEED    50 // Speed percentage (Originally 12)
#define LEFT_TURN_SPEED           8  // Speed percentage (Originally 6)
#define RIGHT_TURN_SPEED        8  // Speed percentage (Originally 7)

// Value for turning directions (do not change)
#define CCW                  1     // rotate robot counter clockwise
#define CW                     2     // rotate robot clockwise

// RSLK specified mechanics (do not change)
#define wheelDiameter       6.999    // in centimeters
#define cntPerRevolution    360      // Number of encoder (rising) pulses every time the wheel turns completely

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
  }


void forward(int left_speed, int right_speed){
  resetLeftEncoderCnt();
  resetRightEncoderCnt();
  
  bool hitObstacle = false;
  enableMotor(BOTH_MOTORS);
  setMotorDirection(BOTH_MOTORS, MOTOR_DIR_FORWARD);


   while(hitObstacle==false){
    
    setRawMotorSpeed(LEFT_MOTOR, left_speed);
    setRawMotorSpeed(RIGHT_MOTOR, right_speed);

    if(digitalRead(BP_SW_PIN_0) == 0)
      break;

    if(digitalRead(BP_SW_PIN_1) == 0)
      break;
    
   }
   disableMotor(BOTH_MOTORS);
   delay (2000);
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
