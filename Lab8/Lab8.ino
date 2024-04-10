#include <Servo.h>
#include "SimpleRSLK.h"

#define DELAY_MS            2000   // delay in milliseconds
#define LEFT_MOTOR_SPEED     45  // Speed percentage, originally 12
#define RIGHT_MOTOR_SPEED    36   // Speed percentage, originally 12
#define LEFT_MOTOR_SPEED_BACK    45 // Speed percentage, originally 12
#define RIGHT_MOTOR_SPEED_BACK   37   // Speed percentage, originally 12
#define wheelDiameter       6.999         // in centimeters
#define cntPerRevolution    360           // Number of encoder (rising) pulses every time the wheel turns completely
#define DISTANCE    18*2.5       // Distances between doorways converted from inches to cm
#define DOORS         4
#define numPings 3
#define minWallDist 12.00         // Threshold distance to wall (cm)
const int trigPin = 32;
const int echoPin = 33;
Servo myservo;  // create servo object to control a servo
                // a maximum of eight servo objects can be created
int result[16];            
void setup() {    
  // put your setup code here, to run once:
  Serial.begin(9600);   // Start serial monitor  

  setupRSLK();            // must be called to setup RSLK peripherals and pins
  
  // initialize two digital pins as outputs.
  pinMode(76, OUTPUT);  //RGB Green LED possible pinMode variables -> P2.1 -> 76 -> GREEN_LED
  pinMode(77, OUTPUT);  //RGB Blue LED possible pinMode variables -> P2.2 -> 77 -> BLUE_LED
  myservo.attach(38);   // attaches the servo on Port 6.1 (P6.1 or pin 23)to the servo object
  myservo.write(90);     // Send it to the default position
  setupWaitBtn(LP_LEFT_BTN);  // Setup left button on Launchpad
}

void loop() {
  int leftSequence[4];
  int rightSequence[4];
  int rightSequenceBack[4];
  int leftSequenceBack[4]; 
  // put your main code here, to run repeatedly:
 // servoSweep(0, 180, 1);    // Function call to start servo sweep from 0-180 degrees
 startProgram();
 for(int i = 0; i < DOORS; i++){
  //Make servo work
        forward(DISTANCE);
        myservo.write(180);
        Serial.println("Hello");
        //leftSequence[i] = detectDoor();
        myservo.write(0);
        //rightSequence[i] = detectDoor();
        delay(1000);
  }

  forward(DISTANCE);
  for(int i = 0; i < DOORS; i++){
        backward(DISTANCE);
        //servoTurn(180);
        rightSequenceBack[i] = detectDoor();
        //servoTurn(0);
        leftSequenceBack[i] = detectDoor();
        delay(1000);
      }
   backward(DISTANCE);
//concatArray(leftSequence, rightSequence, leftSequenceBack, rightSequenceBack);
//for (int k = 0; k < 16; k++){
//  Serial.print(result[k]);
//}
//while(true){
//Serial.println(binaryToDecimal(result, 16));
//Serial.println("");
//}
}

void concatArray(int arr1[], int arr2[], int arr3[], int arr4[]){
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
  
  while (leftPulse < totalPulses && rightPulse < totalPulses) {    // Check Encoders against target pulse count
    leftPulse = getEncoderLeftCnt();                               // Get Left Encoder Count
    rightPulse = getEncoderRightCnt();                             // Get Right Encoder Count

    /* If the left encoder count is less than the right, increase
       the left motor speed by 1 */
    if (leftPulse < rightPulse) {
      setRawMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED + 3);
      setRawMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED);
    }
    
    /* If the right encoder count is less than the left, increase
       the right motor speed by 2 */
    if (rightPulse < leftPulse) {
      setRawMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED);
      setRawMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED + 3);
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
    Serial.println(leftPulse);
    /* If the left encoder count is less than the right, increase
       the left motor speed by 1 */
    if (leftPulse < rightPulse) {
      setRawMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED_BACK + 3);
      setRawMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED_BACK);
    }
    
    /* If the right encoder count is less than the left, increase
       the right motor speed by 2 */
    if (rightPulse < leftPulse) {
      setRawMotorSpeed(LEFT_MOTOR, LEFT_MOTOR_SPEED_BACK);
      setRawMotorSpeed(RIGHT_MOTOR, RIGHT_MOTOR_SPEED_BACK + 3);
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

void startProgram() {
  /* Setup message to print to serial montor */
  String btnMsg = "Push left button on Launchpad to start lab program.\n";
  waitBtnPressed(LP_LEFT_BTN, btnMsg, RED_LED); //Setup button, msg, LED
  delay(1000);
  
}

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
  float middleDistance =  pings[(numPings / 2)];

  if(middleDistance > minWallDist){
    return 1;
  }else{
    return 0;
  }
}

//RETURNS THE DISTANCE MEASURED BY THE HC-SR04 DISTANCE SENSOR
float getDistance() {
  float echoTime;                   //variable to store the time it takes for a ping to bounce off an object
  //float calculatedDistanceInches;         //variable to store the distance calculated from the echo time
  float calculatedDistanceCentimeters;         //variable to store the distance calculated from the echo time

  //send out an ultrasonic pulse that's 10us long
  digitalWrite(trigPin, LOW); //ensures a clean pulse beforehand
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);

  echoTime = pulseIn(echoPin, HIGH);      //use the pulsein command to see how long it takes for the
                                          //pulse to bounce back to the sensor in microseconds
  //calculatedDistanceInches = echoTime / 148.0;  //calculate the distance in inches of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)
  calculatedDistanceCentimeters = echoTime / 58.0;  //calculate the distance in centimeters of the object that reflected the pulse (half the bounce time multiplied by the speed of sound)

  return calculatedDistanceCentimeters;              //send back the distance that was calculated
}

long binaryToDecimal(int arr[], int size) {
  long decimal = 0;
  for (int i = 0; i < size; i++) {
    decimal = decimal * 2 + arr[i];
  }
  return decimal;
}
