//MAZE RUNNING ROBOT W/ LEFT WALl FOLLOWING ALGORITHM

//IR SENSOR SETUP
#include <SharpIR.h>
#define model 1080 //sensor model
//input pins for each sensor
#define sensorFpin A1
#define sensorLpin A0
#define sensorRpin A2
//variables to store each sensors distance
int fDistance;
int lDistance;
int rDistance;
//new instance of the SharpIR class for each sensor
SharpIR sensorF = SharpIR(sensorFpin, model);
SharpIR sensorL = SharpIR(sensorLpin, model);
SharpIR sensorR = SharpIR(sensorRpin, model);

//MOTOR SETUP
//left motor pins
int enMotorL = 9; //left motor speed pin
int motorLF = 8; //left motor forward
int motorLB = 7; //left motor backward
//right motor pins
int enMotorR = 6; //right motor speed pin
int motorRF = 5; //right motor forward
int motorRB = 3; //right motor backward
//motor speed
int maxSpeed = 191;
int halfSpeed = 127;
int stop = 0;

//OTHER VARIABLES
char newDir = 'S'; //robot direction control (set to STOP by default)
char currentDir; //stores current direction for comparison to new dir
int thresholdLower = 8; //threshold of sensor distance input before direction needs updating, lower value
int thresholdMid = 10; //threshold of sensor distance input before direction needs updating, middle value
int thresholdUpper = 14; //threshold of sensor distance input before direction needs updating, higher value



void setup() {

  //set motor pins as output
  pinMode(motorLF, OUTPUT);
  pinMode(motorRF, OUTPUT);
  pinMode(motorLB, OUTPUT);
  pinMode(motorRB, OUTPUT);
  pinMode(enMotorR, OUTPUT);
  pinMode(enMotorL, OUTPUT);

  //set default speed of motors
  analogWrite(enMotorR, maxSpeed); // 0-255
  analogWrite(enMotorL, maxSpeed); // 0-255

  //set default state of motors
    digitalWrite(motorLF, LOW);
    digitalWrite(motorLB, LOW);
    digitalWrite(motorRF, LOW);
    digitalWrite(motorRB, LOW);
}

void loop() {

  checkSensors(); //check sensor values

  calculateAdjustment(); //calculate direction adjustement

  setDirection(); //apply adjustment

}

void checkSensors() { //checks sensor and stores distance
  fDistance = sensorF.distance();
  lDistance = sensorL.distance();
  rDistance = sensorR.distance();

}

void calculateAdjustment() { //calclates next adjustment for following left wall, all conditions are in a priortity order
  // if (fDistance < thresholdMid && lDistance < thresholdMid && rDistance < thresholdMid) {
  //    newDir = 'S';
  if (fDistance < thresholdLower) {
    newDir = 'R'; // Turn right if front is blocked
  } else if (lDistance < thresholdLower) {
    newDir = 'r'; // curve right if too close to the left wall
  } else if (lDistance > thresholdUpper) {
    newDir = 'L'; // turn left if not next to left wall
  } else if (lDistance > thresholdMid) {
    newDir = 'l'; // curve left if too far from the left wall
  } else {
    newDir = 'F'; // Move forward if no obstacles are detected
  }
}

void setDirection() {
  if (newDir == currentDir) { //prevent direction from setting if already set
    ; //do nothing
  } 
  else if (newDir == 'F') { //forward
    analogWrite(enMotorR, maxSpeed); //speed right
    analogWrite(enMotorL, maxSpeed);  //speed left
    digitalWrite(motorLF, HIGH); //left forward
    digitalWrite(motorLB, LOW);
    digitalWrite(motorRF, HIGH); //right forward
    digitalWrite(motorRB, LOW);

  } 
  else if (newDir == 'S') { //stop
    analogWrite(enMotorR, stop); //speed right
    analogWrite(enMotorL, stop);  //speed left
    digitalWrite(motorLF, LOW);
    digitalWrite(motorLB, LOW);
    digitalWrite(motorRF, LOW);
    digitalWrite(motorRB, LOW);

  } 
  else if (newDir == 'L') { //left turn
    analogWrite(enMotorR, maxSpeed); //speed right
    analogWrite(enMotorL, maxSpeed);  //speed left
    digitalWrite(motorLF, LOW);
    digitalWrite(motorLB, HIGH); //left back
    digitalWrite(motorRF, HIGH); //right back
    digitalWrite(motorRB, LOW);

  } 
  else if (newDir == 'R') { //right turn
    analogWrite(enMotorR, maxSpeed); //speed right
    analogWrite(enMotorL, maxSpeed);  //speed left
    digitalWrite(motorLF, HIGH);
    digitalWrite(motorLB, LOW);
    digitalWrite(motorRF, LOW);
    digitalWrite(motorRB, HIGH);
  }
  else if (newDir == 'r') { //right curve
    analogWrite(enMotorR, halfSpeed); //speed right 
    analogWrite(enMotorL, maxSpeed);  //speed left
    digitalWrite(motorLF, HIGH); //left forward
    digitalWrite(motorLB, LOW);
    digitalWrite(motorRF, HIGH); //right forward
    digitalWrite(motorRB, LOW);
  }
  else if (newDir == 'l') { //left curve
    analogWrite(enMotorR, maxSpeed); //speed right
    analogWrite(enMotorL, halfSpeed);  //speed left
    digitalWrite(motorLF, HIGH); //left forward
    digitalWrite(motorLB, LOW);
    digitalWrite(motorRF, HIGH); //right forward
    digitalWrite(motorRB, LOW);
  }
  currentDir = newDir;
}

