//Software to test a maze running robot sensor and motors
//robot will stop if objects are close

//IR SENSOR SETUP
// Include library
#include <SharpIR.h>
// Define model and input pins
#define sensorFpin A0
#define sensorLpin A1
#define sensorRpin A2
#define model 1080
//variable to store distance
int fDistance;
int lDistance;
int rDistance;
// Create a new instance of the SharpIR class
SharpIR sensorF = SharpIR(sensorFpin, model);
SharpIR sensorL = SharpIR(sensorLpin, model);
SharpIR sensorR = SharpIR(sensorRpin, model);

//PIN DECLERATIONS
//left motor
int enMotorL = 9; //left motor speed pin
int motorLF = 8; //left motor forward
int motorLB = 7; //left motor backward
//right motor
int enMotorR = 3; //right motor speed pin
int motorRF = 5; //right motor forward
int motorRB = 4; //right motor backward
//other variables
int fSensorState; //stores distance state of front sensor
int lSensorState; //stores distance state of left sensor
int rSensorState; //stores distance state of right sensor
char newDir = 'S'; //robot direction control (set to STOP by default)
char currentDir; //stores current direction for comparison to new dir
int threshold = 15; //threshold of sensor distance input before direction needs updating


void setup() {
  Serial.begin(115200); // Open serial monitor
  Serial.println("Starting");

  //set motor pins as output
  pinMode(motorLF, OUTPUT);
  pinMode(motorRF, OUTPUT);
  pinMode(motorLB, OUTPUT);
  pinMode(motorRB, OUTPUT);
  pinMode(enMotorR, OUTPUT);
  pinMode(enMotorL, OUTPUT);

  //set default speed of motors
  analogWrite(enMotorR, 255); // 0-255
  analogWrite(enMotorL, 255); // 0-255

  //set default state of motors
    digitalWrite(motorLF, LOW);
    digitalWrite(motorLB, LOW);
    digitalWrite(motorRF, LOW);
    digitalWrite(motorRB, LOW);
}


void loop() {
  Serial.println("Checking Sensors");
  checkSensors();
  Serial.println("Calculating Direction");
  calculateDirection();
  Serial.println("Setting New Direction");
  setDirection();
  Serial.println("Waiting (Comment out delay in loop function to stop this)");
  delay(100); //delay to monitor serial, COMMENT OUT

}

void checkSensors() {
  fDistance = sensorF.distance();
  lDistance = sensorL.distance();
  rDistance = sensorR.distance();
  Serial.println("Front Sensor Distance: " + fDistance);
  Serial.println("Left Sensor Distance: " + lDistance);
  Serial.println("Right Sensor Distance: " + rDistance);
}

void calculateDirection () {
  if (fDistance > threshold) { //if robot front is too close to object
    newDir = 'S';                    //STOP
    }

  else if (lDistance > threshold) { //if robot front is too close to object
    newDir = 'S';                    //STOP
    }

  else if (rDistance > threshold) { //if robot front is too close to object
    newDir = 'S';                    //STOP
    }

  else { //if no obstacle go forward
    newDir = 'F';
  }
}

void setDirection() {
  if (newDir == currentDir) { //checks if already moving in new direciton
  Serial.println("No update to direction made");
    ; //do nothing
  }

  else if (newDir == 'F') { //forward
    digitalWrite(motorLF, HIGH);   //left wheel forward
    digitalWrite(motorLB, LOW);
    digitalWrite(motorRF, HIGH);  //right wheel forward
    digitalWrite(motorRB, LOW);
    Serial.println("Moving Forward");
  }

  else if (newDir == 'S') { //stop
    digitalWrite(motorLF, LOW);
    digitalWrite(motorLB, LOW);
    digitalWrite(motorRF, LOW);
    digitalWrite(motorRB, LOW);
    Serial.println("STOPPING");
  } 

  currentDir = newDir; //stores updated direction
  Serial.print("Current Dir; ");
  Serial.println(currentDir);
}
