//Software to test motors

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
char newDir = 'S'; //robot direction control (set to STOP by default)
char currentDir; //stores current direction for comparison to new dir


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
  newDir = 'F'
  Serial.println("Setting New Direction");
  setDirection();
  delay(1000);
  
  newDir = 'S'
  Serial.println("Setting New Direction");
  setDirection();
  delay(1000);

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
