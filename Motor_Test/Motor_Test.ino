//Software to test a maze running robot

//IR SENSOR SETUP
// Include library
#include <SharpIR.h>
// Define model and input pins
#define sensorFpin A0
#define model 1080
//variable to store distance
int fDistance;
int lDistance;
int rDistance;
// Create a new instance of the SharpIR class
SharpIR sensorF = SharpIR(sensorFpin, model);


//other variables
int fSensorState; //stores distance state of front sensor
int threshold = 15; //threshold of sensor distance input before direction needs updating


void setup() {
  Serial.begin(115200); // Open serial monitor
  Serial.println("Starting");

}


void loop() {
  Serial.println("Checking Sensors");
  checkSensors();
  delay(100); //delay to monitor serial, COMMENT OUT

}

void checkSensors() {
  fDistance = sensorF.distance();
  Serial.println(fDistance);
}

void calculateDirection () {
  if (fDistance > threshold) { //if robot front is too close to object
    newDir = 'S';                    //STOP
    }

  else { //if no obstacle go forward
    newDir = 'F';
  }
}
