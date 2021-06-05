#include "./src/hardware/Drivetrain.h"
#include "./src/hardware/Sensors.h"
#include "./src/hardware/IMU.h"


void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  Drivetrain::init(1, 3, 3, 2);
  Sensors::init();


  // Run selftest
  // while(selfTest){};
  Drivetrain::resetPosition();

  IMU::init();

  delay(1000);
}

float holdHeading;

void loop() {
  // -------
  //  Sense
  // -------
  Sensors::periodic();
  IMU::Position pos = IMU::getPosition();
  float usDist = Sensors::getUltrasonicDistance()->getSmoothed();

  // -------
  //  Think
  // -------
  // Default to Case 0 operation
  int mode = 0;
  
  // Select navigation mode
  if(Sensors::isOnMarker()){
    mode = 4;
  } else if(Sensors::getLeftIR()->getSmoothed()) {
    mode = 3;
  } else if(usDist > 0 && usDist < 20){
    mode = 2;
  } else if(Sensors::getRightIR()->getSmoothed()){
    mode = 1;
  }

  // -----
  //  Act
  // -----
  // Output location for plotting
  // IMU::toPlot();
  Serial.println(mode);

  switch(mode){
  case 4:
    Drivetrain::setOutput(0,0);
    break;
  case 3:
    driveHeading(100, pos.heading+1);
    break;
  case 2:
    if(usDist > 12){
      driveHeading(210, pos.heading+(13-usDist)*0.05);
      holdHeading = pos.heading;
    } else if (usDist < 7){
      driveHeading(210, pos.heading+(8-usDist)*0.05);
      holdHeading = pos.heading;
    } else {
      driveHeading(210, holdHeading);
    }
    break;
  case 1:
    driveHeading(100, pos.heading+1);
    break;
  case 0:
    driveHeading(225, IMU::headingTo(0, 200));
    break;
  default: // If none of the cases, something is wrong
    Drivetrain::setOutput(0, 0);
  }

}

void driveHeading(int speed, float hdg){
  IMU::Position pos = IMU::getPosition();
  float err = hdg-pos.heading;
  // Serial.println(err);
  Drivetrain::setOutput(speed - err*500, speed + err*500);
}

void pivotHeading(int heading){
  IMU::Position pos = IMU::getPosition();
  float err = heading-pos.heading;
  Drivetrain::setOutput(err*250, -err*250);
}

int testState = 0;

/**
 * Run a self-test
 * @return false when complete
*/
bool selfTest(){
  int usd = Sensors::getUltrasonicDistance()->getSmoothed();
  if(
    testState == 0 && usd == -1 ||
    testState == 1 && usd < 10  && usd > 0 ||
    testState == 2 && !Sensors::getLeftIR()->getSmoothed()  || 
    testState == 3 && Sensors::getLeftIR()->getSmoothed()  || 
    testState == 4 && !Sensors::getRightIR()->getSmoothed()  || 
    testState == 5 && Sensors::getRightIR()->getSmoothed()  || 
    testState == 6 && Sensors::getDownwardSensor()->getSmoothed() < 10 ||
    testState == 7 && Sensors::getDownwardSensor()->getSmoothed() > 10 ||
    testState == 8 && Drivetrain::leftEncoder->getPosition() > 20 ||
    testState == 9 && Drivetrain::rightEncoder->getPosition() > 20
  ) {
    Serial.print("Completed test: ");
    Serial.println(testState);

    
    testState ++;

    
    if(testState == 8){
      Drivetrain::resetPosition();
    } else if(testState == 9){
      Drivetrain::resetPosition();
    } else if (testState == 10){
      Serial.println("All tests complete");
    }
  }
  return testState < 10;
}