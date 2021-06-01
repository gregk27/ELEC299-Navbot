#include "./src/hardware/Drivetrain.h"
#include "./src/hardware/Sensors.h"
#include "./src/hardware/IMU.h"


void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  Drivetrain::init(1, 3, 3, 2);
  Sensors::init();
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
  int mode = 4;
  
  // Select navigation mode
  if(Sensors::isOnMarker()){
    // mode = 4;
  } else if(Sensors::getLeftIR()->getSmoothed()) {
    // mode = 3;
  } else if(usDist > 0 && usDist < 20){
    mode = 2;
  } else if(Sensors::getRightIR()->getSmoothed()){
    // mode = 1;
  }

  // -----
  //  Act
  // -----
  // Output location for plotting
  // IMU::toPlot();
  Serial.println(usDist);

  switch(mode){
  case 4:
    Drivetrain::setOutput(0,0);
    break;
  case 3:
    pivotHeading(pos.heading-1);
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
    pivotHeading(pos.heading-1);
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
  Drivetrain::setOutput(speed - err*200, speed + err*200);
}

void pivotHeading(int heading){
  IMU::Position pos = IMU::getPosition();
  float err = heading-pos.heading;
  Drivetrain::setOutput(err*250, -err*250);
}