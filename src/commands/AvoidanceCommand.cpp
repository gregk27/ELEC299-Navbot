#include "AvoidanceCommand.h"
#include "../hardware/Sensors.h"
#include "../hardware/IMU.h"
#include "../hardware/Drivetrain.h"

using namespace Sensors;

void AvoidanceCommand::driveHeading(int speed, float hdg){
  IMU::Position pos = IMU::getPosition();
  float err = hdg-pos.heading;
  // Serial.println(err);
  Drivetrain::setOutput(speed - err*500, speed + err*500);
}

void AvoidanceCommand::pivotHeading(int heading){
  IMU::Position pos = IMU::getPosition();
  float err = heading-pos.heading;
  Drivetrain::setOutput(err*250, -err*250);
}

AvoidanceCommand::AvoidanceCommand(){
}

void AvoidanceCommand::init(){
  IMU::Position pos = IMU::getPosition();
  holdHeading = pos.heading;
}

void AvoidanceCommand::periodic(){
  float usDist = getUltrasonicDistance()->getSmoothed();
  IMU::Position pos = IMU::getPosition();
  if(getLeftIR()->getSmoothed()){
    driveHeading(100, pos.heading+1);
  } else if(usDist > 0 && usDist < 20){
    if(usDist > 12){
      driveHeading(210, pos.heading+(13-usDist)*0.05);
      holdHeading = pos.heading;
    } else if (usDist < 7){
      driveHeading(210, pos.heading+(8-usDist)*0.05);
      holdHeading = pos.heading;
    } else {
      driveHeading(210, holdHeading);
    }
  } else if(getRightIR()->getSmoothed()){
    driveHeading(100, pos.heading+1);
  }
}

void AvoidanceCommand::end(){
  Drivetrain::setOutput(0,0);
}

bool AvoidanceCommand::isFinished(){
  float usDist = getUltrasonicDistance()->getSmoothed();
  return !(Sensors::getLeftIR()->getSmoothed() || Sensors::getRightIR()->getSmoothed() || (usDist > 0 && usDist < 20));
}
