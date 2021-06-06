#include "AvoidanceCommand.h"
#include "../hardware/Sensors.h"
#include "../hardware/IMU.h"
#include "../hardware/Drivetrain.h"
#include "../utils/List.h"

using namespace Sensors;
extern List<IMU::Position> path;

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
  // Remove the last path position to increase clearance of obstacle
  path.pop();
}

void AvoidanceCommand::periodic(){
  float usDist = getUltrasonicDistance()->getSmoothed();
  IMU::Position pos = IMU::getPosition();
  if(getLeftIR()->getSmoothed()){
    driveHeading(100, pos.heading+1);
  } else if(usDist > 0 && usDist < 20){
    // TODO: Increase cleareance to allow for sloppy positional tracking
    // Save path around obstacle for return path
    if(millis() % 250 == 0){
      path.add(pos);
    }
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
  // Add point at end of avoidance
  path.add(IMU::getPosition());
}

bool AvoidanceCommand::isFinished(){
  float usDist = getUltrasonicDistance()->getSmoothed();
  return !(Sensors::getLeftIR()->getSmoothed() || Sensors::getRightIR()->getSmoothed() || (usDist > 0 && usDist < 20));
}
