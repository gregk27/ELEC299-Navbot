#include "AvoidanceCommand.h"
#include "../hardware/Sensors.h"
#include "../hardware/IMU.h"
#include "../hardware/Drivetrain.h"
#include "../utils/List.h"

using namespace Sensors;
extern List<IMU::Position> path;

AvoidanceCommand::AvoidanceCommand(){
}

void AvoidanceCommand::init(){
  IMU::Position pos = IMU::getPosition();
  holdHeading = pos.heading;
  // Remove the last path position to increase clearance of obstacle
  path.pop();
  Drivetrain::setOutput(0,0);
  // while(1);
}

void AvoidanceCommand::periodic(){
  float usDist = getUltrasonicDistance()->getLast();
  Serial.println(usDist);
  if(getLeftIR()->getLast()){
    Drivetrain::setOutput(0, 175);
  } else if(usDist > 0 && usDist < 20){
    // TODO: Increase cleareance to allow for sloppy positional tracking
    // Save path around obstacle for return path
    if(millis() % 250 == 0){
      path.add(IMU::getPosition());
    }
    // Exponential error to prevent overcorrecting
    float err = pow(15-usDist, 3)*0.1;
    Drivetrain::setOutput(200-err, 200+err);
  } else if(getRightIR()->getLast()){
    Drivetrain::setOutput(0, 175);
  } else {
    
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
