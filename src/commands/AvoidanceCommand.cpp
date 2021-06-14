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
  // path.pop();
  // Stop the robot quickly to prevent collision
  Drivetrain::setOutput(0,0);
  timeout = millis()+250;
  // while(1);
}

void AvoidanceCommand::periodic(){
  // Support an internal timeout to improve reliability
  if(millis() < timeout) return;
  float usDist = getUltrasonicDistance()->getLast();
  Serial.println(usDist);
  if(getLeftIR()->getLast()){
    Drivetrain::setOutput(-100, 175);
    endTimeout = millis() + 100;
  } else if(usDist > 0 && usDist < 20){
    // TODO: Increase cleareance to allow for sloppy positional tracking
    // Save path around obstacle for return path
    if(millis() % 250 == 0){
      path.add(IMU::getPosition());
    }
    // Exponential error to prevent overcorrecting
    float err = pow(15-usDist, 3)*0.005;
    Drivetrain::setOutput(200-err, 200+err);
    endTimeout = millis() + 250;
  } else if(getRightIR()->getLast()){
    Drivetrain::setOutput(-175, 175);
    endTimeout = millis() + 100;
  } else {
    Drivetrain::setOutput(200,200);
  }
}

void AvoidanceCommand::end(){
  Drivetrain::setOutput(0,0);
  // Add point at end of avoidance
  path.add(IMU::getPosition());
}

bool AvoidanceCommand::isFinished(){
  return !isObstacle() && millis() > endTimeout;
}

bool AvoidanceCommand::isObstacle(){
  float usDist = getUltrasonicDistance()->getLast();
  return (Sensors::getLeftIR()->getLast() || Sensors::getRightIR()->getLast() || (usDist > 0 && usDist < 20));
}
