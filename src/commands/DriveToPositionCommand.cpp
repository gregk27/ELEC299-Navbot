#include "./DriveToPositionCommand.h"
#include "../hardware/Drivetrain.h"

using namespace IMU;

float DriveToPositionCommand::kP = 400;
float DriveToPositionCommand::kI = 10;

DriveToPositionCommand::DriveToPositionCommand(float x, float y, int speed, float tol){
  targetX = x;
  targetY = y;
  this->tol = tol;
  this->speed = speed;
  accumErr = 0;
}

void DriveToPositionCommand::periodic(){
  Position pos = getPosition();
  float err = headingTo(targetX, targetY) - pos.heading;
  // If more than 90 degrees off, pivot to a more reasonable heading
  if(abs(err) > PI/2){
    Drivetrain::setOutput(-err*kP/2, err*kP/2);
  } else {
    accumErr += err;
    
    Drivetrain::setOutput(speed-err*kP-accumErr*kI, speed+err*kP+accumErr*kI);
  }
}

void DriveToPositionCommand::end() {
  Drivetrain::setOutput(0, 0);
}

bool DriveToPositionCommand::isFinished() {
  Position pos = getPosition();
  return (abs(targetX - pos.x) <= tol && abs(targetY - pos.y) <= tol);
}