#include "./DriveToPositionCommand.h"
#include "../hardware/Drivetrain.h"
#include "../utils/List.h"
#include "../../Scheduler.h"
#include "./TurnToHeadingCommand.h"

using namespace IMU;
// List declared in ino file
extern List<Position> path;


DriveToPositionCommand::DriveToPositionCommand(float x, float y, byte speed, float tol, PID_v2 *controller){
  targetX = x;
  targetY = y;
  this->tol = tol;
  this->speed = speed;
  this->controller = controller;
}

void DriveToPositionCommand::init(){
  path.add(IMU::getPosition());
  controller->SetTunings(150, 5, 20);
  controller->Start(0, 0, 0);
  controller->SetOutputLimits(-255, 255);
}

void DriveToPositionCommand::periodic(){
  Position pos = getPosition();
  // Save position every 500 ms
  if(millis() % 500 == 0){
    path.add(pos);
  }
  float err = angleTo(headingTo(targetX, targetY));
  float correctOut = controller->Run(err);
  // If more than 45 degrees off, pivot to a more reasonable heading
  if(abs(err) > PI/2){
    Drivetrain::setOutput(correctOut, -correctOut, speed);
  } else {
    Drivetrain::setOutput(speed+correctOut, speed-correctOut);
  }
}

void DriveToPositionCommand::end() {
  Drivetrain::setOutput(0, 0);
}

bool DriveToPositionCommand::isFinished() {
  Position pos = getPosition();
  return (abs(targetX - pos.x) <= tol && abs(targetY - pos.y) <= tol);
}