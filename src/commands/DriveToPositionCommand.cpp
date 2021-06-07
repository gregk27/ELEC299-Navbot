#include "./DriveToPositionCommand.h"
#include "../hardware/Drivetrain.h"
#include "../utils/List.h"
#include "../../Scheduler.h"
#include "./TurnToHeadingCommand.h"

using namespace IMU;
// List declared in ino file
extern List<Position> path;

float DriveToPositionCommand::kP = 250;
float DriveToPositionCommand::kI = 10;

DriveToPositionCommand::DriveToPositionCommand(float x, float y, int speed, float tol)
  :controller(150, 10, 25, PID::Direct){
  targetX = x;
  targetY = y;
  this->tol = tol;
  this->speed = speed;
  accumErr = 0;
}

void DriveToPositionCommand::init(){
  controller.Start(IMU::getPosition().heading, 0, headingTo(targetX, targetY));
  controller.SetOutputLimits(-255, 255);
}

void DriveToPositionCommand::periodic(){
  Position pos = getPosition();
  // Save position every 500 ms
  if(millis() % 500 == 0){
    path.add(pos);
  }
  float target = headingTo(targetX, targetY);
  controller.Setpoint(target);
  float correctOut = controller.Run(pos.heading);
  // If more than 90 degrees off, pivot to a more reasonable heading
  if(abs(target-pos.heading) > PI/2){
    Scheduler::master->interrupt(new TurnToHeadingCommand(target-pos.heading, false, 200, 0.5, 3000));
    Drivetrain::setOutput(0, 0);
  } else {
    Drivetrain::setOutput(speed-correctOut, speed+correctOut);
  }
}

void DriveToPositionCommand::end() {
  path.add(IMU::getPosition());
  Drivetrain::setOutput(0, 0);
}

bool DriveToPositionCommand::isFinished() {
  Position pos = getPosition();
  return (abs(targetX - pos.x) <= tol && abs(targetY - pos.y) <= tol);
}