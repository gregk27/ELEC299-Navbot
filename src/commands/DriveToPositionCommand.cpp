#include "./DriveToPositionCommand.h"
#include "../hardware/Drivetrain.h"
#include "../utils/List.h"
#include "../../Scheduler.h"
#include "./TurnToHeadingCommand.h"

using namespace IMU;
// List declared in ino file
extern List<Position> path;


DriveToPositionCommand::DriveToPositionCommand(float x, float y, byte speed, float tol, PID_v2 *controller) {
  targetX = x;
  targetY = y;
  this->tol = tol;
  this->speed = speed;
  this->controller = controller;
  this->controller->SetTunings(150, 10, 25);
}

void DriveToPositionCommand::init(){
  controller->Start(IMU::getPosition().heading, 0, headingTo(targetX, targetY));
  controller->SetOutputLimits(-255, 255);
  Serial.println("INIT");
}

void DriveToPositionCommand::periodic(){
  Position pos = getPosition();
  // Save position every 500 ms
  if(millis() % 500 == 0){
    path.add(pos);
  }
  float target = headingTo(targetX, targetY);
  controller->Setpoint(target);
  float correctOut = controller->Run(pos.heading);
  Serial.println(target-pos.heading);
  // If more than 60 degrees off, pivot to a more reasonable heading
  if(abs(target-pos.heading) > PI/3){
    // Scheduler::master->interrupt(new TurnToHeadingCommand(target-pos.heading, false, 200, 0.5, 3000));
    Drivetrain::setOutput(-correctOut, correctOut);
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