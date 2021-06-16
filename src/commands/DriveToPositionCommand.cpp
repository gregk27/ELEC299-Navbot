#include "./DriveToPositionCommand.h"
#include "../hardware/Drivetrain.h"
#include "../../Scheduler.h"

using namespace Odom;


DriveToPositionCommand::DriveToPositionCommand(float x, float y, byte speed, float tol, PID_v2 *controller, List<Location> *path){
  targetX = x;
  targetY = y;
  this->tol = tol;
  this->speed = speed;
  this->controller = controller;
  this->path = path;
}

void DriveToPositionCommand::init(){
  // Save position to path
  if(path) path->add(Odom::getPosition());
  // Configure PID controller
  controller->SetTunings(150, 5, 20);
  controller->Start(0, 0, 0);
  controller->SetOutputLimits(-255, 255);
}

void DriveToPositionCommand::periodic(){
  Position pos = getPosition();
  // Save position every 50 iterations of scheduler
  if(Scheduler::master->getIteration() % 50 == 0 && path){
    path->add(pos);
  }
  // Compute heading error and feed to PID controller
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
  // Stop when both x and y position are within tolerance
  return (abs(targetX - pos.x) <= tol && abs(targetY - pos.y) <= tol);
}