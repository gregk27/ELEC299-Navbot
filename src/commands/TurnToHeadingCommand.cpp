#include "./TurnToHeadingCommand.h"
#include "../hardware/Drivetrain.h"


TurnToHeadingCommand::TurnToHeadingCommand(float target, bool absolute, int speed, float tol, unsigned long timeout, PID_v2 *controller) {
  this->target = target;
  this->absolute = absolute;
  this->speed = speed;
  this->tol = tol;
  this->timeout = timeout;
  this->controller = controller;
}

void TurnToHeadingCommand::setTarget(float target, bool absolute){
  // Update variables
  this->target = target;
  this->absolute = absolute;
    // If using a relative heading, then add the current one before proceeding
  if(!absolute) target += Odom::getPosition().heading;
}


void TurnToHeadingCommand::init(){
  // If using a relative heading, then add the current one before proceeding
  if(!absolute) target += Odom::getPosition().heading;

  // Start the PID controller
  controller->SetTunings(300, 5, 25);
  controller->Start(0, 0, 0);
  controller->SetOutputLimits(-255, 255);
  // Initialise timeout
  timeout += millis();
}

void TurnToHeadingCommand::periodic(){
  // Get heading error and feed into controller
  float hdgOut = controller->Run(Odom::angleTo(target));
  // Pivot up to max speed
  Drivetrain::setOutput(hdgOut, -hdgOut, speed);
}

void TurnToHeadingCommand::end(){
  Drivetrain::setOutput(0,0);
}

bool TurnToHeadingCommand::isFinished(){
  // End when within tolerance or timed out
  return abs(Odom::angleTo(target)) < tol || millis() > timeout;
}