#include "./TurnToHeadingCommand.h"
#include "../hardware/Drivetrain.h"


float TurnToHeadingCommand::kP_POS = 0;

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
  if(!absolute) target += IMU::getPosition().heading;
}


void TurnToHeadingCommand::init(){
  // Save initial encoder positions
  initialLeftPos = Drivetrain::leftEncoder->getPosition();
  initialRightPos = Drivetrain::rightEncoder->getPosition();

  // If using a relative heading, then add the current one before proceeding
  if(!absolute) target += IMU::getPosition().heading;

  // Start the PID controller
  controller->SetTunings(300, 5, 25);
  controller->Start(0, 0, 0);
  controller->SetOutputLimits(-255, 255);

  timeout += millis();
  // Serial.println(target);
}

void TurnToHeadingCommand::periodic(){
  float hdgOut = controller->Run(IMU::angleTo(target));
  // Serial.print(hdgOut);
  // Serial.print("\t");
  // Serial.print(IMU::getPosition().heading);
  // Serial.print("\t");
  // Serial.print(IMU::angleTo(target));
  // Serial.print("\t");
  // Serial.println(controller->GetSetpoint());
  // Serial.print(5);
  // Serial.print("\t");
  // Serial.println(-5);
  Drivetrain::setOutput(hdgOut, -hdgOut, speed);
}

void TurnToHeadingCommand::end(){
  Drivetrain::setOutput(0,0);
}

bool TurnToHeadingCommand::isFinished(){
  // Serial.print(millis());
  // Serial.print("\t");
  // Serial.println(timeout);
  Serial.println(IMU::angleTo(target));
  return abs(IMU::angleTo(target)) < tol || millis() > timeout;
}