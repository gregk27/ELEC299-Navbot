#include "./TurnToHeadingCommand.h"
#include "../hardware/Drivetrain.h"


float TurnToHeadingCommand::kP_POS = 0;

TurnToHeadingCommand::TurnToHeadingCommand(float target, bool absolute, byte speed, float tol, unsigned int timeout)
  :controller(400, 2, 25, PID::Direct) {
  this->target = target;
  this->absolute = absolute;
  this->speed = speed;
  this->tol = tol;
  this->timeout = timeout;
}

void TurnToHeadingCommand::init(){
  // Save initial encoder positions
  initialLeftPos = Drivetrain::leftEncoder->getPosition();
  initialRightPos = Drivetrain::rightEncoder->getPosition();

  // If using a relative heading, then add the current one before proceeding
  if(!absolute) target += IMU::getPosition().heading;

  // Start the PID controller
  controller.Start(IMU::getPosition().heading, 0, target);
  controller.SetOutputLimits(-255, 255);

  timeout += millis();
  // Serial.println(target);
}

void TurnToHeadingCommand::periodic(){
  float hdgOut = controller.Run(IMU::getPosition().heading);
  // Serial.print(hdgOut);
  // Serial.print("\t");
  // Serial.print(IMU::getPosition().heading);
  // Serial.print("\t");
  // Serial.print(controller.GetSetpoint());
  // Serial.print("\t");
  // Serial.print(5);
  // Serial.print("\t");
  // Serial.println(-5);
  float lDelta = initialLeftPos - Drivetrain::leftEncoder->getPosition();
  float rDleta = initialRightPos - Drivetrain::rightEncoder->getPosition();
  float lrErr = abs(lDelta)-abs(rDleta);
  Drivetrain::setOutput(-hdgOut-lrErr*kP_POS, hdgOut+lrErr*kP_POS, speed);
}

void TurnToHeadingCommand::end(){
  Drivetrain::setOutput(0,0);
}

bool TurnToHeadingCommand::isFinished(){
  // Serial.print(millis());
  // Serial.print("\t");
  // Serial.println(timeout);
  return (abs(target-IMU::getPosition().heading) < tol) || millis() > timeout;
}