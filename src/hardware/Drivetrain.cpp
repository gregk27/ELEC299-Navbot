#include "./Drivetrain.h"
#include <Arduino.h>
#include <AFMotor.h>

using namespace Drivetrain;

// -------------------
// Helper functions
// -------------------

void leftEncoderCallback(){
  leftEncoder->onRisingEdge();
}

void rightEncoderCallback(){
  rightEncoder->onRisingEdge();
}

/**
 * Set output value for a drivetrain side
 * @param m Pointer to the side's motor
 * @param e Pointer to the side's encoder
 * @param value Ouput value to use
*/
void setSide(AF_DCMotor *m, Encoder *e, int value){
  // Set encoder and motor directions
  e->setDirection(value);
  if(value == 0){
    m->run(RELEASE);
    m->setSpeed(0);
    return;
  }
  m->run(value < 0 ? BACKWARD : FORWARD);
  // Clamp value to motor operating range (0-255)
  value = value < 0 ? -value : value;
  value = value > 220 ? 220 : value;
  // Set the motor speed
  m->setSpeed(value);
}


// -------------------
// Namepsace functions
// -------------------

AF_DCMotor* Drivetrain::leftMotor;
AF_DCMotor* Drivetrain::rightMotor;
Encoder* Drivetrain::leftEncoder;
Encoder* Drivetrain::rightEncoder;


void Drivetrain::init(int lMotor, int rMotor, int lEncoder, int rEncoder) {
  leftMotor = new AF_DCMotor(lMotor);
  rightMotor = new AF_DCMotor(rMotor);

  leftEncoder = new Encoder();
  rightEncoder = new Encoder();

  attachInterrupt(digitalPinToInterrupt(lEncoder), leftEncoderCallback, CHANGE);
  attachInterrupt(digitalPinToInterrupt(rEncoder), rightEncoderCallback, CHANGE);

  leftMotor->run(BRAKE);
  rightMotor->run(BRAKE);
  
}

void Drivetrain::setOutput(int l, int r){
  setSide(leftMotor, leftEncoder, l);
  setSide(rightMotor, rightEncoder, r);
}

int Drivetrain::getStraightCorrection(){
  int err = rightEncoder->getPosition() - leftEncoder->getPosition();
  // Serial.print(Drivetrain::leftEncoder->getPosition());
  // Serial.print("\t");
  // Serial.print(Drivetrain::rightEncoder->getPosition());
  // Serial.print("\t");
  // Serial.println(err);
  return err*12;
}

int Drivetrain::averagePosition(){
  return (leftEncoder->getPosition() + rightEncoder->getPosition())/2;
}

void Drivetrain::resetPosition(){
  leftEncoder->reset();
  rightEncoder->reset();
}

