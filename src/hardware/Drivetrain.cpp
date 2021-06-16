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
void setSide(AF_DCMotor *m, Encoder *e, int value, int max){
  // Set encoder and motor directions
  e->setDirection(value);
  if(value == 0){
    m->run(RELEASE);
    m->setSpeed(0);
    return;
  }
  m->run(value < 0 ? BACKWARD : FORWARD);
  // Clamp value to motor operating range: provided max, then (0-255)
  value = abs(value);
  value = value > max ? max : value;
  value = value > 255 ? 255 : value;
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

void Drivetrain::setOutput(int l, int r, int max){
  setSide(leftMotor, leftEncoder, l, max);
  setSide(rightMotor, rightEncoder, r, max);
}

void Drivetrain::setOutput(int l, int r){
  setOutput(l, r, 255);
}


int Drivetrain::getStraightCorrection(){
  int err = rightEncoder->getPosition() - leftEncoder->getPosition();
  return err*12;
}

int Drivetrain::averagePosition(){
  return (leftEncoder->getPosition() + rightEncoder->getPosition())/2;
}

void Drivetrain::resetPosition(){
  leftEncoder->reset();
  rightEncoder->reset();
}

