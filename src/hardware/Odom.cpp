#include "./Odom.h"
#include <Arduino.h>
#include "./Drivetrain.h"
#include "./Gyro.h"

// Positional computations taken from: https://ocw.mit.edu/courses/electrical-engineering-and-computer-science/6-186-mobile-autonomous-systems-laboratory-january-iap-2005/study-materials/odomtutorial.pdf

using namespace Odom;

// -------------------
// Helper functions
// -------------------

#define INTERVAL 50
#define WIDTH 13.25

// The robot's position
Position pos;

// The last left encoder count
float lastLeft = 0;
// The last right encoder count
float lastRight = 0;

// On B timer, compute new position
ISR(TIMER1_COMPB_vect){
  float dLeft = Drivetrain::leftEncoder->getPositionCm() - lastLeft;
  float dRight = Drivetrain::rightEncoder->getPositionCm() - lastRight;

  lastLeft = Drivetrain::leftEncoder->getPositionCm();
  lastRight = Drivetrain::rightEncoder->getPositionCm();

  float dCentre = (dLeft+dRight)/2.0;

  pos.x += dCentre*sin(pos.heading);
  pos.y += dCentre*cos(pos.heading);
  pos.heading = Gyro::getYaw()->getSmoothed();
}

// -------------------
// Namepsace functions
// -------------------

void Odom::init(){
  // Prepare speed timer interrupt, see https://busylog.net/FILES2DW/doc8161.pdf page 134
  noInterrupts();
  TCCR1A = 4; // Set clock to mode 4 (Clear timer on compare match)
  TCCR1B = 8 | 5; // Set bit 3 for CTC mode, lowest 3 bits as 101 to use 1024 prescaling
              // 16,000,000/1024 = 15,625 Hz, tick once per 0.064ms
  OCR1B= (int) (10/0.064); // Get count required for 50ms delay (Use B timer)
  TIMSK1 |= 4; // Enable B timer
  interrupts();

  // Initialise positional information
  pos.x = 0;
  pos.y = 0;
  pos.heading = 0;
  Drivetrain::resetPosition();
}

Position Odom::getPosition(){
  return pos;
}

void Odom::toPlot(){
  Serial.print(pos.x);
  Serial.print(F("\t"));
  Serial.print(pos.y);
  Serial.print(F("\t"));
  Serial.print(pos.heading*180/PI);
  Serial.println(F(""));
}

float Odom::headingTo(float x, float y){
  return atan2((x-pos.x), y-pos.y);
}

float Odom::angleTo(float h){
  float err = h-pos.heading;
  if(err > PI) return err-2*PI;
  else if (err < -PI) return err+2*PI;
  else return err;
}