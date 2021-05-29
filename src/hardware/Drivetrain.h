#ifndef DRIVETRAIN_H
#define DRIVETRAIN_H
#include <AFMotor.h>
#include "./Encoder.h"


namespace Drivetrain { 
  /**
   * Drivetrain's left motor
  */
  extern AF_DCMotor *leftMotor;
  /**
   * Drivetrain's right motor
  */
  extern AF_DCMotor *rightMotor;

  /**
   * Left motor's encoder
  */
  extern Encoder *leftEncoder;
  /**
   * Right motor's encoder
  */
  extern Encoder *rightEncoder;

  /**
   * Initialise the drivetrain
   * @param lMotor The motor shield terminal for the left motor
   * @param rMotor The motor shield terminal for the right motor
   * @param lEncoder The pin for the left encoder
   * @param lEncoder The pin for the right encoder
  */
  void init(int lMotor, int rMotor, int lEncoder, int rEncoder);

  /**
   * Set the motor output
   * @param l The output for the left motor, -255 to 255
   * @param r THe output for the right motor, -255 to 255
  */
  void setOutput(int l, int r);

  /**
   * Get correction values for straight driving
   * @return corrective value, add to left and subtract from right
  */
  int getStraightCorrection();

  /**
   * Get the average position of the encoders, in ticks
  */
  int averagePosition();

  /**
   * Reset the encoders to 0
  */
  void resetPosition();

}


#endif