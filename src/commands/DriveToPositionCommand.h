#ifndef DRIVETOPOINTCOMMAND_H
#define DRIVETOPOINTCOMMAND_H

#include <PID_v2.h>
#include "./Command.h"
#include "../hardware/IMU.h"

class DriveToPositionCommand: public Command {
  private:
    float targetX;
    float targetY;
    float tol;
    byte speed;

    PID_v2 *controller;

  public:
    DriveToPositionCommand(float x, float y, byte speed, float tol, PID_v2 *controller);

    void init() override;
    void periodic() override;
    void end() override;
    bool isFinished() override;

};

#endif