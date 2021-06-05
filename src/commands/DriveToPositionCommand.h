#ifndef DRIVETOPOINTCOMMAND_H
#define DRIVETOPOINTCOMMAND_H

#include "./Command.h"
#include "../hardware/IMU.h"

class DriveToPositionCommand: public Command {
  private:
    float targetX;
    float targetY;
    float tol;
    int speed;
    int accumErr;

    static float kP;
    static float kI;

  public:
    DriveToPositionCommand(float x, float y, int speed, float tol);

    void periodic() override;
    void end() override;
    bool isFinished() override;

};

#endif