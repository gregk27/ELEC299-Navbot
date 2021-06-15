#ifndef DRIVETOPOINTCOMMAND_H
#define DRIVETOPOINTCOMMAND_H

#include <PID_v2.h>
#include "./Command.h"
#include "../hardware/IMU.h"
#include "../utils/List.h"

class DriveToPositionCommand: public Command {
  protected:
    float targetX;
    float targetY;

    PID_v2 *controller;
  
  private:
    float tol;
    byte speed;

    List<IMU::Location> *path;

  public:
    DriveToPositionCommand(float x, float y, byte speed, float tol, PID_v2 *controller, List<IMU::Location> *path);

    // Functions are virtual to allow for overriding
    
    virtual void init() override;
    void periodic() override;
    void end() override;
    virtual bool isFinished() override;

};

#endif