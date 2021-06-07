#ifndef COMPUTEPATHCOMMAND_H
#define COMPUTEPATHCOMMAND_H

#include <PID_v2.h>
#include "./Command.h"
#include "../hardware/IMU.h"
#include "../utils/List.h"

/**
 * Path to schedule commands to follow a specified path (in reverse order)
 * Used to backtrack from end to start
*/
class ComputePathCommand: public Command {
  private:
    List<IMU::Position> *path;
    PID_v2 *controller;

  public:
    ComputePathCommand(List<IMU::Position> *path, PID_v2 *controller);

    void init() override;
    bool isFinished() override;

};

#endif