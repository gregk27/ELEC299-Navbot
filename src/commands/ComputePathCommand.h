#ifndef COMPUTEPATHCOMMAND_H
#define COMPUTEPATHCOMMAND_H

#include "./Command.h"
#include "../utils/List.h"
#include "../hardware/IMU.h"

class ComputePathCommand: public Command {
  private:
    List<IMU::Location> *pathIn;
    List<IMU::Location> **pathOut;

  public:
    ComputePathCommand(List<IMU::Location> *pathIn, List<IMU::Location> **pathOut);

    void init() override;
    bool isFinished() override;

};

#endif