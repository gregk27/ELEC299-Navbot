#ifndef COMPUTEPATHCOMMAND_H
#define COMPUTEPATHCOMMAND_H

#include "./Command.h"
#include "../utils/List.h"
#include "../hardware/IMU.h"

class ComputePathCommand: public Command {
  private:
    List<IMU::Position> *pathIn;
    List<IMU::Position> *pathOut;

  public:
    ComputePathCommand(List<IMU::Position> *pathIn, List<IMU::Position> *pathOut);

    void init() override;
    bool isFinished() override;

};

#endif