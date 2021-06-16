#ifndef COMPUTEPATHCOMMAND_H
#define COMPUTEPATHCOMMAND_H

#include "./Command.h"
#include "../utils/List.h"
#include "../hardware/Odom.h"

class ComputePathCommand: public Command {
  private:
    List<Odom::Location> *pathIn;
    List<Odom::Location> **pathOut;

  public:
    ComputePathCommand(List<Odom::Location> *pathIn, List<Odom::Location> **pathOut);

    void init() override;
    bool isFinished() override;

};

#endif