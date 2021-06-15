#ifndef DRIVEPATHCOMMAND_H
#define DRIVEPATHCOMMAND_H

#include <PID_v2.h>
#include "./DriveToPositionCommand.h"
#include "../hardware/IMU.h"
#include "../utils/List.h"

class DrivePathCommand: public DriveToPositionCommand {
  private:
    int idx;
    bool reverse;
    List<IMU::Location> *path;

    bool setTargetNode(int idx);

  public:
    DrivePathCommand(List<IMU::Location> *path, bool reverse, byte speed, float tol, PID_v2 *controller);

    void init() override;
    bool isFinished() override;

};

#endif