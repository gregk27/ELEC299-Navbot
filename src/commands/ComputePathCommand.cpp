#include "./ComputePathCommand.h"
#include "./DriveToPositionCommand.h"
#include "../../Scheduler.h"

ComputePathCommand::ComputePathCommand(List<IMU::Position> *path){
  this->path = path;
}

void ComputePathCommand::init(){
  // Schedule commands to reverse track along path
  for(int i = path->size()-1; i >= 0; i--){
    IMU::Position pos = (*path)[i];
    Scheduler::master->addCommand(new DriveToPositionCommand(pos.x, pos.y, 200, 5));
  }
  // Add command to drive to start from first point
  Scheduler::master->addCommand(new DriveToPositionCommand(0, 0, 200, 1));
}

bool ComputePathCommand::isFinished(){
  return true;
}