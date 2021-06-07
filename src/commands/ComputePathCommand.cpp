#include "./ComputePathCommand.h"
#include "./DriveToPositionCommand.h"
#include "../../Scheduler.h"
#include "../utils/MemoryFree.h"

ComputePathCommand::ComputePathCommand(List<IMU::Position> *path){
  this->path = path;
}

void ComputePathCommand::init(){
  // Schedule commands to reverse track along path
  for(int i = path->size()-1; i >= 0; i--){
    IMU::Position pos = (*path)[i];
    Serial.print(i);
    Serial.print(":");
    Serial.print(pos.x);
    Serial.print("\t");
    Serial.println(pos.y);
    delay(100);
    Serial.println(freeMemory());
    Scheduler::master->addCommand(new DriveToPositionCommand(pos.x, pos.y, 150, 10));
  }
  // Add command to drive to start from first point
  Scheduler::master->addCommand(new DriveToPositionCommand(0, 0, 200, 1));
}

bool ComputePathCommand::isFinished(){
  return true;
}