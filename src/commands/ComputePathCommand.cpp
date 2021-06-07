#include "./ComputePathCommand.h"
#include "./DriveToPositionCommand.h"
#include "../../Scheduler.h"
#include "../utils/memtest.h"

ComputePathCommand::ComputePathCommand(List<IMU::Position> *path, PID_v2 *controller){
  this->path = path;
  this->controller = controller;
}

void ComputePathCommand::init(){
  Serial.print("Building path from: ");
  Serial.print(path->size());
  Serial.println(" points");
  Serial.println(memtest());
  // Schedule commands to reverse track along path
  for(int i = path->size()-1; i >= 0; i--){
    IMU::Position pos = (*path)[i];
    Serial.print(i);
    Serial.print(":");
    Serial.print(pos.x);
    Serial.print("\t");
    Serial.print(pos.y);
    Serial.print("\t");
    DriveToPositionCommand *c = new DriveToPositionCommand(pos.x, pos.y, 150, 10, controller);
    Serial.println((int) c);
    Scheduler::master->addCommand(c);
    Serial.println(memtest());
    delay(100);
  }
  Serial.println("Loop done");
  // Add command to drive to start from first point
  DriveToPositionCommand *c = new DriveToPositionCommand(0, 0, 200, 10, controller);
  Serial.println(memtest());
  Scheduler::master->addCommand(c);
  Serial.println("Path created");
}

bool ComputePathCommand::isFinished(){
  return true;
}