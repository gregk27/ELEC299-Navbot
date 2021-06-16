#include "./DrivePathCommand.h"

DrivePathCommand::DrivePathCommand(List<Odom::Location> **path, bool reverse, byte speed, float tol, PID_v2 *controller)
: DriveToPositionCommand(0, 0, speed, tol, controller, 0x0){
  this->path = path; 
  this->reverse = reverse;
}

bool DrivePathCommand::setTargetNode(int idx){
    // If it's travelled the full path, then end
    if(idx >= (*path)->size()) return false;
    // Otherwise, target the next node
    Odom::Location pos = (**path)[idx];
    targetX = pos.x;
    targetY = pos.y;
    return true;
}

void DrivePathCommand::init(){
  // Reverse the path if requested
  if(reverse){
    (*path)->reverse();
  }
  // Initialise parent command
  DriveToPositionCommand::init();
  // Set navigation to first node
  idx = 0;
  setTargetNode(0);
}

bool DrivePathCommand::isFinished(){
  // If the robot has arrived at the current node, move on to the next
  if(DriveToPositionCommand::isFinished()){
    idx ++;
    // If updating the node fails, then the path has been completed
    return !setTargetNode(idx);
  }
  return false;
}