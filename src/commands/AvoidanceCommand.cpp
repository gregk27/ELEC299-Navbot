#include "AvoidanceCommand.h"
#include "../hardware/Sensors.h"
#include "../hardware/Drivetrain.h"
#include "../../Scheduler.h"

using namespace Sensors;

// Non-class function to save position on path
void AvoidanceCommand::savePosition(){
    // If path is null, then do nothing
    if(!path) return;
    Odom::Position pos = Odom::getPosition();
    // Add the point left and behind of the vehicle to increase clearance
    // Equations from https://gamedev.stackexchange.com/a/79779
    int xOffset = -50;
    int yOffset = 0;
    path->add({
      (int) (pos.x + cos(-pos.heading)*(xOffset) - sin(-pos.heading)*(yOffset)),
      (int) (pos.y + cos(-pos.heading)*(yOffset) + sin(-pos.heading)*(xOffset))
    });
}

AvoidanceCommand::AvoidanceCommand(List<Odom::Location> *path){
  this->path = path;
}

void AvoidanceCommand::init(){
  // Stop the drivetrain and wait before proceeding
  Drivetrain::setOutput(0,0);
  timeout = millis()+250;
}

void AvoidanceCommand::periodic(){
  // Save the position periodically
  if(Scheduler::master->getIteration() % 20 == 0){
    savePosition();
  }

  // Support an internal timeout to improve reliability
  if(millis() < timeout) return;

  float usDist = getUltrasonicDistance()->getLast();
  if(getLeftIR()->getLast()){ // Case 3
    // Pivot, but maintain some forward motion to ensure obstacle remains in sight
    Drivetrain::setOutput(-100, 175);
    endTimeout = millis() + 100;
  
  } else if(usDist > 0 && usDist < 20){ // Case 2
    // Exponential error to prevent under/overcorrecting in various areas
    float err = pow(15-usDist, 3)*0.005;
    Drivetrain::setOutput(200-err, 200+err);
    endTimeout = millis() + 250;
  
  } else if(getRightIR()->getLast()){ // Case 1
    // Pivot in-place as robot should now be very close to obstacle
    Drivetrain::setOutput(-175, 175);
    endTimeout = millis() + 100;
  
  } else { // Case 0
    // Drive ahead while waiting for command to end to increase distance
    Drivetrain::setOutput(200,200);
  }
}

void AvoidanceCommand::end(){
  Drivetrain::setOutput(0,0);
  // Add point at end of avoidance
  savePosition();
}

bool AvoidanceCommand::isFinished(){
  // Finish when obstacle no longer detected, and some time has passed to increase clearance
  return !isObstacle() && millis() > endTimeout;
}

bool AvoidanceCommand::isObstacle(){
  float usDist = getUltrasonicDistance()->getLast();
  return (Sensors::getLeftIR()->getLast() || Sensors::getRightIR()->getLast() || (usDist > 0 && usDist < 20));
}
