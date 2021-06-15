#include "./SearchCommand.h"
#include "../hardware/Sensors.h"
#include "../../Scheduler.h"

void SearchCommand::setTarget(float x, float y){
    targetX = x;
    targetY = y;
}

SearchCommand::SearchCommand(byte speed, float centre, byte width, byte height, PID_v2 *controller, PID_v2 *turnController)
: DriveToPositionCommand(0,0,speed, 10, controller, 0x0){
    this->centre = centre;
    this->width = width;
    this->height = height;
    turnCommand = new TurnToHeadingCommand(0, true, 200, 0.1, 3000, turnController);
}

void SearchCommand::init(){
    i = 0;
    targetY = IMU::getPosition().y;
    DriveToPositionCommand::init();
    // Change to better tunings
    controller->SetTunings(100, 0, 10);
}


bool SearchCommand::isFinished(){
    // Stop when the marker is found
    if(Sensors::isOnMarker()){
        return true;
    }
    
    /*  Driving pattern:
    *   |_______________
    *   3              | 2
    *   _______________| 1
    *   | 0
    */
    if(DriveToPositionCommand::isFinished()){
        Serial.println(i);
        switch(i % 4){
            case 0:
                // Drive to right extreme
                setTarget(centre-width, targetY);
                // Turn to heading
                turnCommand->setTarget(-PI/2, true);
                Scheduler::master->interrupt(turnCommand);
            break;
            case 1:
                // Drive up
                setTarget(targetX, targetY+height);
                // Turn to heading
                turnCommand->setTarget(0, true);
                Scheduler::master->interrupt(turnCommand);
            break;
            case 2:
                // Drive to left extreme
                setTarget(centre+width, targetY);
                // Turn to heading
                turnCommand->setTarget(PI/2, true);
                Scheduler::master->interrupt(turnCommand);
            break;
            case 3:
                // Drive up
                setTarget(targetX, targetY+height);
                // Turn to heading
                turnCommand->setTarget(0, true);
                Scheduler::master->interrupt(turnCommand);
            break;
        }
        i ++;
    }
    return false;
}
