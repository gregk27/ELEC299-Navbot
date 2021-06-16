#include "./ComputePathCommand.h"
#include "../utils/MemoryFree.h"

// Macros to facilitate using [] operator on pointers
#define PATHIN (*pathIn)
#define PATHOUT (**pathOut)

ComputePathCommand::ComputePathCommand(List<Odom::Location> *pathIn, List<Odom::Location> **pathOut){
    this->pathIn = pathIn;
    this->pathOut = pathOut;
}

void ComputePathCommand::init(){
    // Add current position at end of path
    pathIn->add(Odom::getPosition());

    int count = pathIn->size();

    // Print out data used for constructing path
    Serial.print(F("Building path from: "));
    Serial.print(count);
    Serial.println(F(" nodes"));
    for(int i=0; i<pathIn->size(); i++){
        Serial.print(-PATHIN[i].x);
        Serial.print(F(","));
        Serial.println(PATHIN[i].y);
    }

    // Either initialise or re-initialise pathOut at same size as pathIn
    if(!pathOut){
        pathOut = new List<Odom::Location> *;
    }
    if(*pathOut && (*pathOut)->getCapacity() < count) delete pathOut;
    *pathOut = new List<Odom::Location>(count);

    // Start working from current position
    Odom::Location current = PATHIN[count-1];
    int currIdx = count-1;

    // Loop until first node found
    while(currIdx != 0){
        float minAngle = 2*PI;
        int bestIdx = 0;

        // Check each node for best option
        for(int i=0; i<count; i++){
            // Dont comare against self
            if(i == currIdx) continue;
            Odom::Location tmp = PATHIN[i];
            // Calculate angle between points, take negative due to orientation
            float angle = -atan2(current.y-tmp.y, current.x-tmp.x);
            // Adjust coordinate space
            angle = angle >= PI/2 ? angle-PI/2 : angle+3*PI/2;

            // If it's a better point, then use it
            // The robot shouldn't backtrack due to how it navigates out
            if(angle < minAngle && angle > PI/2){
                minAngle = angle;
                bestIdx = i;
            }
        }
        // Save the best point and use it for next iteration
        Serial.print(F("Adding: "));
        Serial.println(bestIdx);
        (*pathOut)->add(PATHIN[bestIdx]);
        PATHOUT[(*pathOut)->size()-1].x += 15;
        current = PATHIN[bestIdx];
        currIdx = bestIdx;
    }

    // Print out resulting path
    Serial.println(F("\nComputed Path:"));
    Serial.println((*pathOut)->size());
    for(int i=0; i<(*pathOut)->size(); i++){
        Serial.print(-PATHOUT[i].x);
        Serial.print(F(","));
        Serial.println(PATHOUT[i].y);
    }

    Serial.println(F("Pathfinding complete"));
}

bool ComputePathCommand::isFinished(){
    // Since the command does everthing in init, it will be finshed instantly
    return true;
}