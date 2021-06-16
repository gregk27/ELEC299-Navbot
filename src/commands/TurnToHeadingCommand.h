#ifndef TURNTOHEADINGCOMMAND_H
#define TURNTOHEADINGCOMMAND_H

#include <PID_v2.h>
#include "./Command.h"
#include "../hardware/Odom.h"

/**
 * Command to pivot to a specified heading (relative or absolute)
*/
class TurnToHeadingCommand: public Command {
  private:
    /** Target heading */
    float target;
    /** Flag indicating if target is absolute */
    bool absolute;
    /** Tolerance for turn */
    float tol;
    /** Maximum turning speed */
    int speed;
    
    /** PID controller */
    PID_v2 *controller;

    /** Timeout to prevent getting stuck */
    unsigned long timeout;

  public:
    /**
     * Create a new Turn to Heading command
     * @param target Target heading (radians)
     * @param absolute If true, turns to absolute heading. If false turns amount relative to position
     * @param speed Maximum wheel speed in turn
     * @param tol Tolerance for turn
     * @param timeout Maximum command duration (ms)
     * @param controller PID controller to use
    */
    TurnToHeadingCommand(float target, bool absolute, int speed, float tol, unsigned long timeout, PID_v2 *controller);
    
    /**
     * Set the target heading, can be used to change after creation
     * @param target Target heading (radians)
     * @param absolute If true, turns to absolute heading. If false turns amount relative to position
    */
    void setTarget(float target, bool absolute);

    void init() override;
    void periodic() override;
    void end() override;
    bool isFinished() override;


};

#endif