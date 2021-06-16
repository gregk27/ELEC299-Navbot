#ifndef DRIVETOPOINTCOMMAND_H
#define DRIVETOPOINTCOMMAND_H

#include <PID_v2.h>
#include "./Command.h"
#include "../hardware/Odom.h"
#include "../utils/List.h"

/**
 * Command to drive to a specific position
*/
class DriveToPositionCommand: public Command {
  protected:
    /** Target x position */
    float targetX;
    /** Target y position */
    float targetY;

    /** PID controller to use (passed in to save memory)*/
    PID_v2 *controller;
  
  private:
    /** Tolerance for stop location */
    float tol;
    /** Baseline speed to travel at */
    byte speed;

    /** Path to add position to */
    List<Odom::Location> *path;

  public:
    /**
     * Create a new Drive To Position Command
     * @param x The x position to drive to
     * @param y The y position to drive to
     * @param speed The baseline speed to travel at
     * @param tol The tolerance for stopping location
     * @param controller The PID controller to use
    */
    DriveToPositionCommand(float x, float y, byte speed, float tol, PID_v2 *controller, List<Odom::Location> *path);

    // Some functions are virtual to allow for overriding
    
    virtual void init() override;
    void periodic() override;
    void end() override;
    virtual bool isFinished() override;

};

#endif