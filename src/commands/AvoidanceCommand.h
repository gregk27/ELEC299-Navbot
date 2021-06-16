#ifndef AVOIDANCECOMMAND_H
#define AVOIDANCECOMMAND_H

#include "./Command.h"
#include "../hardware/Odom.h"
#include "../utils/List.h"

/**
 * Command to execute obstacle avoidance
 * For rough outline of operation, see nav.md
*/
class AvoidanceCommand: public Command {
  private:
    /** Timeout used to pause change of operation case */
    unsigned long timeout;
    /** Timeout to prevent ending of command, used to increase clearance */
    unsigned long endTimeout;
    /** Path that position will be saved to */
    List<Odom::Location> *path;

    /**
     * Save the robot's position to the path
     * This will also add some offset to increase clearance on return
    */
    void savePosition();

  public:
    /**
     * Create a new AvoidanceCommand
     * @param path Path to save position to
    */
    AvoidanceCommand(List<Odom::Location> *path);

    void init() override;
    void periodic() override;
    void end() override;
    bool isFinished() override;

    /**
     * Function used to detect obstacle based on following:
     *  - Either IR sensor detects someting
     *  - Ultrasonic sensors detects something within 0-20 cm
     * @returns True if obstacle is detected
    */
    static bool isObstacle();

};

#endif