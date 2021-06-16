#ifndef DRIVEPATHCOMMAND_H
#define DRIVEPATHCOMMAND_H

#include <PID_v2.h>
#include "./DriveToPositionCommand.h"
#include "../hardware/Odom.h"
#include "../utils/List.h"

/**
 * Command to drive along a path
*/
class DrivePathCommand: public DriveToPositionCommand {
  private:
    /** Index of current node */
    int idx;
    /** Flag indicating if path should be navigated in reverse */
    bool reverse;
    /** Path to be navigated */
    List<Odom::Location> **path;

    /**
     * Set the target position to that of a node
     * @param idx Index of the node
     * @return False if path is complete
    */
    bool setTargetNode(int idx);

  public:
    /**
     * Create a new Drive Path Command
     * @param path Double-pointer to the path to follow
     * @param reverse Flag to indicate if path should be navigated in reverse
     * @param speed Baseline speed the robot should travel
     * @param tol Tolerance for reaching each node
     * @param controller PID controller to use
    */
    DrivePathCommand(List<Odom::Location> **path, bool reverse, byte speed, float tol, PID_v2 *controller);

    void init() override;
    bool isFinished() override;

};

#endif