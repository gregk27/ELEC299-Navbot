#ifndef COMPUTEPATHCOMMAND_H
#define COMPUTEPATHCOMMAND_H

#include "./Command.h"
#include "../utils/List.h"
#include "../hardware/Odom.h"

/**
 * Command to compute an improved path using modified Jarvis March algorithm
*/
class ComputePathCommand: public Command {
  private:
    /** Pointer to the input path */
    List<Odom::Location> *pathIn;
    /** Double-pointer to output path */
    List<Odom::Location> **pathOut;

  public:
    /**
     * Create a new Compute Path Command
     * @param pathIn Pointer to path used as input data
     * @param pathOut Pointer to path used for output, will be initialised by command
    */
    ComputePathCommand(List<Odom::Location> *pathIn, List<Odom::Location> **pathOut);

    void init() override;
    bool isFinished() override;

};

#endif