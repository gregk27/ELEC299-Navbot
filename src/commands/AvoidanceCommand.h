#ifndef AVOIDANCECOMMAND_H
#define AVOIDANCECOMMAND_H

#include "./Command.h"

class AvoidanceCommand: public Command {
  private:
    float holdHeading;
    void driveHeading(int speed, float heading);
    void pivotHeading(int heading);

  public:
    AvoidanceCommand();

    void init() override;
    void periodic() override;
    void end() override;
    bool isFinished() override;

};

#endif