#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "./src/utils/List.h"
#include "./src/commands/Command.h"

class Scheduler: public Command{
  private:

    List<Command *> schedule;
    int currentCommand=0;

  public:
    static Scheduler *master;

    Scheduler();

    void init() override;
    void periodic() override;
    void end() override;
    bool isFinished() override;

    /**
     * Add a new command to the schedule
     * @param command Pointer to the command to be added
    */
    void addCommand(Command *command);
    /**
     * Add a delay to the schedule
     * @param duration Duration of the delay, in ms
    */
    void addDelay(unsigned int duration);
};

#endif