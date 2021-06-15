#ifndef SCHEDULER_H
#define SCHEDULER_H

#include "./src/utils/List.h"
#include "./src/commands/Command.h"

class Scheduler: public Command{
  private:

    List<Command *> schedule;
    int currentCommand=0;
    Command *interruptCommand = 0x0;
    unsigned int iteration;

  public:
    static Scheduler *master;

    Scheduler(int maxCommands);

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
     * Interrupt the currently running command with another
     * Current command will resume upon completion
     * An interrupt comnmand cannot be interrupted
     * @param command The new command to run
     * @return true if interrupt started, otherwise false
    */
    bool interrupt(Command *command);
    /**
     * Add a delay to the schedule
     * @param duration Duration of the delay, in ms
    */
    void addDelay(unsigned int duration);

    /**
     * Get the number of iterations since starting
    */
    unsigned int getIteration();
};

#endif