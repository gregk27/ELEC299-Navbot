#ifndef Command_H
#define  Command_H
#include <Arduino.h>

/**
 * Base class represeting all commands
*/
class Command {
  public:
    /**
     * Function called to initialise the command
    */
    virtual void init(){}
    /**
     * Function called periodically
    */
    virtual void periodic(){}
    /**
     * Function called when command ends
    */
    virtual void end(){}
    /**
     * Function called to determine if command is finished
     * @return True when command is finished
    */
    virtual bool isFinished(){return false;}
};

#endif