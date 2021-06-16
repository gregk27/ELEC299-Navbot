#ifndef SEARCHCOMMAND_H
#define SEARCHCOMMAND_H

#include "./DriveToPositionCommand.h"
#include "./TurnToHeadingCommand.h"

/**
 * Command to execute a zig-zag search pattern
 * Pattern format is:
 *     <---> width
 *┖────────┒ ˄ height
 *    ┎────┚ ˅
 *    | <- centre 
*/
class SearchCommand: public DriveToPositionCommand {
    private:
        /** X position at centre of pattern */
        float centre;
        /** Width of pattern */
        byte width;
        /** Height between pattern steps */
        byte height;
        /** Counter to track current state in pattern */
        int i;

        /**
         * Command used for turns, singal instance used to reduce memory use
        */
        TurnToHeadingCommand *turnCommand;

        /**
         * Set the target position
         * @param x X target
         * @param y Y target
        */
        void setTarget(float x, float y);
        
    public:
        /**
         * Create a new Search Command
         * @param speed Baseline driving sped
         * @param centre Centre of pattern
         * @param width Width of pattern half
         * @param height Height of pattern step
         * @param controller PID controller to be used for driving
         * @param turnController Secondary PID controller to be used for turning
        */
        SearchCommand(byte speed, float centre, byte width, byte height, PID_v2 *controller, PID_v2 *turnController);
        
        void init() override;
        bool isFinished() override;

};

#endif