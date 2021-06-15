#include "./DriveToPositionCommand.h"
#include "./TurnToHeadingCommand.h"

class SearchCommand: public DriveToPositionCommand {
    private:
        float centre;
        byte width;
        byte height;
        float initialY;
        int i;

        TurnToHeadingCommand *turnCommand;

    public:
        SearchCommand(byte speed, float centre, byte width, byte height, PID_v2 *controller, PID_v2 *turnController);
        
        void setTarget(float x, float y);
        
        void init() override;
        bool isFinished() override;
    
};