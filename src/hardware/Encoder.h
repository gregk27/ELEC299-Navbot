#ifndef ENCODER_H
#define ENCODER_H
#include <math.h>

/**
 * Class for provided encoder
 * Must only be instantiated using new to prevent errors with speed calculation
*/
class Encoder {
  private:
    /** Tick count */
    volatile int count = 0;
    /** Current average speed, ticks/second*/
    volatile float speed = 0;
    /** Time of last tick event */
    volatile unsigned long lastTick = 0;
    /** Count at last time interval */
    volatile int lastCount = 0;

    /** Current travel direction, positive forward negative reverse*/
    char direction = 1;

  public:
    
    /** Minimum time interval between ticks */
    int interval = 2;
    /** Number of encoder ticks in a revolution */
    int ticksPerRev = 40;
    /** Distance travelled (m) per revolution */
    double distPerRev = M_PI * 0.068;
    /** 
     * Interval used for speed calculation, in ms
     * Can only be set before instantiating any encoders
    */
    static int speedInterval;

    /** Constructor to inisialise speed timer for encoder*/
    Encoder();
    /** Function to be called on rising edge */
    void onRisingEdge();
    /** Function that will be automatically called at frequency determined by speedInterval*/
    void onSpeedInterval();

    /**
     * Set the encoder's direction for counting
     * @param direction If positive, will count forward, nevagive will count reverse
    */
    void setDirection(int direction);
    /**
     * Reset encoder to 0
    */
    void reset();

    /**
     * Get position in ticks
    */
    int getPosition();
    /**
     * Get position in revolutions, uses ticksPerRev
    */
    float getPositionRev();
    /**
     * Get position in cm, uses ticksPerRev and distPerRev
    */
    float getPositionCm();
    /**
     * Get speed in ticks/second
    */
    float getSpeed();
    /**
     * Get speed in revolutions/second, uses ticksPerRev
    */
    float getSpeedRev();
    /**
     * Get speed in revolutions/minute, uses ticksPerRev
    */
    float getSpeedRPM();
    /**
     * Get speed in m/second, uses ticksPerRev and distPerRev
    */
    float getSpeedM();
};

#endif