#ifndef SENSORS_H
#define SENSORS_H
#include "../utils/SmoothedValue.h"
#include "./Gyro.h"

namespace Sensors {

  /** Timestamp of last ultrasonic sensor update, used to enforce 60ms wait between updates*/
  extern int USLastCall;
  /** Threshold used by downard sensor to detect marker */
  extern int DSThreshold;

  extern bool ignoreSensors;

  /**
   * Initialise sensors
  */
  void init();
  /**
   * Call from loop to update
  */
  void periodic();

  /**
   * Get left IR sensor detection
   * @return True if sensor detects an obstacle
  */
  SmoothedValue<bool>* getLeftIR();
  
  /**
   * Get right IR sensor detection
   * @return True if sensor detects an obstacle
  */
  SmoothedValue<bool>* getRightIR();

  /**
   * Get the distance returned from the ultrasonic sensor
   * @return Distance read, in cm. Returns -1 if out of range (>50cm), -2 if sensor not ready
  */
  SmoothedValue<float>* getUltrasonicDistance();

  /**
   * Get downward sensor reading
   * @return The downward sensor's raw reading
  */
  SmoothedValue<int>* getDownwardSensor();
  
  /**
   * Check if the robot is on the marker using the downward sensor
   * @return True if smoothed sensor reading is above DSThreshold
  */
  bool isOnMarker();
}

#endif