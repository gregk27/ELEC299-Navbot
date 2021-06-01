#ifndef SENSORS_H
#define SENSORS_H
#include "../utils/SmoothedValue.h"

namespace Sensors {

  extern int USLastCall;
  extern int DSThreshold;

  /**
   * Initialise sensors
  */
  void init();
  /**
   * Call from loop to update
  */
  void periodic();

  SmoothedValue<bool>* getLeftIR();
  SmoothedValue<bool>* getRightIR();

  /**
   * Get the distance returned from the ultrasonic sensor
   * @return Distance read, in cm. Returns -1 if out of range (>50cm), -2 if sensor not ready
  */
  SmoothedValue<float>* getUltrasonicDistance();

  SmoothedValue<int>* getDownwardSensor();
  bool isOnMarker();
}

#endif