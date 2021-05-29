#ifndef SENSORS_H
#define SENSORS_H

namespace Sensors {

  extern int USLastCall;
  extern int DSThreshold;

  /**
   * Initialise sensors
  */
  void init();

  bool getLeftIR();
  bool getRightIR();

  /**
   * Get the distance returned from the ultrasonic sensor
   * @return Distance read, in cm. Returns -1 if out of range (>50cm), -2 if sensor not ready
  */
  float getUltrasonicDistance();

  int getDownwardSensor();
  bool isOnMarker();
}

#endif