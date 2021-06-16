#ifndef GYRO_H
#define GYRO_h
#include "../utils/SmoothedValue.h"

namespace Gyro {
  /**
   * Initialise the gyroscope
  */
  void init();
  /**
   * Call periodically to update the gyro yaw
  */
  void periodic();

  /**
   * Get the Gyro's yaw (heading)
  */
  SmoothedValue<float> *getYaw();
}

#endif