#ifndef GYRO_H
#define GYRO_h
#include "../utils/SmoothedValue.h"

namespace Gyro {
  void init();
  void periodic();
  
  SmoothedValue<float> *getYaw();
}

#endif