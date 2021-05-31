#ifndef IMU_H
#define IMU_H

namespace IMU {
  /**
   * Structure representing the robot's position, relative to start
  */
  struct Position {
    float x;
    float y;
    float heading;
  };

  /**
    * Initialise the IMU
    * This should be done after intialising the drivetrain and sensors
  */
  void init();

  /**
   * Get the robot's position based on sensor readings
  */
  Position getPosition();

}

#endif