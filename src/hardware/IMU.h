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

  /**
   * Print data in format accepted by ArduPlot
  */
  void toPlot();
  
  /**
   * Get the heading in radians to a target point
   * @param x x position of the point
   * @param y y position of the point
   * @return The heading in radians facing the point
  */
  float headingTo(float x, float y);
}

#endif