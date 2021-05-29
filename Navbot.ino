#include "./src/hardware/Drivetrain.h"
#include "./src/hardware/Sensors.h"
#include "./src/hardware/IMU.h"


void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  Drivetrain::init(1, 3, 3, 2);
  Sensors::init();
  IMU::init();
}

void loop() {
  IMU::Position pos = IMU::getPosition();
  Serial.print(pos.x);
  Serial.print("\t");
  Serial.print(pos.y);
  Serial.print("\t");
  Serial.println(pos.heading);
}