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
  Serial.print(pos.heading*180/PI);
  Serial.print("\t");
  Serial.println(atan2(0-pos.y, 300-pos.x)*180/PI);
  driveHeading(200, atan2((0-pos.x)*2, 200-pos.y));
  delay(50);
  while(pos.y > 200){
    Drivetrain::setOutput(0, 0);

  };
}

void driveHeading(int speed, float hdg){
  IMU::Position pos = IMU::getPosition();
  float err = hdg-pos.heading;
  // Serial.println(err);
  Drivetrain::setOutput(speed - err*200, speed + err*200);
}