#include "./src/hardware/Drivetrain.h"
#include "./src/hardware/Sensors.h"
#include "./src/hardware/IMU.h"


void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  Drivetrain::init(1, 3, 3, 2);
  Sensors::init();
  IMU::init();

  delay(1000);
}

int state = 0;

void loop() {
  IMU::Position pos = IMU::getPosition();
  IMU::toPlot();
  if(state == 0){
    int sc = Drivetrain::getStraightCorrection();
    Drivetrain::setOutput(220+sc, 200-sc);
    if(millis() > 2000){
      state ++;
    }
  }
  if(state == 1){
    int lErr = -15-Drivetrain::leftEncoder->getPosition();
    int rErr = 15-Drivetrain::rightEncoder->getPosition();
    Drivetrain::setOutput(lErr*25, rErr*25);
    if(abs(-15-Drivetrain::leftEncoder->getPosition()) <= 5 && 
          abs(15-Drivetrain::rightEncoder->getPosition()) <= 5) {
      state ++;
    }
  } else if (state == 2){
    // int sc = Drivetrain::getStraightCorrection();
    Drivetrain::setOutput(200, 200);
    if(millis() > 3000){
      state ++;
    }
  } else if(state == 3){
    driveHeading(225, IMU::headingTo(0, 200));
  }
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