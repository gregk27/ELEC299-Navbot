#include "./src/hardware/Drivetrain.h"
#include "./src/hardware/Sensors.h"
#include "./src/hardware/IMU.h"
#include "./Scheduler.h"

#include "./src/commands/DriveToPositionCommand.h"
#include "./src/commands/TurnToHeadingCommand.h"


void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  Drivetrain::init(1, 3, 3, 2);
  Sensors::init();
  
  Scheduler::master = new Scheduler();
  // Scheduler::master->addCommand(new DriveToPositionCommand(0, 200, 200, 10));
  // Scheduler::master->addDelay(1000);
  // Scheduler::master->addCommand(new DriveToPositionCommand(50, 100, 200, 15));
  // Scheduler::master->addDelay(1000);
  // Scheduler::master->addCommand(new DriveToPositionCommand(0, 0, 200, 5));

  Scheduler::master->addCommand(new TurnToHeadingCommand(PI/2, false, 220, 0.01, 3000));
  Scheduler::master->addDelay(1000);
  Scheduler::master->addCommand(new TurnToHeadingCommand(-PI, false, 220, 0.01, 3000));
  Scheduler::master->addDelay(1000);
  Scheduler::master->addCommand(new TurnToHeadingCommand(PI, true, 220, 0.01, 3000));

  // Run selftest
  Drivetrain::resetPosition();

  IMU::init();


  Scheduler::master->init();
  delay(1000);
}

float holdHeading;

void loop() {
  // -------
  //  Sense
  // -------
  Sensors::periodic();
  IMU::Position pos = IMU::getPosition();
  float usDist = Sensors::getUltrasonicDistance()->getSmoothed();

  // -----
  //  Act
  // -----
  Scheduler::master->periodic();
  // IMU::toPlot();
  // Hang when done
  while(Scheduler::master->isFinished());
}

void driveHeading(int speed, float hdg){
  IMU::Position pos = IMU::getPosition();
  float err = hdg-pos.heading;
  // Serial.println(err);
  Drivetrain::setOutput(speed - err*500, speed + err*500);
}

void pivotHeading(int heading){
  IMU::Position pos = IMU::getPosition();
  float err = heading-pos.heading;
  Drivetrain::setOutput(err*250, -err*250);
}

int testState = 0;

/**
 * Run a self-test
 * @return false when complete
*/
bool selfTest(){
  int usd = Sensors::getUltrasonicDistance()->getSmoothed();
  if(
    testState == 0 && usd == -1 ||
    testState == 1 && usd < 10  && usd > 0 ||
    testState == 2 && !Sensors::getLeftIR()->getSmoothed()  || 
    testState == 3 && Sensors::getLeftIR()->getSmoothed()  || 
    testState == 4 && !Sensors::getRightIR()->getSmoothed()  || 
    testState == 5 && Sensors::getRightIR()->getSmoothed()  || 
    testState == 6 && Sensors::getDownwardSensor()->getSmoothed() < 10 ||
    testState == 7 && Sensors::getDownwardSensor()->getSmoothed() > 10 ||
    testState == 8 && Drivetrain::leftEncoder->getPosition() > 20 ||
    testState == 9 && Drivetrain::rightEncoder->getPosition() > 20
  ) {
    Serial.print("Completed test: ");
    Serial.println(testState);

    
    testState ++;

    
    if(testState == 8){
      Drivetrain::resetPosition();
    } else if(testState == 9){
      Drivetrain::resetPosition();
    } else if (testState == 10){
      Serial.println("All tests complete");
    }
  }
  return testState < 10;
}