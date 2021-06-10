#include "./src/hardware/Drivetrain.h"
#include "./src/hardware/Sensors.h"
#include "./src/hardware/IMU.h"
#include "./src/utils/List.h"
#include "./src/utils/MemoryFree.h"
#include "./Scheduler.h"

#include "./src/commands/DriveToPositionCommand.h"
#include "./src/commands/TurnToHeadingCommand.h"
#include "./src/commands/DrivePathCommand.h"

/**
 * List of positions build while travelling, used to generate return path
*/
List<IMU::Position> path = List<IMU::Position>(32);

PID_v2 pid1(0,0,0,PID::Direct);

// DriveToPositionCommand driveOutCommand(0, 100, 200, 10, &pid1);
// DriveToPositionCommand driveSidewaysCommand(-50, 100, 200, 10, &pid1);
// DrivePathCommand driveBackCommand(&path, true, 200, 15, &pid1);
// DriveToPositionCommand driveToStopCommand(0, 0, 170, 5, &pid1);

void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  // Serial.println("Sizes");
  // Serial.print("Pointer: ");
  // Serial.println(sizeof(void*));
  // Serial.print("DriveToPositionCommand: ");
  // Serial.println(sizeof(DriveToPositionCommand));
  // Serial.print("TurnToHeadingCommand: ");
  // Serial.println(sizeof(TurnToHeadingCommand));
  // Serial.print("ComputePathCommand: ");
  // Serial.println(sizeof(ComputePathCommand));
  // Serial.print("Scheduler: ");
  // Serial.println(sizeof(Scheduler));

  Drivetrain::init(4, 3, 3, 2);
  Drivetrain::setOutput(0,0);
  Sensors::init();

  Scheduler::master = new Scheduler(32);
  // Scheduler::master->addCommand(&driveOutCommand);
  // Scheduler::master->addDelay(1000);
  // Scheduler::master->addCommand(&driveSidewaysCommand);
  // Scheduler::master->addDelay(1000);
  // Scheduler::master->addCommand(&driveBackCommand);
  // Scheduler::master->addDelay(1000);
  // Scheduler::master->addCommand(&driveToStopCommand);

  Scheduler::master->addCommand(new DriveToPositionCommand(0, 100, 220, 20, &pid1, &path));
  Scheduler::master->addCommand(new DriveToPositionCommand(-50, 125, 200, 10, &pid1, &path));
  Scheduler::master->addCommand(new DriveToPositionCommand(50, 175, 200, 10, &pid1, &path));
  // Scheduler::master->addCommand(new DriveToPositionCommand(50, 100, 200, 10, &pid1));
  // Scheduler::master->addCommand(new DriveToPositionCommand(0, 0, 200, 10, &pid1));
  Scheduler::master->addCommand(new DrivePathCommand(&path, true, 200, 15, &pid1));

  // Scheduler::master->addCommand(new TurnToHeadingCommand(PI/2, false, 220, 0.01, 3000));
  // Scheduler::master->addDelay(1000);
  // Scheduler::master->addCommand(new TurnToHeadingCommand(-PI, false, 220, 0.01, 3000));
  // Scheduler::master->addDelay(1000);
  // Scheduler::master->addCommand(new TurnToHeadingCommand(PI, true, 220, 0.01, 3000));

  // Run selftest
  Drivetrain::resetPosition();

  IMU::init();


  delay(1000);
  
  Scheduler::master->init();
  Serial.println(freeMemory());
}

float holdHeading;

void loop() {
  // -------
  //  Sense
  // -------
  Sensors::periodic();
  IMU::Position pos = IMU::getPosition();
  float usDist = Sensors::getUltrasonicDistance()->getSmoothed();

  // Serial.print(Sensors::getLeftIR()->getLast());
  // Serial.print("\t");
  // Serial.print(Sensors::getRightIR()->getLast());
  // Serial.print("\t");
  // Serial.println(Gyro::getYaw()->getSmoothed());


  // -----
  //  Act
  // -----
  // Drivetrain::setOutput(220,220);
  Scheduler::master->periodic();
  // IMU::toPlot();
  // Hang when done
  if(Scheduler::master->isFinished()){
    Serial.println("Done");
    while(1){};
  };
  delay(20);
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