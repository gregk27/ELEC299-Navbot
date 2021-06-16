#include "./src/hardware/Drivetrain.h"
#include "./src/hardware/Sensors.h"
#include "./src/hardware/IMU.h"
#include "./src/utils/List.h"
#include "./Scheduler.h"

#include <PID_v2.h>

#include "./src/commands/DriveToPositionCommand.h"
#include "./src/commands/TurnToHeadingCommand.h"
#include "./src/commands/DrivePathCommand.h"
#include "./src/commands/AvoidanceCommand.h"
#include "./src/commands/SearchCommand.h"
#include "./src/commands/ComputePathCommand.h"

/**
 * List of positions build while travelling, used to generate return path
*/
List<IMU::Location> path = List<IMU::Location>(32);
/**
 * List of position generatred by ComputePathCommand 
*/
List<IMU::Location> *retPath;

/**
 * PID Controller shared by main navigation commands (saves alot of memory)
*/
PID_v2 pid1(0,0,0,PID::Direct);
/**
 * Secondary PID controller, used when primary already in use
*/
PID_v2 pid2(0,0,0,PID::Direct);

/**
 * Command used for avoidance routine
 * Instantiated once to reduce memory use
*/
AvoidanceCommand *avoidance = new AvoidanceCommand();

void setup() {
  Serial.begin(115200);
  Serial.println("Starting");

  //
  Drivetrain::init(4, 3, 3, 2);
  Drivetrain::setOutput(0,0);
  Sensors::init();

  // Create Scheduler
  Scheduler::master = new Scheduler(16);

  // Schedule main navigation commands
  Scheduler::master->addCommand(new DriveToPositionCommand(0, 150, 220, 20, &pid1, &path));
  Scheduler::master->addCommand(new DriveToPositionCommand(0, 200, 220, 5, &pid1, 0x0));
  // Scheduler::master->addCommand(new SearchCommand(150, 0, 30, 20, &pid1, &pid2));
  // Scheduler::master->addCommand(new DriveToPositionCommand(-50, 125, 200, 10, &pid1, &path));
  // Scheduler::master->addCommand(new DriveToPositionCommand(50, 175, 200, 10, &pid1, &path));
  Scheduler::master->addCommand(new ComputePathCommand(&path, &retPath));
  Scheduler::master->addCommand(new DrivePathCommand(&retPath, false, 200, 15, &pid1));
  // Scheduler::master->addCommand(new AvoidanceCommand());
 
 
  // Run selftest
  // while(selfTest()){};

  // Final setup
  delay(1000);
  Drivetrain::resetPosition();
  IMU::init();
  Scheduler::master->init();
}


void loop() {
  // -------
  //  Sense
  // -------
  Sensors::periodic();
  IMU::Position pos = IMU::getPosition();
  float usDist = Sensors::getUltrasonicDistance()->getSmoothed();
  // Serial.println("LOOP");

  // -------
  //  Think
  // -------
  // Serial.print(Sensors::getLeftIR()->getSmoothed());
  // Serial.print("\t");
  // Serial.print(Sensors::getRightIR()->getSmoothed());
  // Serial.print("\t");
  // Serial.println(usDist);
  // If there is an obstacle detected, interrupt scheduler with avoidance routine
  if(avoidance->isObstacle()){//Sensors::getLeftIR()->getLast() || Sensors::getRightIR()->getLast()){//} || (usDist > 0 && usDist < 20)){
    // Serial.println("AVOID");
    // Serial.print(Sensors::getLeftIR()->getSmoothed());
    // Serial.print("\t");
    // Serial.print(Sensors::getRightIR()->getSmoothed());
    // Serial.print("\t");
    // Serial.println(usDist);
    // Nothing will happen if there is already an interrupting command
    Scheduler::master->interrupt(avoidance);
  }

  // -----
  //  Act
  // -----
  // Run scheduler
  Scheduler::master->periodic();
  // IMU::toPlot();

  // Hang when done
  if(Scheduler::master->isFinished()){
    Serial.println("Done");
    while(1){};
  };
  delay(10);
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