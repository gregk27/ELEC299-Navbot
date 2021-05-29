#include "./Sensors.h"
#include <Arduino.h>

#define LEFT_IR_PIN A3
#define RIGHT_IR_PIN A2

#define US_ECHO_PIN A4
#define US_TRIG_PIN A5
#define US_MIN_INTERVAL 60
#define US_DURATION_TO_DISTANCE 0.01724137931 // 1/58, from docs

#define DS_PIN A1

using namespace Sensors;

int Sensors::USLastCall = 0;
int Sensors::DSThreshold = 80;

void Sensors::init(){
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);

  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  digitalWrite(US_TRIG_PIN, LOW);

  pinMode(DS_PIN, INPUT);
}

bool Sensors::getLeftIR(){
  return !digitalRead(LEFT_IR_PIN);
}
bool Sensors::getRightIR(){
  return !digitalRead(RIGHT_IR_PIN);

}

float Sensors::getUltrasonicDistance(){
  if(millis() - USLastCall > US_MIN_INTERVAL){
    // Send trigger pulse
    digitalWrite(US_TRIG_PIN, LOW);
    delayMicroseconds(50);
    digitalWrite(US_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG_PIN, LOW);

    int duration = pulseIn(US_ECHO_PIN, HIGH);
    USLastCall = millis();
    // Return distance, capped at 50cm
    return duration * US_DURATION_TO_DISTANCE > 50 ? -1 : duration * US_DURATION_TO_DISTANCE;
  }
  return -2;
}

int Sensors::getDownwardSensor(){
  return analogRead(DS_PIN);
}
bool Sensors::isOnMarker(){
  return analogRead(DS_PIN) > DSThreshold;
}