#include "./Sensors.h"
#include <Arduino.h>
#include "../utils/SmoothedValue.h"

// TODO: Confirm pin order
#define LEFT_IR_PIN 9
#define RIGHT_IR_PIN 10

#define US_ECHO_PIN A3
#define US_TRIG_PIN A2
#define US_MIN_INTERVAL 60
#define US_DURATION_TO_DISTANCE 0.01724137931 // 1/58, from docs

#define DS_PIN A1

using namespace Sensors;


template <>
int SmoothedValue<int>::getSmoothed(){
  int sum = 0;
  for(int i=0; i<numSamples; i++){
    sum += samples[i];
  }
  return sum/numSamples;
}

template <>
float SmoothedValue<float>::getSmoothed(){
  float sum = 0;
  for(int i=0; i<numSamples; i++){
    sum += samples[i];
  }
  return sum/numSamples;
}

template <>
bool SmoothedValue<bool>::getSmoothed(){
  float sum = 0.0f;
  for(int i=0; i<numSamples; i++){
    sum += samples[i];
  }
  return sum/numSamples > 0.5;
}

int Sensors::USLastCall = 0;
int Sensors::DSThreshold = 80;

SmoothedValue<bool> leftIR(5, SmoothFunctions::smoothBool);
SmoothedValue<bool> rightIR(5, SmoothFunctions::smoothBool);
SmoothedValue<float> ultrasonic(5, [](float* samples, int count)->float {
  float sum = 0.0f;
  int badSamples = 0;
  for(int i=0; i<count; i++){
    if(samples[i] >= 0) sum += samples[i];
    else badSamples ++;
  }
  Serial.println(badSamples);
  // Need 3 good samples to get value
  if(badSamples >= 2){
    return -2;
  } else {
    return sum/(count-badSamples);
  }
});
SmoothedValue<int> downward(3, SmoothFunctions::smoothInt);

void Sensors::init(){
  pinMode(LEFT_IR_PIN, INPUT);
  pinMode(RIGHT_IR_PIN, INPUT);

  pinMode(US_TRIG_PIN, OUTPUT);
  pinMode(US_ECHO_PIN, INPUT);
  digitalWrite(US_TRIG_PIN, LOW);

  pinMode(DS_PIN, INPUT);

  Gyro::init();
}

void Sensors::periodic(){
  // Sample obstacle sensors
  leftIR.addSample(!digitalRead(LEFT_IR_PIN));
  rightIR.addSample(!digitalRead(RIGHT_IR_PIN));
  
  // Sample ultrasonic sensor
  if(millis() - USLastCall > US_MIN_INTERVAL){
    // Send trigger pulse
    digitalWrite(US_TRIG_PIN, LOW);
    delayMicroseconds(50);
    digitalWrite(US_TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(US_TRIG_PIN, LOW);

    int duration = pulseIn(US_ECHO_PIN, HIGH, 8000);
    USLastCall = millis();
    // Return distance, capped at 50cm
    ultrasonic.addSample(duration * US_DURATION_TO_DISTANCE > 50 ? -2 : duration * US_DURATION_TO_DISTANCE);
  }

  downward.addSample(analogRead(DS_PIN));
  Gyro::periodic();
}

SmoothedValue<bool>* Sensors::getLeftIR(){
  return &leftIR;
}
SmoothedValue<bool>* Sensors::getRightIR(){
  return &rightIR;
}

SmoothedValue<float>* Sensors::getUltrasonicDistance(){
  return &ultrasonic;
}

SmoothedValue<int>* Sensors::getDownwardSensor(){
  return &downward;
}
bool Sensors::isOnMarker(){
  return downward.getSmoothed() > DSThreshold;
}