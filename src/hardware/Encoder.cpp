#include "./Encoder.h"
#include <Arduino.h>
#include "../utils/List.h"

/** Array of encoders to have periodic functions called */
List<Encoder *> encoders = List<Encoder *>(2);

// Set speed interval to 250 ms
int Encoder::speedInterval = 250;

Encoder::Encoder(){
  // Initialise interrupts
  if(encoders.size() == 0){
    // Prepare speed timer interrupt, see https://busylog.net/FILES2DW/doc8161.pdf page 134
    noInterrupts();
    TCCR1A = 4; // Set clock to mode 4 (Clear timer on compare match)
    TCCR1B = 8 | 5; // Set bit 3 for CTC mode, lowest 3 bits as 101 to use 1024 prescaling
                // 16,000,000/1024 = 15,625 Hz, tick once per 0.064ms
    OCR1A= (int) (speedInterval/0.064); // Get count required for 500ms delay
    TIMSK1 |= 2; 
    interrupts();
  }
  // Save encoder to list
  encoders.add(this);
}

void Encoder::onRisingEdge(){
  if(millis()-lastTick > interval){
    // Add direction (+/- 1) to count
    count += direction;
    lastTick = millis();
  }
}

void Encoder::onSpeedInterval(){
  speed = (count - lastCount)/(speedInterval*0.001);
  lastCount = count;
}

void Encoder::setDirection(int direction){
  this->direction = direction < 0 ? -1 : 1;
}

void Encoder::reset(){
  this->count = 0;
}

int Encoder::getPosition(){
  return count;
}

float Encoder::getPositionRev(){
  return count / (ticksPerRev*1.0);
}

float Encoder::getPositionCm(){
  return count / (ticksPerRev/distPerRev);
}

float Encoder::getSpeed(){
  return speed;
}

float Encoder::getSpeedRev(){
  return speed / (ticksPerRev*1.0);
}

float Encoder::getSpeedRPM(){
  return (speed / (ticksPerRev*1.0)) * 60;
}

float Encoder::getSpeedM(){
  return (speed / (ticksPerRev/distPerRev)) / 100;
}

ISR(TIMER1_COMPA_vect) {
  for(int i=0; i<encoders.size(); i++){
    encoders[i]->onSpeedInterval();
  }
}