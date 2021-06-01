#include "./SmoothedValue.h"

int SmoothFunctions::smoothInt(int *samples, int count){
  int sum = 0;
  for(int i=0; i<count; i++){
    sum += samples[i];
  }
  return sum/count;
}  

float SmoothFunctions::smoothFloat(float *samples, int count){
  float sum = 0.0f;
  for(int i=0; i<count; i++){
    sum += samples[i];
  }
  return sum/count;
}

bool SmoothFunctions::smoothBool(bool *samples, int count){
  float sum = 0.0f;
  for(int i=0; i<count; i++){
    sum += samples[i];
  }
  return sum/count > 0.5;
}