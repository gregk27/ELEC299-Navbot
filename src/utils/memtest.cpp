#include "memtest.h"
#include <Arduino.h>

unsigned int memtest(unsigned short int step=1) {
  byte *buffer;
  
  // 2k max
  for (unsigned int i = 2048; i; i -= step) {
    buffer = (byte *)malloc(i);
    
    if (buffer) {
      free(buffer);
      return i;
    }
  }

  return -1;
}