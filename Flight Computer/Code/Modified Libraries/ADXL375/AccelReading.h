#ifndef AccelReading_h
#define AccelReading_h

#include "Arduino.h"

class AccelReading
{
  public:
    AccelReading();
    void init(int16_t x, int16_t y, int16_t z, uint8_t scalingFactor = 1);
    uint32_t accelSize();
    void printDebug();
    uint32_t x;
    uint32_t y;
    uint32_t z;
};

#endif
