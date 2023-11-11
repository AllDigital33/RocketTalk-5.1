#include "Arduino.h"
#include "AccelReading.h"

AccelReading::AccelReading()
{
}

void AccelReading::init(int16_t _x, int16_t _y, int16_t _z, uint8_t scalingFactor)
{
  x = _x * scalingFactor;
  y = _y * scalingFactor;
  z = _z * scalingFactor;
}

uint32_t AccelReading::accelSize()
{
  return sqrt(sq(x) + sq(y) + sq(z));
}

void AccelReading::printDebug()
{
  Serial.print(x);
  Serial.print("\t");
  Serial.print(y);
  Serial.print("\t");
  Serial.print(z);
  Serial.print("\t");
  Serial.println(accelSize());
}

