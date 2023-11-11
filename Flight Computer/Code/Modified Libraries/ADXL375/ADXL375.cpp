#include "Arduino.h"
#include "ADXL375.h"
#include "SPI.h"

ADXL375::ADXL375()
{
}

void ADXL375::init()
{
  SPI.begin();
  SPI.setDataMode(SPI_MODE3); 
  
  pinMode(PIN_SPI_SS, OUTPUT);
  digitalWrite(PIN_SPI_SS, HIGH);

  // set the data sampling rate to 3200Hz
  setDataRate(0b00001111);
}

void ADXL375::startMeasuring()
{
  writeRegister(ADXL375_REG_POWER_CTL, 0x08);
}

void ADXL375::setDataRate(uint8_t rate)
{
  writeRegister(ADXL375_REG_BW_RATE, rate);
}

AccelReading ADXL375::getXYZ()
{
  uint8_t data[6];
  _multiReadRegister(ADXL375_REG_DATAX0, data, 6);

  AccelReading xyz;
  xyz.init(
    data[0] | data[1]<<8,
    data[2] | data[3]<<8,
    data[4] | data[5]<<8,
    ADXL375_XYZ_READ_SCALE_FACTOR
  );
  
  return xyz;
}

uint8_t ADXL375::readRegister(uint8_t regAddress)
{
  uint8_t data[1];
  _multiReadRegister(regAddress, data, 1);
  return data[0];
}

// input is in g
void ADXL375::setShockThreshold(uint8_t shockThreshold)
{
  uint8_t scaledValue = shockThreshold * 1000 / ADXL375_THRESH_SHOCK_SCALE;
  writeRegister(ADXL375_REG_THRESH_SHOCK, scaledValue);
}

// returns the number of items in the FIFO buffer
uint8_t ADXL375::getFIFOBufferSize()
{
  return readRegister(ADXL375_REG_FIFO_STATUS) & 0b00111111;
}

void ADXL375::startShockDetection()
{
  setShockAxes(true,true,true);

  // shock duration to test for 625 μs/LSB
  writeRegister(ADXL375_REG_DUR, 2);

  // reset then enable FIFO trigger mode
  setFIFOMode(ADXL375_FIFO_MODE_BYPASS);
  setFIFOMode(ADXL375_FIFO_MODE_TRIGGER, ADXL375_TRIGGER_INT1_PIN, 16);

  // set the interrupt bit for shock detection
  uint8_t intEnable = readRegister(ADXL375_REG_INT_ENABLE);
  writeRegister(ADXL375_REG_INT_ENABLE, intEnable | 0b01000000);

  startMeasuring();
}

void ADXL375::setShockAxes(bool x, bool y, bool z)
{
  uint8_t shockAxesData = 0b00000000;
  if(x) shockAxesData |= 0b100;
  if(y) shockAxesData |= 0b010;
  if(z) shockAxesData |= 0b001;

  // the axes we want to detect shocks on
  writeRegister(ADXL375_REG_SHOCK_AXES, shockAxesData);
}

void ADXL375::setFIFOMode(uint8_t mode, uint8_t pin, uint8_t samples)
{
  writeRegister(ADXL375_REG_FIFO_CTL, mode<<6 | pin<<5 | samples);
}

void ADXL375::_multiReadRegister(uint8_t regAddress, uint8_t values[], uint8_t numberOfBytes)
{
  // Since we're performing a read operation, the most significant bit of the register address should be set.
  regAddress |= 0x80;

  // set the multi-read byte if required
  if (numberOfBytes > 1) {
    regAddress |= 0x40;
  }
  
  // set the Chip select pin low to start an SPI packet.
  digitalWrite(PIN_SPI_SS, LOW);
  // transfer the starting register address that needs to be read.
  SPI.transfer(regAddress);
  
  // read the data
  for(int i=0; i<numberOfBytes; i++){
    values[i] = SPI.transfer(0x00);
  }

  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(PIN_SPI_SS, HIGH);
}

void ADXL375::writeRegister(uint8_t regAddress, uint8_t value)
{
    //Set Chip Select pin low to signal the beginning of an SPI packet.
  digitalWrite(PIN_SPI_SS, LOW);
  //Transfer the register address over SPI.
  SPI.transfer(regAddress);
  //Transfer the desired register value over SPI.
  SPI.transfer(value);
  //Set the Chip Select pin high to signal the end of an SPI packet.
  digitalWrite(PIN_SPI_SS, HIGH);
}

