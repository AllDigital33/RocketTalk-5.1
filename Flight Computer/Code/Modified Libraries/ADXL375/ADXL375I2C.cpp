/******************************************************************************
hacked ADXL375 library
******************************************************************************/


#include "ADXL375I2C.h"

//****************************************************************************//
//
//  Settings and configuration
//
//****************************************************************************//

//Constructor -- Specifies default configuration
ADX375::ADX375( void )
{
	//Construct with these default settings

	settings.commInterface = I2C_MODE; //Default to I2C

	//settings.I2CAddress = 0x53; //Default, jumper open is 0x77
	settings.I2CAddress = 0x1D; //Default, jumper open is 0x77
	_hardPort = &Wire; //Default to Wire port

	//settings.chipSelectPin = 10; //Select CS pin for SPI
	
	//These are deprecated settings

}

void ADX375::readIt()
{
	uint8_t readInt = readRegister(ADX375_REG_POWER_CTL); 
	Serial.print("read: ");Serial.println(readInt);
}

void ADX375::startMeasuring()
{
  writeRegister(ADX375_REG_POWER_CTL, 0x08);
}

void ADX375::setOutput()
{
  writeRegister(ADX375_REG_BW_RATE, 0x0A);
}

void ADX375::setFifo()
{
  writeRegister(ADX375_REG_POWER_CTL, 0x80);
}

void ADX375::setInterupt()
{
  writeRegister(ADX375_REG_INT_ENABLE, 0x80);
}

void ADX375::startup()
{
  // set output to 100 hz
  //writeRegister(ADX375_REG_BW_RATE, 0x0A);
 // writeRegister(ADX375_REG_BW_RATE, 0x0C);  //400Hz
  //writeRegister(ADX375_REG_BW_RATE, 0x0D);  //800Hz  

  // reset then enable FIFO trigger mode
  
  //setFIFOMode(ADX375_FIFO_MODE_STREAM);
  //setFIFOMode(ADX375_FIFO_MODE_TRIGGER, ADX375_TRIGGER_INT1_PIN, 16);


  // set the interrupt bit for shock detection
//  uint8_t intEnable = readRegister(ADX375_REG_INT_ENABLE);
//  writeRegister(ADX375_REG_INT_ENABLE, intEnable | 0b01000000);

  startMeasuring();

}

// returns the number of items in the FIFO buffer
uint8_t ADX375::getFIFOBufferSize()
{
  return readRegister(ADX375_REG_FIFO_STATUS) & 0b00111111;
}


//****************************************************************************//
//
//  Configuration section
//
//  This uses the stored SensorSettings to start the IMU
//  Use statements such as "mySensor.settings.commInterface = SPI_MODE;" to 
//  configure before calling .begin();
//
//****************************************************************************//
uint8_t ADX375::begin()
{
	delay(2);  //Make sure sensor had enough time to turn on. ADX375 requires 2ms to start up.

	//Check the settings structure values to determine how to setup the device
	switch (settings.commInterface)
	{

	case I2C_MODE:
		
		switch(_wireType)
		{
			case(HARD_WIRE):
				_hardPort->begin(); //The caller can begin their port and set the speed. We just confirm it here otherwise it can be hard to debug.
				break;
			case(SOFT_WIRE):
			#ifdef SoftwareWire_h
				_softPort->begin(); //The caller can begin their port and set the speed. We just confirm it here otherwise it can be hard to debug.
			#endif
				break;
		}
		break;


	default:
		break;
	}

	//Check communication with IC before anything else
	uint8_t chipID = readRegister(0x00); //Should return 0x60 or 0x58
	//Serial.print("ID: ");Serial.println(chipID);
	if(chipID != 0xE5 && chipID != 0x60) // Is this BMP or BME?
	return(chipID); //This is not BMP nor BME!
	
	return(readRegister(0x00)); //Should return 0x60
}



//Begin comm with ADX375 over I2C
bool ADX375::beginI2C(TwoWire &wirePort)
{
	_hardPort = &wirePort;
	_wireType = HARD_WIRE;

	settings.commInterface = I2C_MODE;
	
	//settings.I2CAddress = 0x77; //We assume user has set the I2C address using setI2CAddress()
	if(begin() == 229) return(true); 
	Serial.print("Bad ID: ");Serial.println(begin());//Begin normal init with these settings. Should return chip ID of 0x58 for ADXL
	return(false);
}

//Begin comm with ADX375 over software I2C
#ifdef SoftwareWire_h
bool ADX375::beginI2C(SoftwareWire& wirePort)
{
	_softPort = &wirePort;
	_wireType = SOFT_WIRE;

	settings.commInterface = I2C_MODE;
	//settings.I2CAddress = 0x77; //We assume user has set the I2C address using setI2CAddress()

	if(begin() == 229) return(true); //Begin normal init with these settings. Should return chip ID of 0x58 for BMP
	if(begin() == 0x60) return(true); //Begin normal init with these settings. Should return chip ID of 0x60 for BME
	return(false);
}
#endif



//****************************************************************************//
//
//  Utility
//
//****************************************************************************//
void ADX375::readRegisterRegion(uint8_t *outputPointer , uint8_t offset, uint8_t length)
{
	//define pointer that will point to the external space
	uint8_t i = 0;
	char c = 0;

	switch (settings.commInterface)
	{

	case I2C_MODE:
		switch(_wireType)
		{
			case(HARD_WIRE):
				_hardPort->beginTransmission(settings.I2CAddress);
				_hardPort->write(offset);
				_hardPort->endTransmission();

				// request bytes from slave device
				_hardPort->requestFrom(settings.I2CAddress, length);
				while ( (_hardPort->available()) && (i < length))  // slave may send less than requested
				{
					c = _hardPort->read(); // receive a byte as character
					*outputPointer = c;
					outputPointer++;
					i++;
				}
				break;
			case(SOFT_WIRE):
			#ifdef SoftwareWire_h
				_softPort->beginTransmission(settings.I2CAddress);
				_softPort->write(offset);
				_softPort->endTransmission();

				// request bytes from slave device
				_softPort->requestFrom(settings.I2CAddress, length);
				while ( (_softPort->available()) && (i < length))  // slave may send less than requested
				{
					c = _softPort->read(); // receive a byte as character
					*outputPointer = c;
					outputPointer++;
					i++;
				}
			#endif
				break;
		}
		break;


	default:
		break;
	}

}

uint8_t ADX375::readRegister(uint8_t offset)
{
	//Return value
	uint8_t result = 0;
	uint8_t numBytes = 1;
	switch (settings.commInterface) {

	case I2C_MODE:
		switch(_wireType)
		{
			case(HARD_WIRE):
				_hardPort->beginTransmission(settings.I2CAddress);
				_hardPort->write(offset);
				_hardPort->endTransmission();

				_hardPort->requestFrom(settings.I2CAddress, numBytes);
				while ( _hardPort->available() ) // slave may send less than requested
				{
					result = _hardPort->read(); // receive a byte as a proper uint8_t
				}
				break;
			
			case(SOFT_WIRE):
			#ifdef SoftwareWire_h
				_softPort->beginTransmission(settings.I2CAddress);
				_softPort->write(offset);
				_softPort->endTransmission();

				_softPort->requestFrom(settings.I2CAddress, numBytes);
				while ( _softPort->available() ) // slave may send less than requested
				{
					result = _softPort->read(); // receive a byte as a proper uint8_t
				}
			#endif
				break;
		}
		
		break;

	default:
		break;
	}
	return result;
}

int16_t ADX375::readRegisterInt16( uint8_t offset )
{
	uint8_t myBuffer[2];
	readRegisterRegion(myBuffer, offset, 2);  //Does memory transfer
	int16_t output = (int16_t)myBuffer[0] | int16_t(myBuffer[1] << 8);
	
	return output;
}

void ADX375::writeRegister(uint8_t offset, uint8_t dataToWrite)
{
	switch (settings.commInterface)
	{
	case I2C_MODE:
		//Write the byte

		switch(_wireType)
		{
			case(HARD_WIRE):
				_hardPort->beginTransmission(settings.I2CAddress);
				_hardPort->write(offset);
				_hardPort->write(dataToWrite);
				_hardPort->endTransmission();
				break;
			case(SOFT_WIRE):
			#ifdef SoftwareWire_h
				_softPort->beginTransmission(settings.I2CAddress);
				_softPort->write(offset);
				_softPort->write(dataToWrite);
				_softPort->endTransmission();
			#endif
				break;
		}
		break;
		

	default:
		break;
	}
}

void ADX375::setFIFOMode(uint8_t mode, uint8_t pin, uint8_t samples)
{
  writeRegister(ADX375_REG_FIFO_CTL, mode<<6 | pin<<5 | samples);
}

void ADX375::_multiReadRegister(uint8_t regAddress, uint8_t values[], uint8_t numberOfBytes)
{
  // Since we're performing a read operation, the most significant bit of the register address should be set.
  regAddress |= 0x80;

  // set the multi-read byte if required
  if (numberOfBytes > 1) {
    regAddress |= 0x40;
  }
  
  // read the data
  for(int i=0; i<numberOfBytes; i++){
    values[i] = SPI.transfer(0x00);
  }

  //Set the Chips Select pin high to end the SPI packet.
  digitalWrite(PIN_SPI_SS, HIGH);
}

AccelReading ADX375::getXYZ()
{
  uint8_t data[6];
  _multiReadRegister(ADX375_REG_DATAX0, data, 6);

  AccelReading xyz;
  xyz.init(
    data[0] | data[1]<<8,
    data[2] | data[3]<<8,
    data[4] | data[5]<<8,
    ADX375_XYZ_READ_SCALE_FACTOR
  );
  
  return xyz;
}



