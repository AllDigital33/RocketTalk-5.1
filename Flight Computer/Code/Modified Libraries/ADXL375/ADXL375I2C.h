#ifndef ADX375_h
#define ADX375_h

#include "Arduino.h"
#include "AccelReading.h"
#include "SPI.h"
#include <Wire.h>

#define I2C_MODE 0
#define SPI_MODE 1

#define NO_WIRE 0
#define HARD_WIRE 1
#define SOFT_WIRE 2



#define ADX375_REG_DEVID               0x00    // Device ID
#define ADX375_REG_THRESH_SHOCK        0x1D    // Shock threshold
#define ADX375_REG_OFSX                0x1E    // X-axis offset
#define ADX375_REG_OFSY                0x1F    // Y-axis offset
#define ADX375_REG_OFSZ                0x20    // Z-axis offset
#define ADX375_REG_DUR                 0x21    // Shock duration
#define ADX375_REG_LATENT              0x22    // Shock latency
#define ADX375_REG_WINDOW              0x23    // Tap window
#define ADX375_REG_THRESH_ACT          0x24    // Activity threshold
#define ADX375_REG_THRESH_INACT        0x25    // Inactivity threshold
#define ADX375_REG_TIME_INACT          0x26    // Inactivity time
#define ADX375_REG_ACT_INACT_CTL       0x27    // Axis enable control for activity and inactivity detection
#define ADX375_REG_SHOCK_AXES          0x2A    // Axis control for single/double tap
#define ADX375_REG_ACT_SHOCK_STATUS    0x2B    // Source for single/double tap
#define ADX375_REG_BW_RATE             0x2C    // Data rate and power mode control
#define ADX375_REG_POWER_CTL           0x2D    // Power-saving features control
#define ADX375_REG_INT_ENABLE          0x2E    // Interrupt enable control
#define ADX375_REG_INT_MAP             0x2F    // Interrupt mapping control
#define ADX375_REG_INT_SOURCE          0x30    // Source of interrupts
#define ADX375_REG_DATA_FORMAT         0x31    // Data format control
#define ADX375_REG_DATAX0              0x32    // X-axis data 0
#define ADX375_REG_DATAX1              0x33    // X-axis data 1
#define ADX375_REG_DATAY0              0x34    // Y-axis data 0
#define ADX375_REG_DATAY1              0x35    // Y-axis data 1
#define ADX375_REG_DATAZ0              0x36    // Z-axis data 0
#define ADX375_REG_DATAZ1              0x37    // Z-axis data 1
#define ADX375_REG_FIFO_CTL            0x38    // FIFO control
#define ADX375_REG_FIFO_STATUS         0x39    // FIFO status

#define ADX375_XYZ_READ_SCALE_FACTOR   49      // scaling factor when reading xyz data
#define ADX375_THRESH_SHOCK_SCALE      780     // scaling factor for shock threshold register

#define ADX375_FIFO_MODE_BYPASS        0b00
#define ADX375_FIFO_MODE_FIFO          0b01
#define ADX375_FIFO_MODE_STREAM        0b10
#define ADX375_FIFO_MODE_TRIGGER       0b11

#define ADX375_TRIGGER_INT1_PIN        17
#define ADX375_TRIGGER_INT2_PIN        18
//#define PIN_SPI_SS				        5


//Class SensorSettings.  This object is used to hold settings data.  The application
//uses this classes' data directly.  The settings are adopted and sent to the sensor
//at special times, such as .begin.  Some are used for doing math.
//
//This is a kind of bloated way to do this.  The trade-off is that the user doesn't
//need to deal with #defines or enums with bizarre names.
//
//A power user would strip out SensorSettings entirely, and send specific read and
//write command directly to the IC. (ST #defines below)
//
// Mike modified from SensorSettings to ADXLSensorSettings, as name was already used by another library
struct ADXLSensorSettings
{
  public:
	
	//Main Interface and mode settings
    uint8_t commInterface;
    uint8_t I2CAddress;
    uint8_t chipSelectPin;
	
	//Deprecated settings
	uint8_t runMode;
	uint8_t tStandby;
	uint8_t filter;
	uint8_t tempOverSample;
	uint8_t pressOverSample;
	uint8_t humidOverSample;
    float tempCorrection; // correction of temperature - added to the result
};




class ADX375  //modified from ADX
{
  public:
    //settings
    ADXLSensorSettings settings;
//	SensorCalibration calibration;
	int32_t t_fine;
	
	//Constructor generates default SensorSettings.
	//(over-ride after construction if desired)
    ADX375( void );	
	//Call to apply SensorSettings.
	//This also gets the SensorCalibration constants
    uint8_t begin( void );
    bool beginSPI(uint8_t csPin); //Communicate using SPI
    bool beginI2C(TwoWire &wirePort = Wire); //Called when user provides Wire port    
    
	//ReadRegisterRegion takes a uint8 array address as input and reads
	//a chunk of memory into that array.
    void readRegisterRegion(uint8_t*, uint8_t, uint8_t );
	//readRegister reads one register
    uint8_t readRegister(uint8_t);

    //Reads two regs, LSByte then MSByte order, and concatenates them
	//Used for two-byte reads
	int16_t readRegisterInt16( uint8_t offset );
	//Writes a byte;
    void writeRegister(uint8_t, uint8_t);    
    
//    void init();
    void startMeasuring();
    void readIt();    
    void setOutput();
    void setInterupt();
    void setFifo();
    void startup();
    AccelReading getXYZ();
//    uint8_t readRegister(uint8_t regAddress);
//    void writeRegister(uint8_t regAddress, uint8_t value);
//    void setShockThreshold(uint8_t shockThreshold);
//    void setShockAxes(bool x = true, bool y = true, bool z = true);
//    void startShockDetection();
//    void setDataRate(uint8_t rate);
    uint8_t getFIFOBufferSize();
    void setFIFOMode(uint8_t mode, uint8_t trigger = 0, uint8_t samples = 0);
  private:
    uint8_t _wireType = HARD_WIRE; //Default to Wire.h
    TwoWire *_hardPort = NO_WIRE; //The generic connection to user's chosen I2C hardware
    
	#ifdef SoftwareWire_h
	SoftwareWire *_softPort = NO_WIRE; //Or, the generic connection to software wire port
	#endif
    void _multiReadRegister(uint8_t regAddress, uint8_t values[], uint8_t numberOfBytes = 1);
};

#endif
