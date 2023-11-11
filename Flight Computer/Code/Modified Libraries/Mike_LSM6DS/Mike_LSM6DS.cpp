/* 
 *  
 *  Created by Mike B
 *  Library may be used freely and without limit with attribution.
 *  
 */
 
#include "Mike_LSM6DS.h"


/**************************************************************************/
/*!
    Sets the gyro data rate.
    @param  data_rate
            The the gyro data rate. Must be a `lsm6ds_data_rate_t`.
*/


void LSM6DS::setAccelRateScale(uint8_t data_rate) {

  // Pg. 67 in Datasheet
  // bits 1-4 rate, 5-6 scale, 7 res/LPF, 8=0

  _i2c_bus->writeByte(mLSM6DS_I2CADDR, mLSM6DS_CTRL1_XL, data_rate); //set accelerometer rate / scale
  
}

void LSM6DS::setGyroRateScale(uint8_t data_rate) {

  // Pg. 68 in Datasheet
  // bits 1-4 rate, 5-6 scale, 7 FS+125, 8=0

  _i2c_bus->writeByte(mLSM6DS_I2CADDR, mLSM6DS_CTRL2_G, data_rate); //set gyro rate / scale
  
}

void LSM6DS::setAnyRegister(uint8_t theRegister, uint8_t data_byte) {

  // used to update any register

  _i2c_bus->writeByte(mLSM6DS_I2CADDR, theRegister, data_byte); //set any register
  
}

int16_t LSM6DS::readTempData()
{
  uint8_t rawData[2];  
  _i2c_bus->readBytes(mLSM6DS_I2CADDR, mLSM6DS_OUT_TEMP_L, 2, &rawData[0]);  // Read the two raw data registers sequentially into data array
  Serial.println(rawData[0],BIN);
  Serial.println(rawData[1],BIN);
  int16_t theValue = (rawData[1] << 8) + rawData[0]; 
  Serial.println(theValue,BIN);
  return theValue;
  
  //return  (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
  
}

void LSM6DS::readAccelData(int16_t * destination) {

  uint8_t rawData[6];  // x/y/z accel register data stored here
  
  _i2c_bus->readBytes(mLSM6DS_I2CADDR, mLSM6DS_OUTX_L_A, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
    
}

void LSM6DS::readAllData(int16_t * destination) {

  uint8_t rawData[12];  // x/y/z accel register data stored here
  
  _i2c_bus->readBytes(mLSM6DS_I2CADDR, mLSM6DS_OUTX_L_G, 12, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 
    
  destination[3] = (int16_t) (((int16_t)rawData[7] << 8) | rawData[6]);   // Turn the MSB and LSB into a signed 16-bit value
  destination[4] = (int16_t) (((int16_t)rawData[9] << 8) | rawData[8]);  
  destination[5] = (int16_t) (((int16_t)rawData[11] << 8) | rawData[10]);     
    
}

uint8_t LSM6DS::whoAmI() {

  uint8_t rawData;  // x/y/z accel register data stored here
  
  _i2c_bus->readBytes(mLSM6DS_I2CADDR, mLSM6DS_WHOAMI, 1, &rawData);  // Read the six raw data registers sequentially into data array

  return rawData;
    
}



void LSM6DS::readGyroData(int16_t * destination) {

  uint8_t rawData[6];  // x/y/z gyro register data stored here
  
  _i2c_bus->readBytes(mLSM6DS_I2CADDR, mLSM6DS_OUTX_L_G, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t) (((int16_t)rawData[1] << 8) | rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) (((int16_t)rawData[3] << 8) | rawData[2]);  
  destination[2] = (int16_t) (((int16_t)rawData[5] << 8) | rawData[4]); 

}


/*

void LSM6DS::readGyroData(int16_t * destination) {

  uint8_t rawData[6];  // x/y/z gyro register data stored here
  
  _i2c_bus->readBytes(mLSM6DS_I2CADDR, mLSM6DS_OUTX_L_G, 6, &rawData[0]);  // Read the six raw data registers sequentially into data array
  destination[0] = (int16_t) ((rawData[1] << 8) + rawData[0]);   // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = (int16_t) ((rawData[3] << 8) + rawData[2]);  
  destination[2] = (int16_t) ((rawData[5] << 8) + rawData[4]); 

}

*/

/*

void Adafruit_LSM6DS::_read(void) {
  // get raw readings
  Adafruit_BusIO_Register data_reg = Adafruit_BusIO_Register(
      i2c_dev, spi_dev, ADDRBIT8_HIGH_TOREAD, LSM6DS_OUT_TEMP_L, 14);

  uint8_t buffer[14];
  data_reg.read(buffer, 14);

  rawTemp = buffer[1] << 8 | buffer[0];
  temperature = (rawTemp / 256.0) + 25.0;

  rawGyroX = buffer[3] << 8 | buffer[2];
  rawGyroY = buffer[5] << 8 | buffer[4];
  rawGyroZ = buffer[7] << 8 | buffer[6];

  rawAccX = buffer[9] << 8 | buffer[8];
  rawAccY = buffer[11] << 8 | buffer[10];
  rawAccZ = buffer[13] << 8 | buffer[12];

  lsm6ds_gyro_range_t gyro_range = getGyroRange();
  float gyro_scale = 1; // range is in milli-dps per bit!
  if (gyro_range == ISM330DHCX_GYRO_RANGE_4000_DPS)
    gyro_scale = 140.0;
  if (gyro_range == LSM6DS_GYRO_RANGE_2000_DPS)
    gyro_scale = 70.0;
  if (gyro_range == LSM6DS_GYRO_RANGE_1000_DPS)
    gyro_scale = 35.0;
  if (gyro_range == LSM6DS_GYRO_RANGE_500_DPS)
    gyro_scale = 17.50;
  if (gyro_range == LSM6DS_GYRO_RANGE_250_DPS)
    gyro_scale = 8.75;
  if (gyro_range == LSM6DS_GYRO_RANGE_125_DPS)
    gyro_scale = 4.375;

  gyroX = rawGyroX * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;
  gyroY = rawGyroY * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;
  gyroZ = rawGyroZ * gyro_scale * SENSORS_DPS_TO_RADS / 1000.0;

  lsm6ds_accel_range_t accel_range = getAccelRange();
  float accel_scale = 1; // range is in milli-g per bit!
  if (accel_range == LSM6DS_ACCEL_RANGE_16_G)
    accel_scale = 0.488;
  if (accel_range == LSM6DS_ACCEL_RANGE_8_G)
    accel_scale = 0.244;
  if (accel_range == LSM6DS_ACCEL_RANGE_4_G)
    accel_scale = 0.122;
  if (accel_range == LSM6DS_ACCEL_RANGE_2_G)
    accel_scale = 0.061;

  accX = rawAccX * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
  accY = rawAccY * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
  accZ = rawAccZ * accel_scale * SENSORS_GRAVITY_STANDARD / 1000;
}
*/

/*
void LSM6DS::setGyroDataRate(uint8_t data_rate) {

  uint8_t bytes[4], STAT;
  bytes[0] = gyro_fs & (0xFF);
  bytes[1] = (gyro_fs >> 8) & (0xFF);
  bytes[2] = 0x00;
  bytes[3] = 0x00;
  _i2c_bus->writeByte(LSM6DS_I2CADDR, EM7180_LoadParamByte0, bytes[0]); //Gyro LSB
  _i2c_bus->writeByte(LSM6DS_I2CADDR, EM7180_LoadParamByte1, bytes[1]); //Gyro MSB
  _i2c_bus->writeByte(LSM6DS_I2CADDR, EM7180_LoadParamByte2, bytes[2]); //Unused
  _i2c_bus->writeByte(LSM6DS_I2CADDR, EM7180_LoadParamByte3, bytes[3]); //Unused
  _i2c_bus->writeByte(LSM6DS_I2CADDR, EM7180_ParamRequest, 0xCB); //Parameter 75; 0xCB is 75 decimal with the MSB set high to indicate a paramter write processs
  _i2c_bus->writeByte(LSM6DS_I2CADDR, EM7180_AlgorithmControl, 0x80); //Request parameter transfer procedure
  STAT = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge); //Check the parameter acknowledge register and loop until the result matches parameter request byte
  while(!(STAT==0xCB)) {
  STAT = _i2c_bus->readByte(EM7180_ADDRESS, EM7180_ParamAcknowledge);
  }
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_ParamRequest, 0x00); //Parameter request = 0 to end parameter transfer process
  _i2c_bus->writeByte(EM7180_ADDRESS, EM7180_AlgorithmControl, 0x00); // Re-start algorithm



}

*/