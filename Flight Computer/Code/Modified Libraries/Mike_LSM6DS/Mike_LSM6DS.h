/*!
 *  @file Mike_LSM6DS.h
 *
 * 	I2C Driver base for LSM6DS 6-DoF Accelerometer and Gyroscope
 *  For use with I2CDev
 *
 * */

#ifndef Mike_LSM6DS_h
#define Mike_LSM6DS_h

#include "I2Cdev.h"   



// LSM6DS register map
// see https://www.st.com/resource/en/datasheet/lsm6dso32.pdf
//


#define mLSM6DS_I2CADDR 0x6A        ///< LSM6DS default i2c address (5.)
//#define mLSM6DS_I2CADDR 0x6B        ///< LSM6DS default i2c address (4.6)
#define mLSM6DS_FUNC_CFG_ACCESS 0x1 ///< Enable embedded functions register
#define mLSM6DS_INT1_CTRL 0x0D      ///< Interrupt control for INT 1
#define mLSM6DS_INT2_CTRL 0x0E      ///< Interrupt control for INT 2
#define mLSM6DS_WHOAMI 0xF          ///< Chip ID register
#define mLSM6DS_CTRL1_XL 0x10       ///< Main accelerometer config register
#define mLSM6DS_CTRL2_G 0x11        ///< Main gyro config register
#define mLSM6DS_CTRL3_C 0x12        ///< Main configuration register
#define mLSM6DS_CTRL8_XL 0x17       ///< High and low pass for accel
#define mLSM6DS_CTRL10_C 0x19       ///< Main configuration register
#define mLSM6DS_WAKEUP_SRC 0x1B     ///< Why we woke up
#define mLSM6DS_OUT_TEMP_L 0x20     ///< L-H First data register (temperature low)
#define mLSM6DS_OUT_TEMP_H 0x21     ///< L-H Second data register (temperature high)
#define mLSM6DS_OUTX_L_G 0x22       ///< L-H XYZ First gyro data register
#define mLSM6DS_OUTX_L_A 0x28       ///< L-H XYZ First accel data register
#define mLSM6DS_STEPCOUNTER 0x4B    ///< 16-bit step counter
#define mLSM6DS_TAP_CFG 0x58        ///< Tap/pedometer configuration
#define mLSM6DS_WAKEUP_THS 0x5B     ///< Single and double-tap function threshold register
#define mLSM6DS_WAKEUP_DUR 0x5C     ///< Free-fall, wakeup, timestamp and sleep mode duration
#define mLSM6DS_MD1_CFG 0x5E        ///< Functions routing on INT1 register

#define mLSM6DS_PIN_CTRL 0x02       ///< Enable pull-up on SDO
#define mLSM6DS_CTRL4_C 0x13        ///< Additional Gyro Settings 
#define mLSM6DS_CTRL5_C 0x14        ///< Additional Accel Settings
#define mLSM6DS_CTRL6_C 0x15        ///< Additional Accel and Gyro Settings
#define mLSM6DS_CTRL7_G 0x16        ///< Additional Accel and Gyro Settings
#define mLSM6DS_CTRL9_XL 0x18       ///< Accel DEN settings
#define mLSM6DS_X_OFS_USR 0x73      ///< Accel X offset user (subtracted in 2's C)
#define mLSM6DS_Y_OFS_USR 0x74      ///< Accel Y offset user (subtracted in 2's C)
#define mLSM6DS_Z_OFS_USR 0x75      ///< Accel Z offset user (subtracted in 2's C)
#define mLSM6DS_I2CADDR_DEFAULT 0x6C





class LSM6DS
{
  public: 
  
  
  void setAccelRateScale(uint8_t data_rate); 
  void setGyroRateScale(uint8_t data_rate); 
  void setAnyRegister(uint8_t theRegister, uint8_t data_byte); 
  
  
  int16_t readTempData();
  void readAccelData(int16_t * destination);
  void readGyroData(int16_t * destination);
  void readAllData(int16_t * destination);
  uint8_t whoAmI();
  
  
  I2Cdev* _i2c_bus;  
  
  
/*  
  USFS(uint8_t intPin, bool passThru, I2Cdev* i2c_bus);
  float uint32_reg_to_float (uint8_t *buf);
  float int32_reg_to_float (uint8_t *buf);
  void float_to_bytes (float param_val, uint8_t *buf);
  void EM7180_set_gyro_FS (uint16_t gyro_fs);
  void EM7180_set_mag_acc_FS (uint16_t mag_fs, uint16_t acc_fs);
  void EM7180_set_integer_param (uint8_t param, uint32_t param_val);
  void EM7180_set_float_param (uint8_t param, float param_val);
  void readSENtralQuatData(float * destination);
  void readSENtralAccelData(int16_t * destination);
  void readSENtralGyroData(int16_t * destination);
  void readSENtralMagData(int16_t * destination);
  void initEM7180(uint8_t accBW, uint8_t gyroBW, uint16_t accFS, uint16_t gyroFS, uint16_t magFS, uint8_t QRtDiv, uint8_t magRt, uint8_t accRt, uint8_t gyroRt, uint8_t baroRt);
  int16_t readSENtralBaroData();
  int16_t readSENtralTempData();
  void SENtralPassThroughMode();
  void M24512DFMwriteByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t  data);
  void M24512DFMwriteBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest);
  uint8_t M24512DFMreadByte(uint8_t device_address, uint8_t data_address1, uint8_t data_address2);
  void M24512DFMreadBytes(uint8_t device_address, uint8_t data_address1, uint8_t data_address2, uint8_t count, uint8_t * dest);
  void MadgwickQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
  void MahonyQuaternionUpdate(float ax, float ay, float az, float gx, float gy, float gz, float mx, float my, float mz);
  void getChipID();
  void loadfwfromEEPROM();
  uint8_t checkEM7180Status();
  uint8_t checkEM7180Errors();
  private:
  uint8_t _intPin;
  bool _passThru;
  float _q[4];
  float _beta;
  float _deltat;
  float _Kp;
  float _Ki;
  */

};

#endif
