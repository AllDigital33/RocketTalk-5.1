/*File   : MS5607.cpp
  Author : Amit Ate
  Email  : amit@uravulabs.com
  Company: Uravu Labs
  
  modified to break up six calls into individual calls
*/

#include <math.h>
#include <MS5607_MB.h>
#include <Wire.h>

MS5607::MS5607(){

}

MS5607::MS5607(short address){
  this->MS5607_ADDR = address;
}

// Initialise coefficient by reading calibration data
char MS5607::begin(){
  Wire.begin();
  return(readCalibration());
}

char MS5607::resetDevice(void){
  Wire.beginTransmission(MS5607_ADDR);
  Wire.write(RESET);
  char error = Wire.endTransmission();
  if(error == 0){
    delay(3);     // wait for internal register reload
    return(1);
  }else{return(0);}
}

// read calibration data from PROM
char MS5607::readCalibration(){
  if(resetDevice() &&
    readUInt_16(PROM_READ+2, C1) &&
    readUInt_16(PROM_READ+4, C2) &&
    readUInt_16(PROM_READ+6, C3) &&
    readUInt_16(PROM_READ+8, C4) &&
    readUInt_16(PROM_READ+10, C5) &&
    readUInt_16(PROM_READ+12, C6)
  ){
    return (1);
  }else{return(0);}
}

// convert raw data into unsigned int
char MS5607::readUInt_16(char address, unsigned int &value){
  unsigned char data[2];	//4bit
	data[0] = address;
	if (readBytes(data,2))
	{
		value = (((unsigned int)data[0]*(1<<8))|(unsigned int)data[1]);
		return(1);
	}
	value = 0;
	return(0);
}

char MS5607::setValuesMB(unsigned long DTX, unsigned long DPX) {
   DT = DTX;
   DP = DPX;
   return(1);
}

// read number of bytes over i2c
char MS5607::readBytes(unsigned char *values, char length){
	

	Wire.beginTransmission(MS5607_ADDR);
  Wire.write(values[0]);

	char error = Wire.endTransmission();
	if (error == 0)
	{
		Wire.requestFrom(MS5607_ADDR,length);
		while(!Wire.available()) ; // wait until bytes are ready
		for(int x=0; x < length ; x++)
		{
			values[x] = Wire.read();
		}
		return(1);
	}
	return(0);
}

// send command to start measurement
char MS5607::startMeasurment(void){
  Wire.beginTransmission(MS5607_ADDR);
  Wire.write(R_ADC);
  char error = Wire.endTransmission();
  if(error == 0){
    delay(3);
    return(1);
  }else{return(0);}
}

// send command to start conversion of temp/pressure
char MS5607::startConversion(char CMD){
  Wire.beginTransmission(MS5607_ADDR);
  Wire.write(CMD);
  char error = Wire.endTransmission();
  
  if(error == 0){
    delay(Conv_Delay);
    return(1);
  }else{return(0);}
}

// MODIFIED send command to start measurement
char MS5607::startMeasurmentMB(void){
  Wire.beginTransmission(MS5607_ADDR);
  Wire.write(R_ADC);
  char error = Wire.endTransmission();
  if(error == 0){
    //delay(3);
    return(1);
  }else{return(0);}
}

// MODIFIED send command to start conversion of temp/pressure
char MS5607::startConversionMB(char CMD){
  Wire.beginTransmission(MS5607_ADDR);
  Wire.write(CMD);
  char error = Wire.endTransmission();
  if(error == 0){
    //delay(Conv_Delay);
    return(1);
  }else{return(0);}
}



// read raw digital values of temp & pressure from MS5607
char MS5607::readDigitalValue(void){
    
    if(startConversion(CONV_D1)){
      if(startMeasurment()){
        if(getDigitalValue(DP));
      }
    }else{return 0;}

    if(startConversion(CONV_D2)){
      if(startMeasurment()){
        if(getDigitalValue(DT));
      }
    }else{return 0;}
 
    return 1;
}

char MS5607::getDigitalValue(unsigned long &value){
  int length = 3;
  unsigned char data[3];
    Wire.requestFrom(MS5607_ADDR,length);
    while(!Wire.available()) ; // wait until bytes are ready
    for(int x = 0; x < length; x++)
    {
      data[x] = Wire.read();
    }
    value = (unsigned long)data[0]*1<<16|(unsigned long)data[1]*1<<8|(unsigned long)data[2];
    return(1);
  }
  
char MS5607::getDigitalValueMB(unsigned long &value){
  int length = 3;
  unsigned char data[3];
    Wire.requestFrom(MS5607_ADDR,length);
    while(!Wire.available()) ; // wait until bytes are ready
    for(int x=0; x < length; x++)
    {
      data[x] = Wire.read();
    }
    value = (unsigned long)data[0]*1<<16|(unsigned long)data[1]*1<<8|(unsigned long)data[2];

    return(1);
  }  

  float MS5607::getTemperature(void){
    dT = (float)DT - ((float)C5)*((int)1<<8);
    TEMP = 2000.0 + dT * ((float)C6)/(float)((long)1<<23);
    return TEMP/100 ;
  }

  float MS5607::getPressure(void){
    dT = (float)DT - ((float)C5)*((int)1<<8);
    TEMP = 2000.0 + dT * ((float)C6)/(float)((long)1<<23);
    OFF = (((int64_t)C2)*((long)1<<17)) + dT * ((float)C4)/((int)1<<6);
    SENS = ((float)C1)*((long)1<<16) + dT * ((float)C3)/((int)1<<7);
    float pa = (float)((float)DP/((long)1<<15));
    float pb = (float)(SENS/((float)((long)1<<21)));
    float pc = pa*pb;
    float pd = (float)(OFF/((float)((long)1<<15)));
    P = pc - pd;
    return P/100;
  }

  float MS5607::getTemperature2(void){
    dT = (float)DT - ((float)C5)*(float)((int)1<<8);
    TEMP = (float)2000.0 + dT * ((float)C6)/(float)((long)1<<23);
    return TEMP / (float) 100 ;
  }

  float MS5607::getPressure2(void){
    dT = (float)DT - ((float)C5) * (float)((int)1<<8);
    TEMP = (float) 2000.0 + dT * ((float)C6)/(float)((long)1<<23);
    OFF = (((int64_t)C2)*(float)((long)1<<17)) + dT * ((float)C4)/((int)1<<6);
    SENS = ((float)C1)*(float)((long)1<<16) + dT * ((float)C3)/((int)1<<7);
    float pa = (float)((float)DP/(float)((long)1<<15));
    float pb = (float)(SENS/((float)((long)1<<21)));
    float pc = pa*pb;
    float pd = (float)(OFF/((float)((long)1<<15)));
    P = pc - pd;
    return P / (float) 100;
  }



  // set OSR and select corresponding values for conversion commands & delay
  void MS5607::setOSR(short OSR_U){
    this->OSR = OSR_U;
    switch (OSR) {
      case 256:
        CONV_D1 = 0x40;
        CONV_D2 = 0x50;
        Conv_Delay = 1;
        break;
      case 512:
        CONV_D1 = 0x42;
        CONV_D2 = 0x52;
        Conv_Delay = 2;
        break;
      case 1024:
        CONV_D1 = 0x44;
        CONV_D2 = 0x54;
        Conv_Delay = 3;
        break;
      case 2048:
        CONV_D1 = 0x46;
        CONV_D2 = 0x56;
        Conv_Delay = 5;
        break;
      case 4096:
        CONV_D1 = 0x48;
        CONV_D2 = 0x58;
        Conv_Delay = 10;
        break;
      default:
        CONV_D1 = 0x40;
        CONV_D2 = 0x50;
        Conv_Delay = 1;
        break;
    }
  }

  float MS5607::getAltitude(void){
    float h,p;
    p = getTemperature();
    p = 0.0;
    p = getPressure();
    p = p/P0;
    h = (44330.0 * (1 - pow(p, (float)0.190284)));
    return h;
  }

/* 

>>>>>> OLD CODE <<<<<<<<<

  float MS5607::getAltitude(void){
    float h,t,p;
    t = getTemperature();
    p = getPressure();
    p = P0/p;
    h = 153.84615*(pow(p,0.19) - 1)*(t+273.15);
    return h;
  }
  
  float MS5607::getAltitude2(void){
    float h,t,p;
    t = getTemperature2();
    p = getPressure2();
    p = P0/p;
    h = (float) 153.84615 * (pow(p,(float) 0.19) - (float) 1.0) * ( t + (float) 273.15);
    return h;
  }
  
  
>>>>>> END OLD CODE <<<<<<<<<
 
  
  sample BME code
  
  return(44330.0*(1-pow(Pressure/baseline-pressure,1/5.255)));
  
  GENESIS CODE
  
     float MS5607::getAltitude(void){
    float h,t,p;
    t = getTemperature();
    p = getPressure();
    p = p / (float) 1013.25;
    h = pow(p,(float)0.190284);
    h = ((float)1.0 - p)* (float)145366.45 / (float)3.28084;    
    return h;
  }
  
  // Given a pressure measurement (mb) and the pressure at a baseline (mb),
// return altitude change (in meters) for the delta in pressures.
double MS8607::altitudeChange(double currentPressure, double baselinePressure)
{
  return (44330.0 * (1 - pow(currentPressure / baselinePressure, 1 / 5.255)));
}


BOSCH

The altitude in meters can be calculated with the international barometric formula:

H = 44330 * [1 - (P/p0)^(1/5.255) ]


H = altitude (m)
P = measured pressure (Pa) from the sensor
p0 = reference pressure at sea level (e.g. 1013.25hPa)


  
*/
  
  
  
  
  
  
  
  
  