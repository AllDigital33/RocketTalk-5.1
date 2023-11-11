

/* RocketTalk New Board Diagnostics 5.1
 *  
 *  Used to test all sensors, inputs and outputs on a new RT board
 *  Modified for RocketTalk Build 5.1 pins and hardware
 *  
 *  This is ALL you need for health check and calibration...
 *  
 *  
 */

  const bool sGPS = true;
  const bool sRadio = false;
  const bool sADXL = true;
  const bool sLSM = true;
  const bool sMS56 = true;
  const bool sLogging = true;


  float tempX;
  float tempY;
  float tempZ;


  #define GPSmodel 10 // (only Ublox Max-M10 logic in this code
// 5.1 pin configuration
  #define relayMain 41
  #define continuityMainPin 40
  #define relayDrogue 14
  #define continuityDroguePin 17
  #define relaySeparation 16
  #define continuitySeparationPin 15
  #define relaySustainer 22
  #define continuitySustainerPin 21
  #define fore_pin 37
  #define aft_pin 38
  #define sep_pin 39
  #define relayCamera 36
  #define led1_pin 33
  #define led2_pin 34
  #define led3_pin 35  
  #define intADXL 28
  #define int2ADXL 31
  #define radioPower_pin 12 //radio power
  #define GPSpower_pin 11 //GPS power  
  #define voltage_pinA A9 
  #define RXD1 0
  #define TXD1 1
  #define RXD2 7
  #define TXD2 8
  #define int1LSM 29
  #define int2LSM 30
  #define radioSet 32
  // also grounded pins 2,3,4,5,6,9,10,20,24,25,26,27

//Little FS flash
  #include <LittleFS.h>
  char buf[512] = "";
  char fname1[32] = "/testfile.txt";
  LittleFS_QSPIFlash myfs;
  File file, file1, file3;


//Teensy
  #include "Arduino.h"

// track the toggle logic
  bool tLED1 = false;
  bool tLED2 = false;
  bool tLED3 = false;
  bool tMain = false;
  bool tDrogue = false;
  bool tSep = false;
  bool tSus = false;
  bool tRadio = false;
  bool tGPS = false;
  bool tCam = false;



//ADXL375 Accelerometer 
  #include "ADXL375I2C.h"
  ADX375 Accel;

// For MS5607 barometer
  #include<MS5607_MB.h>
  MS5607 P_Sens;


// For LSM6DSO32 
   #include "Mike_LSM6DS.h"
   LSM6DS LSM6;  
   // QUATERNION STUFF
   #include "math.h"
   struct quaternion {
     float r;    /* real bit */
     float x, y, z;  /* imaginary bits */
    };
   quaternion masterQuaternion; 
   quaternion tempQuaternion;
   float angX, angY, angZ;

  struct configStruct {
   bool drogue = true; // fire drogue event on apogee
   bool main = true; // fire main event on descent
   float mainAltitude = 1500.0; // deploy main at 1500 feet
   bool twoStage = false; // two stage flight logic
   bool separation = false; // fire separation charge on two stage
   bool logging = true; // enable flash logging thread loop
   bool gps = false; // enable GPS checking
   bool radio = false; // enable radio checking
   bool sampling = true; //enable sampling thread loop
   bool baro = true; // enable barometer checking
   bool ADXL = true; // enable ADXL checking
   bool LSM = true; // enable LSM checking
   int machLockout = 6000;  // seconds to lock-out baro for mach
   int altitudeMin = 500;  // 500 feet minimum before pyro events
   float apogeeSpeed = 5.0; //apogee below 5fps speed
   float apogeeVelocity = 500.0; //apogee below integrated velocity of 500 on adxl
   int stagingMaxTime = 25000; // 25 seconds post launch max for staging
   float stagingSpeed = 500.0; // need to be > 500fps for staging
   int burnoutPlus = 2000; // separation two seconds after burnout
   int burnoutArm = 3000; // 3G - must hit three G before allowing burnout
   int sustainerTime = 2000; // ignite sustainer two seconds after separation
   float stagingTilt = 35.0; // 35 degree lock-out for firing sustainer
   int pyroDuration = 1000; // 1.0 second hold for pyros
   //bool invertAft = false; // used for latch switch (NO) vs normaly closed switch
   bool invertSep = false; // used for latch switch (NO) vs normaly closed switch
   
  };
  configStruct configs;



  struct workingStruct {  // working variables
    char phase[25];   // Normally should be set to STARTUP
    char callsign[10];  //Radio requires Amatuer Radio Licence
    // Testing Phase variables
    char FlightLogMaster[800];
    char RadioLogMaster[400];
    char FlightSummaryMaster[600];
    byte radioOff = 0;
    char eventString[200];
    byte logFastMode = 0; //used to keep log files open during ascent
    byte flightLogOpen = 0;
    byte gpsLogOpen = 0;
    byte radioLogOpen = 0;
    char sendFlight[150];
    char sendSummary1[250];
    char sendSummary2[250];
    char sendSummary3[250];
    char timeHeader[50];
    int summaryRepeat = 0;
    byte LaunchNumber = 0;  // used to name files on the SD card for each unique launch
    char LaunchNumberStr[16];
    unsigned long eventTimer = 0;
    byte triggerCount = 0;
    byte triggerCount2 = 0;
    byte error = 0;
    char errorMessage[100];
    byte camera = 0;
    unsigned long cameraLockout = 0;
    float voltage = 0;    
    int eLogCount = 0;
    int eLogPos = 0;
    char eventLogQ[15][200];
    int eLogLast = 0;
    int gpsLogCount = 0;
    int gpsLogPos = 0;
    char gpsLogQ[15][200];
    int gpsLogLast = 0;  
    char gpsString[200];  
    int radioLogCount = 0;
    int radioLogPos = 0;
    char radioLogQ[15][200];
    int radioLogLast = 0;  
    char radioString[200];   
    byte summaryWrite = 0;  
    int retries = 0;
    unsigned long loggingClock;
    bool loggingOK = true;
    unsigned long sampleClock;
    bool sampleOK = true;    
    float CPUtemp = 0.0;
    float CPUtempMax = 0.0;
    float accelPre = 0;
    int lockout2 = 0;
    byte eraseFlash = 0;

  };
  workingStruct working;


  struct eventStruct {  // event variables
    
    char launchTime[50];
    unsigned long launchClock;    
    bool burnout;
    float burnoutSeconds;
    unsigned long burnoutClock;
    unsigned long separationClock;
    unsigned long sustainerClock;  
    unsigned long apogeeClock;  
    unsigned long mainClock;
    unsigned long landedClock;
    unsigned long machClock;
    byte launch;
    byte apogee;
    byte landed;
    char apogeeTime[50];
    char apogeeType[5];    
    char landedTime[50];
    char launchType[5];
    char foreTime[50];
    char aftTime[50];  
    char sepTime[50]; 
    byte retries = 0;
    byte clockSet;
    int ascentSeconds;
    int descentSeconds;
    byte EventFore = 0;
    byte EventAft = 0;
    byte EventSep = 0;
    byte pyroSeparation = 0;
    byte pyroSustainer = 0;
    byte pyroMain = 0;
    byte pyroDrogue = 0;
    byte continuityMain = 0;
    byte continuityDrogue = 0;
    byte continuitySeparation = 0;
    byte continuitySustainer = 0;
    byte stagingArmed = 0;
  };
  eventStruct events;

  struct timerStruct {  // timer variables

  // logging intervals
    int eventLogInterval = 200;
    int radioLogInterval = 500;
    int gpsLogInterval = 500;

    int flightLogInterval = 1000;
  // Housekeeping
    unsigned long gpsSendTimer = 0;
    unsigned long gpsLogTimer = 0;
    unsigned long sendFlightTimer = 0;
    unsigned long voltageTimer = 0;
    unsigned long eventsTimer = 0;
    unsigned long continuityTimer = 0;
    unsigned long radioRXTimer = 0;
    unsigned long radioTXTimer = 0;
    unsigned long flightLogTimer = 0;
    unsigned long radioSendCallSignTimer = 90000;
    unsigned long pyroOffTimer = 0;
    unsigned long pyroContinuityTimer = 0;
    unsigned long loggingHealth = 0;
    unsigned long sampleHealth = 0;
    int gpsSendInterval = 5000;
    int sendFlightInterval = 4000;
    int voltageInterval = 1000; 
    int eventsInterval = 1000;
    int continuityInterval = 500;
    int radioRXInterval = 500;
    int radioTXInterval = 1200;
    int radioSendCallSignInterval = 90000;
    unsigned long PhaseTimerA;  // used for different events within each phase
    unsigned long PhaseTimerB;
    unsigned long PhaseTimerC;    
    unsigned long pyroMainTimer = 0;
    unsigned long pyroDrogueTimer = 0;
    unsigned long pyroSeparationTimer = 0;
    unsigned long pyroSustainerTimer = 0;
  };
  timerStruct timers;
  
  
  struct LSMStruct {  //LSM6DSO32 variables
    int count;
    float x, y, z;   
    float linZ;
    float avgZ;
    float tempAvgZ;
    float maxZ;
    float minZ;
    float velocity;
    float velocityLast; 
    byte apogee;    
    unsigned long last;   
    float launchMaxZ;  
    int bias;
    int tempbias;
    int biasCount;     
    byte burnout = 0;    
    byte burnoutArm = 0;   
    
    float gX, gY, gZ;  // raw avg
    float tilt = 0.0;
    float tiltMax = 0.0;
    float tiltMaxLaunch = 0.0;
    float yaw;
    float pitch;
    float roll;
  };
  LSMStruct LSM6D;

// global for the barometer tracking 
  struct baroStruct {
    float Altitude = 0.0;
    float AltitudeMax = 0.0;
    float tempAltitude = 0.0;
    float lastAltitude;
    float Baseline = 0.0;
    float TempF;    
    float TempMax;    
    float Speed = 0.0;  // speed in FPS
    float LandedSpeed = 0.0;
    float SpeedMax = 0.0; 
    float Pressure = 0.0;
    float Tempraw = 0.0;
    float Altraw = 0.0;
    unsigned long launchDetectDelay;
    byte apogee;  
    int count;  
    byte launch;
  };
  baroStruct baro;

  //ADXL375 Accelerometer 
  #include "ADXL375I2C.h"
  struct adxlStruct {  // ADXL Accelerometer Variables
    int count;
    float x, y, z;   
    float linZ;
    float avgZ;
    float tempAvgZ;
    float maxZ;
    float minZ;
    float velocity;
    float velocityLast;    
    int bias;
    int tempbias;
    int biasCount;    
    byte apogee;    
    unsigned long last;   
    float launchMaxZ;  
    byte burnout = 0;
    byte burnoutArm = 0; 
    byte launch = 0;
  };
  adxlStruct adxl;

  // GPS
  struct gpsDataStruct {  // GPS variables
    char vGNRMC[200]; 
    char vGNGGA[200]; 
    char vGNGNS[200]; 
    char gpsDate[15];
    char gpsTime[15];
    char fix[15];
    char quality[15];
    char latRaw[15];
    float latDec;
    float longDec;
    char longRaw[15];
    char longFriendly[15];
    char latFriendly[15];
    char gpsAltitudeMeters[15];
    char gpsAltitudeFeet[15];
    char altitudeAGL[15];
    char gpsSpeed[15];
    char angle[15];
    char sat[15];
    char posMode[15];    
    float baseline;
    float maxAGLfeet;
    char gpsLogLine[300];
    char gpsSendLine[150];
    int gpsStatus;
  };
  gpsDataStruct gpsData;
  bool GPSgood = false;
  
  char aGNRMC[20][20]={0x0}; //for parsing
  char aGNGGA[20][20]={0x0};
  char aGNGNS[20][20]={0x0};
  int fldcnt=0; //for parsing

// *** Radio Setup
  char radioHeader[5];
  char radioMessage[150];
  char radioMessageS[200];
  int newWord = 0;
  char theWord[100];

//added for send queue
  char rSend[15][300];
  byte rSendCount = 0;
  byte rSendLast = 0;
  byte rSendPos = 0;
  char vFullMessageR[300];

  struct debugStruct {
    int adxlCount;
    int baroCount;
    int LSM6DCount;
    int logCount;
    int count1;
    int count2;
    unsigned long debugTimer;
    unsigned long adxlStart;
    unsigned long baroStart;
    unsigned long LSMstart;
    int adxlTime;
    int baroTime;
    int LSMtime;
    int GPScount;
    int GPSquality; //99 timeout, 0 none, # sats
    int gpsTime;
  };
  debugStruct debugVars;

// For threading
  int id_logging, id_sampling;

// For the flash memory
//  #include <LittleFS_NAND.h>
  #include <LittleFS.h>
  #include <stdarg.h>
  uint64_t fTot, totSize1;
  
  #include <TimeLib.h>  // for logging time


// Temperature from CPU
  extern float tempmonGetTemp(void);  

// just for SD
  #include "SD.h"
  #include <SPI.h> 
  File tempFile;

//************************************************************************************************************************************************************  SETUP
//************************************************************************************************************************************************************  SETUP
void setup() {

// Pin setup
  pinMode(radioPower_pin, OUTPUT);  digitalWrite(radioPower_pin, LOW);
  pinMode(radioSet, OUTPUT);  digitalWrite(radioSet, LOW);
  pinMode(GPSpower_pin, OUTPUT);  digitalWrite(GPSpower_pin, LOW);
  pinMode(relayMain, OUTPUT);  digitalWrite(relayMain, LOW);
  pinMode(relayDrogue, OUTPUT);  digitalWrite(relayDrogue, LOW);
  pinMode(relaySustainer, OUTPUT);  digitalWrite(relaySustainer, LOW);
  pinMode(relaySeparation, OUTPUT);  digitalWrite(relaySeparation, LOW);
  pinMode(relayCamera, OUTPUT);  digitalWrite(relayCamera, LOW);
  pinMode(led1_pin, OUTPUT);  digitalWrite(led1_pin, LOW);
  pinMode(led2_pin, OUTPUT);  digitalWrite(led2_pin, LOW);
  pinMode(led3_pin, OUTPUT);  digitalWrite(led3_pin, LOW);
  pinMode(fore_pin, INPUT_PULLUP);
  pinMode(aft_pin, INPUT_PULLUP);
  pinMode(sep_pin, INPUT_PULLUP);
  pinMode(intADXL, INPUT);
  pinMode(int2ADXL, INPUT);  
  pinMode(int1LSM, INPUT  ); //only set to input - now using int2
  pinMode(int2LSM, INPUT_PULLUP);
  pinMode(voltage_pinA, INPUT);
  pinMode(continuityMainPin, INPUT_PULLUP);   
  pinMode(continuityDroguePin, INPUT_PULLUP);   
  pinMode(continuitySustainerPin, INPUT_PULLUP);   
  pinMode(continuitySeparationPin, INPUT_PULLUP);   
  // Grounded pins for RFI:  2,3,4,5,6,9,10,17,20,24,25,26,27,32
  pinMode(2,INPUT);pinMode(3,INPUT);pinMode(4,INPUT);pinMode(5,INPUT);pinMode(6,INPUT);
  pinMode(9,INPUT);pinMode(10,INPUT);pinMode(20,INPUT);pinMode(24,INPUT);
  pinMode(25,INPUT);pinMode(26,INPUT);pinMode(27,INPUT);
  pinMode(13,OUTPUT); digitalWrite(13, LOW);//internal LED

  
  delay(4000);
  Serial.begin(115200);
  delay(100);
  Serial.println("RocketTalk New Board Diagnostics and Health Check v5.1 hardware");
  Serial.println(" ");Serial.println(" ");



  // Initialize GPS Serial
  if(sGPS) {
      Serial1.begin(9600);
  }

  printCommands();

}

//************************************************************************************************************************************************************  MAIN LOOP
//************************************************************************************************************************************************************  MAIN LOOP

void loop() {

  char chIn = 255;
  if ( Serial.available() ) {
    do {
      if ( chIn != '1' && chIn != '2' && chIn != '3' && chIn != '4' && chIn != '5' && chIn != '6' && chIn != '7' && chIn != '8' && chIn != '9' && chIn != 'a' && chIn != 'b' && chIn != 'c' && chIn != 'd' && chIn != 'e' && chIn != 'f' && chIn != 'g' && chIn != 'h' && chIn != 'i' && chIn != 'j' && chIn != 'k' && chIn != 'l' && chIn != 'm' && chIn != 'n' && chIn != 'o' && chIn != 'p' && chIn != 'q' && chIn != 'r' && chIn != 's' )
        chIn = Serial.read();
      else
        Serial.read();
    }
    while ( Serial.available() );
  }



  
  if ( chIn == '1' ) {
     checkSensors();
     printCommands();
  }
  if ( chIn == '2' ) {
     I2Cscan();
     printCommands();
  }
  if ( chIn == '3' ) {
     GPStest();
     printCommands();
  }
  if ( chIn == '4' ) {
     flashLogging();
     printCommands();
  }
  if ( chIn == '5' ) {
     checkVoltage();
     printCommands();
  }
  if ( chIn == '6' ) {
     checkContinuity();
     printCommands();
  }
  if ( chIn == '7' ) {
     checkEvents();
     printCommands();
  }
  if ( chIn == '8' ) {
     checkTemp();
     printCommands();
  }
  if ( chIn == '9' ) {
     radioTest();
     printCommands();
  }


  
  if ( chIn == 'a' ) {
     toggleA();
     printCommands();
  }

  if ( chIn == 'b' ) {
     toggleB();
     printCommands();
  }
  if ( chIn == 'c' ) {
     toggleC();
     printCommands();
  }
  if ( chIn == 'd' ) {
     toggleD();
     printCommands();
  }
  if ( chIn == 'e' ) {
     toggleE();
     printCommands();
  }
  if ( chIn == 'f' ) {
     toggleF();
     printCommands();
  }
  if ( chIn == 'g' ) {
     toggleG();
     printCommands();
  }
  if ( chIn == 'h' ) {
     toggleH();
     printCommands();
  }
  if ( chIn == 'i' ) {
     toggleI();
     printCommands();
  }
  if ( chIn == 'j' ) {
     toggleJ();
     printCommands();
  }
  if ( chIn == 'k' ) {
     calibrateADXL();
     printCommands();
  }
  if ( chIn == 'l' ) {
     calibrateLSMa();
     printCommands();
  }
  if ( chIn == 'm' ) {
     calibrateLSMg();
     printCommands();
  }
  if ( chIn == 'n' ) {
     calibrateBaro();
     printCommands();
  }
  if ( chIn == 'o' ) {
     quaternionDrift();
     printCommands();
  }

  if ( chIn == 'p' ) {
     formatFlash();
     printCommands();
  }
  if ( chIn == 'q' ) {
     eraseFlash();
     printCommands();
  }

  
  if ( chIn == 'r' ) {
     printFlashDirectory();
     printCommands();
  }
  if ( chIn == 's' ) {
     copy2SD();
     printCommands();
  }


  
}


void printCommands() {
  Serial.println("  ");
  Serial.println("----------------------------------------------------------------------------------------------------------------------------");
  Serial.println("COMMANDS:");
  Serial.println("   1-Check Sensors,  2-I2C Scan, 3-Check GPS, 4-Check flash logging, 5-voltage, 6-Continuity, 7-Event pins, 8-Temp, 9-Radio Test ");
  Serial.println("   TOGGLE:  a-LED1, b=LED2, c=LED3, d=pyro MAIN, e=pyro DROGUE, f=pyro SEP, g=pyro SUS, h=Radio Power, i=GPS power, j=Camera");
  Serial.println("   CALIBRATE:  k-ADXL, l-LSM-Accel, m-LSM-Gyro  n-Baro  o-quant drift");  
  Serial.println("   FLASH:  p-Low Format, q-Quick Erase, r-Directory, s-copy to SD");  
  Serial.println("----------------------------------------------------------------------------------------------------------------------------");  
  Serial.println("  ");
}

void I2Cscan() {

 byte error, address;
  int nDevices;
  Serial.println("Scanning I2C Bus...");
  Wire.begin();
  delay(1000);
  
  nDevices = 0;
  for(address = 1; address < 127; address++ )
  {
    // The i2c_scanner uses the return value of
    // the Write.endTransmisstion to see if
    // a device did acknowledge to the address.
    Wire.beginTransmission(address);
    error = Wire.endTransmission();
 
    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
 
      nDevices++;
    }
    else if (error==4)
    {
      Serial.print("Unknown error at address 0x");
      if (address<16)
        Serial.print("0");
      Serial.println(address,HEX);
    }    
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
  Wire.end(); 

  
}

void checkSensors() {
  
  Serial.println("Checking Sensors...");Serial.println(" ");
  Wire.begin();
  
  delay(300);
  Wire.setClock(400000); // 400kHz I2C clock.  (can only be 100k, 400k, 1000k)
  delay(400);
  bool sADXL2 = false;
  bool sMS562 = false;
  bool sLSM2 = false;

    float baroHumidity = 0.0;
    baro.Baseline = 0.0; // baseline altitude to subtract
    float oldAltitude = 0.0;
    unsigned long speedtimer = 0;
    unsigned long baroReadTimer = 0;
    float altFilter = 0.0;
    unsigned long oldTime = 0.0;
    int altFilterCount = 0;
    float avgFilter = 0.0;
    int fSpeed = 0;
    float fSpeedSample[10] = {0.0};
   
    short CONV_D1_MB = 0x48;          // corresponding temp conv. command for OSR
    short CONV_D2_MB = 0x58;          // corresponding pressure conv. command for OSR  
    unsigned long MDP, MDT;
    float MdT, MTEMP, MP;
    int baroReady = 0;
    unsigned long tempTimer = 0;
    float P_val,T_val,H_val;







  

  // ====== MS5607 Barometer setup code and local variables ============
    if(sMS56) {
      Serial.print("MS5607 Barometer:  ");
      if(!P_Sens.begin()){
        Serial.println("Error in Communicating with MS5607");
      }else{
        Serial.println("Initialization successful!");
        sMS562 = true;
      }
    } else {
      Serial.println("MS5607 Barometer Disabled in Config");
    }
  // ====== ADXL375  setup code and local variables ============
    if(sADXL) {
      Serial.print("ADXL Accelerometer:  ");
      Accel.settings.commInterface = I2C_MODE;
      //Accel.settings.I2CAddress = 0x53;
      Accel.settings.I2CAddress = 0x1D;
      if (Accel.beginI2C() == false) { //Begin communication over I2C 
         Serial.println("Error in Communicating with the ADXL");
      } else {
         Serial.println("Initialization successful!");
         sADXL2 = true;
      }
    } else {
      Serial.println("ADXL Accelerometer Disabled in Config");
    }

  // ====== LSM6DS  setup code and local variables ============
    if(sLSM) {
      Serial.print("LSM Accel/Gyro:  ");
      uint8_t theID;
      theID = LSM6.whoAmI();
      if(theID == 108) { // HEX 6C
        Serial.println("Initialization successful!");
        sLSM2 = true;
      } else {
        Serial.println("Error in Communicating with the LSM6DS Accel/Gyro");
      }
    } else {
      Serial.println("ADXL Accelerometer Disabled in Config");
    }

  Serial.println(" ");
  unsigned long theStop;
  int theCounter = 0;

  // ====== MS5607  Performance Test ============
   if(sMS562) {
        P_Sens.setOSR(4096); 
        delay(500);
      // first run
        P_Sens.startConversionMB(CONV_D1_MB); delay(10);
        P_Sens.startMeasurmentMB(); delay(10);
        P_Sens.getDigitalValueMB(MDP); delay(10);
        P_Sens.startConversionMB(CONV_D2_MB); delay(10);
        P_Sens.startMeasurmentMB(); delay(10);
        P_Sens.getDigitalValueMB(MDT);
        //get first baseline
        P_Sens.setValuesMB(MDT, MDP);    
        T_val = P_Sens.getTemperature();
        H_val = P_Sens.getAltitude();  

       theCounter = 0;
       theStop = millis() + 1000;
     while(millis() < theStop) {
        P_Sens.startConversionMB(CONV_D1_MB); delay(10);
        P_Sens.startMeasurmentMB(); delay(10);
        P_Sens.getDigitalValueMB(MDP); delay(10);
        P_Sens.startConversionMB(CONV_D2_MB); delay(10);
        P_Sens.startMeasurmentMB(); delay(10);
        P_Sens.getDigitalValueMB(MDT);
        //get first baseline
        P_Sens.setValuesMB(MDT, MDP);    
        T_val = P_Sens.getTemperature();
        H_val = P_Sens.getAltitude();        
        theCounter++; 
     }
     Serial.print("MS5607 Baro Performance = ");
     Serial.print(theCounter);
     Serial.println("  should be 20");
   }

  // ====== ADXL375  Performance Test ============
   if(sADXL2) {
      //accel stored calibration settings (unique for each accelerometer)
      Accel.writeRegister(ADX375_REG_OFSX, 0x00);
      Accel.writeRegister(ADX375_REG_OFSY, 0x00);
      Accel.writeRegister(ADX375_REG_OFSZ, 0x00);
      //other accelerometer settings
      Accel.writeRegister(ADX375_REG_INT_ENABLE, 0x80); //set interput on
      Accel.writeRegister(ADX375_REG_BW_RATE, 0x0C); //set rate to 400hz (0x0C)
      Accel.writeRegister(ADX375_REG_DATA_FORMAT, 0x0B); //set three bits D0, D1, D3 to on
      Accel.writeRegister(ADX375_REG_FIFO_CTL, 0x00);
      Accel.startup();
      delay(100);
     theCounter = 0;
     theStop = millis() + 1000;
     while(millis() < theStop) {
        if(digitalRead(intADXL) == HIGH) {  //ADXL interput pin high      
        uint8_t buffer[6];
        Accel.readRegisterRegion(buffer, 0x32, 6);        
        theCounter++; 
        }
     }
     Serial.print("ADXL Performance = ");
     Serial.print(theCounter);
     Serial.println("  should be 395");
   }

  // ====== LSM Accel/Gyro  Performance Test ============
   if(sLSM2) {
      
      LSM6.setAccelRateScale(B01100100); // 01100100 :  0110 = 416hz, 01 = 32g, 0 = no LPF, 0 = default 
      delay(10);
      LSM6.setGyroRateScale(B01101100); // 01101100 :  0110 = 416hz, 11 = 2000 dps, 0 = FS, 0 = default 
      //LSM6.setAnyRegister(mLSM6DS_INT1_CTRL,B00000010); //set int1 to new gyro values
      delay(10);
      LSM6.setAnyRegister(mLSM6DS_INT1_CTRL,B00000000); //set int1 to new gyro values
      delay(10);
      LSM6.setAnyRegister(mLSM6DS_INT2_CTRL,B00000001); //set int2 to new gyro values

      
     theCounter = 0;
     theStop = millis() + 1000;
     int16_t accelRead[3]; 
     int16_t gyroRead[3]; 
           
     while(millis() < theStop) {
      if(digitalRead(int2LSM) == HIGH) {  //get an interupt from the gyro at 500hz 
        LSM6.readAccelData(accelRead); 
        LSM6.readGyroData(gyroRead); 
             
        theCounter++; 
        
        }
     }
     Serial.print("LSM Accel/Gyro Performance = ");
     Serial.print(theCounter);
     Serial.println("  should be 400");
   }

  Wire.end();
}

void flashLogging() {

  if(sLogging) {
    if (!myfs.begin()) {
      Serial.println("LittleFS Flash:  *** ERROR starting Little FS ***");
      return;
    } else {
      Serial.println("LittleFS Flash:  Successfully started");
    }
    Serial.println("Process:  Deleting File");
    myfs.remove(fname1);
    Serial.println("Process:  Writing File");
    strcpy(buf,"Testing");
    file = myfs.open(fname1, FILE_WRITE);
    delay(10);
    file.println(buf);
    file.close();   
    Serial.println("Process:  Reading File");
    char buf2[1];
    char buf3[100];
    file = myfs.open(fname1, FILE_READ);
    bool fLetter = false;
    bool fCorrect = false;
    int bufCount = 0;
    while(file.available()) {
     file.read(buf2, 1);
     buf3[bufCount] = buf2[0];
     bufCount++;
    }
    file.close();
    char *ptr = strstr(buf3, "Test");
    if (ptr != NULL) { /* Substring found */
      Serial.println("Process:  Successful File Read");
    } else {
      Serial.println("Process:  File Read Failed");
      return;
    }
    Serial.println("Process:  Conducting performance test...");
    unsigned long theStop;
    int theCounter = 0;
    strcpy(buf,"Line 1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,Line 1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,Line 1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10,1,2,3,4,5,6,7,8,9,10");
    myfs.remove(fname1);
    delay(200);
    theStop = millis() + 1000;
    while(millis() < theStop) {
     file = myfs.open(fname1, FILE_WRITE);
     file.println(buf);
     file.close(); 
     theCounter++;
    }
    Serial.print("Flash Logging Speed Result = ");
    Serial.print(theCounter);
    Serial.println("  should be at least 13 (98 after low level format)");   
    
  } else {
     Serial.println("Logging and Flash is disabled in the configuration");
  }

}


void checkVoltage(){
      // New 5.0 hardware (using 330k ohm / 100k ohm):  8.15 = 580, 7.0v = 497, 6.9v = 490


      Serial.println("Checking Voltage...");Serial.println(" ");
      int value = 0;
      float tempf = 0.0;
      float vin = 0.0;
      float calcVolts = 0.0;
      // get the samples
      for (int i=1; i <= 10; i++){  //ten samples
          value = value + analogRead(voltage_pinA);
       }
      value = value / 10;
     if (value < 490) vin = 0.0f;
      if (value > 580) vin = 100.0f;
      
      
      if (value <= 580 && value >= 490) {
        tempf = (float)value - (float)490.0;
        vin = (float) tempf * (float) 1.1111;  // 90 is spread. 100/90 = 1.1111
        if (vin > 100) vin = 100;
        if (vin < 0) vin = 0;
      }
      working.voltage = vin;
      tempf = (float)value - (float)490.0;
      calcVolts = (float) 6.9 + ((float) tempf * (float) .0138888);
      if(calcVolts <= 6.9) calcVolts = 0;
      

      Serial.print("Voltage Raw Value:  ");Serial.println(value);
      Serial.print("Voltage Calc Volts:  ");Serial.println(calcVolts,2);
      Serial.print("Voltage Calc Battery %:  ");Serial.println(vin,0);
      
}

void checkContinuity(){


      Serial.println("Checking Continuity...");Serial.println(" ");

      // Note, requires battery voltage to work!

      int value = 0;
      // get the samples
      for (int i=1; i <= 10; i++){  //ten samples
          value = value + analogRead(voltage_pinA);
       }
      value = value / 10;
      if (value < 400) { //zzz set to correct zero point
        Serial.println("ABORT - No Battery Voltage, so Continuity check inaccurate");
      } else {
       
      Serial.print("MAIN Pyro Continuity = ");
      if(digitalRead(continuityMainPin) == LOW) {
        Serial.println("Closed");
      } else {
        Serial.println("Open");
      }

      Serial.print("DROGUE Pyro Continuity = ");
      if(digitalRead(continuityDroguePin) == LOW) {
        Serial.println("Closed");
      } else {
        Serial.println("Open");
      }

       Serial.print("SEPARATION Pyro Continuity = ");
      if(digitalRead(continuitySeparationPin) == LOW) {
        Serial.println("Closed");
      } else {
        Serial.println("Open");
      }     

      Serial.print("SUSTAINER Pyro Continuity = ");
      if(digitalRead(continuitySustainerPin) == LOW) {
        Serial.println("Closed");
      } else {
        Serial.println("Open");
      }
   }
}


void checkEvents(){


      Serial.println("Checking Event Connections...");Serial.println(" ");
 
      Serial.print("FORWARD Connection = ");
      if(digitalRead(fore_pin) == LOW) {
        Serial.println("Closed");
      } else {
        Serial.println("Open");
      }

      Serial.print("AFT Connection = ");
      if(digitalRead(aft_pin) == LOW) {
        Serial.println("Closed");
      } else {
        Serial.println("Open");
      }

       Serial.print("SEPARATION Connection = ");
      if(digitalRead(sep_pin) == LOW) {
        Serial.println("Closed");
      } else {
        Serial.println("Open");
      }     
   
}


void checkTemp(){


      Serial.println("Checking CPU Temperature...");Serial.println(" ");
 
      Serial.print("CPU Temp (f) = ");
      working.CPUtemp  = (tempmonGetTemp() * 9.0f / 5.0f) + 32.0f;
      Serial.println(working.CPUtemp);
      
}




void toggleA() {
  Serial.print("LED1 Toggle is now ");
  if(tLED1) {
    tLED1 = false;
    Serial.println("OFF");
    digitalWrite(led1_pin, LOW);
  } else {
    tLED1 = true;
    Serial.println("ON");
    digitalWrite(led1_pin, HIGH);
  }
 
}


void toggleB() {
  Serial.print("LED2 Toggle is now ");
  if(tLED2) {
    tLED2 = false;
    Serial.println("OFF");
    digitalWrite(led2_pin, LOW);
  } else {
    tLED2 = true;
    Serial.println("ON");
    digitalWrite(led2_pin, HIGH);
  }
 
}


void toggleC() {
  Serial.print("LED3 Toggle is now ");
  if(tLED3) {
    tLED3 = false;
    Serial.println("OFF");
    digitalWrite(led3_pin, LOW);
  } else {
    tLED3 = true;
    Serial.println("ON");
    digitalWrite(led3_pin, HIGH);
  }
 
}

void toggleD() {
  Serial.print("MAIN Toggle is now ");
  if(tMain) {
    tMain = false;
    Serial.println("OFF");
    digitalWrite(relayMain, LOW);
  } else {
    tMain = true;
    Serial.println("ON");
    digitalWrite(relayMain, HIGH);
  }
}

void toggleE() {
  Serial.print("DROGUE Toggle is now ");
  if(tDrogue) {
    tDrogue = false;
    Serial.println("OFF");
    digitalWrite(relayDrogue, LOW);
  } else {
    tDrogue = true;
    Serial.println("ON");
    digitalWrite(relayDrogue, HIGH);
  }
}

void toggleF() {
  Serial.print("SEP Toggle is now ");
  if(tSep) {
    tSep = false;
    Serial.println("OFF");
    digitalWrite(relaySeparation, LOW);
  } else {
    tSep = true;
    Serial.println("ON");
    digitalWrite(relaySeparation, HIGH);
  }
}

void toggleG() {
  Serial.print("SUSTAINER Toggle is now ");
  if(tSus) {
    tSus = false;
    Serial.println("OFF");
    digitalWrite(relaySustainer, LOW);
  } else {
    tSus = true;
    Serial.println("ON");
    digitalWrite(relaySustainer, HIGH);
  }
}

void toggleH() {
  Serial.print("RADIO POWER Toggle is now ");
  if(tRadio) {
    tRadio = false;
    Serial.println("OFF");
    digitalWrite(radioPower_pin, LOW);
  } else {
    tRadio = true;
    Serial.println("ON");
    digitalWrite(radioPower_pin, HIGH);
  }
}

void toggleI() {
  Serial.print("GPS POWER Toggle is now ");
  if(tGPS) {
    tGPS = false;
    Serial.println("OFF");
    digitalWrite(GPSpower_pin, LOW);
  } else {
    tGPS = true;
    Serial.println("ON");
    digitalWrite(GPSpower_pin, HIGH);
  }
}

void toggleJ() {
  Serial.print("CAMERA Toggle is now ");
  if(tCam) {
    tCam = false;
    Serial.println("OFF");
    digitalWrite(relayCamera, LOW);
  } else {
    tCam = true;
    Serial.println("ON");
    digitalWrite(relayCamera, HIGH);
  }
}


void formatFlash() {

  if(sLogging) {
    if (!myfs.begin()) {
      Serial.println("LittleFS Flash:  *** ERROR starting Little FS ***");
      return;
    } else {
      Serial.println("LittleFS Flash:  Successfully started");
    }
    Serial.println(" ");
    Serial.println("Initiated a low-level format of flash. This will takes about 16.5 minutes...");
    myfs.lowLevelFormat('.');
    Serial.println("Done formating flash memory");

    
  } else {
     Serial.println("Logging and Flash is disabled in the configuration");
  }


  
}

void GPStest() {     //*********************************  Check the GPS code for UBLOX M10S ********************************************

  if(sGPS) {
    Serial.println("Checking GPS Status...");Serial.println(" ");
    //turn on GPS power
    tGPS = true;
    digitalWrite(GPSpower_pin, HIGH);
    delay(1500);
    GPSbasic();
    if(GPSgood) {
      Serial.println("GPS Basic Communication Successful");
    } else {
      Serial.println("GPS Timeout ERROR!");
      return;
    }
    Serial.println("Configuring GPS...");
    configGPS10();
    Serial.println(" ");
    delay(500);
    CheckGPS10();
    if(events.clockSet == 1) {
     Serial.print("GPS time is active and set to:  ");
     Serial.print(gpsData.gpsDate);
     Serial.print("  ");
     Serial.println(gpsData.gpsTime);
    } else {
      Serial.println("GPS time is not set yet");
    }
    if(gpsData.gpsStatus == 2) {
     Serial.println("GPS has active satellite fix");
     Serial.print("Number of Satellites:  ");
     Serial.println(gpsData.sat);
     Serial.print("Longitude:  ");
     Serial.println(gpsData.longFriendly);
     Serial.print("Lattitude:  ");
     Serial.println(gpsData.latFriendly);
     Serial.print("Raw Altitude(M):  ");
     Serial.println(gpsData.gpsAltitudeMeters);

    } else {
      Serial.println("GPS does not have a satellite fix");
    }



    

    


    
  } else {
    Serial.println("GPS is disabled in the configuration");
  }
  
}

void GPSbasic() {

 // very basic test to check for RMC at startup under 9600 baud 

  unsigned long startTime = millis();
  startTime = startTime + 1500; // 1500ms timeout for reading GPS RMC
  int vRMC = false;
  int vGNS = false;
  int vDone = false;
  strcpy(gpsData.vGNRMC, "");
  strcpy(gpsData.vGNGNS, ""); 
  int UTCoffset = -7;
  char ReadString[100] = "";
  int vCheck = false;

  // *** Get new serial data from the GPS ***
  while(Serial1.available()){  //flush the serial buffer so you don't get an old reading
    Serial1.read();} 
  while (vCheck == false) {
   int size = Serial1.readBytesUntil(13,ReadString,99);
   char *ptr = strstr(ReadString, "$GNRMC");
   if (ptr != NULL) /* Substring found */
     {strcpy(gpsData.vGNRMC, ReadString);
      GPSgood = true;
      return;
     }
    strcpy(ReadString, "");
    if (millis() > startTime) { // timeout
      gpsData.gpsStatus = 0;
      GPSgood = false;
      return;
    }
   }
}

void CheckGPS10() {    //*********************************  Check the GPS code for UBLOX M10S ********************************************


  unsigned long startTime = millis();
  startTime = startTime + 500; // 500ms timeout for reading GPS

  int vRMC = false;
  int vGNS = false;
  int vDone = false;
  strcpy(gpsData.vGNRMC, "");
  strcpy(gpsData.vGNGNS, ""); 
  int UTCoffset = -7;
  char ReadString[100] = "";
  int vCheck = false;


  // *** Get new serial data from the GPS ***
  while(Serial1.available()){  //flush the serial buffer so you don't get an old reading
    Serial1.read();} 
  while (vCheck == false) {
   int size = Serial1.readBytesUntil(13,ReadString,99);
   //if(serialDebug) Serial.println(ReadString);
   char *ptr = strstr(ReadString, "$GNRMC");
   if (ptr != NULL) /* Substring found */
     {strcpy(gpsData.vGNRMC, ReadString);
      vRMC=true; 
     }
   char *ptr2 = strstr(ReadString, "$GNGNS");
   if (ptr2 != NULL) /* Substring found */
     {strcpy(gpsData.vGNGNS, ReadString);
      vGNS=true; 
     }    
    if (vGNS && vRMC) vCheck = true;  //success - got both sentences from GPS now exit
    strcpy(ReadString, "");
    if (millis() > startTime) { // timeout
      gpsData.gpsStatus = 0;
      Serial.println("GPS timeout");
     
      return;
    }
   }


   
  // *** success, got new data from GPS without timeout ***


   char tempGNRMC[200];
   char tempGNGNS[200];
   strcpy(tempGNRMC,gpsData.vGNRMC);
   strcpy(tempGNGNS, gpsData.vGNGNS);
   parseit(tempGNRMC,aGNRMC);
   parseit(tempGNGNS,aGNGNS); 
   //Serial.println(gpsData.vGNRMC);
   //Serial.println(gpsData.vGNGNS);

   // check for time to be set
   if(strlen(aGNRMC[9]) > 0 && events.clockSet == 0) {
        setTheTime();
        events.clockSet = 1; 
   }
   
    if(strcmp(aGNRMC[2],"A") != 0) {  // no fix - abort
      gpsData.gpsStatus = 1;
      return;
     }

  // Success - Good GPS data and has fix, so proceed to load data

   // *******************  Populate the GPS Data from NMEA data  ************************

    

    fldcnt = 0;
    char target[20]= "";
    const char *slash = "/"; const char *dash = "-"; const char *colon = ":"; const char *comma = ","; const char *space = " "; 

    //GPS - date-time load
    populateTime(); 
    strcpy(gpsData.fix, aGNRMC[2]); //fix
    strcpy(gpsData.quality, aGNGNS[6]); //quality
    strcpy(gpsData.latRaw, aGNGNS[2]); //Latitude
    strcat(gpsData.latRaw, aGNGNS[3]); //Latitude Dir
    strcpy(gpsData.longRaw, aGNGNS[4]); //Longitude
    strcat(gpsData.longRaw, aGNGNS[5]); //Longitude Dir
    gpsData.latDec = GpsToDecimalDegrees(aGNGNS[2], aGNGNS[3]);  //Lat Decimal
    gpsData.longDec = GpsToDecimalDegrees(aGNGNS[4], aGNGNS[5]); //Long Decimal
    char str_float[20];
    dtostrf(gpsData.latDec, 8, 6, str_float);
    remove_spaces(str_float, gpsData.latFriendly); //Lat Dec String
    dtostrf(gpsData.longDec, 8, 6, str_float);
    remove_spaces(str_float, gpsData.longFriendly); //Long Dec String   
    strcpy(gpsData.gpsAltitudeMeters, aGNGNS[9]); //Altitude Meters
    //calculate Feet
    char tempNice[100] = "";  
    strcpy(tempNice, ""); 
    strcpy(tempNice, aGNGNS[9]);
    float dTemp = 0.0;
    dTemp = atof(tempNice);
    dTemp = dTemp * 3.28084; //convert to feet
    dtostrf(dTemp, 8, 0, str_float);
    char sResult[12];
    remove_spaces(str_float, sResult);
    strcpy(gpsData.gpsAltitudeFeet, sResult); //Altitude FeeT
    strcpy(gpsData.posMode,aGNGNS[6]);
    strcpy(gpsData.gpsSpeed,"NA");
    strcpy(gpsData.angle,"NA");
    strcpy(gpsData.sat,aGNGNS[7]);

    //max alt AGL
    if(gpsData.baseline == 0.0) gpsData.baseline = dTemp;
    if((dTemp-gpsData.baseline) > gpsData.maxAGLfeet) gpsData.maxAGLfeet = (dTemp - gpsData.baseline);  // record max AGL    
    //get AGL string
    dTemp = dTemp - gpsData.baseline;
    if(dTemp < 15 && dTemp > -15) dTemp = 0.0; //zero out noise if in the ballpark of 0 AGL
    strcpy(str_float, ""); 
    dtostrf(dTemp, 8, 0, str_float);
    remove_spaces(str_float, sResult);   
    strcpy(gpsData.altitudeAGL, sResult);   
    
    gpsData.gpsStatus = 2;
  
}


void configGPS10() {

    delay(100); // Little delay before flushing
    Serial1.flush();
    delay(100);
    // *** Change Baud Rate to 115200 ****
    byte packet[] = {0xB5, 0x62, 0x06, 0x8A, 0x0C, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x52, 0x40, 0x00, 0xC2, 0x01, 0x00, 0xF4, 0xB1,}; sendPacket(packet, sizeof(packet)); //115200 baud
    //byte packet[] = {0xB5,0x62,0x06,0x8A,0x0C,0x00,0x01,0x01,0x00,0x00,0x01,0x00,0x52,0x40,0x00,0x84,0x03,0x00,0xB8,0xFB,}; sendPacket(packet, sizeof(packet)); //230400 baud
    delay(100); // Little delay before flushing
    Serial1.flush();
    Serial1.begin(115200);
    delay(100);
    // *** Change GNSS Subscriptions ****
    byte packet13[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0x22,0x00,0x31,0x10,0x00,0xFE,0x97,}; sendPacket(packet13, sizeof(packet13));delay(10); //remove BeiDou
    byte packet14[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0x25,0x00,0x31,0x10,0x01,0x02,0xA7,}; sendPacket(packet14, sizeof(packet14));delay(10); //add GLONAS
    delay(100);
    // *** Change outpaut rate ****
    byte packet6[] = {0xB5,0x62,0x06,0x8A,0x0A,0x00,0x01,0x01,0x00,0x00,0x01,0x00,0x21,0x30,0x32,0x00,0x20,0x5F,}; sendPacket(packet6, sizeof(packet6));// 20 hz / 50 ms
    //byte packet6[] = {0xB5,0x62,0x06,0x8A,0x0A,0x00,0x01,0x01,0x00,0x00,0x01,0x00,0x21,0x30,0x19,0x00,0x07,0x2D,}; sendPacket(packet6, sizeof(packet6));// 40 hz / 25 ms
    delay(100);
    // *** Filter and set NMEA codes out ****
    byte packet0[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0xCA,0x00,0x91,0x20,0x00,0x16,0x1F,}; sendPacket(packet0, sizeof(packet0));delay(10); //remove GLL
    byte packet2[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0xC0,0x00,0x91,0x20,0x00,0x0C,0xED,}; sendPacket(packet2, sizeof(packet2));delay(10); //remove GSA
    byte packet3[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0xC5,0x00,0x91,0x20,0x00,0x11,0x06,}; sendPacket(packet3, sizeof(packet3));delay(10); //remove GSV
    byte packet4[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0xB1,0x00,0x91,0x20,0x00,0xFD,0xA2,}; sendPacket(packet4, sizeof(packet4));delay(10); //remove VTG
    byte packet11[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0xBB,0x00,0x91,0x20,0x00,0x07,0xD4,}; sendPacket(packet11, sizeof(packet11));delay(10); //remove GGA
    byte packet12[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0xB6,0x00,0x91,0x20,0x01,0x03,0xBC,}; sendPacket(packet12, sizeof(packet12));delay(10); //add in GNS
    delay(100);
    // *** Change Dynamic Model to 4G ****    
    byte packet15[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0x21,0x00,0x11,0x20,0x08,0xF5,0x5A,}; sendPacket(packet15, sizeof(packet15));delay(10); //Change to Air 4G  
}

void sendPacket(byte *packet, byte len)
{
    for (byte i = 0; i < len; i++)
     {  Serial1.write(packet[i]);  }

}

void printPacket(byte *packet, byte len)
{   char temp[3];
    for (byte i = 0; i < len; i++)
    {   sprintf(temp, "%.2X", packet[i]);  
        Serial.print(temp);
        if (i != len - 1)
        {  Serial.print(' ');  }
    }
    Serial.println();
}

void setTheTime() {  // set the time using the GPS - note the UTC offset



    char target[20]="";
    int vHour;
    int vMinute;
    int vSecond;
    int vYear;
    int vMonth;
    int vDay;
    
    populateTime();
    
    strncpy (target, aGNRMC[9]+2, 2 ); //month
    vMonth = atoi(target);strcpy(target, "");
    strncpy (target, aGNRMC[9], 2 ); //day
    vDay = atoi(target);strcpy(target, "");
    strncpy (target, aGNRMC[9]+4, 2 ); //year
    vYear = atoi(target);strcpy(target, "");  
    strncpy (target, aGNRMC[1], 2 ); //hour
    vHour = atoi(target);strcpy(target, "");           
    strncpy (target, aGNRMC[1]+2, 2 ); //minute
    vMinute = atoi(target);strcpy(target, "");   
    strncpy (target, aGNRMC[1]+4, 2 ); //second
    vSecond = atoi(target);strcpy(target, "");   
    // pause (hack) to get millis aligned to zero for millis logging time
    int n = 999;
    char buf[16];
    while(n != 0) {
      ltoa(millis(),buf,10);
      char daMilli[5] = {buf[strlen(buf)-3],buf[strlen(buf)-2],buf[strlen(buf)-1],'\0'};
      n = atoi(daMilli);
    }
    setTime(vHour, vMinute, vSecond, vDay,vMonth,vYear);
    adjustTime(-8 * 3600);   

    
}

float GpsToDecimalDegrees(char nmeaPos[20], char quadrant[5])
{
  float v= 0;
  if(strlen(nmeaPos)>5)
  {
    char integerPart[3+1];
    int digitCount= (nmeaPos[4]=='.' ? 2 : 3);
    memcpy(integerPart, nmeaPos, digitCount);
    integerPart[digitCount]= 0;
    nmeaPos+= digitCount;
    v= atoi(integerPart) + atof(nmeaPos)/60.;
    if(quadrant[0]=='W' || quadrant[0]=='S')
      v= -v;
  }
  return v;
}

void remove_spaces(const char *input, char *result)
{
  int i, j = 0;
  for (i = 0; input[i] != '\0'; i++) {
    if (!isspace((unsigned char) input[i])) {
      result[j++] = input[i];
    }
  }
  result[j] = '\0';
}

void parseit(char *record, char arr[20][20]) {

  const byte segmentSize = 19 ;  // Segments longer than this will be skipped
  char scratchPad[segmentSize + 1]; // A buffer to pull the progmem bytes into for printing/processing
  int i = 0;
  // declare three pointers to the progmem string
  char * nextSegmentPtr = record; // points to first character of segment
  char * delimiterPtr = record;   // points to detected delimiter character, or eos NULL
  char * endOfData = record + strlen(record); // points to last character in string
  byte len; // number of characters in current segment
  while (1) {
    delimiterPtr = strchr_P(nextSegmentPtr, ','); // Locate target character in progmem string.
    len = delimiterPtr - nextSegmentPtr;
    if (delimiterPtr == nullptr) { // Hit end of string
      len = endOfData - nextSegmentPtr;
    }
    if (len <= segmentSize) {
      memcpy_P(scratchPad, nextSegmentPtr, len);
      scratchPad[len] = '\0'; // Append terminator to extracted characters.
      strcpy(arr[i],scratchPad); 
    }
    else {
      strcpy(arr[i],"overflow");       
    }
    if (delimiterPtr == nullptr) { // ----- Exit while loop here -----
      break;
    }
    i++;
    nextSegmentPtr = nextSegmentPtr + len + 1;
  } // end while  
}

void populateTime() {  // load GPS NMEA into structure 

    fldcnt = 0;
    char target[20]= "";
    const char *slash = "/"; const char *dash = "-"; const char *colon = ":"; const char *comma = ","; const char *space = " "; 

    //GPS - date
    strcpy(gpsData.gpsDate, "");
    strncpy(target, aGNRMC[9]+2, 2 ); //month
    strcat(gpsData.gpsDate, target);
    strcat(gpsData.gpsDate, slash);
    strncpy (target, aGNRMC[9], 2 ); //day
    strcat(gpsData.gpsDate, target);
    strcat(gpsData.gpsDate, slash);    
    strcat(gpsData.gpsDate, "20");      
    strncpy (target, aGNRMC[9]+4, 2 ); //year
    strcat(gpsData.gpsDate, target);
    //GPS - time
    strcpy(gpsData.gpsTime, "");
    strncpy(target, aGNRMC[1], 2 ); //hour
    strcpy(gpsData.gpsTime, target);
    strcat(gpsData.gpsTime, colon);
    strncpy(target, aGNRMC[1]+2, 2 ); //minute
    strcat(gpsData.gpsTime, target);
    strcat(gpsData.gpsTime, colon);         
    strncpy (target, aGNRMC[1]+4, 2 ); //second
    strcat(gpsData.gpsTime, target);    
    //
}

//****************************************************************************************  CALIBRATE FUNCTIONS *******************************************************************



void calibrateADXL() {


  
  Serial.println("Calibrate ADXL -- Make sure all offsets are set to zero");Serial.println(" ");
  Wire.begin();
  
  delay(300);
  Wire.setClock(400000); // 400kHz I2C clock.  (can only be 100k, 400k, 1000k)
  delay(400);
  bool sADXL2 = false;
  bool sMS562 = false;
  bool sLSM2 = false;

    float baroHumidity = 0.0;
    baro.Baseline = 0.0; // baseline altitude to subtract
    float oldAltitude = 0.0;
    unsigned long speedtimer = 0;
    unsigned long baroReadTimer = 0;
    float altFilter = 0.0;
    unsigned long oldTime = 0.0;
    int altFilterCount = 0;
    float avgFilter = 0.0;
    int fSpeed = 0;
    float fSpeedSample[10] = {0.0};
   
    short CONV_D1_MB = 0x48;          // corresponding temp conv. command for OSR
    short CONV_D2_MB = 0x58;          // corresponding pressure conv. command for OSR  
    unsigned long MDP, MDT;
    float MdT, MTEMP, MP;
    int baroReady = 0;
    unsigned long tempTimer = 0;
    float P_val,T_val,H_val;


  // ====== ADXL375  setup code and local variables ============
    if(sADXL) {
      Serial.print("ADXL Accelerometer:  ");
      Accel.settings.commInterface = I2C_MODE;
      //Accel.settings.I2CAddress = 0x53;
      Accel.settings.I2CAddress = 0x1D;
      if (Accel.beginI2C() == false) { //Begin communication over I2C 
         Serial.println("Error in Communicating with the ADXL");
      } else {
         Serial.println("Initialization successful!");
         sADXL2 = true;
      }
    } else {
      Serial.println("ADXL Accelerometer Disabled in Config");
    }



  // ====== ADXL375  Calibration Test ============
   if(sADXL2) {
      //accel stored calibration settings (unique for each accelerometer)
      Accel.writeRegister(ADX375_REG_OFSX, 0x00);
      Accel.writeRegister(ADX375_REG_OFSY, 0x00);
      Accel.writeRegister(ADX375_REG_OFSZ, 0x00);
      //other accelerometer settings
      Accel.writeRegister(ADX375_REG_INT_ENABLE, 0x80); //set interput on
      Accel.writeRegister(ADX375_REG_BW_RATE, 0x0C); //set rate to 400hz (0x0C)
      Accel.writeRegister(ADX375_REG_DATA_FORMAT, 0x0B); //set three bits D0, D1, D3 to on
      Accel.writeRegister(ADX375_REG_FIFO_CTL, 0x00);
      Accel.startup();
      delay(100);


  float cXm = 0.0;
  float cXp = 0.0;
  float cYm = 0.0;
  float cYp = 0.0;
  float cZm = 0.0;
  float cZp = 0.0;
  float cX = 0.0;
  float cY = 0.0;
  float cZ = 0.0;
  

  Serial.println("Cycle through six orientations at 20 sec each to calibrate");Serial.println(" ");
  Serial.print("X-  Launch Position.  Hit t key when stable.");
  keyWait();
  Serial.println("    testing...");
  ADXLsample();
  cXm = tempX;
  Serial.print("     Result: "); Serial.println(tempX,2);
  Serial.print("X+  Nose down position.  Hit t key when stable.");
  keyWait();
  Serial.println("    testing...");
  ADXLsample();
  cXp = tempX;
  Serial.print("     Result: "); Serial.println(tempX,2);  
  Serial.print("     X Calibration = ");
  cX = ((float) 1000.0 + cXm) + ((float) -1000.0 + cXp);
  cX = cX / (float) 2.0;
  cX = cX * (float) -1.0;
  Serial.println(cX,2);

  Serial.print("Y-  Terminals Up.  Hit t key when stable.");
  keyWait();
  Serial.println("    testing...");
  ADXLsample();
  cYm = tempY;
  Serial.print("     Result: "); Serial.println(tempY,2);
  Serial.print("Y+  Terminals Down.  Hit t key when stable.");
  keyWait();
  Serial.println("    testing...");
  ADXLsample();
  cYp = tempY;
  Serial.print("     Result: "); Serial.println(tempY,2);  
  Serial.print("     Y Calibration = ");
  cY = ((float) 1000.0 + cYm) + ((float) -1000.0 + cYp);
  cY = cY / (float) 2.0;
  cY = cY * (float) -1.0;
  Serial.println(cY,2);


  Serial.print("Z-  Board facing down.  Hit t key when stable.");
  keyWait();
  Serial.println("    testing...");
  ADXLsample();
  cZm = tempZ;
  Serial.print("     Result: "); Serial.println(tempZ,2);
  Serial.print("Z+  Board Up.  Hit t key when stable.");
  keyWait();
  Serial.println("    testing...");
  ADXLsample();
  cZp = tempZ;
  Serial.print("     Result: "); Serial.println(tempZ,2);  
  Serial.print("     Z Calibration = ");
  cZ = ((float) 1000.0 + cZm) + ((float) -1000.0 + cZp);
  cZ = cZ / (float) 2.0;
  cZ = cZ * (float) -1.0;
  Serial.println(cZ,2);

   }


  Wire.end();
}

void keyWait() {

  char chIn = 255;
  int theTest = 0;
  while(theTest == 0) {
  if ( Serial.available() ) {
    do {
      if ( chIn != 't')
        chIn = Serial.read();
      else
        Serial.read();
    }
    while ( Serial.available() );
  }

  if ( chIn == 't' ) theTest = 1;
  }

  
}


void ADXLsample() {

  // Change these to zero to start a new ADXL
  float ADXLxOffset = -362.26;//-392.5;
  float ADXLyOffset = -497.97;//-495;
  float ADXLzOffset = -59.12;//-40.5;

  unsigned long theStop;
  int theCounter = 0;
  int16_t x, y, z;


     theStop = millis() + 20000;
     tempX = 0;
     tempY=0;
     tempZ=0;
     theCounter=0;
     
     while(millis() < theStop) {
        
          if(digitalRead(intADXL) == HIGH) {  //ADXL interput pin high      
          uint8_t buffer[6];
          Accel.readRegisterRegion(buffer, 0x32, 6);        
          x = (int16_t) (((int16_t)buffer[1] << 8) | buffer[0]);
          y = (int16_t) (((int16_t)buffer[3] << 8) | buffer[2]);
          z = (int16_t) (((int16_t)buffer[5] << 8) | buffer[4]);
          adxl.x = (float)x * (float)49.3;
          adxl.y = (float)y * (float)49.3;
          adxl.z = (float)z * (float)49.3;
          adxl.x += ADXLxOffset;
          adxl.y += ADXLyOffset;
          adxl.z += ADXLzOffset;            
          theCounter++;
          tempX += adxl.x;
          tempY += adxl.y;
          tempZ += adxl.z; 
          

        }
          //done with one second sample
     }
          tempX = (float) tempX / (float) theCounter;
          tempY = (float) tempY / (float) theCounter;
          tempZ = (float) tempZ / (float) theCounter;
          //Serial.print("AVG ADXL Values:    X ="); Serial.print(tempX);
          //Serial.print("    Y ="); Serial.print(tempY);
          //Serial.print("    Z ="); Serial.println(tempZ);


  
}

void calibrateLSMa() {
  


  
  Serial.println("Calibrate LSM6DS Accelerometer -- Make sure all offsets are set to zero");Serial.println(" ");
  Wire.begin();
  
  delay(300);
  Wire.setClock(400000); // 400kHz I2C clock.  (can only be 100k, 400k, 1000k)
  delay(400);
  bool sADXL2 = false;
  bool sMS562 = false;
  bool sLSM2 = false;
          
      
    float baroHumidity = 0.0;
    baro.Baseline = 0.0; // baseline altitude to subtract
    float oldAltitude = 0.0;
    unsigned long speedtimer = 0;
    unsigned long baroReadTimer = 0;
    float altFilter = 0.0;
    unsigned long oldTime = 0.0;
    int altFilterCount = 0;
    float avgFilter = 0.0;
    int fSpeed = 0;
    float fSpeedSample[10] = {0.0};
   
    short CONV_D1_MB = 0x48;          // corresponding temp conv. command for OSR
    short CONV_D2_MB = 0x58;          // corresponding pressure conv. command for OSR  
    unsigned long MDP, MDT;
    float MdT, MTEMP, MP;
    int baroReady = 0;
    unsigned long tempTimer = 0;
    float P_val,T_val,H_val;

  // ====== LSM6DS  setup code and local variables ============
    if(sLSM) {
      Serial.print("LSM Accel/Gyro:  ");
      uint8_t theID;
      theID = LSM6.whoAmI();
      if(theID == 108) { // HEX 6C
        Serial.println("Initialization successful!");
        sLSM2 = true;
      } else {
        Serial.println("Error in Communicating with the LSM6DS Accel/Gyro");
      }
    } else {
      Serial.println("LSM6D Accelerometer Disabled in Config");
    }

  Serial.println(" ");



  // ====== LSM Accel/Gyro  Performance Test ============

      
      LSM6.setAccelRateScale(B01100100); // 01100100 :  0110 = 416hz, 01 = 32g, 0 = no LPF, 0 = default 
      delay(10);
      LSM6.setGyroRateScale(B01101100); // 01101100 :  0110 = 416hz, 11 = 2000 dps, 0 = FS, 0 = default 
      //LSM6.setAnyRegister(mLSM6DS_INT1_CTRL,B00000010); //set int1 to new gyro values
      delay(10);
      LSM6.setAnyRegister(mLSM6DS_INT1_CTRL,B00000000); //set int1 to new gyro values
      delay(10);
      LSM6.setAnyRegister(mLSM6DS_INT2_CTRL,B00000001); //set int2 to new gyro values


  float cXm = 0.0;
  float cXp = 0.0;
  float cYm = 0.0;
  float cYp = 0.0;
  float cZm = 0.0;
  float cZp = 0.0;
  float cX = 0.0;
  float cY = 0.0;
  float cZ = 0.0;
  

  Serial.println("Cycle through six orientations at 20 sec each to calibrate");Serial.println(" ");
  Serial.print("X-  Nose Down Position.  Hit t key when stable.");
  keyWait();
  Serial.println("    testing...");
  LSMaSample();
  cXm = tempX;
  Serial.print("     Result: "); Serial.println(tempX,2);
  Serial.print("X+  Launch Position.  Hit t key when stable.");
  keyWait();
  Serial.println("    testing...");
  LSMaSample();
  cXp = tempX;
  Serial.print("     Result: "); Serial.println(tempX,2);  
  Serial.print("     X Calibration = ");
  cX = ((float) 1000.0 + cXm) + ((float) -1000.0 + cXp);
  cX = cX / (float) 2.0;
  cX = cX * (float) -1.0;
  Serial.println(cX,2);

  Serial.print("Y-  Terminals Down.  Hit t key when stable.");
  keyWait();
  Serial.println("    testing...");
  LSMaSample();
  cYm = tempY;
  Serial.print("     Result: "); Serial.println(tempY,2);
  Serial.print("Y+  Terminals Up.  Hit t key when stable.");
  keyWait();
  Serial.println("    testing...");
  LSMaSample();
  cYp = tempY;
  Serial.print("     Result: "); Serial.println(tempY,2);  
  Serial.print("     Y Calibration = ");
  cY = ((float) 1000.0 + cYm) + ((float) -1000.0 + cYp);
  cY = cY / (float) 2.0;
  cY = cY * (float) -1.0;
  Serial.println(cY,2);


  Serial.print("Z-  Board facing down.  Hit t key when stable.");
  keyWait();
  Serial.println("    testing...");
  LSMaSample();
  cZm = tempZ;
  Serial.print("     Result: "); Serial.println(tempZ,2);
  Serial.print("Z+  Board Up.  Hit t key when stable.");
  keyWait();
  Serial.println("    testing...");
  LSMaSample();
  cZp = tempZ;
  Serial.print("     Result: "); Serial.println(tempZ,2);  
  Serial.print("     Z Calibration = ");
  cZ = ((float) 1000.0 + cZm) + ((float) -1000.0 + cZp);
  cZ = cZ / (float) 2.0;
  cZ = cZ * (float) -1.0;
  Serial.println(cZ,2);


  Wire.end();
}

void LSMaSample() {

  // Change these to zero to start a new LSM6D
      float AxOffset = -2.95;//-5;
      float AyOffset = -16.55;//4.5;
      float AzOffset = -11.85;//-22;

      float Ax, Ay, Az;  
  unsigned long theStop;
  int theCounter = 0;
     theCounter = 0;
     theStop = millis() + 1000;
     int16_t accelRead[3]; 
     int16_t gyroRead[3]; 

        tempX = 0;
        tempY=0;
        tempZ=0;
        theCounter=0;
         
     

     theStop = millis() + 20000;
     while(millis() < theStop) {


         if(digitalRead(int2LSM) == HIGH) {  //get an interupt from the gyro at 500hz 
          LSM6.readAccelData(accelRead); 

            Ax = accelRead[0] + AxOffset;
            Ay = accelRead[1] + AyOffset;
            Az = accelRead[2] + AzOffset;
             
            // Now we'll calculate the accleration value into actual g's
            Ax = Ax * (float) 0.976;  // .976 for 32G, .488 16G, .244 8G, .122 4G, .061 2G (datasheet pg 25)
            Ay = Ay * (float) 0.976;
            Az = Az * (float) 0.976;
            LSM6D.count++;
            LSM6D.x = Ax;
            LSM6D.y = Ay;
            LSM6D.z = Az;

          theCounter++;
          tempX += LSM6D.x;
          tempY += LSM6D.y;
          tempZ += LSM6D.z; 
          }

     }

          tempX = (float) tempX / (float) theCounter;
          tempY = (float) tempY / (float) theCounter;
          tempZ = (float) tempZ / (float) theCounter;


  
}








void calibrateLSMg() {
  
  // Change these to zero to start a new LSM6D
      float GxOffset = -.42325;//-.1407280; 
      float GyOffset = -.171512;//.2104891; 
      float GzOffset = 0.0136;//-.01120;//-.2850000;  

  
  Serial.println("Calibrate LSM6DS Gyro -- Make sure all offsets are set to zero. Three 30 second runs.");Serial.println(" ");
  Wire.begin();
  
  delay(300);
  Wire.setClock(400000); // 400kHz I2C clock.  (can only be 100k, 400k, 1000k)
  delay(400);
  bool sADXL2 = false;
  bool sMS562 = false;
  bool sLSM2 = false;
      float Ax, Ay, Az;      
      
    float baroHumidity = 0.0;
    baro.Baseline = 0.0; // baseline altitude to subtract
    float oldAltitude = 0.0;
    unsigned long speedtimer = 0;
    unsigned long baroReadTimer = 0;
    float altFilter = 0.0;
    unsigned long oldTime = 0.0;
    int altFilterCount = 0;
    float avgFilter = 0.0;
    int fSpeed = 0;
    float fSpeedSample[10] = {0.0};
   
    short CONV_D1_MB = 0x48;          // corresponding temp conv. command for OSR
    short CONV_D2_MB = 0x58;          // corresponding pressure conv. command for OSR  
    unsigned long MDP, MDT;
    float MdT, MTEMP, MP;
    int baroReady = 0;
    unsigned long tempTimer = 0;
    float P_val,T_val,H_val;

     int16_t allRead[6]; 
      int16_t gyroRead[3];       
      float Gx, Gy, Gz = 0.0;  

  // ====== LSM6DS  setup code and local variables ============
    if(sLSM) {
      Serial.print("LSM Accel/Gyro:  ");
      uint8_t theID;
      theID = LSM6.whoAmI();
      if(theID == 108) { // HEX 6C
        Serial.println("Initialization successful!");
        sLSM2 = true;
      } else {
        Serial.println("Error in Communicating with the LSM6DS Accel/Gyro");
      }
    } else {
      Serial.println("LSM6D Accelerometer Disabled in Config");
    }

  Serial.println(" ");
  unsigned long theStop;
  int theCounter = 0;


  // ====== LSM Accel/Gyro  Performance Test ============
   if(sLSM2) {
      
      LSM6.setAccelRateScale(B01100100); // 01100100 :  0110 = 416hz, 01 = 32g, 0 = no LPF, 0 = default 
      delay(10);
      LSM6.setGyroRateScale(B01101100); // 01101100 :  0110 = 416hz, 11 = 2000 dps, 0 = FS, 0 = default 
      //LSM6.setAnyRegister(mLSM6DS_INT1_CTRL,B00000010); //set int1 to new gyro values
      delay(10);
      LSM6.setAnyRegister(mLSM6DS_INT1_CTRL,B00000000); //set int1 to new gyro values
      delay(10);
      LSM6.setAnyRegister(mLSM6DS_INT2_CTRL,B00000001); //set int2 to new gyro values

      
     theCounter = 0;
     theStop = millis() + 1000;
     int16_t accelRead[3]; 
     int16_t gyroRead[3]; 
           
      float tempX;
      float tempY;
      float tempZ;

      float angX;
      float angY;
      float angZ;

      unsigned long microTimer = 0;
      int totalTime = 0;
      int dontCount = 1;
     
     Serial.println ("60 Second LSM-Gyro Calibration Output. Use inverse for offsets...");
     theStop = millis() + 61000;
     
     while(millis() < theStop) {
        unsigned long oneSec;
        tempX = 0;
        tempY=0;
        tempZ=0;
        theCounter=0;
        oneSec = millis() + 30000;
        while(millis() < oneSec) {
         if(digitalRead(int2LSM) == HIGH) {  //get an interupt from the gyro at 500hz 
              totalTime = micros() - microTimer;
              microTimer = micros();
              
              LSM6.readGyroData(gyroRead);   
              //now convert the gyro readings into degrees per second, based on the lsb per degree from the datasheet 
              Gx = (float)gyroRead[0] * (float) .070;  // 4000=140, 2000=70, 1000=35, 500=17.5, 250=8.75, 125 = 4.375 (datasheet pg 25)
              Gy = (float)gyroRead[1] * (float) .070;
              Gz = (float)gyroRead[2] * (float) .070;
              //adjust for still calibration offsets
              Gx += GxOffset;
              Gy += GyOffset;
              Gz += GzOffset;
              if(dontCount == 0) {
                angX = (float) (angX + ((float) totalTime / (float) 1000000.0 * Gx));
                angY = (float) (angY + ((float) totalTime / (float) 1000000.0 * Gy));
                angZ = (float) (angZ + ((float) totalTime / (float) 1000000.0 * Gz));
                
              } else {
                dontCount = 0;
              }
         
            theCounter++;
            tempX += Gx;
            tempY += Gy;
            tempZ += Gz; 
          }

        }
        dontCount = 1; // don't count serial print time in the drift
        
          //done with one second sample
          
          tempX = (float) tempX / (float) theCounter;
          tempY = (float) tempY / (float) theCounter;
          tempZ = (float) tempZ / (float) theCounter;
          Serial.print("AVG LSM6D Gyro Values:    X ="); Serial.print(tempX,8);
          Serial.print("    Y ="); Serial.print(tempY,8);
          Serial.print("    Z ="); Serial.print(tempZ,8);
          Serial.print("    DRIFT DEG:  X=");Serial.print(angX,2);
          Serial.print("   Y ="); Serial.print(angY,2);
          Serial.print("   Z ="); Serial.println(angZ,2);
          
     }
   }

  Wire.end();
}


void calibrateBaro() {
  
  Serial.println("Checking MS5607 Barometer Calibration");Serial.println(" ");
  Wire.begin();
  
  delay(300);
  Wire.setClock(400000); // 400kHz I2C clock.  (can only be 100k, 400k, 1000k)
  delay(400);
  bool sADXL2 = false;
  bool sMS562 = false;
  bool sLSM2 = false;

    float baroHumidity = 0.0;
    baro.Baseline = 0.0; // baseline altitude to subtract
    float oldAltitude = 0.0;
    unsigned long speedtimer = 0;
    unsigned long baroReadTimer = 0;
    float altFilter = 0.0;
    unsigned long oldTime = 0.0;
    int altFilterCount = 0;
    float avgFilter = 0.0;
    int fSpeed = 0;
    float fSpeedSample[10] = {0.0};
   
    short CONV_D1_MB = 0x48;          // corresponding temp conv. command for OSR
    short CONV_D2_MB = 0x58;          // corresponding pressure conv. command for OSR  
    unsigned long MDP, MDT;
    float MdT, MTEMP, MP;
    int baroReady = 0;
    unsigned long tempTimer = 0;
    float P_val,T_val,H_val;

  // ====== MS5607 Barometer setup code and local variables ============
    if(sMS56) {
      Serial.print("MS5607 Barometer:  ");
      if(!P_Sens.begin()){
        Serial.println("Error in Communicating with MS5607");
      }else{
        Serial.println("Initialization successful!");
        sMS562 = true;
      }
    } else {
      Serial.println("MS5607 Barometer Disabled in Config");
    }

  Serial.println(" ");
  unsigned long theStop;
  unsigned long oneSec;
  int theCounter = 0;

  // ====== MS5607  Performance Test ============
   if(sMS562) {
        P_Sens.setOSR(4096); 
        delay(500);
      // first run
        P_Sens.startConversionMB(CONV_D1_MB); delay(10);
        P_Sens.startMeasurmentMB(); delay(10);
        P_Sens.getDigitalValueMB(MDP); delay(10);
        P_Sens.startConversionMB(CONV_D2_MB); delay(10);
        P_Sens.startMeasurmentMB(); delay(10);
        P_Sens.getDigitalValueMB(MDT);
        //get first baseline
        P_Sens.setValuesMB(MDT, MDP);    
        T_val = P_Sens.getTemperature();
        H_val = P_Sens.getAltitude();  

       theCounter = 0;
       Serial.println ("30 Second Barometer Output...");
       theStop = millis() + 30000;
     
     while(millis() < theStop) {
      oneSec = millis() + 5000;
        while(millis() < oneSec) {
          P_Sens.startConversionMB(CONV_D1_MB); delay(10);
          P_Sens.startMeasurmentMB(); delay(10);
          P_Sens.getDigitalValueMB(MDP); delay(10);
          P_Sens.startConversionMB(CONV_D2_MB); delay(10);
          P_Sens.startMeasurmentMB(); delay(10);
          P_Sens.getDigitalValueMB(MDT);
          //get first baseline
          P_Sens.setValuesMB(MDT, MDP);    
          T_val = P_Sens.getTemperature();
          H_val = P_Sens.getAltitude();        
            baro.Altraw = H_val;
             baro.TempF = (T_val * (float) 1.8) + (float) 32.0; //convert to F
             if (baro.TempF > baro.TempMax) baro.TempMax = baro.TempF;
           
             baro.tempAltitude = H_val * (float) 3.28084;
             if (baro.tempAltitude > baro.AltitudeMax) baro.AltitudeMax = baro.tempAltitude;

          
          theCounter++; 
        }
        Serial.println("===========================================");
        Serial.print("Baro Altitude (F):  ");
        Serial.print(baro.tempAltitude);
        Serial.println("  -  should be around 375");
        Serial.print("Baro Max Altitude:  ");
        Serial.println(baro.AltitudeMax);
        Serial.print("Baro Temp (f):  ");
        Serial.println(baro.TempF);
        Serial.print("Baro Temp Max:  ");
        Serial.println(baro.TempMax);
        Serial.print("Five Second Count:  ");
        Serial.print(theCounter);
        Serial.println("  -  should be around 98");
        theCounter = 0;
        baro.TempMax = 0;
        baro.AltitudeMax = 0;                
        
     }
     Serial.println("MS5607 Baro Compelete");
     
    } else {
      Serial.println("MS5607 Barometer Disabled in Config");
    }


  Wire.end();
}


void printFlashDirectory() {



  Serial.println("Printing Flash Directory: ");
  if (!myfs.begin()) {
    Serial.println("Error starting spidisk");
  }

  
  printDirectory(myfs.open("/"), 0);
  Serial.println();
  freeSpaces();

}

  void freeSpaces() {

  unsigned long bFree = 0;
  bFree = myfs.totalSize() - myfs.usedSize();
  Serial.printf("Bytes Used: %llu, Bytes Total:%llu", myfs.usedSize(), myfs.totalSize());
  Serial.print("    Bytes Free: ");
  Serial.println(bFree);
  Serial.println();
}

void printDirectory(File dir, int numTabs) {
  //dir.whoami();
  uint64_t fSize = 0;
  uint32_t dCnt = 0, fCnt = 0;
  if ( 0 == dir ) {
    Serial.printf( "\t>>>\t>>>>> No Dir\n" );
    return;
  }
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      Serial.printf("\n %u dirs with %u files of Size %u Bytes\n", dCnt, fCnt, fSize);
      fTot += fCnt;
      totSize1 += fSize;
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }

    if (entry.isDirectory()) {
      Serial.print("DIR\t");
      dCnt++;
    } else {
      Serial.print("FILE\t");
      fCnt++;
      fSize += entry.size();
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println(" / ");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
    //Serial.flush();
  }
}


void copy2SD() {
    Serial.println("Copying Flash to SD card... ");
    // Initializing the SD Card
    if(!SD.begin(BUILTIN_SDCARD)){
        Serial.println(F("SD Card Mount Failed"));
        return;
    } else {
        Serial.println(F("Good SD Card mounted"));
    }
  if (!myfs.begin()) {
    Serial.println("Error starting Flash Disk");
    return;
  }
  delay(500);
    
   uint64_t fSize = 0;
  // Cycle through directory
      File mdir;
      mdir = myfs.open("/");
    while (true) {  
      File entry =  mdir.openNextFile();
      if (! entry) {
        // no more files
        Serial.println("No more flash files to copy ");
        totSize1 += fSize;
        break;
      }
      
      if (entry.isDirectory()) {
        Serial.print("Skipping Directory: ");
        Serial.println(entry.name());
      } else {
        Serial.print("Copying File ");
        Serial.print(entry.name());
        Serial.print("   Size = ");
        fSize = entry.size();      
        Serial.println(fSize);
        // Copy it to SD
        // Delete and open from SD
        char sName[30];
        strcpy(sName,"/");  
        strcat(sName,entry.name());
        SD.remove(sName);
        tempFile = SD.open(sName, FILE_WRITE);
        // Open for reading and writing from flash
        char buf2[1];
        file = myfs.open(entry.name(), FILE_READ);
       
        while(file.available()) {         
         file.read(buf2, 1);
         tempFile.print(buf2[0]);
        }
        file.close();
        tempFile.close();        
      }
      entry.close();
    }
    
}


void eraseFlash() {
  Serial.println("Quick formatting flash...");
  myfs.quickFormat();
  //myfs.lowLevelFormat('.');
  Serial.println("Done formating flash memory");
  Serial.println(" ");

}


void radioTest() {
  
    Serial.println("Initiating Radio Test...");
    Serial.println(" ");
    digitalWrite(radioSet, HIGH); //turn radio set pin to low
    delay(50);
    digitalWrite(radioPower_pin, HIGH); //turn on Radio power
    delay(3000); // time for power to come up
    Serial2.begin(9600);
    char s[5];
    strcpy(s,"");
    unsigned long testtime = millis() + 500;
    s[0] = 0xaa;
    s[1] = 0xfa;
    s[2] = 0xaa;
    s[3] = 0x0d;
    s[4] = 0x0a;
    Serial2.println(s);
    int x = 0;
    while(x < 1) {
      checkRadio();
      if(newWord == 1) {
        Serial.print("Radio Rx:  ");
        Serial.println(theWord);
        char *ptr = strstr(theWord, "LORA6100");
        if (ptr != NULL) Serial.println("Radio Test Successful!"); 
        digitalWrite(radioSet, LOW); //turn radio set pin to low
        delay(2000);
        Serial2.begin(19200);
        Serial2.print("@M,33,Hello World,!");
        Serial2.flush();
        Serial.println(" "); 
        Serial.println("Message Sent"); 
        
        x = 1;
        
      }
      if(millis() > testtime) {
        Serial.println("Radio Timeout");
        digitalWrite(radioSet, LOW); //turn radio set pin to low
        delay(200);
        Serial2.begin(19200);
        x = 2;
      }
      
      
    }



  
}

void checkRadio() {  // **********  CHECK RADIO  *****************
   newWord = 0;
   char receivedChar;
   if (Serial2.available() > 0) {
  //  Serial.println("Rx...");
    unsigned long testTime = millis() + 100;
    
    strcpy(theWord, "");
    //Serial.println("radio inbound detect");

    while (newWord != 1) {
       if(Serial2.available()) {

         receivedChar = Serial2.read();
         //Serial.print(receivedChar);
         append(theWord, receivedChar);
         if(receivedChar == 0x0a) {
            newWord = 1;
            break;   
         }
       }
       if(millis() > testTime) {  // exit and evaluate if it takes too long
          newWord =1;
          Serial.println("RX timeout exit error");
          Serial.println(theWord);
          break;
       }
       delay(1);
     }
     Serial.println(" ");
     //ProcessRadio();
   }
}

void append(char* s, char c) {  //used to concat char to char array
        int len = strlen(s);
        s[len] = c;
        s[len+1] = '\0';
}




void quaternionDrift() {
  
  // Change these to zero to start a new LSM6D
      float GxOffset = -.42325;//-.1407280; 
      float GyOffset = -.171512;//.2104891; 
      float GzOffset = 0.0136;//-.01120;//-.2850000;  

  
  Serial.println("LSM6DS Gyro -- Make sure all offsets are set. Quaternion drift test.");Serial.println(" ");
  Wire.begin();
  
  delay(300);
  Wire.setClock(400000); // 400kHz I2C clock.  (can only be 100k, 400k, 1000k)
  delay(400);
  bool sADXL2 = false;
  bool sMS562 = false;
  bool sLSM2 = false;
      float Ax, Ay, Az;      
      
    float baroHumidity = 0.0;
    baro.Baseline = 0.0; // baseline altitude to subtract
    float oldAltitude = 0.0;
    unsigned long speedtimer = 0;
    unsigned long baroReadTimer = 0;
    float altFilter = 0.0;
    unsigned long oldTime = 0.0;
    int altFilterCount = 0;
    float avgFilter = 0.0;
    int fSpeed = 0;
    float fSpeedSample[10] = {0.0};
   
    short CONV_D1_MB = 0x48;          // corresponding temp conv. command for OSR
    short CONV_D2_MB = 0x58;          // corresponding pressure conv. command for OSR  
    unsigned long MDP, MDT;
    float MdT, MTEMP, MP;
    int baroReady = 0;
    unsigned long tempTimer = 0;
    float P_val,T_val,H_val;

     int16_t allRead[6]; 
      int16_t gyroRead[3];       
      float Gx, Gy, Gz = 0.0;  

  // ====== LSM6DS  setup code and local variables ============
    if(sLSM) {
      Serial.print("LSM Accel/Gyro:  ");
      uint8_t theID;
      theID = LSM6.whoAmI();
      if(theID == 108) { // HEX 6C
        Serial.println("Initialization successful!");
        sLSM2 = true;
      } else {
        Serial.println("Error in Communicating with the LSM6DS Accel/Gyro");
      }
    } else {
      Serial.println("LSM6D Accelerometer Disabled in Config");
    }

  Serial.println(" ");
  unsigned long theStop;
  int theCounter = 0;


  // ====== LSM Accel/Gyro  Performance Test ============
   if(sLSM2) {
      
      LSM6.setAccelRateScale(B01100100); // 01100100 :  0110 = 416hz, 01 = 32g, 0 = no LPF, 0 = default 
      delay(10);
      LSM6.setGyroRateScale(B01101100); // 01101100 :  0110 = 416hz, 11 = 2000 dps, 0 = FS, 0 = default 
      //LSM6.setAnyRegister(mLSM6DS_INT1_CTRL,B00000010); //set int1 to new gyro values
      delay(10);
      LSM6.setAnyRegister(mLSM6DS_INT1_CTRL,B00000000); //set int1 to new gyro values
      delay(10);
      LSM6.setAnyRegister(mLSM6DS_INT2_CTRL,B00000001); //set int2 to new gyro values

      
     theCounter = 0;
     theStop = millis() + 1000;
     int16_t accelRead[3]; 
     int16_t gyroRead[3]; 
           
      float tempX;
      float tempY;
      float tempZ;

      float angX;
      float angY;
      float angZ;

      unsigned long microTimer = 0;
      int totalTime = 0;
      int dontCount = 1;

     Serial.println ("Hold Still. Reading offset for 60 seconds...");
     theStop = millis() + 60000;
     
     while(millis() < theStop) {
        unsigned long oneSec;
        tempX = 0;
        tempY=0;
        tempZ=0;
        theCounter=0;
        oneSec = millis() + 10000;
        while(millis() < oneSec) {
         if(digitalRead(int2LSM) == HIGH) {  //get an interupt from the gyro at 500hz 
              totalTime = micros() - microTimer;
              microTimer = micros();
              
              LSM6.readGyroData(gyroRead);   
              //now convert the gyro readings into degrees per second, based on the lsb per degree from the datasheet 
              Gx = (float)gyroRead[0] * (float) .070;  // 4000=140, 2000=70, 1000=35, 500=17.5, 250=8.75, 125 = 4.375 (datasheet pg 25)
              Gy = (float)gyroRead[1] * (float) .070;
              Gz = (float)gyroRead[2] * (float) .070;
              //adjust for still calibration offsets
              Gx += GxOffset;
              Gy += GyOffset;
              Gz += GzOffset;
         
            theCounter++;
            tempX += Gx;
            tempY += Gy;
            tempZ += Gz; 
          }

        }
          tempX = (float) tempX / (float) theCounter;
          tempY = (float) tempY / (float) theCounter;
          tempZ = (float) tempZ / (float) theCounter;
          Serial.print("OFFSET:  X=");Serial.print(tempX,5);
          Serial.print("   Y ="); Serial.print(tempY,5);
          Serial.print("   Z ="); Serial.println(tempZ,5);
          GxOffset = GxOffset + ((float) -1.0 * tempX);
          GyOffset = GyOffset + ((float) -1.0 * tempY);
          GzOffset = GzOffset + ((float) -1.0 * tempZ);
          Serial.print("New Offsets:  X=");Serial.print(GxOffset,5);
          Serial.print("   Y ="); Serial.print(GyOffset,5);
          Serial.print("   Z ="); Serial.println(GzOffset,5);          
     }

   // *****  now test the drift with raw and quaternions
     gyroPrelaunch();

      
     Serial.println ("90 Second LSM-Gyro Drift Output with 10 sec output.");
     theStop = millis() + 90000;

      elapsedMicros lastMicros; //for Gyro shift
      unsigned long gyroReset = 0;
      float dt = 0.0;
      int LSM6DBurnoutCount = 0;
     
     while(millis() < theStop) {
        unsigned long oneSec;
        int td;
        tempX = 0;
        tempY=0;
        tempZ=0;
        theCounter=0;
        oneSec = millis() + 10000;
        while(millis() < oneSec) {
         if(digitalRead(int2LSM) == HIGH) {  //get an interupt from the gyro at 500hz 
              totalTime = micros() - microTimer;
              microTimer = micros();




              td = lastMicros;
              lastMicros = 0;
              dt = (float) td / (float) 1000000.0;  // this is the time interval in microseconds
              
              LSM6.readGyroData(gyroRead);   
              //now convert the gyro readings into degrees per second, based on the lsb per degree from the datasheet 
              Gx = (float)gyroRead[0] * (float) .070;  // 4000=140, 2000=70, 1000=35, 500=17.5, 250=8.75, 125 = 4.375 (datasheet pg 25)
              Gy = (float)gyroRead[1] * (float) .070;
              Gz = (float)gyroRead[2] * (float) .070;
              //adjust for still calibration offsets
              Gx += GxOffset;
              Gy += GyOffset;
              Gz += GzOffset;

              theCounter++;
              tempX += Gx;
              tempY += Gy;
              tempZ += Gz; 

              // now factor for fraction of a second 
              Gx = Gx * dt;
              Gy = Gy * dt;
              Gz = Gz * dt;

            // done getting what we need from the Gyro. Got Gx, Gy, Gz represented in degrees moved since last sampled 
            // now convert hardware orientation to our quaternion orientation. Quaternion functions expects x = roll, y = pitch, z = yaw
            float qX, qY, qZ;
            qX = Gx; qY = Gy; qZ = Gz; //v 4.5 hardware x+ is up 
            //-------- QUATERNION CALCULATIONS      -------------// 
            gyroRotate(qX, qY, qZ); // does the four quaternion functions and updates angX, angY, angZ
            //-------- CALCULATE THE TILT VALUES      -------------// 
            float testA;
            float testB;
            testA = angY;
            if(testA < 0) testA = testA * -1;
            testB = angZ;
            if(testB < 0) testB = testB * -1;
            if(testA > testB) {
              LSM6D.tilt = testA;        
            } else {
              LSM6D.tilt = testB;
            }
            if((millis() > 15000) && (LSM6D.tilt > LSM6D.tiltMax)) LSM6D.tiltMax = LSM6D.tilt;
            LSM6D.roll = angX;
            LSM6D.pitch = angY;
            LSM6D.yaw = angZ;
            LSM6D.gX = Gx;
            LSM6D.gY = Gy;
            LSM6D.gZ = Gz;

          }

        }
          tempX = (float) tempX / (float) theCounter;
          tempY = (float) tempY / (float) theCounter;
          tempZ = (float) tempZ / (float) theCounter;
          Serial.print("QUAT DEG:  X=");Serial.print(angX,2);
          Serial.print("   Y ="); Serial.print(angY,2);
          Serial.print("   Z ="); Serial.print(angZ,2);
          Serial.print("   TILT ="); Serial.println(LSM6D.tilt,2);       
     }


     
   }

  Wire.end();
}


//***********************************************************        TILT QUATERNION HERE    ************************************************************
//***********************************************************        TILT QUATERNION HERE    ************************************************************



void gyroPrelaunch() {

      int16_t accelRead[3]; 
      float Ax, Ay, Az;      
      float AxOffset = -2.95;//-5;
      float AyOffset = -16.55;//4.5;
      float AzOffset = -11.85;//-22;
  
  // set the baseline quaternion tilt values using accel data before launch  
  masterQuaternion = { .r = 1, .x = 0, .y = 0, .z = 0 };

  LSM6.readAccelData(accelRead); //populate accelRead[3] array with raw sample data from accelerometer
  Ax = accelRead[0] + AxOffset;
  Ay = accelRead[1] + AyOffset;
  Az = accelRead[2] + AzOffset;
                  
  double a1 = 0.00, a2 = 0.00;       //Roll & Pitch are the angles which rotate by the axis X and y    
  double x_Buff = float(Az);
  double y_Buff = float(Ay);
  double z_Buff = float(Ax);
  //set angles for pitch and yaw
  a1 = atan2(y_Buff , z_Buff) * 57.3; //pitch
  a2 = atan2((- x_Buff) , sqrt(y_Buff * y_Buff + z_Buff * z_Buff)) * 57.3; //yaw
    
  //now integrate the angles into our freshly minted master quaternion (ignoring roll) 
  gyroRotate(0,a1,a2);

}


void gyroRotate(float gRoll, float gPitch, float gYaw) {

  // R, P, Y inputs should be degrees factored for time interval

  //first convert the gyro tiny rotation into a half euler and store it in a temporary quaternion
  quaternionInitHalfEuler(gRoll, gPitch, gYaw);

  //now combine it with the masterQuaternion to get an integrated rotation
  quaternionMultiply();

  //now normalize the masterQuaternion in case it went out of bounds
  quaternionNormalize();

  //now we need to translate the masterQuaternion back to angles that humans can read
  quaternionAxisAngles();   
}


void quaternionAxisAngles() {
 
  quaternion p = masterQuaternion;

  // Compute the X (roll) angle in radians
  if((1-2*(p.x*p.x+p.y*p.y)) != 0) {
     angX = atan(2*(p.r*p.x+p.y*p.z)/(1-2*(p.x*p.x+p.y*p.y)));
   } else {
    if((p.r*p.x+p.y*p.z) > 0) {
      angX = M_PI / 2.0f;
    } else {
      if((p.r*p.x+p.y*p.z) < 0) {
        angX = -1.0f * M_PI / 2.0f;
      } else {
        angX = 9999999;
        // SIGNAL ERROR DIVIDE BY ZERO!!!
      }
    }
  }
  // Convert x (roll) from radian to degrees
  angX = angX * 180.0f / M_PI;


  //Compute the Y (pitch) angle in radians
  if((2*(p.x*p.y-p.z*p.x)) <= -1) {
    angY = -1.0f * M_PI / 2.0f;
  } else {
    if((2*(p.r*p.y-p.z*p.x)) >= 1) {
      angY = M_PI / 2.0f;
    } else {
      angY = asin(2*(p.r*p.y-p.z*p.x));
    }
  }
  // Convert y (pitch) from radian to degrees
  angY = angY * 180.0f / M_PI;  


  // Compute the Z (Yaw) angle in radians
  if((1-2*(p.x*p.x+p.y*p.y)) != 0) {
     angZ = atan(2*(p.r*p.z+p.x*p.y)/(1-2*(p.y*p.y+p.z*p.z)));
   } else {
    if((p.r*p.z+p.x*p.y) > 0) {
      angZ = M_PI / 2.0f;
    } else {
      if((p.r*p.z+p.x*p.y) < 0) {
        angZ = -1.0f * M_PI / 2.0f;
      } else {
        angZ = 9999999;
        // SIGNAL ERROR DIVIDE BY ZERO!!!
      }
    }
  }
  // Convert z (Yaw) from radian to degrees
  angZ = angZ * 180.0f / M_PI;
  
}

void quaternionNormalize() {

  quaternion p = masterQuaternion;
  float testP = 0.0;

  testP = (p.r * p.r) + (p.x * p.x) + (p.y * p.y) + (p.z * p.z);
  if(testP > 1.0) {
     p.r = p.r * (float)(1.0f / sqrtf(testP));
     p.x = p.x * (float)(1.0f / sqrtf(testP));
     p.y = p.y * (float)(1.0f / sqrtf(testP));
     p.z = p.z * (float)(1.0f / sqrtf(testP));
  }
  masterQuaternion = p;  
}

void quaternionMultiply() {
  
  // combine t with the masterQuaternion to get an integrated rotation
  quaternion p = masterQuaternion;
  quaternion t = tempQuaternion;

  masterQuaternion.r = (p.r * t.r) + (-p.x * t.x) + (-p.y * t.y) + (-p.z * t.z);
  masterQuaternion.x = (p.r * t.x) + (p.x * t.r) + (p.y * t.z) + (-p.z * t.y);
  masterQuaternion.y = (p.r * t.y) + (-p.x * t.z) + (p.y * t.r) + (p.z * t.x);
  masterQuaternion.z = (p.r * t.z) + (p.x * t.y) + (-p.y * t.x) + (p.z * t.r);

}


void quaternionInitHalfEuler(float gRoll, float gPitch, float gYaw) {

  // store the tiny rotation from the gyro in a new temporary quaternion
  // roll, pitch, yaw input is in degrees

  float s_x, c_x;
  float s_y, c_y;
  float s_z, c_z;

  float x = (gRoll / 2.0f) * M_PI/180.0f;  // half the degree and convert it to radians
  float y = (gPitch / 2.0f) * M_PI/180.0f;
  float z = (gYaw / 2.0f) * M_PI/180.0f;

  s_x = sin(x); c_x = cos(x);
  s_y = sin(y); c_y = cos(y);
  s_z = sin(z); c_z = cos(z);

  // This is our quaternion that represents the rotation difference
  tempQuaternion.r = c_x * c_y * c_z + s_x * s_y * s_z;
  tempQuaternion.x = s_x * c_y * c_z - c_x * s_y * s_z;
  tempQuaternion.y = c_x * s_y * c_z + s_x * c_y * s_z;
  tempQuaternion.z = c_x * c_y * s_z - s_x * s_y * c_z;

}
