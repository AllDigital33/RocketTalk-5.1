

/*   ***********  Rocket Talk Vehicle Code - copyright 2016 - 2023 @Pad33space / @AllDigital  ***********

NOTES:
 *   Compile at 396 Mhz clock speed for lowest CPU temperatures - Is stable
 *   
 *   
 * 
 */



  const bool serialDebug = false;  // set to true to get Serial output for debugging
  const bool debugMode = false; // Serial output sensor stats to ensure nothing screwed up
  
  #define GPSmodel 10 // remove old 8 and 9 code

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


//Teensy
  #include <TeensyThreads.h>
  #include "Arduino.h"

  
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
   bool gps = true; // enable GPS checking
   bool radio = true; // enable radio checking
   bool sampling = true; //enable sampling thread loop
   bool baro = true; // enable barometer checking
   bool ADXL = true; // enable ADXL checking
   bool LSM = true; // enable LSM checking
   int machLockout = 6000;  // milliseconds to lock-out baro for mach (L+)
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
   int logFlush = 1000; // millis to hard physical save log files to flash
   
  };
  configStruct configs;



  struct workingStruct {  // working variables
    char phase[25];   // Normally should be set to STARTUP
    char callsign[10];  //Radio requires Amatuer Radio Licence
    // Testing Phase variables
    char FlightLogMaster[900];
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
    unsigned long errorTimer = 0;
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
    int errorInterval = 30000; // send errors every 30 sec 
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
    int avgCounter = 0;
    float tempAvg = 0.0;
  };
  gpsDataStruct gpsData;
  
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

struct errorStruct {
  // 0 = unchecked, 1 = error condition, 2 = healthy
  byte sampleLoop = 0;
  byte loggingLoop = 0;
  byte ADXL = 0;
  byte LSM = 0;
  byte MS56 = 0;
  byte GPS = 0;
  byte flash = 0;
  byte voltage = 0;
  byte CPUtemp = 0;
  byte I2C = 0;
  byte crash = 0;
  byte count = 0;
  byte sendIt = 0;
  byte radio = 0;
  char message[200];
};
  errorStruct errors;

// For threading
  int id_logging, id_sampling;

// For the flash memory
  #include <LittleFS.h>
  #include <stdarg.h>
  
  #include <TimeLib.h>  // for logging time
  LittleFS_QSPIFlash myfs;  //these need to stay global 
  File FlightFile;
  File SummaryFile;
  File GPSFile;
  File RadioFile;

// Temperature from CPU
  extern float tempmonGetTemp(void);  



//***********************************************************        SETUP    ************************************************************
//***********************************************************        SETUP     ************************************************************


void setup() {


// Pin setup v5.1
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
  
  Serial.begin(115200);

  if(CrashReport) {
    Serial.print(CrashReport);
    errors.crash = 1;
    if(debugMode) while(1); // hard stop if debug mode
  } else {
    errors.crash = 2; //healthy
  }

  if(serialDebug) Serial.println("Starting up....RocketTalk v51 Master");

  // Power On Radio  (Radio at 435.92 Mhz / 4800b radio-radio, but 19.2b cpu-radio)
  if(configs.radio) {
    digitalWrite(radioPower_pin, HIGH); //turn on Radio power
    Serial2.begin(19200);
  }
  
  if(errors.crash == 2) {
    delay(3000); // time for power to come up - otherwise if we crashed midflight get back to business fast  
    digitalWrite(led1_pin,HIGH);delay(300);digitalWrite(led1_pin,LOW);
    digitalWrite(led2_pin,HIGH);delay(300);digitalWrite(led2_pin,LOW);
    digitalWrite(led3_pin,HIGH);delay(300);digitalWrite(led3_pin,LOW);  
    // test radio
    radioHardwareTest();
  }

  

  // Initialize GPS
  if(configs.gps) {
    digitalWrite(GPSpower_pin, HIGH); //turn on GPS power
    delay(1000);
    if(GPSmodel == 8) {
      Serial1.begin(9600);
      configGPS89(); 
    }
    if(GPSmodel == 9) {
       Serial1.begin(38400); // for M9N 
       configGPS89();
    }
     if(GPSmodel == 10) {
      Serial1.begin(9600);
      configGPS10(); 
    }   
  }



  //create separate thread loop for sampling
  if(configs.sampling) {
    id_sampling = threads.addThread(sample_thread,0, 2048,0);
    threads.setMicroTimer(5);
    threads.setTimeSlice(id_sampling, 1); 
  }
  threads.delay(500);
  
  //create separate thread loop for logging
  if(configs.logging) {
    timers.flightLogInterval = 100; //10x sec
    id_logging = threads.addThread(logging_thread,0, 4096,0);
    threads.setMicroTimer(5);
    threads.setTimeSlice(id_logging, 1); 
  }
  threads.setTimeSlice(0, 1); 
  threads.delay(500);
  
  
  // Initialize the Event switches
  if (digitalRead(fore_pin) == HIGH) { events.EventFore = 1; } else {events.EventFore = 0;}
  if (digitalRead(aft_pin) == HIGH) { events.EventAft = 1; } else {events.EventAft = 0;}
  //if (configs.invertAft && events.EventAft == 1) { events.EventAft = 0; } else {events.EventAft = 1;} // Aft switch invert logic
  if (digitalRead(sep_pin) == HIGH) { events.EventSep = 1; } else {events.EventSep = 0;}  
  if (configs.invertSep && events.EventSep == 1) { events.EventSep = 0; } else {events.EventSep = 1;} // Aft switch invert logic

  
  getCPUtemp();  
  readVoltage();
  strcpy(working.callsign, "KK6EVH");
  //check continuity before starting up
  threads.delay(100);
  pyroContinuity(); 
  if(serialDebug) Serial.println(">>> Starting Main Loop...");
  if(errors.crash == 1) {
    PhaseBasicInit();
  } else {
    PhaseStartupInit();
  }
  threads.delay(3000);
  working.loggingClock = millis() + 10000; //let the logging start before errors
  working.sampleClock = millis() + 10000; //let the sampling start before errors
  
}



//***********************************************************        MAIN LOOP START    ************************************************************
//***********************************************************        MAIN LOOP START    ************************************************************

void loop() {


  //=================  Performance DEBUG Diagnostic Checks ==================== //
   if(debugMode) {
     if(millis() > debugVars.debugTimer) {
       timers.flightLogInterval = 10; // 50X per second
       int dTimerInterval = 5000;
       timers.gpsLogInterval = 1000;
       int dDivide = dTimerInterval / 1000; //whole seconds only
       Serial.println("DIAGNOSTICS DATA:");
       Serial.println("Transactions per second:");
       Serial.print("   ADXL:  ");Serial.print(debugVars.adxlCount / dDivide);Serial.println("  - should be 390");
       Serial.print("   LSM:  ");Serial.print(debugVars.LSM6DCount / dDivide);Serial.println("  - should be 399");
       Serial.print("   BARO:  ");Serial.print(debugVars.baroCount / dDivide);Serial.println("  - should be 38");
       float dSmall = (float)debugVars.logCount / (float) dDivide;
       Serial.print("   Flash Logging:  ");Serial.print(dSmall,1);Serial.println("  - should be 50");
       
       dSmall = (float)debugVars.GPScount / (float) dDivide;
       if(debugVars.GPSquality == 99){
        Serial.println("   GPS Check:  Timeout Error!");
       } else {
          Serial.print("   GPS Check: ");Serial.print(dSmall,1);Serial.print("  Sats: ");
          if(debugVars.GPSquality == 1){
            Serial.print("No Connection Yet");
          } else {
            Serial.print(debugVars.GPSquality);
          }
          Serial.println("  --  Count should be ~1");
          float dSmall = (float)debugVars.gpsTime / (float) debugVars.GPScount;
          Serial.print("   GPS clock set: ");if(events.clockSet == 1) {Serial.println("Yes");}else{Serial.println("No");}
          Serial.print("   GPS Avg Time (ms): ");Serial.println(dSmall,1);
       }
       
       Serial.println("RAW VALUES:");
       Serial.print("   adxl-avgZ: ");Serial.println(adxl.avgZ);
       Serial.print("   LSM-avgZ: ");Serial.println(LSM6D.avgZ);
       Serial.print("   Raw Altitude: ");Serial.println(baro.tempAltitude);
       Serial.print("   Altitude: ");Serial.println(baro.Altitude);
       Serial.print("   AltMax: ");Serial.println(baro.AltitudeMax);
       Serial.print("   Speed: ");Serial.println(baro.Speed);
       Serial.print("   TempF: ");Serial.println(baro.TempF);
       Serial.print("   Tilt: ");Serial.println(LSM6D.tilt);
       Serial.print("   Voltage: ");Serial.println(working.voltage);
       Serial.print("   CPU Temp: ");Serial.println(working.CPUtemp);
       Serial.print("   Launch Num: ");Serial.println(working.LaunchNumberStr);
       Serial.print("   Sample Stack Remaining: ");Serial.println(threads.getStackRemaining(id_sampling));
       Serial.print("   Logging Stack Remaining: ");Serial.println(threads.getStackRemaining(id_logging));
       if(errors.count > 0) {
        Serial.print("   ERRORS: ");Serial.println(errors.message);
       }

               
       //Serial.println("  adxl-Time: ");Serial.print(debugVars.adxlTime / debugVars.adxlCount);
       //Serial.println("  LSM-Time: ");Serial.print(debugVars.LSMtime / debugVars.LSM6DCount);
       Serial.println("**********************************************************************************************************************");
       debugVars.adxlCount = 0;
       debugVars.LSM6DCount = 0;
       debugVars.baroCount = 0;
       debugVars.logCount = 0;
       debugVars.LSMtime = 0;
       debugVars.baroTime = 0;
       debugVars.adxlTime = 0;
       debugVars.GPScount = 0;
       debugVars.GPSquality = 0;
       debugVars.gpsTime = 0;
       debugVars.debugTimer = millis() + dTimerInterval;
      threads.yield();
     }
   }
   //=================  End Performance DEBUG Diagnostics Check ==================== //

  
  //=================  HOUSEKEEPING ==================== //

  if(millis() > timers.voltageTimer) {
    readVoltage();
    timers.voltageTimer = millis() + timers.voltageInterval;
    getCPUtemp();
  }

  if(configs.gps) {
    if(millis() > timers.gpsSendTimer || millis() > timers.gpsLogTimer) {
      if(GPSmodel == 8 || GPSmodel == 9) CheckGPS89();
      if(GPSmodel == 10) CheckGPS10();
      
      if(millis() > timers.gpsSendTimer) {
        if (gpsData.gpsStatus == 2) {
          gpsRadioString();
          strcpy(radioHeader, "@G"); strcpy(radioMessage, gpsData.gpsSendLine);
          RadioSend();             
        }        
        timers.gpsSendTimer = millis() + timers.gpsSendInterval;
      }
      if(millis() > timers.gpsLogTimer) {
        if (gpsData.gpsStatus == 2) {
          gpsLogString();
          strcpy(working.gpsString, gpsData.gpsLogLine); 
          gpsLogQ();             
        } else {
          gpsLogQ();  // string is already populated with raw        
        }
        timers.gpsLogTimer = millis() + timers.gpsLogInterval;
      }
    threads.yield();  
    }
  }
  
  if(millis() > timers.sendFlightTimer) {
    flightRadioString();
    strcpy(radioHeader, "@F"); strcpy(radioMessage, working.sendFlight);
    RadioSend(); 
    timers.sendFlightTimer = millis() + timers.sendFlightInterval;
    threads.yield();
  }  

  if(millis() > timers.eventsTimer) {
    EjectEventCheck();
    timers.eventsTimer = millis() + timers.eventsInterval;
    threads.yield();
  }
  if(millis() > timers.radioRXTimer) {
    if(configs.radio) {
      checkRadio();
      timers.radioRXTimer = millis() + timers.radioRXInterval;
      threads.yield();
    }
  }

  if(millis() > timers.radioTXTimer && rSendCount > 0) {
    if(configs.radio) {
      RadioSendQueue();
      timers.radioTXTimer = millis() + timers.radioTXInterval; //1200 default spacing
      threads.yield();
    }
  }

  if(millis() > timers.errorTimer) {
    errorCheck();
    timers.errorTimer = millis() + timers.errorInterval;
  }

  if(millis() > timers.radioSendCallSignTimer) {
    strcpy(radioHeader, "@C"); strcpy(radioMessage, working.callsign);
    RadioSend(); 
    timers.radioSendCallSignTimer = millis() + timers.radioSendCallSignInterval;
    threads.yield();
  }

  if(millis() > timers.pyroOffTimer) {
    pyroTimers();
    timers.pyroOffTimer = millis() + 200;  //check pyros for off every .2 sec
  }

  if(millis() > timers.pyroContinuityTimer) {
    pyroContinuity();
    timers.pyroContinuityTimer = millis() + 2000;  //check pyros for off every 2 sec
  }

  if(millis() > timers.loggingHealth) {  // make sure logging has not errored out
    if(millis() > (working.loggingClock + 2000) && working.loggingOK == true) {
      if(serialDebug) Serial.println(F("Flash Error - logging stopped"));
      if(serialDebug) Serial.println(millis());
      if(serialDebug) Serial.println(working.loggingClock);
      working.loggingOK = false;
      errors.loggingLoop = 1;
      strcpy(radioHeader, "@M"); strcpy(radioMessage, "FLASH ERROR: Logging has stopped");
      RadioSend(); 
    }
    if(millis() < (working.loggingClock + 2000) && working.loggingOK == false) {
      if(serialDebug) Serial.println(F("Flash OK - logging resumed"));
      working.loggingOK = true;
      strcpy(radioHeader, "@M"); strcpy(radioMessage, "Logging has resumed");
      RadioSend(); 
    }
    if(millis() < (working.loggingClock + 2000)) errors.loggingLoop = 2; // good health :)
    timers.loggingHealth = millis() + 3000;
  }

  if(millis() > timers.sampleHealth) {  // make sure sampling thread has not errored out
    if(millis() > (working.sampleClock + 2000) && working.sampleOK == true) {
      if(serialDebug) Serial.println(F("ERROR sample thread stopped"));
      working.sampleOK = false;
      errors.sampleLoop = 1;
      strcpy(radioHeader, "@M"); strcpy(radioMessage, "ERROR: Sample Thread has stopped");
      RadioSend(); 
    }
    if(millis() < (working.sampleClock + 2000) && working.sampleOK == false) {
      working.sampleOK = true;
      strcpy(radioHeader, "@M"); strcpy(radioMessage, "Sample Thread has resumed");
      RadioSend(); 
    }
    if(millis() < (working.sampleClock + 2000)) errors.sampleLoop = 2; // good health :)
    timers.sampleHealth = millis() + 3000; // check once a second
  }


   

 //=================  PHASE LOOPS  ==================== //

  if (strcmp(working.phase, "STARTUP") == 0) PhaseStartup();
  if (strcmp(working.phase, "WAITING") == 0) PhaseWaiting();
  if (strcmp(working.phase, "LAUNCH") == 0) PhaseLaunch();
  if (strcmp(working.phase, "DESCENT") == 0) PhaseDescent();
  if (strcmp(working.phase, "XLANDED") == 0) PhaseLanded();
  if (strcmp(working.phase, "BASIC") == 0) PhaseBasic();

  threads.yield();
}

//***********************************************************        PHASES      ************************************************************
//***********************************************************        PHASES      ************************************************************


// ==========================  PHASE STARTUP ===========================
void PhaseStartupInit() {
  
  timers.eventLogInterval = 200;
  timers.radioLogInterval = 500;
  timers.gpsLogInterval = 2000;
  timers.gpsSendInterval = 5000;
  timers.flightLogInterval = 5000;  // log every 5 seconds during startup
  timers.sendFlightInterval = 10000; // should be >= log interval 
  timers.voltageInterval = 1000; 
  timers.eventsInterval = 500;
  timers.continuityInterval = 500;
  timers.radioRXInterval = 500;
  timers.radioTXInterval = 1200;
  timers.radioSendCallSignInterval = 90000;
  timers.errorInterval = 30000;
  timers.errorTimer = millis() + 4000;
  strcpy(working.phase, "STARTUP");
  strcpy(radioHeader, "@M"); strcpy(radioMessage, "Hello. Initiating Startup...");
  RadioSend();   
  timers.PhaseTimerA = millis() + 5000;  //transmit errors during startup
  timers.PhaseTimerB = millis() + 60000; // wait at least 60 seconds for startup to characterize barometer
  working.retries = 0;
}
void PhaseStartup() {
  
  if(millis() > timers.PhaseTimerB) {
      if (gpsData.gpsStatus == 2) {   // ready to rock. Move to waiting phase.
        strcpy(working.eventString, "STARTUP Phase Complete - Moving to Waiting Phase - All OK"); 
        eventLog(); 
        PhaseWaitingInit();
        return;
      } else {  // try for ten seconds more 15 times
        working.retries++;
        timers.PhaseTimerB = millis() + 10000; 
      }
      // check for GPS timeout and abort to Waiting phase
      if (working.retries == 15) {
        strcpy(working.eventString, "STARTUP Phase Complete - Moving to Waiting Phase - GPS Timeout ERROR"); 
        eventLog(); 
        PhaseWaitingInit();
        return;    
      }
  }  
}

// ==========================  PHASE WAITING ===========================

void PhaseWaitingInit() {

    //Recheck continuity
    events.continuityMain = 0;
    events.continuityDrogue = 0;
    events.continuitySeparation = 0;
    events.continuitySustainer = 0;
    pyroContinuity();
    timers.gpsLogInterval = 2000;
    timers.gpsSendInterval = 11000;   
    timers.flightLogInterval = 5000;  // log every 5 seconds during waiting
    timers.sendFlightInterval = 10000; // should be >= log interval 

    events.launchClock = 0;
    events.machClock = 0;
    events.launch = 0;
    errors.sendIt = 1; //send a health check

    timers.flightLogTimer = millis();
    strcpy(working.phase, "WAITING");
    strcpy(radioHeader, "@M"); strcpy(radioMessage, "Waiting for launch mode");
    RadioSend();            
    timers.sendFlightTimer = millis(); //force radio flight update
    if(serialDebug) Serial.println(F("Phase:  Waiting for launch mode"));
    //reset variables after warm-up 
    baro.Baseline = baro.tempAltitude; //forces a reset of baseline to altitude  
    adxl.minZ = 0;
    adxl.maxZ = 0;
    LSM6D.minZ = 0;
    LSM6D.maxZ = 0;
    LSM6D.tiltMax = 0;
    baro.AltitudeMax = 0;
    baro.SpeedMax = 0;
    gpsData.maxAGLfeet = 0;
    gpsData.baseline = 0.0;
    working.triggerCount = 0;
    working.triggerCount2 = 0;
}

void PhaseWaiting() {

    // *** LAUNCH TEST ONE - LSM6D ACCELEROMETER ***
    //three samples >1g in a row
    if (LSM6D.linZ > 1500) {  // use 1.5g for now as launch detect 
      working.triggerCount++;
      if(serialDebug) digitalWrite(led1_pin, HIGH);
      threads.yield();
      threads.delay(10); // allow enough time to get new accel reads
      threads.yield();
      if(working.triggerCount == 3) { // We have a real launch!!
        strcpy(working.eventString, "LAUNCH DETECT!  LIFT OFF (Accel Detect)"); 
        eventLog(); 
        PhaseLaunchInit();
        return;
      }
    }   
    if (working.triggerCount > 0 && LSM6D.linZ < 1000) {
        working.triggerCount = 0;  // reset counter if there was a false alarm
        if(serialDebug) digitalWrite(led1_pin, LOW);     
    }

    // *** LAUNCH TEST TWO - BAROMETER ***  (conservative backup)
    // three samples of >50 ft in a row -- use 100ms delay to prevent radio interference 
    if (baro.Altitude > 500.0 && millis() > baro.launchDetectDelay) {  // use 150' for now as absolute launch detect 
      working.triggerCount2++;
      if(serialDebug) digitalWrite(led1_pin, HIGH);
      baro.launchDetectDelay = millis() + 100; //100ms to prevent false positive
      if(working.triggerCount2 == 3) {
        strcpy(working.eventString, "LAUNCH DETECT!  LIFT OFF (Baro Detect)"); eventLog(); 
        baro.launch = 1;
        PhaseLaunchInit();
        return;
      }
    }
     if (working.triggerCount2 > 0 && baro.Altitude < 25.0) {
        working.triggerCount2 = 0;  // reset counter if there was a false alarm
        if(serialDebug) digitalWrite(led1_pin, LOW);     
    }  

}


// ==========================  PHASE LAUNCH - PARTY ON! ===========================

void PhaseLaunchInit() {

    events.launchClock = millis();
    events.machClock = millis() + configs.machLockout;
    timers.gpsLogInterval = 500;
    timers.gpsSendInterval = 5000;
    timers.flightLogInterval = 20;  // log 40X per second at launch
    timers.sendFlightInterval = 3000; // every 4 seconds on ascent
    timers.radioSendCallSignTimer = millis() + 90000;  //keep the radio free
    strcpy(working.phase, "LAUNCH");
    timers.flightLogTimer = millis();
    threads.yield();
    strcpy(radioHeader, "@E"); strcpy(radioMessage, "LAUNCH");
    RadioSend();            
    timers.sendFlightTimer = millis(); //force radio flight update
    if(serialDebug) Serial.println(F("LAUNCH DETECT!"));
    events.launch = 1;
    getTimeNow();
    strcpy(events.launchTime, working.timeHeader); //record launch time in string
    baro.lastAltitude = baro.Altitude; 
    working.triggerCount = 0; //reset for next phase
    working.triggerCount2 = 0; 
    timers.PhaseTimerA = millis() + 5000;  //used for launch max tracking
    timers.PhaseTimerB = millis() + 20;  //used for launch max tracking
    if(serialDebug) digitalWrite(led1_pin, HIGH); 
}

void PhaseLaunch() {

    if(millis() > timers.PhaseTimerA) {  // max variable tracking
        LSM6D.tiltMaxLaunch = LSM6D.tiltMax; // catch the max tilt for summary but not the apogee tilt
        LSM6D.launchMaxZ = LSM6D.maxZ; // capture maxZ before the boom
        adxl.launchMaxZ = adxl.maxZ; // capture maxZ before the boom
        timers.PhaseTimerA = millis() + 5000;
    }
 
    if(millis() > timers.PhaseTimerB) {  // check for events every 20ms

       if (baro.AltitudeMax > configs.altitudeMin && millis() > events.machClock) {  // wait until >500' and 6 sec to check everything
         
         // ******** TWO STAGE LOGIC **********
         if(configs.twoStage) {  
          //-------------  Arm Staging  ------------------------ 
          if(millis() < (events.launchClock + configs.stagingMaxTime) && baro.Speed > configs.stagingSpeed && events.burnout == 1) {
            events.stagingArmed = 1;
          } else {
            events.stagingArmed = 0;
          }

          //-------------  Staging Separation  ------------------------ 
          if(events.stagingArmed == 1 && millis() > (events.burnoutClock + configs.burnoutPlus) && events.pyroSeparation == 0 && adxl.velocity > 500) {  // do separation
            // Armed + burnout time + accel velocity>500 + pyro=0 
            events.separationClock = millis();
            if(configs.separation) pyroSeparation();
          }
          //-------------  Sustainer Ignition  ------------------------ 
          if(events.pyroSeparation == 1 && events.stagingArmed == 1 && events.pyroSustainer == 0) {
             // now check time and tilt
             if(millis() > (events.separationClock + configs.sustainerTime) && LSM6D.tilt < configs.stagingTilt && adxl.velocity > 500) {
               events.sustainerClock = millis();
               working.accelPre = adxl.avgZ;  // capture for 2nd Mach lockout
               pyroSustainer();
             }
          }

          //-------------  Check for a second lock-out  ------------------------ 
          if(events.pyroSustainer == 1 && adxl.avgZ > (working.accelPre + (float) 3000.0) && working.lockout2 == 0) { // We have successful ignition of sustainer, so lock out
             working.lockout2 = 1;
             events.machClock = millis() + configs.machLockout;
             strcpy(radioHeader, "@M");   strcpy(radioMessage, "Confirmed Sustainer Acceleration");
             RadioSend();  
          }
         } // ******** END TWO STAGE LOGIC **********


          //-------------  Standard Apogee Logic  ------------------------ 
           float testVelocity;
           /* Only using LSM6D velocity for apogee until ADXL can be resolved
            * if(adxl.maxZ < 31000) {  // use the LSM accelerometer below 31G
              testVelocity = LSM6D.velocity;
           } else {
              testVelocity = adxl.velocity;
           }
           */
           testVelocity = LSM6D.velocity;
           if(baro.Speed < configs.apogeeSpeed && testVelocity < configs.apogeeVelocity) {
            working.triggerCount2++;
            strcpy(working.eventString, "Standard Apogee Trigger"); eventLog(); 
            threads.delay(20);
            if(working.triggerCount2 == 5) { // increase to make 100ms in samples
              strcpy(working.eventString, "APOGEE STANDARD DETECT"); eventLog();    
              PhaseDescentInit();
              return;
            }
           }
  
          //-------------  BARO Backup Apogee Logic  ------------------------ 
           if(baro.Altitude < (baro.AltitudeMax - 300) && baro.Altitude <= baro.lastAltitude) { //sampling faster than baro
              working.triggerCount++;
              strcpy(working.eventString, "Barometer Apogee Trigger"); eventLog();    
              threads.delay(50); // add time to get another baro sample        
           } else {
              working.triggerCount = 0;
           }
           if(working.triggerCount == 3) {  // We have apogee!  
            strcpy(working.eventString, "APOGEE BAROMETER DETECT"); eventLog();    
            PhaseDescentInit();
            return;
           }
           baro.lastAltitude = baro.Altitude;
       } //end Max>500
      timers.PhaseTimerB = millis() + 20;
    } // end phase timerB event loop
} // End Launch Phase

// ==========================  PHASE DESCENT ===========================

void PhaseDescentInit() {

    if(configs.drogue) pyroDrogue(); //Fire Drogue at Apogee 
    events.apogeeClock = millis();
    events.ascentSeconds = (millis() - events.launchClock) / 1000;
    getTimeNow();
    strcpy(events.apogeeTime, working.timeHeader); //record apogee time
    events.apogee = 1;
    strcpy(working.phase,"DESCENT");           
    strcpy(radioHeader, "@E"); strcpy(radioMessage, "APOGEE");
    RadioSend();  
    timers.sendFlightTimer = millis(); //force radio flight update
    timers.flightLogInterval = 20;  // 40X per second for a while
    timers.sendFlightInterval = 2000; // every 2 seconds on descent
    timers.gpsLogInterval = 500;
    timers.gpsSendInterval = 4000;
    baro.lastAltitude = baro.Altitude;   
    if(serialDebug) Serial.println(F("APOGEE DETECT!"));    
    working.triggerCount = 0; //reset for next phase
    working.triggerCount2 = 0; 
    if(serialDebug) digitalWrite(led1_pin, LOW); 
    timers.PhaseTimerA = millis() + 20000;  //used for setting logging rate down post apogee
    timers.PhaseTimerB = millis() + 200;  // check for main deploy every 200ms
    timers.PhaseTimerC = millis() + 5000; // check for landed
}

void PhaseDescent() {

    //slow down log rate
    if(millis() > timers.PhaseTimerA) {
      //slow the logging down twenty seconds after apogee 40X to 10X
      timers.flightLogInterval = 100;
      timers.PhaseTimerA = millis() + 999000;
    }

    //-------------  Main Deploy Logic  ------------------------ 
    if(configs.main) {
      if(millis() > timers.PhaseTimerB && events.pyroMain == 0) {
        if(baro.Altitude < configs.mainAltitude && millis() > (events.launchClock + configs.machLockout)) {
          working.triggerCount++;
        } else {
          working.triggerCount = 0;
        }
        if(working.triggerCount == 3) {
          events.mainClock = millis();
          pyroMain();
        }
        timers.PhaseTimerB = millis() + 200;  
      }
    }
    

    //-------------  Landed Detect Logic  ------------------------    
    if(millis() > timers.PhaseTimerC) {

        if (baro.Altitude < (baro.lastAltitude + 10) && baro.Altitude > (baro.lastAltitude - 10)) {   // LANDED detect based on standing still barometer 
            working.triggerCount2++;
            strcpy(working.eventString, "Barometer Altitude Landed Trigger"); eventLog();  
        } else {
            baro.lastAltitude = baro.Altitude;
            working.triggerCount2 = 0;
            if(baro.Altitude != 0.0 && baro.Speed != 0.0) baro.LandedSpeed = baro.Speed;
        }
        if(working.triggerCount2 == 3) { // We have landed!
          PhaseLandedInit();
          return; 
        }
        timers.PhaseTimerC = millis() + 5000;
    }
}

// ==========================  PHASE LANDED ===========================

void PhaseLandedInit() {

    events.landedClock = millis();
    events.descentSeconds = (millis() - events.apogeeClock) / 1000;
    getTimeNow();
    strcpy(events.landedTime, working.timeHeader); //record apogee time
    events.landed = 1;
    strcpy(working.phase,"XLANDED");           
    strcpy(radioHeader, "@E"); strcpy(radioMessage, "LANDED");
    RadioSend();  
    timers.sendFlightTimer = millis(); //force radio flight update
    timers.flightLogInterval = 1000;  // log slow on the ground for recovery
    timers.sendFlightInterval = 10000; // every 10 seconds on the ground
    timers.gpsLogInterval = 2000;
    timers.gpsSendInterval = 5000;
    if(serialDebug) Serial.println(F("LANDED DETECT!"));    
    timers.PhaseTimerA = millis() + 6000;  //used for sending summary
    // log summary
    flightSummaryString();
    working.summaryWrite = 1;
    
}

void PhaseLanded() {

      if (millis() > timers.PhaseTimerA) {  // send the summary 
        summaryRepeater();
        timers.PhaseTimerA = millis() + 6000;  
      }     
      
}

// ==========================  PHASE BASIC ===========================

void PhaseBasicInit() {

   strcpy(working.phase, "BASIC");
   strcpy(radioHeader, "@M");   strcpy(radioMessage, "Hello. Now in BASIC mode...");
   RadioSend(); 
   timers.flightLogInterval = 500;  // log every half second in basic mode
   timers.sendFlightInterval = 4200; // every 5 seconds in basic mode
   timers.gpsLogInterval = 2000;
   timers.gpsSendInterval = 4000;
   timers.errorTimer = millis();

}

void PhaseBasic() {

  // nothing to do but hang out and log and send data back...
  
}

//***********************************************************        PYRO EVENTS      ************************************************************


void pyroSeparation() {
  
  if(events.pyroSeparation != 1) {
    timers.pyroSeparationTimer = millis() + configs.pyroDuration;
    digitalWrite(relaySeparation, HIGH);  // fire away!
    events.pyroSeparation = 1;
    strcpy(working.eventString, "Pyro Separation FIRED!"); eventLog(); 
    strcpy(radioHeader, "@M");   strcpy(radioMessage, "Pyro Separation FIRED!");
    RadioSend();    
  }

}

void pyroSustainer() {

  if(events.pyroSustainer != 1) {
    timers.pyroSustainerTimer = millis() + configs.pyroDuration;
    digitalWrite(relaySustainer, HIGH);  // fire away!
    events.pyroSustainer = 1;
    //events.machClock = millis() + configs.machLockout;  // reset the mach lock-out for sustainer ignition phase
    strcpy(working.eventString, "Pyro Sustainer FIRED!"); eventLog(); 
    strcpy(radioHeader, "@M");   strcpy(radioMessage, "Pyro Sustainer FIRED!");
    RadioSend();    
  }

}


void pyroDrogue() {
  
  if(events.pyroDrogue != 1) {
    timers.pyroDrogueTimer = millis() + configs.pyroDuration;
    digitalWrite(relayDrogue, HIGH);  // fire away!
    events.pyroDrogue = 1;
    strcpy(working.eventString, "Pyro Drogue FIRED!"); eventLog(); 
    strcpy(radioHeader, "@M");   strcpy(radioMessage, "Pyro Drogue FIRED!");
    RadioSend();    
  }

}

void pyroMain() {
    if(events.pyroMain != 1) {
      timers.pyroMainTimer = millis() + configs.pyroDuration;
      digitalWrite(relayMain, HIGH);  // fire away!
      events.pyroMain = 1;
      strcpy(working.eventString, "Pyro Main FIRED!"); eventLog(); 
      strcpy(radioHeader, "@M");   strcpy(radioMessage, "Pyro Main FIRED!");
    RadioSend();    
  }

}

void pyroBoom() {  //FTS
  // DANGER!!  EMERGENCY DETONATE OF ALL EVENT SEPARATION PYROS!
  // Separation, Main, Drogue in that order

  if(configs.separation) { pyroSeparation(); threads.delay(500);}
  pyroMain();
  threads.delay(1000);
  pyroDrogue();
  strcpy(radioHeader, "@M");   strcpy(radioMessage, "Confirmed all FTS Pyro Events");
  strcpy(working.eventString, "Confirmed all BOOM pyro events "); eventLog(); 
  RadioSend();   
}

void pyroTimers() {

  if(timers.pyroMainTimer != 0 && millis() > timers.pyroMainTimer) { //shut off main pyro
    digitalWrite(relayMain, LOW);
    timers.pyroMainTimer = 0;
  }
  if(timers.pyroSeparationTimer != 0 && millis() > timers.pyroSeparationTimer) { //shut off separation pyro
    digitalWrite(relaySeparation, LOW);
    timers.pyroSeparationTimer = 0;
  }
  if(timers.pyroSustainerTimer != 0 && millis() > timers.pyroSustainerTimer) { //shut off sustainer pyro
    digitalWrite(relaySustainer, LOW);
    timers.pyroSustainerTimer = 0;
  }
  if(timers.pyroDrogueTimer != 0 && millis() > timers.pyroDrogueTimer) { //shut off drogue pyro
    digitalWrite(relayDrogue, LOW);
    timers.pyroDrogueTimer = 0;
  }
}

void pyroContinuity() {


    if(digitalRead(continuityMainPin) == LOW && events.continuityMain == 0) {
      events.continuityMain = 1;
      strcpy(working.eventString, "Main Continuity Closed"); eventLog(); 
    }
    if(digitalRead(continuityMainPin) == HIGH && events.continuityMain == 1) {
      events.continuityMain = 0;
      strcpy(working.eventString, "Main Continuity Open"); eventLog(); 
    }  

    if(digitalRead(continuityDroguePin) == LOW && events.continuityDrogue == 0) {
        events.continuityDrogue = 1;
        strcpy(working.eventString, "Drogue Continuity Closed"); eventLog(); 
        if(serialDebug) Serial.println("Drogue Continuity CLOSED"); 
    }
    if(digitalRead(continuityDroguePin) == HIGH && events.continuityDrogue == 1) {
      events.continuityDrogue = 0;
      strcpy(working.eventString, "Drogue Continuity Open"); eventLog(); 
    }  

    if(digitalRead(continuitySeparationPin) == LOW && events.continuitySeparation == 0) {
      events.continuitySeparation = 1;
      strcpy(working.eventString, "Separation Continuity Closed"); eventLog(); 
    }
    if(digitalRead(continuitySeparationPin) == HIGH && events.continuitySeparation == 1) {
      events.continuitySeparation = 0;
      strcpy(working.eventString, "Separation Continuity Open"); eventLog(); 
    } 
 
    if(digitalRead(continuitySustainerPin) == LOW && events.continuitySustainer == 0) {
      events.continuitySustainer = 1;
      strcpy(working.eventString, "Sustainer Continuity Closed"); eventLog(); 
    }
    if(digitalRead(continuitySustainerPin) == HIGH && events.continuitySustainer == 1) {
      events.continuitySustainer = 0;
      strcpy(working.eventString, "Sustainer Continuity Open"); eventLog(); 
    }        
}


//***********************************************************        DATA LOGGING THREAD    ************************************************************
//***********************************************************        DATA LOGGING THREAD     ************************************************************
//***********************************************************        DATA LOGGING THREAD     ************************************************************


void logging_thread() {

  unsigned long eventLogTimer =0;
  unsigned long gpsLogTimer = 0;
  unsigned long radioLogTimer = 0;
  unsigned long flushTimer = 0;
  unsigned long flushTimer2 = 0;
  unsigned long flushTimer3 = 0;
  elapsedMillis checkIn = 0;

  //Initialize the flash memory
  if(serialDebug) Serial.println(F("Initializing Flash Memory"));
  if (!myfs.begin()) {
        if(serialDebug) Serial.println(F("Flash Mount Failed"));
        errors.flash = 1;
        if(serialDebug) digitalWrite(led3_pin,HIGH);
  } else {
    if(serialDebug) Serial.println(F("Flash Mount Success"));
     errors.flash = 2; //healthy
     getLaunchNum();
     startLogs(); // start the log files and open the files
  }
  
  threads.delay(4000); // wait for sampling to start
  if(serialDebug) Serial.println(">>> Starting Data Logging Loop...");

  // ============  LOGGING MAIN LOOP  =============== //
  while(1) { 
  
   if(configs.logging && errors.flash != 1) {
   
   // ============  LOG FLUSH (PHYSICALLY SAVE)  =============== //
   if(millis() > flushTimer) {
     FlightFile.flush();
     // also check for flash space while working
     unsigned long bFree = 0;
     bFree = myfs.totalSize() - myfs.usedSize();
     if(bFree < 30000) {
       strcpy(working.eventString,"Flash Low Disk Space Error");
       eventLog();
     }
     if(bFree < 15000) { //shut down due to low space
       FlightFile.println("SD CARD IS FULL. STOPPING LOG");
       FlightFile.flush();
       errors.flash = 1;
       configs.logging = false;
     }
     flushTimer = millis() + configs.logFlush; // physically save every second
   }

   // Staggering the log flush to not cause long gaps in flight data (can take 200ms to flush)
   // ============  LOG FLUSH FOR RADIO (PHYSICALLY SAVE)  =============== //
   if(millis() > flushTimer2) {
     RadioFile.flush();
     flushTimer2 = millis() + 8300; // physically save every 8.3 seconds radio
     threads.yield();
   }
   
   // ============  LOG FLUSH FOR GPS (PHYSICALLY SAVE)  =============== //
   if(millis() > flushTimer3) {
     GPSFile.flush();
     flushTimer3 = millis() + 5150; // physically save every 5.15 seconds
     threads.yield();
   }
   
   
   //============  ERASE FLASH COMMAND  =============== //
   if(working.eraseFlash == 1) {
    working.eraseFlash = 0;
    threads.delay(5);
    myfs.quickFormat();
    //myfs.lowLevelFormat();
    threads.delay(50);
    startLogs();
    strcpy(working.eventString, ""); strcat(working.eventString, "Flash format complete");         
    eventLog(); 
    if(serialDebug) Serial.println(F("Flash format complete"));
    strcpy(radioHeader, "@M");
    strcpy(radioMessage, "Flash Format Complete");
    RadioSend();    
     
   }

   // ============  FLIGHT LOG REGULAR LOOP  =============== //
   if(millis() > timers.flightLogTimer) {
     if(debugMode) debugVars.logCount++;
     flightLogString();
     flightLog();
     timers.flightLogTimer = millis() + timers.flightLogInterval;
     threads.yield();
   }

   // ============  FLIGHT EVENT QUEUE LOOP  =============== //
   if(millis() > eventLogTimer) {
     if(working.eLogCount > 0) {
       while(working.eLogCount > 0) {
        working.eLogLast++;
        if(working.eLogLast == 11) working.eLogLast = 1;
        strcpy(working.FlightLogMaster, working.eventLogQ[working.eLogLast]);
        flightLog();
        working.eLogCount--;
       }
     }
     eventLogTimer = millis() + timers.eventLogInterval; // check queue every 200ms
   }

   // ============  GPS QUEUE LOOP  =============== //
   if(millis() > gpsLogTimer) {
     if(working.gpsLogCount > 0) {
       while(working.gpsLogCount > 0) {
        working.gpsLogLast++;
        if(working.gpsLogLast == 11) working.gpsLogLast = 1;
        strcpy(gpsData.gpsLogLine, working.gpsLogQ[working.gpsLogLast]);
        GPSLog();
        working.gpsLogCount--;
       }
     }
     gpsLogTimer = millis() + timers.gpsLogInterval;
   }

   // ============  RADIO QUEUE LOOP  =============== //
   if(millis() > radioLogTimer) {
     if(working.radioLogCount > 0) {
       while(working.radioLogCount > 0) {
        working.radioLogLast++;
        if(working.radioLogLast == 11) working.radioLogLast = 1;
        strcpy(working.RadioLogMaster, working.radioLogQ[working.radioLogLast]);
        radioLog();
        working.radioLogCount--;
       }
     }
     radioLogTimer = millis() + timers.radioLogInterval;
   }

   if(working.summaryWrite == 1) SummaryLog();  
   
  } // == END IF LOGGING and ERROR ===
     if (checkIn > 250) { 
          working.loggingClock = millis(); //heartbeat clock to make sure thread is alive
          checkIn = 0;
     }
     threads.yield();
 } // == WHILE
} //== END LOGGING THREAD ==




void startLogs() {

// open the files (for performance they never close)
     //Flight
     char theFileName[50];
     strcpy(theFileName,"/flight");
     strcat(theFileName, working.LaunchNumberStr);
     strcat(theFileName, ".txt"); 
     FlightFile.close(); // just in case it was open before
     FlightFile = myfs.open(theFileName, FILE_WRITE);
     // GPS
     strcpy(theFileName,"/gps");
     strcat(theFileName, working.LaunchNumberStr);
     strcat(theFileName, ".txt"); 
     GPSFile.close();
     GPSFile = myfs.open(theFileName, FILE_WRITE);
     //Radio 
     strcpy(theFileName,"/radio");
     strcat(theFileName, working.LaunchNumberStr);
     strcat(theFileName, ".txt"); 
     RadioFile.close();
     RadioFile = myfs.open(theFileName, FILE_WRITE);

//initialize all log files 
     strcpy(working.FlightLogMaster, "====================== NEW FLIGHT LOG INITIATED =============================="); 
     flightLog();
     strcpy(working.FlightLogMaster, "phase, T-clk, alt, alt-max, speed, speed-max, temp, temp max, bLaunch, bApogee, temp-raw, pressure, alt-raw, ADXL, adxl.x, adxl.y, adxl.z, adxl.avgZ, adxl.linZ, adxl.maxZ, adxl.minZ, adxl.vel, adxl.burnoutArm,adxl.burnout, adxl.apogee, LSM6D-A, lsm.x, lsm.y, lsm.z, lsm.avgZ, lsm.linZ, lsm.maxZ, lsm.minZ, lsm.vel, lsm.burnoutArm,lsm.burnout, lsm.apogee, LSM-Gyro,lsm.gX, lsm.gY, lsm.gZ, lsm.yaw, lsm.pitch, lsm.roll, lsm.tilt, lsm.tiltMax, EVENTS, voltage, launch, burnout, apogee, fore, aft, sustainer, pyro-sep, pyro-sus, pyro-main, pyro-drogue, contMain, contDrogue, contSep, contSust, StagArmed, CPU temp, CPU tempMax, ERRORS, SampleLoop, LoggingLoop, ADXL, LSM, MS56, GPS, Flash, voltage, CPUtemp, I2C,crash");
     flightLog();
     strcpy(working.RadioLogMaster, "====================== NEW RADIO LOG INITIATED =============================="); 
     radioLog();   
     strcpy(gpsData.gpsLogLine,"====================== NEW GPS LOG INITIATED =============================="); 
     GPSLog();
     strcpy(gpsData.gpsLogLine,"date,time, fix, quality, lat, long, ASL alt(F), speed(knots), angle, Satellites, friendly lat, friendly lon, AGL MaX f, altitude AGL-f"); 
     GPSLog();  

}


void eventLog() {  //Creates event log queue
    //set working.eventString before calling
    if(configs.logging) {
      working.eLogCount ++;
      working.eLogPos ++;
      if (working.eLogPos == 11) working.eLogPos = 1;
      strcpy(working.eventLogQ[working.eLogPos], working.eventString);  
    }
}


void flightLog() {
  // set FlightLogMaster before calling  
     getTimeHeader();  
     FlightFile.print(working.timeHeader);
     FlightFile.println(working.FlightLogMaster);
     threads.yield();
   
} 

void gpsLogQ() {  //Creates event log queue
    //set working.gpsString before calling the queue
    working.gpsLogCount++;
    working.gpsLogPos++;
    if (working.gpsLogPos == 11) working.gpsLogPos = 1;
    strcpy(working.gpsLogQ[working.gpsLogPos], working.gpsString);  
}

void GPSLog() {
  // set gpsLogLine before calling  
   getTimeHeader();
   GPSFile.print(working.timeHeader);
   GPSFile.println(gpsData.gpsLogLine);
   working.gpsLogOpen = 0;
   threads.yield();
}  

void radioLogQ() {  //Creates event log queue
    //set working.radioString before calling the queue
    working.radioLogCount ++;
    working.radioLogPos ++;
    if (working.radioLogPos == 11) working.radioLogPos = 1;
    strcpy(working.radioLogQ[working.radioLogPos], working.radioString);  
}

void radioLog() {
  // set RadioLogMaster before calling  
   getTimeHeader();
   RadioFile.print(working.timeHeader);
   RadioFile.println(working.RadioLogMaster);
   
   working.radioLogOpen = 0;
   threads.yield();
} 

void SummaryLog() {
  // set FlightSummaryMaster before calling  
   char theFileName[50] = "/sum";
   strcat(theFileName, working.LaunchNumberStr);
   strcat(theFileName, ".txt"); 
   SummaryFile = myfs.open(theFileName, FILE_WRITE);
   SummaryFile.println("launch number, launch time, apogee time, landed time, aft event time, fore event time, ascent seconds, descent seconds, burnout seconds, AGL Baro, AGL GPS, AGL Accel, Launch Accel Max, Max temp, Max tilt, Batt voltage, Speed baro fps, mps, mph, SpeedA mps, Descent rate fps, landed rate fps, Landed GPS coord");
   getTimeHeader();
   SummaryFile.print(working.timeHeader);
   SummaryFile.println(working.FlightSummaryMaster);
   SummaryFile.close();
   working.summaryWrite = 0;  
   threads.yield();
   
} 


//==========================================================================  LOW MEM SPRINTF FUNCTIONS
// Note:  SprintF is not thread safe (big issues) with Teensy 4.1, so these functions were writen to replace SprintF


void iToStr(int x, char res[20], int d)
{
    if(x == 0) {
      strcpy(res,"0");
      if(d==2) strcpy(res,"00");
      if(d==3) strcpy(res,"000");
      return;
    }
    bool neg = false;
    if(x < 0) {
      x = x * -1;
      neg = true;
    }
    
    int i = 0;
    while (x) {
        res[i++] = (x % 10) + '0';
        x = x / 10;
    }
     while (i < d)
        res[i++] = '0';
    reverse(res, i);
    res[i] = '\0';
    if(neg) {
      char temp1[20];
      strcpy(temp1,"-");
      strcat(temp1,res);
      strcpy(res,temp1);
    }
    
}

void bToStr(byte x, char res[20], int d)
{
    if(x == 0) {
      strcpy(res,"0");
      return;
    }
  
    int i = 0;
    while (x) {
        res[i++] = (x % 10) + '0';
        x = x / 10;
    }
     while (i < d)
        res[i++] = '0';
    reverse(res, i);
    res[i] = '\0';
    
}

void ulongToStr(unsigned long x, char res[20], int d)
{
    if(x == 0) {
      strcpy(res,"0");
      if(d==2) strcpy(res,"00");
      if(d==3) strcpy(res,"000");
      return;
    }
    int i = 0;
    while (x) {
        res[i++] = (x % 10) + '0';
        x = x / 10;
    }
    while (i < d)
        res[i++] = '0';
    reverse(res, i);
    res[i] = '\0';
}

void fToStr(float n, char res[20], int afterpoint)
{
    bool neg = false;
    if(n < 0) {
      n = n * -1;
      neg = true;
    }
    int ipart = (int)n;
    int i = 0;
    float fpart = n - (float)ipart;
    if(ipart == 0) {
      strcpy(res,"0");
      i = 1;
    } else {
      i = intToStr(ipart, res, 0);
    }
    if (afterpoint != 0) {
        res[i] = '.';
        fpart = fpart * pow(10, afterpoint);
        intToStr((int)fpart, res + i + 1, afterpoint);
    }
    if(neg) {
      char temp1[20];
      strcpy(temp1,"-");
      strcat(temp1,res);
      strcpy(res,temp1);
    }
    
}

void reverse(char* str, int len)
{
    int i = 0, j = len - 1, temp;
    while (i < j) {
        temp = str[i];
        str[i] = str[j];
        str[j] = temp;
        i++;
        j--;
    }
}
 
int intToStr(int x, char str[], int d) //only used for the float function
{
    int i = 0;
    while (x) {
        str[i++] = (x % 10) + '0';
        x = x / 10;
    }
    while (i < d)
        str[i++] = '0';
    reverse(str, i);
    str[i] = '\0';
    return i;
}


//==========================================================================


void fCatI(int tempI) {
  char tempS[20];
  iToStr(tempI, tempS, 0);
  strcat(working.FlightLogMaster, tempS);
  strcat(working.FlightLogMaster, ",");
}
void fCatF(float tempF,int dPoints) {
  char tempS[20];
  fToStr(tempF, tempS, dPoints);
  strcat(working.FlightLogMaster, tempS);
  strcat(working.FlightLogMaster, ",");
}
void fCatU(unsigned long tempU) {
  char tempS[20];
  ulongToStr(tempU, tempS, 0);
  strcat(working.FlightLogMaster, tempS);
  strcat(working.FlightLogMaster, ",");
}
void fCatB(byte tempB) {
  char tempS[20];
  bToStr(tempB, tempS, 0);
  strcat(working.FlightLogMaster, tempS);
  strcat(working.FlightLogMaster, ",");
}
void fCatS(char tempS[20]) {
  strcat(working.FlightLogMaster, tempS);
  strcat(working.FlightLogMaster, ",");
}



void flightLogString() {

  char labelString[20];
  strcpy(working.FlightLogMaster, "");  //zero out the string
  //------------ BASICS ------------------
  fCatS(working.phase);
  float tempTime = 0.0;
  if(events.launch == 1) tempTime = (float)(millis() - events.launchClock)/(float) 1000.0; 
  fCatF(tempTime,3);  
  //------------ BARO ------------------
  fCatF(baro.Altitude,0);
  fCatF(baro.AltitudeMax,0);
  fCatF(baro.Speed,0);
  fCatF(baro.SpeedMax,0);
  fCatF(baro.TempF,0);
  fCatF(baro.TempMax,0);
  fCatI(baro.launch);
  fCatI(baro.apogee);
  fCatF(baro.Tempraw,8);
  fCatF(baro.Pressure,8);
  fCatF(baro.Altraw,4);
  //------------ ADXL ------------------  
  strcpy(labelString, "ADXL:");
  fCatS(labelString);
  fCatF(adxl.x,0);
  fCatF(adxl.y,0);
  fCatF(adxl.z,0);
  if(adxl.count > 0) {
    adxl.avgZ = (adxl.tempAvgZ / (float) adxl.count);
    adxl.count = 0;
    adxl.tempAvgZ = 0.0;
  } else {
    adxl.avgZ = adxl.z;
  }
  fCatF(adxl.avgZ,0);
  fCatF(adxl.linZ,0);
  fCatF(adxl.maxZ,0);
  fCatF(adxl.minZ,0);
  fCatF(adxl.velocity,0);
  fCatI(adxl.burnoutArm);
  fCatI(adxl.burnout);
  fCatI(adxl.apogee);
  //------------ LSM6D ACCEL ------------------  
  strcpy(labelString, "LSM-A:");
  fCatS(labelString);
  fCatF(LSM6D.x,0);
  fCatF(LSM6D.y,0);
  fCatF(LSM6D.z,0);
  if(LSM6D.count > 0) {
    LSM6D.avgZ = (LSM6D.tempAvgZ / (float) LSM6D.count);
    LSM6D.count = 0;
    LSM6D.tempAvgZ = 0.0;
  } else {
    LSM6D.avgZ = LSM6D.z;
  }
  fCatF(LSM6D.avgZ,0);
  fCatF(LSM6D.linZ,0);
  fCatF(LSM6D.maxZ,0);
  fCatF(LSM6D.minZ,0);
  fCatF(LSM6D.velocity,0);
  fCatI(LSM6D.burnoutArm);
  fCatI(LSM6D.burnout);
  fCatI(LSM6D.apogee);
  //------------ LSM6D GYRO ------------------
  strcpy(labelString, "LSM-G");
  fCatS(labelString);
  fCatF(LSM6D.gX,4);
  fCatF(LSM6D.gY,4);
  fCatF(LSM6D.gZ,4);
  fCatF(LSM6D.yaw,0);
  fCatF(LSM6D.pitch,0);
  fCatF(LSM6D.roll,0);
  fCatF(LSM6D.tilt,1);
  fCatF(LSM6D.tiltMax,1);
  //------------ EVENTS  ------------------
  strcpy(labelString, "EVENTS:");
  fCatS(labelString);
  fCatF(working.voltage,0);
  fCatI(events.launch);
  fCatI(events.burnout);
  fCatI(events.apogee);
  fCatI(events.EventFore);
  fCatI(events.EventAft);
  fCatI(events.EventSep);
  fCatI(events.pyroSeparation);
  fCatI(events.pyroSustainer);
  fCatI(events.pyroMain);
  fCatI(events.pyroDrogue);
  fCatI(events.continuityMain);
  fCatI(events.continuityDrogue);
  fCatI(events.continuitySeparation);
  fCatI(events.continuitySustainer);
  fCatI(events.stagingArmed);
  fCatF(working.CPUtemp,0);
  fCatF(working.CPUtempMax,0);
  //------------ ERRORS and HEALTH  ------------------  1=error, 2=healthy, 0=not initiated
  strcpy(labelString, "ERRORS:");
  fCatS(labelString);
  fCatI(errors.sampleLoop);
  fCatI(errors.loggingLoop);
  fCatI(errors.ADXL);
  fCatI(errors.LSM);
  fCatI(errors.MS56);
  fCatI(errors.GPS);
  fCatI(errors.flash);
  fCatI(errors.voltage);
  fCatI(errors.CPUtemp);
  fCatI(errors.I2C);
  fCatI(errors.crash);  
}




void flightRadioString() {
  
//Send 4.0 version:  phase(1char), alt, alt max, speed, speed max, accel-z, tilt, tilt max, temp, max temp, fore, aft, sep, voltage,
//                   pyroMain,pyroDrogue,pyroSustainer,pyroSeparation,continuityMain,continuityDrogue,continuitySustainer,continuitySeparation

  char comma[5] = ",";
  char tempstr[25];
  char tempS[20];
  int tempi = 0;
  float tempf = 0.0;
  
  strcpy(working.sendFlight, "");  //zero out the string
  strcpy(tempstr, working.phase);
  tempstr[1] = '\0'; //truncate the phase (poorman style)
  strcat(working.sendFlight, tempstr);strcat(working.sendFlight, comma); //only use the first letter

  tempf = baro.Altitude;
  strcpy(tempS,"");fToStr(tempf, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);
  tempf = baro.AltitudeMax;
  strcpy(tempS,"");fToStr(tempf, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);
  tempf = baro.Speed;
  strcpy(tempS,"");fToStr(tempf, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);
  tempf = baro.SpeedMax;
  strcpy(tempS,"");fToStr(tempf, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);
  if(adxl.maxZ < 31000) { // use LSM Accel if below 31G
     tempf = LSM6D.linZ;     
  } else {
     tempf = adxl.linZ;
  }
  strcpy(tempS,"");fToStr(tempf, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);
  if(adxl.maxZ < 31000) { // use LSM Accel if below 31G
     tempf = LSM6D.maxZ;     
  } else {
     tempf = adxl.maxZ;
  }
  strcpy(tempS,"");fToStr(tempf, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);
  tempf = LSM6D.tilt;
  strcpy(tempS,"");fToStr(tempf, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);
  tempf = LSM6D.tiltMax;
  strcpy(tempS,"");fToStr(tempf, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);
  tempf = working.CPUtemp;
  strcpy(tempS,"");fToStr(tempf, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);
  tempf = working.CPUtempMax;
  strcpy(tempS,"");fToStr(tempf, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);
  tempi = events.EventFore;
  strcpy(tempS,"");iToStr(tempi, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);
  tempi = events.EventAft;
  strcpy(tempS,"");iToStr(tempi, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);
  tempi = events.EventSep;
  strcpy(tempS,"");iToStr(tempi, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);
  tempf = working.voltage;
  strcpy(tempS,"");fToStr(tempf, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);  
  tempi = events.pyroMain;
  strcpy(tempS,"");iToStr(tempi, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);
  tempi = events.pyroDrogue;
  strcpy(tempS,"");iToStr(tempi, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);  
  tempi = events.pyroSustainer;
  strcpy(tempS,"");iToStr(tempi, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);  
  tempi = events.pyroSeparation;
  strcpy(tempS,"");iToStr(tempi, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);  
  tempi = events.continuityMain;
  strcpy(tempS,"");iToStr(tempi, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);  
  tempi = events.continuityDrogue;
  strcpy(tempS,"");iToStr(tempi, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);  
  tempi = events.continuitySustainer;
  strcpy(tempS,"");iToStr(tempi, tempS, 0);
  strcat(working.sendFlight, tempS);strcat(working.sendFlight, comma);  
  tempi = events.continuitySeparation;
  strcpy(tempS,"");iToStr(tempi, tempS, 0);
  strcat(working.sendFlight, tempS);   

}


void getTimeHeader(){
  //return is in working.timeHeader
  
  strcpy(working.timeHeader, "");
  if(events.clockSet == 1) {
    const char *slash = "/"; const char *dash = "-"; const char *colon = ":"; 
    char tempS[20];
    int tempI = 0;
    tempI = month();
    strcpy(tempS,"");iToStr(tempI, tempS, 2);
    strcat(working.timeHeader, tempS);
    strcat(working.timeHeader, slash);     
    tempI = day();
    strcpy(tempS,"");iToStr(tempI, tempS, 2);
    strcat(working.timeHeader, tempS);
    strcat(working.timeHeader, slash); 
    tempI = year();
    strcpy(tempS,"");iToStr(tempI, tempS, 0);
    strcat(working.timeHeader, tempS);
    strcat(working.timeHeader, dash); 
    tempI = hour();
    strcpy(tempS,"");iToStr(tempI, tempS, 2);
    strcat(working.timeHeader, tempS);
    strcat(working.timeHeader, colon); 
    tempI = minute();
    strcpy(tempS,"");iToStr(tempI, tempS, 2);
    strcat(working.timeHeader, tempS);
    strcat(working.timeHeader, colon);
    tempI = second();
    strcpy(tempS,"");iToStr(tempI, tempS, 2);
    strcat(working.timeHeader, tempS);  
    //add milliseconds
    char buf[16];
    ltoa(millis(),buf,10);
    char daMilli[5] = {buf[strlen(buf)-3],buf[strlen(buf)-2],buf[strlen(buf)-1],'\0'};
    strcat(working.timeHeader,".");
    strcat(working.timeHeader, daMilli);
    strcat(working.timeHeader, ",");             
    } else {
       //just use the full milliseconds
       char buf[16];
       ltoa(millis(),buf,10);
       strcpy(working.timeHeader, "ms");
       strcat(working.timeHeader, buf);
       strcat(working.timeHeader, ",");     
    }    
}


void getLaunchNum() {
  // read the current launch number and increment it from disk
  int thevalue;
  char buf[1];
  if(serialDebug) Serial.println(F("Getting Launch Number"));
  
  File sdFile; 
  strcpy(buf,"");
  sdFile = myfs.open("launch.txt", FILE_READ);
  int fx = 0;
  if(serialDebug) Serial.print("Launch.txt file: ");
  if(sdFile.available()) {
    sdFile.read(buf,1);
    if(serialDebug) Serial.print(buf);
   // theline[0] = buf;
   // theline[1] = '\n';
   // theline[2] = '\0';
    fx++;
  }
  sdFile.close();
  if(fx > 0) {
    if(serialDebug) Serial.print(F(" value: "));if(serialDebug) Serial.println(atoi(buf));
    if(serialDebug) Serial.print(F("Last Launch Number: "));
    if(serialDebug) Serial.println(buf);
    thevalue = atoi(buf);
  } else {
    if(serialDebug) Serial.println(F("error opening launch.txt"));
    thevalue = 0;
  }
  if (errors.crash == 2) { //nominal
    if (thevalue == 9) thevalue = 0; // rotate through 9 launch files to save space
    working.LaunchNumber = thevalue + 1;  //increment from last launch
    strcpy(working.LaunchNumberStr, "");
    int tempi = working.LaunchNumber;
    char tempS[20];
    strcpy(tempS,"");iToStr(tempi, tempS, 0);
    strcat(working.LaunchNumberStr, tempS);
    // now overwrite the last launch.txt value 
  
    myfs.remove("launch.txt");
    sdFile = myfs.open("launch.txt", FILE_WRITE);
    sdFile.println(working.LaunchNumber);    
    sdFile.close();
  } else {
    working.LaunchNumber = thevalue;  //we crashed so keep the old launch num
    strcpy(working.LaunchNumberStr, "");
    int tempi = working.LaunchNumber;
    char tempS[20];
    strcpy(tempS,"");iToStr(tempi, tempS, 0);
    strcat(working.LaunchNumberStr, tempS);   
  }
  
  if(serialDebug) Serial.print(F("New Launch Number: "));
  if(serialDebug) Serial.println(working.LaunchNumber);
} 

void ltrim(char *src)
{
    char *dst;
        /* find position of first non-space character */
    for (dst=src; *src == ' '; src++) {;}
        /* nothing to do */
    if (dst==src) return;
        /* K&R style strcpy() */
    while ((*dst++ = *src++)) {;}
    return;
}



void flightSummaryString() {

//launch number, launch time, apogee time, landed time, aft event time, fore event time, ascent seconds, descent seconds, burnout seconds, AGL Baro, AGL GPS, AGL Accel, Launch Accel Max, Max temp, Max tilt, Batt voltage, Speed baro fps, mps, mph, SpeedA mps, Descent rate fps, landed rate fps, Landed GPS coord

  char comma[5] = ",";
  char str_int[16];

  strcpy(working.FlightSummaryMaster, "");  //zero out the string
  strcat(working.FlightSummaryMaster, working.LaunchNumberStr);strcat(working.FlightSummaryMaster, comma);  //launch number
  strcat(working.FlightSummaryMaster, events.launchTime);strcat(working.FlightSummaryMaster, comma);  //launch time  
  strcat(working.FlightSummaryMaster, events.apogeeTime);strcat(working.FlightSummaryMaster, comma);  //apogee time    
  strcat(working.FlightSummaryMaster, events.landedTime);strcat(working.FlightSummaryMaster, comma);  //landed time  
  if(strcmp(events.aftTime, "") == 0) strcpy(events.aftTime, "none");
  strcat(working.FlightSummaryMaster, events.aftTime);strcat(working.FlightSummaryMaster, comma);  //aft time  
  if(strcmp(events.foreTime, "") == 0) strcpy(events.foreTime, "none");
  strcat(working.FlightSummaryMaster, events.foreTime);strcat(working.FlightSummaryMaster, comma);  //fore time  

  strcpy(str_int, "");sprintf (str_int, "%d" , events.ascentSeconds);
  strcat(working.FlightSummaryMaster, str_int);strcat(working.FlightSummaryMaster, comma);  //ascent seconds
  strcpy(str_int, "");sprintf (str_int, "%d" , events.descentSeconds);
  strcat(working.FlightSummaryMaster, str_int);strcat(working.FlightSummaryMaster, comma);  //descent seconds  
  strcpy(str_int, "");sprintf (str_int, "%1.1f" , events.burnoutSeconds);
  strcat(working.FlightSummaryMaster, str_int);strcat(working.FlightSummaryMaster, comma);  //placeholder for burnout seconds
  strcpy(str_int, "");sprintf (str_int, "%1.0f" , baro.AltitudeMax);
  strcat(working.FlightSummaryMaster, str_int);strcat(working.FlightSummaryMaster, comma);  //baro max altitude
  strcpy(str_int, "");sprintf (str_int, "%1.0f" , gpsData.maxAGLfeet);
  strcat(working.FlightSummaryMaster, str_int);strcat(working.FlightSummaryMaster, comma);  //GPS max altitude (AGL)
  strcat(working.FlightSummaryMaster, "n/a");strcat(working.FlightSummaryMaster, comma);  //placeholder for Accel altitude AGL
  strcpy(str_int, "");sprintf (str_int, "%1.0f" , adxl.launchMaxZ);
  strcat(working.FlightSummaryMaster, str_int);strcat(working.FlightSummaryMaster, comma);  //launch Accel Max g's
  strcpy(str_int, ""); sprintf (str_int, "%1.0f" , baro.TempMax); ltrim(str_int);    
  strcat(working.FlightSummaryMaster, str_int);strcat(working.FlightSummaryMaster, comma); // max temp
  strcpy(str_int, ""); sprintf (str_int, "%1.0f" , LSM6D.tiltMaxLaunch);    
  strcat(working.FlightSummaryMaster, str_int);strcat(working.FlightSummaryMaster, comma); // max tilt    
  strcpy(str_int, ""); sprintf (str_int, "%1.0f" , working.voltage);
  strcat(working.FlightSummaryMaster, str_int);strcat(working.FlightSummaryMaster, comma);  // voltage A 
  //strcpy(str_int, ""); sprintf (str_int, "%3.2f" , vinB);
  //strcat(working.FlightSummaryMaster, str_int);strcat(working.FlightSummaryMaster, comma); // volatage B
  strcpy(str_int, ""); sprintf (str_int, "%1.0f" , baro.SpeedMax);
  strcat(working.FlightSummaryMaster, str_int);strcat(working.FlightSummaryMaster, comma); // baro speed in FPS
  float tempcalc = 0.0;
  tempcalc = baro.SpeedMax * 0.3048;
  strcpy(str_int, ""); sprintf (str_int, "%1.0f" , tempcalc);
  strcat(working.FlightSummaryMaster, str_int);strcat(working.FlightSummaryMaster, comma); // baro speed in MPS
  tempcalc = baro.SpeedMax * 0.681818;
  strcpy(str_int, ""); sprintf (str_int, "%1.0f" , tempcalc);
  strcat(working.FlightSummaryMaster, str_int);strcat(working.FlightSummaryMaster, comma); // baro speed in MPH    
  strcat(working.FlightSummaryMaster, "n/a");strcat(working.FlightSummaryMaster, comma);  //placeholder for Accel speed
  tempcalc = baro.AltitudeMax / (float)events.descentSeconds;
  tempcalc = tempcalc * -1;
  strcpy(str_int, ""); sprintf (str_int, "%1.0f" , tempcalc);
  strcat(working.FlightSummaryMaster, str_int);strcat(working.FlightSummaryMaster, comma); // average descent rate in FPS
  strcpy(str_int, ""); sprintf (str_int, "%1.0f" , baro.LandedSpeed);
  strcat(working.FlightSummaryMaster, str_int);strcat(working.FlightSummaryMaster, comma); // landed speed in FPS
  // put map friendly GPS coord here
  strcat(working.FlightSummaryMaster, gpsData.latFriendly);strcat(working.FlightSummaryMaster, comma); //GPS lat friendly
  strcat(working.FlightSummaryMaster, gpsData.longFriendly); //GPS long friendly  
  
  //Now do the radio sentences 
  
   //S1:  @S1,#,launch number, launch time, apogee time, landed time, aft event time, fore event time, ascent seconds, descent seconds, 
   //S2:  @S2,#,burnout seconds, AGL Baro, AGL GPS, AGL Accel, Launch Accel Max, Max temp, Max tilt, Batt voltage, 
   //S3:  @S3,#,Speed baro fps, mps, mph, SpeedA mps, Descent rate fps, landed rate fps, Landed GPS coord

  strcpy(working.sendSummary1, "");  //zero out the string
  strcat(working.sendSummary1, working.LaunchNumberStr);strcat(working.sendSummary1, comma);  //launch number
  strcat(working.sendSummary1, events.launchTime);strcat(working.sendSummary1, comma);  //launch time  
  strcat(working.sendSummary1, events.apogeeTime);strcat(working.sendSummary1, comma);  //apogee time    
  strcat(working.sendSummary1, events.landedTime);strcat(working.sendSummary1, comma);  //landed time  
  if(strcmp(events.aftTime, "") == 0) strcpy(events.aftTime, "none");
  strcat(working.sendSummary1, events.aftTime);strcat(working.sendSummary1, comma);  //aft time  
  if(strcmp(events.foreTime, "") == 0) strcpy(events.foreTime, "none");
  strcat(working.sendSummary1, events.foreTime);strcat(working.sendSummary1, comma);  //fore time  
  strcpy(str_int, "");sprintf (str_int, "%d" , events.ascentSeconds);
  strcat(working.sendSummary1, str_int);strcat(working.sendSummary1, comma);  //ascent seconds
  strcpy(str_int, "");sprintf (str_int, "%d" , events.descentSeconds);
  strcat(working.sendSummary1, str_int);  //descent seconds  

  strcpy(str_int, "");sprintf (str_int, "%1.1f" , events.burnoutSeconds); 
  strcat(working.sendSummary2, str_int);strcat(working.sendSummary2, comma);  //burnout seconds
  strcpy(str_int, "");sprintf (str_int, "%1.0f" , baro.AltitudeMax);
  strcat(working.sendSummary2, str_int);strcat(working.sendSummary2, comma);  //baro max altitude
  strcpy(str_int, "");sprintf (str_int, "%1.0f" , gpsData.maxAGLfeet);
  strcat(working.sendSummary2, str_int);strcat(working.sendSummary2, comma);  //GPS max altitude (AGL)
  strcat(working.sendSummary2, "n/a");strcat(working.sendSummary2, comma);  //placeholder for Accel altitude AGL
  strcpy(str_int, "");sprintf (str_int, "%1.0f" , adxl.launchMaxZ);
  strcat(working.sendSummary2, str_int);strcat(working.sendSummary2, comma);  //launch Accel Max g's
  strcpy(str_int, ""); sprintf (str_int, "%1.0f" , baro.TempMax); ltrim(str_int);    
  strcat(working.sendSummary2, str_int);strcat(working.sendSummary2, comma); // max temp
  strcpy(str_int, ""); sprintf (str_int, "%1.0f" , LSM6D.tiltMaxLaunch);    
  strcat(working.sendSummary2, str_int);strcat(working.sendSummary2, comma); // max tilt    
  strcpy(str_int, ""); sprintf (str_int, "%1.0f" , working.voltage);
  strcat(working.sendSummary2, str_int);  // voltage A 

  strcpy(str_int, ""); sprintf (str_int, "%1.0f" , baro.SpeedMax);
  strcat(working.sendSummary3, str_int);strcat(working.sendSummary3, comma); // baro speed in FPS
  tempcalc = 0.0;
  tempcalc = baro.SpeedMax * 0.3048;
  strcpy(str_int, ""); sprintf (str_int, "%1.0f" , tempcalc);
  strcat(working.sendSummary3, str_int);strcat(working.sendSummary3, comma); // baro speed in MPS
  tempcalc = baro.SpeedMax * 0.681818;
  strcpy(str_int, ""); sprintf (str_int, "%1.0f" , tempcalc);
  strcat(working.sendSummary3, str_int);strcat(working.sendSummary3, comma); // baro speed in MPH    
  strcat(working.sendSummary3, "n/a");strcat(working.sendSummary3, comma);  //placeholder for Accel speed
  tempcalc = baro.AltitudeMax / (float)events.descentSeconds;
  tempcalc = tempcalc * -1;
  strcpy(str_int, ""); sprintf (str_int, "%1.0f" , tempcalc);
  strcat(working.sendSummary3, str_int);strcat(working.sendSummary3, comma); // average descent rate in FPS
  strcpy(str_int, ""); sprintf (str_int, "%1.0f" , baro.LandedSpeed);
  strcat(working.sendSummary3, str_int);strcat(working.sendSummary3, comma); // landed speed in FPS
  // put map friendly GPS coord here
  strcat(working.sendSummary3, gpsData.latFriendly);strcat(working.sendSummary3, comma); //GPS lat friendly
  strcat(working.sendSummary3, gpsData.longFriendly); //GPS long friendly  

    //send moved to landing loop
}

void summaryRepeater() {

  //radio send now
   //S1:  @S1,#,launch number, launch time, apogee time, landed time, aft event time, fore event time, ascent seconds, descent seconds, 
   //S2:  @S2,#,burnout seconds, AGL Baro, AGL GPS, AGL Accel, Launch Accel Max, Max temp, Max tilt, Batt voltage, 
   //S3:  @S3,#,Speed baro fps, mps, mph, SpeedA mps, Descent rate fps, landed rate fps, Landed GPS coord

    if(working.summaryRepeat == 3) {
      strcpy(radioHeader, "@S3");
      strcpy(radioMessage, working.sendSummary3);  
      RadioSend();  
      working.summaryRepeat = 1;
    }
    if(working.summaryRepeat == 2) {
      strcpy(radioHeader, "@S2");
      strcpy(radioMessage, working.sendSummary2);  
      RadioSend();  
      working.summaryRepeat = 3;
    }
    if(working.summaryRepeat == 0) working.summaryRepeat = 1;
    if(working.summaryRepeat == 1) {
      strcpy(radioHeader, "@S1");
      strcpy(radioMessage, working.sendSummary1);  
      RadioSend();  
      working.summaryRepeat = 2;
    }
}



//***********************************************************        GPS LOGIC    ************************************************************
//***********************************************************        GPS LOGIC    ************************************************************


void configGPS89() {
    threads.delay(100); // Little delay before flushing
    Serial1.flush();
    threads.delay(100);
    // *** Change Baud Rate to 115200 ****
    //byte packet[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x4B, 0x00, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x51,};  //19200
    byte packet[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBF, 0x78,};  //115200    
    sendPacket(packet, sizeof(packet));
    threads.delay(100); // Little delay before flushing
    Serial1.flush();
    Serial1.begin(115200);
    threads.delay(100);
    // *** Change outpaut rate ****
    // CFG-RATE packet   
    //  10 Hz = B5 62 06 08 06 00 64 00 01 00 01 00 7A 12 
    //   1 Hz = B5 62 06 08 06 00 E8 03 01 00 01 00 01 39
    //  .2 Hz = B5 62 06 08 06 00 88 13 01 00 01 00 B1 49   
    // byte packet[] = {0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12,}; //10 Hz M8N
    if(GPSmodel == 8) {
      byte packet8[] = {0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12,}; //10 Hz M8N
      sendPacket(packet8, sizeof(packet8));
    }
    if(GPSmodel == 9) {
      byte packet6[] = {0xB5,0x62,0x06,0x8A,0x0A,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x21,0x30,0x37,0x00,0x24,0x5F,}; //19Hz 55ms M9N
      sendPacket(packet6, sizeof(packet6));
    }
    threads.delay(100);
    // *** Filter NMEA codes out ****
    // remove GSV B5 62 06 01 08 00 F0 03 00 00 00 00 00 00 02 38
    byte packet0[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38,};
    sendPacket(packet0, sizeof(packet0));
    // remove VTG B5 62 06 01 08 00 F0 05 00 00 00 00 00 00 04 46
    byte packet2[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46,};
    sendPacket(packet2, sizeof(packet2));
    // remove GSA B5 62 06 01 08 00 F0 02 00 00 00 00 00 00 01 31
    byte packet3[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31,};
    sendPacket(packet3, sizeof(packet3));
    // remove GLL B5 62 06 01 08 00 F0 01 00 00 00 00 00 00 00 2A
    byte packet4[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A,};
    sendPacket(packet4, sizeof(packet4));   
    threads.delay(100);
    // *** Change to Airborne 4g mode ****
    // CFG-NAV5 packet
    // Basic Portable = B5 62 06 24 24 00 FF FF 00 03 00 00 00 00 10 27 00 00 05 00 FA 00 FA 00 64 00 2C 01 00 00 00 00 10 27 00 00 00 00 00 00 00 00 47 0F
    // Airborne 4g =    B5 62 06 24 24 00 FF FF 08 03 00 00 00 00 10 27 00 00 05 00 FA 00 FA 00 64 00 2C 01 00 00 00 00 10 27 00 00 00 00 00 00 00 00 4F 1F
    // Stationary  =    B5 62 06 24 24 00 FF FF 02 03 00 00 00 00 10 27 00 00 05 00 FA 00 FA 00 64 00 2C 01 00 00 00 00 10 27 00 00 00 00 00 00 00 00 49 53
//    byte packet[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x0F,}; // Basic Portable 
//    byte packet[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x1F,}; // Airborne 4g M8N
    
    if(GPSmodel == 8) {
      //byte packet[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x1F,}; // Airborne 4g M8N
      //sendPacket(packet, sizeof(packet));
    }    
    if(GPSmodel == 9) {
      byte packet5[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x21, 0x00, 0x11, 0x20, 0x08, 0xF4, 0x51,}; // Airborne 4g for M9N GPS
      sendPacket(packet5, sizeof(packet5));
    }
}


void configGPS10() {

    threads.delay(100); // Little delay before flushing
    Serial1.flush();
    threads.delay(100);
    // *** Change Baud Rate to 115200 ****
    byte packet[] = {0xB5, 0x62, 0x06, 0x8A, 0x0C, 0x00, 0x01, 0x01, 0x00, 0x00, 0x01, 0x00, 0x52, 0x40, 0x00, 0xC2, 0x01, 0x00, 0xF4, 0xB1,}; sendPacket(packet, sizeof(packet)); //115200 baud
    //byte packet[] = {0xB5,0x62,0x06,0x8A,0x0C,0x00,0x01,0x01,0x00,0x00,0x01,0x00,0x52,0x40,0x00,0x84,0x03,0x00,0xB8,0xFB,}; sendPacket(packet, sizeof(packet)); //230400 baud
    threads.delay(100); // Little delay before flushing
    Serial1.flush();
    Serial1.begin(115200);
    threads.delay(100);
    // *** Change GNSS Subscriptions ****
    byte packet13[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0x22,0x00,0x31,0x10,0x00,0xFE,0x97,}; sendPacket(packet13, sizeof(packet13));threads.delay(10); //remove BeiDou
    byte packet14[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0x25,0x00,0x31,0x10,0x01,0x02,0xA7,}; sendPacket(packet14, sizeof(packet14));threads.delay(10); //add GLONAS
    threads.delay(100);
    // *** Change outpaut rate ****
    byte packet6[] = {0xB5,0x62,0x06,0x8A,0x0A,0x00,0x01,0x01,0x00,0x00,0x01,0x00,0x21,0x30,0x32,0x00,0x20,0x5F,}; sendPacket(packet6, sizeof(packet6));// 20 hz / 50 ms
    //byte packet6[] = {0xB5,0x62,0x06,0x8A,0x0A,0x00,0x01,0x01,0x00,0x00,0x01,0x00,0x21,0x30,0x19,0x00,0x07,0x2D,}; sendPacket(packet6, sizeof(packet6));// 40 hz / 25 ms
    threads.delay(100);
    // *** Filter and set NMEA codes out ****
    byte packet0[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0xCA,0x00,0x91,0x20,0x00,0x16,0x1F,}; sendPacket(packet0, sizeof(packet0));threads.delay(10); //remove GLL
    byte packet2[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0xC0,0x00,0x91,0x20,0x00,0x0C,0xED,}; sendPacket(packet2, sizeof(packet2));threads.delay(10); //remove GSA
    byte packet3[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0xC5,0x00,0x91,0x20,0x00,0x11,0x06,}; sendPacket(packet3, sizeof(packet3));threads.delay(10); //remove GSV
    byte packet4[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0xB1,0x00,0x91,0x20,0x00,0xFD,0xA2,}; sendPacket(packet4, sizeof(packet4));threads.delay(10); //remove VTG
    byte packet11[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0xBB,0x00,0x91,0x20,0x00,0x07,0xD4,}; sendPacket(packet11, sizeof(packet11));threads.delay(10); //remove GGA
    byte packet12[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0xB6,0x00,0x91,0x20,0x01,0x03,0xBC,}; sendPacket(packet12, sizeof(packet12));threads.delay(10); //add in GNS
    threads.delay(100);
    // *** Change Dynamic Model to 4G ****    
    byte packet15[] = {0xB5,0x62,0x06,0x8A,0x09,0x00,0x01,0x01,0x00,0x00,0x21,0x00,0x11,0x20,0x08,0xF5,0x5A,}; sendPacket(packet15, sizeof(packet15));threads.delay(10); //Change to Air 4G  
}


void sendPacket(byte *packet, byte len)
{
    for (byte i = 0; i < len; i++)
     {  Serial1.write(packet[i]);  }
    if(serialDebug) printPacket(packet, len);
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
    if (delimiterPtr == nullptr) { // Hit end of string
      len = endOfData - nextSegmentPtr;
    } else {
      len = delimiterPtr - nextSegmentPtr;
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
    if(i==20) {
      //overflow array data
      break;
    }
    nextSegmentPtr = nextSegmentPtr + len + 1;
  } // end while  
}



void CheckGPS89() {    //*********************************  Check the GPS code ********************************************

  if(serialDebug) digitalWrite(led2_pin, HIGH);
  if(debugMode) debugVars.GPScount++;
  unsigned long startTime = millis();
  startTime = startTime + 500; // 500ms timeout for reading GPS
  int vRMC = false;
  int vGGA = false;
  strcpy(gpsData.vGNRMC, "");
  strcpy(gpsData.vGNGGA, ""); 
  char ReadString[100] = "";
  int vCheck = false;

  // *** Get new serial data from the GPS ***
  while(Serial1.available()){  //flush the serial buffer so you don't get an old reading
    Serial1.read();} 
  while (vCheck == false) {
   int size = Serial1.readBytesUntil(13,ReadString,99);
   //if(serialDebug) Serial.println(ReadString);
   if(size > 0) {
      char *ptr = strstr(ReadString, "$GNRMC");
     if (ptr != NULL) /* Substring found */
       {strcpy(gpsData.vGNRMC, ReadString);
        vRMC=true; 
       }
     char *ptr2 = strstr(ReadString, "$GNGGA");
     if (ptr2 != NULL) /* Substring found */
       {strcpy(gpsData.vGNGGA, ReadString);
        vGGA=true; 
       }    
      if (vGGA && vRMC) vCheck = true;  //success - got both sentences from GPS now exit
   }
    strcpy(ReadString, "");
    if (millis() > startTime) { // timeout
      gpsData.gpsStatus = 0;
      if(serialDebug) Serial.println("GPS timeout");
      if(debugMode) debugVars.GPSquality = 99;
      strcpy(working.gpsString, "Error on GPS - Timeout");
      gpsLogQ(); 
      errors.GPS = 1;
      if(serialDebug) digitalWrite(led2_pin, LOW);
      digitalWrite(led3_pin, HIGH);
      if(debugMode) debugVars.gpsTime += (millis() - (startTime-500));
      return;
    }
   }
   errors.GPS = 2; //healthy

   
  // *** success, got new data from GPS without timeout ***

   char tempGNRMC[200];
   char tempGNGGA[200];
   strcpy(tempGNRMC,gpsData.vGNRMC);
   strcpy(tempGNGGA, gpsData.vGNGGA);
   parseit(tempGNRMC,aGNRMC);
   parseit(tempGNGGA,aGNGGA); 
   //Serial.println(gpsData.vGNRMC);
   //Serial.println(gpsData.vGNGGA);
   

   // check for time to be set
   if(strlen(aGNRMC[9]) > 0 && events.clockSet == 0) {
        setTheTime();
        events.clockSet = 1; 
   }
   
    if(strcmp(aGNRMC[2],"A") != 0) {  // no fix - abort
      if(gpsData.gpsStatus == 2) { // lost GPS lock
        strcpy(working.gpsString, "** Lost GPS lock **");
        gpsLogQ();
        if(serialDebug) Serial.println(working.gpsString);
      }
      strcpy(working.gpsString, "No GPS fix:  ");
      strcat(working.gpsString,gpsData.vGNRMC);
      strcat(working.gpsString, " GNS: ");
      strcat(working.gpsString, gpsData.vGNGGA);
      if(debugMode) debugVars.GPSquality = 1;
      if(serialDebug) Serial.println(working.gpsString);
      gpsLogQ(); 
      gpsData.gpsStatus = 1;
      if(serialDebug) digitalWrite(led3_pin, LOW);
      if(debugMode) debugVars.gpsTime += (millis() - (startTime-500));
      return;
     }

  // Success - Good GPS data and has fix, so proceed to load data

   // *******************  Populate the GPS Data from NMEA data  ************************

    

    fldcnt = 0;

    //GPS - date-time load
    populateTime(); 
    strcpy(gpsData.fix, aGNRMC[2]); //fix
    strcpy(gpsData.quality, aGNGGA[6]); //quality
    strcpy(gpsData.latRaw, aGNGGA[2]); //Latitude
    strcat(gpsData.latRaw, aGNGGA[3]); //Latitude Dir
    strcpy(gpsData.longRaw, aGNGGA[4]); //Longitude
    strcat(gpsData.longRaw, aGNGGA[5]); //Longitude Dir
    gpsData.latDec = GpsToDecimalDegrees(aGNGGA[2], aGNGGA[3]);  //Lat Decimal
    gpsData.longDec = GpsToDecimalDegrees(aGNGGA[4], aGNGGA[5]); //Long Decimal
    char str_float[20];
    dtostrf(gpsData.latDec, 8, 6, str_float);
    remove_spaces(str_float, gpsData.latFriendly); //Lat Dec String
    dtostrf(gpsData.longDec, 8, 6, str_float);
    remove_spaces(str_float, gpsData.longFriendly); //Long Dec String   
    strcpy(gpsData.gpsAltitudeMeters, aGNGGA[9]); //Altitude Meters
    //calculate Feet
    char tempNice[100] = "";  
    strcpy(tempNice, ""); 
    strcpy(tempNice, aGNGGA[9]);
    float dTemp = 0.0;
    dTemp = atof(tempNice);
    dTemp = dTemp * 3.28084; //convert to feet
    dtostrf(dTemp, 8, 0, str_float);
    char sResult[12];
    remove_spaces(str_float, sResult);
    strcpy(gpsData.gpsAltitudeFeet, sResult); //Altitude FeeT
    strcpy(gpsData.gpsSpeed,aGNRMC[7]);
    strcpy(gpsData.angle,aGNRMC[8]);
    strcpy(gpsData.sat,aGNGGA[7]);
    if(debugMode) debugVars.GPSquality = atoi(gpsData.sat);
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
    
    if (gpsData.gpsStatus < 2) {
        if(working.radioOff == 1)  working.radioOff = 0; 
        strcpy(working.gpsString, "** GPS acquired GPS fix **");
        gpsLogQ();
        strcpy(radioHeader, "@M");   strcpy(radioMessage, "Rocket acquired GPS fix");
        RadioSend();      
    }
    gpsData.gpsStatus = 2;
    if(serialDebug) digitalWrite(led3_pin, LOW);
    if(debugMode) debugVars.gpsTime += (millis() - (startTime-500));
        
}



void CheckGPS10() {    //*********************************  Check the GPS code for UBLOX M10S ********************************************

  if(debugMode) debugVars.GPScount++;
  unsigned long startTime = millis();
  startTime = startTime + 500; // 500ms timeout for reading GPS
  if(serialDebug) digitalWrite(led1_pin, HIGH);
  int vRMC = false;
  int vGNS = false;
  strcpy(gpsData.vGNRMC, "");
  strcpy(gpsData.vGNGNS, ""); 
  char ReadString[100] = "";
  int vCheck = false;


  // *** Get new serial data from the GPS ***
  while(Serial1.available()){  //flush the serial buffer so you don't get an old reading
    Serial1.read();} 
  while (vCheck == false) {
   int size = Serial1.readBytesUntil(13,ReadString,99);
   if(size > 0) {
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
    }
    strcpy(ReadString, "");
    if (millis() > startTime) { // timeout
      gpsData.gpsStatus = 0;
      if(serialDebug) Serial.println("GPS timeout");
      if(debugMode) debugVars.GPSquality = 99;
      strcpy(working.gpsString, "Error on GPS - Timeout");
      gpsLogQ(); 
      errors.GPS = 1;
      if(serialDebug) digitalWrite(led1_pin, LOW);
      digitalWrite(led3_pin, HIGH);
      if(debugMode) debugVars.gpsTime += (millis() - (startTime-500));
      return;
    }
   }

   errors.GPS = 2; //healthy
   
  // *** success, got new data from GPS without timeout ***

   char tempGNRMC[200];
   char tempGNGNS[200];
   strcpy(tempGNRMC,gpsData.vGNRMC);
   strcpy(tempGNGNS, gpsData.vGNGNS);
   parseit(tempGNRMC,aGNRMC);
   parseit(tempGNGNS,aGNGNS); 

   // check for time to be set
   if(strlen(aGNRMC[9]) > 0 && events.clockSet == 0) {
        setTheTime();
        events.clockSet = 1; 
   }
   
    if(strcmp(aGNRMC[2],"A") != 0) {  // no fix - abort
      if(gpsData.gpsStatus == 2) { // lost GPS lock
        strcpy(working.gpsString, "** Lost GPS lock **");
        gpsLogQ();
        if(serialDebug) Serial.println(working.gpsString);
      }
      strcpy(working.gpsString, "No GPS fix:  ");
      strcat(working.gpsString,gpsData.vGNRMC);
      strcat(working.gpsString, " GNS: ");
      strcat(working.gpsString, gpsData.vGNGNS);
      if(debugMode) debugVars.GPSquality = 1;
      if(serialDebug) Serial.println(working.gpsString);
      gpsData.gpsStatus = 1;
      if(serialDebug) digitalWrite(led1_pin, LOW);
      if(debugMode) debugVars.gpsTime += (millis() - (startTime-500));
      return;
     }

  // Success - Good GPS data and has fix, so proceed to load data

   // *******************  Populate the GPS Data from NMEA data  ************************

    

    fldcnt = 0;

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
    if(debugMode) debugVars.GPSquality = atoi(gpsData.sat);
    //baseline
    if(gpsData.baseline == 0.0) gpsData.baseline = dTemp;
    //use last five readings for zero AGL baseline while waiting
    if(events.launch == 0) {
      gpsData.avgCounter++;
      gpsData.tempAvg += dTemp;
      if(gpsData.avgCounter == 5) {
        gpsData.baseline = (float) (gpsData.tempAvg / (float) gpsData.avgCounter);
        gpsData.avgCounter = 0;
        gpsData.tempAvg = 0.0;
      }
    }
    //max AGL
    if((dTemp-gpsData.baseline) > gpsData.maxAGLfeet) gpsData.maxAGLfeet = (dTemp - gpsData.baseline);  // record max AGL    
    //get AGL string
    dTemp = dTemp - gpsData.baseline;
    if(dTemp < 15 && dTemp > -15) dTemp = 0.0; //zero out noise if in the ballpark of 0 AGL
    strcpy(str_float, ""); 
    dtostrf(dTemp, 8, 0, str_float);
    remove_spaces(str_float, sResult);   
    strcpy(gpsData.altitudeAGL, sResult);   
    
    if (gpsData.gpsStatus < 2) {
        if(working.radioOff == 1)  working.radioOff = 0; 
        strcpy(working.gpsString, "** GPS acquired GPS fix **");
        gpsLogQ();
        strcpy(radioHeader, "@M");   strcpy(radioMessage, "Rocket acquired/regained GPS fix");
        RadioSend();      
    }
    gpsData.gpsStatus = 2;
    if(serialDebug) digitalWrite(led1_pin, LOW);
    if(debugMode) debugVars.gpsTime += (millis() - (startTime-500));
        
}





void gpsLogString() {   // ***** Format the GPS log line *****
   //Log Format:   [@G,]date,time, fix, quality, lat, long, altitude(f), speed(knots), angle, Satellites, friendly lat, friendly lon, AGL max feet AGL,altitudeAGL,!.
   const char *comma = ",";
   strcpy(gpsData.gpsLogLine,""); 
   strcat(gpsData.gpsLogLine, gpsData.gpsDate);strcat(gpsData.gpsLogLine, comma);
   strcat(gpsData.gpsLogLine, gpsData.gpsTime);strcat(gpsData.gpsLogLine, comma);
   strcat(gpsData.gpsLogLine, gpsData.fix);strcat(gpsData.gpsLogLine, comma);
   strcat(gpsData.gpsLogLine, gpsData.quality);strcat(gpsData.gpsLogLine, comma);
   strcat(gpsData.gpsLogLine, gpsData.latRaw);strcat(gpsData.gpsLogLine, comma);
   strcat(gpsData.gpsLogLine, gpsData.longRaw);strcat(gpsData.gpsLogLine, comma);
   strcat(gpsData.gpsLogLine, gpsData.gpsAltitudeFeet);strcat(gpsData.gpsLogLine, comma);
   strcat(gpsData.gpsLogLine, gpsData.gpsSpeed);strcat(gpsData.gpsLogLine, comma);
   strcat(gpsData.gpsLogLine, gpsData.angle);strcat(gpsData.gpsLogLine, comma);
   strcat(gpsData.gpsLogLine, gpsData.sat);strcat(gpsData.gpsLogLine, comma);
   strcat(gpsData.gpsLogLine, gpsData.latFriendly);strcat(gpsData.gpsLogLine, comma);
   strcat(gpsData.gpsLogLine, gpsData.longFriendly);strcat(gpsData.gpsLogLine, comma);
   float tempf = gpsData.maxAGLfeet;
   char str_feet[16];
   strcpy(str_feet,"");fToStr(tempf, str_feet, 0); 
   strcat(gpsData.gpsLogLine, str_feet); ;strcat(gpsData.gpsLogLine, comma);
   strcat(gpsData.gpsLogLine, gpsData.altitudeAGL);
   
}

void gpsRadioString() {  // ***** Format the GPS radio line *****
  //Radio Format: [@G,]time, fix, lat, long, alt(feet), speed, sat, max alt feet AGL -- keep it short for reliability
   const char *comma = ",";
   strcpy(gpsData.gpsSendLine,"T"); 
   //strcat(gpsData.gpsSendLine, gpsData.gpsTime); //removed to shorten
   strcat(gpsData.gpsSendLine, comma);
   strcat(gpsData.gpsSendLine, gpsData.fix);strcat(gpsData.gpsSendLine, comma);
   strcat(gpsData.gpsSendLine, gpsData.latFriendly);strcat(gpsData.gpsSendLine, comma); //now sending decimal
   strcat(gpsData.gpsSendLine, gpsData.longFriendly);strcat(gpsData.gpsSendLine, comma); //now sending decimal
   strcat(gpsData.gpsSendLine, gpsData.altitudeAGL);strcat(gpsData.gpsSendLine, comma); //was gpsData.gpsAltitudeFeet
   strcat(gpsData.gpsSendLine, gpsData.gpsSpeed);strcat(gpsData.gpsSendLine, comma);
   strcat(gpsData.gpsSendLine, gpsData.sat);strcat(gpsData.gpsSendLine, comma);
   float tempf = gpsData.maxAGLfeet;
   char str_feet[16];
   strcpy(str_feet,"");fToStr(tempf, str_feet, 0);  
   strcat(gpsData.gpsSendLine, str_feet); 

}

/**
 * Convert NMEA absolute position to decimal degrees
 * "ddmm.mmmm" or "dddmm.mmmm" really is D+M/60,
 * then negated if quadrant is 'W' or 'S'
 */
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

void populateTime() {  // load GPS NMEA into structure 

    fldcnt = 0;
    char target[20]= "";
    const char *slash = "/"; const char *colon = ":"; 
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
    if(serialDebug) {
      Serial.println(F("The Time has been set"));
      Serial.print(hour());Serial.print(":");
      Serial.print(minute()); Serial.print(":");
      Serial.print(second());
      Serial.print(" ");
      Serial.print(month());
      Serial.print("/");
      Serial.print(day());
      Serial.print("/");
      Serial.print(year()); 
      Serial.println(); 
      Serial.flush();
    }
    
}



//***********************************************************        RADIO LOGIC    ************************************************************
//***********************************************************        RADIO LOGIC     ************************************************************

void radioHardwareTest() {

  // assumes the power has been on for at least 3 seconds
  // puts Lora radio in set mode and sends ID command. Response is the radio ID and name.
  
    if(serialDebug) Serial.println("Initiating Radio Test...");
    digitalWrite(radioSet, HIGH); //turn radio set pin to low
    delay(50);
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
      radioRXtest();
      if(newWord == 1) {
        if(serialDebug) Serial.print("Radio Rx:  ");
        if(serialDebug) Serial.println(theWord);
        char *ptr = strstr(theWord, "LORA6100");
        if (ptr != NULL && serialDebug) Serial.println("Radio Test Successful!"); 
        digitalWrite(radioSet, LOW); //turn radio set pin to low
        Serial2.begin(19200);
        errors.radio = 2; //good
        x = 1;
        
      }
      if(millis() > testtime) {
        if(serialDebug) Serial.println("Radio Timeout");
        digitalWrite(radioSet, LOW); //turn radio set pin to low
        Serial2.begin(19200);
        errors.radio = 1;
        digitalWrite(led3_pin, HIGH);
        digitalWrite(led2_pin, HIGH);
        x = 2;
      }
    }
}

void radioRXtest() {  // **********  CHECK RADIO  *****************
   newWord = 0;
   char receivedChar;
   if (Serial2.available() > 0) {
    unsigned long testTime = millis() + 100;
    strcpy(theWord, "");
    while (newWord != 1) {
       if(Serial2.available()) {
         receivedChar = Serial2.read();
         append(theWord, receivedChar);
         if(receivedChar == 0x0a) {
            newWord = 1;
            break;   
         }
       }
       if(millis() > testTime) {  // exit and evaluate if it takes too long
          newWord =1;
          break;
       }
       delay(1);
     }
   }
}


void RadioSend() {  //New and improved
  //populate global radioHeader (e.g. @M) and radioMessage first
   if(configs.radio) {
    char str_int[16];
    strcpy(radioMessageS, "");
    strcat(radioMessageS, radioHeader);
    strcat(radioMessageS, ",");   
    int tempi = working.LaunchNumber;
    strcpy(str_int,"");iToStr(tempi, str_int, 0);
    strcat(radioMessageS, str_int);
    strcat(radioMessageS, ","); 
    strcat(radioMessageS, radioMessage);      
    strcat(radioMessageS, ",!");
    //done creating string
    //add it to the send queue n times
    rSendCount ++;
    rSendPos ++;
    if (rSendPos == 11) rSendPos = 1;
    strcpy(rSend[rSendPos], radioMessageS);  
   }
}


void RadioSendQueue () {

  // this is designed to cut down on send congestion. Buffer packets 1000ms apart so they don't mangle together
  // Globals:  String rSend[10];  byte rSendCount = 0;  rSendLast = 0; rSendPos = 0; RadioSendQueueRate = 1000
   if(rSendCount > 0) {
     if(serialDebug) digitalWrite(led1_pin, HIGH); //blink onboard light
     rSendLast ++;
     if (rSendLast == 11) rSendLast = 1;
     if(working.radioOff == 0) {
       Serial2.print(rSend[rSendLast]);
       Serial2.flush(); //waits for outgoing transmission to complete
       strcpy(working.radioString, "");
       strcat(working.radioString, "SENT: ");
       strcat(working.radioString, rSend[rSendLast]);
       radioLogQ(); 
     } else {
       strcpy(working.radioString, "");
       strcat(working.radioString, "Skipped Radio Send OFF: ");
       strcat(working.radioString, rSend[rSendLast]);
       radioLogQ();       
     }
     rSendCount = rSendCount - 1; 
     if(serialDebug) digitalWrite(led1_pin, LOW);
   }
}



void checkRadio() {  // **********  CHECK RADIO  *****************
   newWord = 0;
   char receivedChar;
   if (Serial2.available() > 0) {
  //  Serial.println("Rx...");
    unsigned long testTime = millis() + 2000;
    
    strcpy(theWord, "");
    if(serialDebug) Serial.println("radio inbound detect");

    while (newWord != 1) {
       if(Serial2.available()) {

         receivedChar = Serial2.read();
         if(receivedChar == 33) {  // look for ! to end
          newWord = 1;
          append(theWord, receivedChar);
          Serial2.read();  // read the extra end char
         } else {
           append(theWord, receivedChar);
         }
         if(receivedChar < 32 || receivedChar > 126) {  //noise or garbage on the radio - empty and abort to not waste cycles
            while(Serial2.available()) {
              receivedChar = Serial2.read();
              if(millis() > testTime) return;
            }
            return;
            if(serialDebug) Serial.print("**** Radio noise ignored ****");
         }
       }
       if(millis() > testTime) {  // exit and evaluate if it takes too long
          newWord =1;
          if(serialDebug) Serial.println("timeout exit error");
          if(serialDebug) Serial.println(theWord);
          break;
       }
       threads.delay(1);
     }
     ProcessRadio();
   }
}


void ProcessRadio() {   // **********  PROCESS RADIO COMMAND  *****************

      int chkNew = 0;

      if(theWord[0] == 64 && theWord[strlen(theWord)-1] == 33) {   // proper sentence recieved from radio @ !          
          //log it
          strcpy(working.radioString, "");
          strcat(working.radioString, "Radio RX: ");
          strcat(working.radioString, theWord);          
          radioLogQ(); 
          chkNew = 1;

      } else {  // error
          // write the SD card
              strcpy(working.radioString, "");
              strcat(working.radioString, "Rx Radio Malformed Sentence Received: ");
              strcat(working.radioString, theWord);          
              radioLogQ();  
              newWord = 0;
              strcpy(theWord, "");
      }
      // **************  SUMMARY OF COMMANDS  ********************
      //  @S = send status
      //  @A = Abort
      //  @L = Launch
      //  @B = Basic
      //  @R = Reset
      //  @T = Radio Test Request
      //  @D = force Descent Mode
      //  @Z = summary
      //  @V = Video Camera
      //  @C = Call Sign
      //  @P = "Marco"
      //  @O,BOOM = BOOM
      //  @O,MAIN = main pyro, @O,DROGUE = pyro drogue, @O,SEP = separate, @O,SUS = fire sustainer
      //  @0R = Radio off
      //  @1R = Radio on
      //  @9 = Erase Flash
      
      if (chkNew == 1) {  // Real processing goes here            ********************************  RADIO PROCESSING  ******************************

          if (strncmp("@S,",theWord,3) == 0) {
            strcpy(working.radioString, ""); strcat(working.radioString, "Rx STATUS command received by radio");       
            radioLogQ();  
            if(serialDebug) Serial.println(F("Got radio action STATUS"));
            strcpy(radioHeader, "@M");
            strcpy(radioMessage, "Hello from Rocket");
            RadioSend();           
          }
          if (strncmp("@P,",theWord,3) == 0) {
            strcpy(working.radioString, ""); strcat(working.radioString, "Rx Marco command received by radio");       
            radioLogQ();  
            if(serialDebug) Serial.println(F("Got radio action Marco"));
            strcpy(radioHeader, "@M");
            strcpy(radioMessage, "Polo");
            RadioSend();           
          }

          if (strncmp("@0R,",theWord,4) == 0) {
              working.radioOff = 1;
              strcpy(working.radioString, ""); strcat(working.radioString, "Rx RADIO OFF command received by radio");       
              radioLogQ(); 
              if(serialDebug) Serial.println(F("Got radio action RADIO OFF"));          
          }
          if (strncmp("@1R,",theWord,4) == 0) {
              working.radioOff = 0;
              strcpy(working.radioString, ""); strcat(working.radioString, "Rx RADIO ON command received by radio");       
              radioLogQ(); 
              if(serialDebug) Serial.println(F("Got radio action RADIO ON"));
              strcpy(radioHeader, "@M");
              strcpy(radioMessage, "Hello. Radio TX is now ON");
              RadioSend();        
          }
          if (strncmp("@9,",theWord,3) == 0) {
              working.eraseFlash = 1;
              strcpy(working.radioString, ""); strcat(working.radioString, "Rx Erase Flash command received by radio");       
              radioLogQ(); 
              if(serialDebug) Serial.println(F("Got radio action RADIO OFF"));          
          }
          
          
          if (strncmp("@A,",theWord,3) == 0) {  // used to terminate waiting phase and close logs for retention
             //working.phase = "ABORT";  // this will shut down processing
             strcpy(working.phase, "ABORT");
             strcpy(working.eventString, "====================== ABORT RECEIVED =============================="); 
             eventLog();
             threads.delay(500);
             strcpy(radioHeader, "@M");   strcpy(radioMessage, "Rocket ABORT Processed");
             RadioSend();   
             if(serialDebug) Serial.println(F("Completed radio action ABORT"));
          }

          if (strncmp("@L,",theWord,3) == 0 && strcmp("LAUNCH",working.phase) != 0) {  // used to manually transition from waiting to launch if no launch detect
             strcpy(working.eventString, "*** MANUAL LAUNCH DETECT!  (Base Initiated)"); 
             eventLog();
             strcpy(radioHeader, "@M");   strcpy(radioMessage, "Manual Launch Initiation");
             RadioSend();   
             PhaseLaunchInit();
          }
          if (strncmp("@D,",theWord,3) == 0 && strcmp("DESCENT",working.phase) != 0) {  // used to manually transition to descent mode (primarily for testing)
             strcpy(working.eventString, "*** MANUAL DESCENT MODE!  (Base Initiated)"); 
             eventLog();
             strcpy(radioHeader, "@M");   strcpy(radioMessage, "Descent Mode Initiation");
             RadioSend();   
             PhaseDescentInit();
          }
          if (strncmp("@T,",theWord,3) == 0) {  // used to transmit a radio test set of strings 
             radioTest();
          }    
          if (strncmp("@W,",theWord,3) == 0) {  // round-trip radio speed test. Bypass all queues and send 46 bytes
               char theReply[50];
               strcpy(theReply, "@W2,1234567890123456789012345678901234567890,!");
               Serial2.print(theReply);
               Serial2.flush(); 
          }    

          
          if (strncmp("@V,",theWord,3) == 0) {  // toggle video camera on/off
             cameraToggle();
          }        
          
          //Do Reset
          if (strncmp("@R,",theWord,3) == 0) {  // used to reset on the stand if false launch detect back to waiting
             
             strcpy(working.eventString, "*** MANUAL RESET!  (Base Initiated)"); 
             eventLog();
             threads.delay(20);
             SCB_AIRCR = 0x05FA0004; // force a power reset bye bye
             asm volatile ("dsb");
             strcpy(radioHeader, "@M");   strcpy(radioMessage, "Rocket Reset Processed");
             RadioSend();
             ResetLaunch();
          }
          if (strncmp("@B,",theWord,3) == 0) {  // used to put rocket into basic mode after in air reset
             strcpy(working.eventString, "*** BASIC MODE!  (Base Initiated)"); 
             if(serialDebug) Serial.println("BASIC mode recieved over radio");
             eventLog();
             strcpy(radioHeader, "@M");   strcpy(radioMessage, "Basic Mode Initiated");
             RadioSend();    
             PhaseBasicInit();
          }          

          if (strncmp("@O,BOOM",theWord,7) == 0) {  // This is boom under lock and key. Not sure what purpose yet. Fire ejection manually.
             //hope to never use this!!
             strcpy(working.eventString, "*** BOOM ***  (Base Initiated)"); 
             eventLog();
             events.pyroSeparation = 0;events.pyroMain = 0;events.pyroDrogue = 0;
             strcpy(radioHeader, "@M");   strcpy(radioMessage, "BOOM ALL! Received by rocket");
             RadioSend(); 
             pyroBoom();
          }
          if (strncmp("@O,MAIN",theWord,7) == 0) {  // Pyro ignite Main
             strcpy(working.eventString, "*** BOOM MAIN ***  (Base Initiated)"); 
             eventLog();
             events.pyroMain = 0; //reset if second fire
             strcpy(radioHeader, "@M");   strcpy(radioMessage, "BOOM MAIN! Received by rocket");
             RadioSend(); 
             pyroMain();
          }
          if (strncmp("@O,DROGUE",theWord,9) == 0) {  // Pyro ignite Drogue
             strcpy(working.eventString, "*** BOOM DROGUE ***  (Base Initiated)"); 
             eventLog();
             events.pyroDrogue = 0; //reset if second fire
             strcpy(radioHeader, "@M");   strcpy(radioMessage, "BOOM DROGUE! Received by rocket");
             RadioSend(); 
             pyroDrogue();
          }
          if (strncmp("@O,SUS",theWord,6) == 0) {  // Pyro ignite Sustainer
             strcpy(working.eventString, "*** BOOM SUSTAINER ***  (Base Initiated)"); 
             eventLog();
             events.pyroSustainer = 0;
             strcpy(radioHeader, "@M");   strcpy(radioMessage, "BOOM SUSTAINER! Received by rocket");
             RadioSend(); 
             pyroSustainer();
          }
          if (strncmp("@O,SEP",theWord,6) == 0) {  // Pyro ignite Separation
             strcpy(working.eventString, "*** BOOM SEPARATION ***  (Base Initiated)"); 
             eventLog();
             events.pyroSeparation = 0;
             strcpy(radioHeader, "@M");   strcpy(radioMessage, "BOOM SEPARATION! Received by rocket");
             RadioSend(); 
             pyroSeparation();
          }

          
          if (strncmp("@Z,",theWord,3) == 0) {  // Request to force resend the summary manually
             strcpy(working.eventString, "Manual Request of Summary"); 
             eventLog();
             flightSummaryString(); 
             summaryRepeater();
             summaryRepeater();
             summaryRepeater();
          }

          newWord = 0;
          strcpy(theWord,"");
      }
}

void append(char* s, char c) {  //used to concat char to char array
        int len = strlen(s);
        s[len] = c;
        s[len+1] = '\0';
}

void ResetLaunch() {

   strcpy(working.phase, "WAITING");
   PhaseWaitingInit();    
  //say hello on radio
   strcpy(radioHeader, "@M");   strcpy(radioMessage, "Rocket Processing RESET");
   RadioSend(); 
}



void radioTest() {
  threads.delay(500); // make RX radio is clear
  strcpy(radioHeader, "@T");   strcpy(radioMessage, "10");
  RadioSend(); 
  strcpy(radioHeader, "@T");   strcpy(radioMessage, "20-123456789");
  RadioSend(); 
  strcpy(radioHeader, "@T");   strcpy(radioMessage, "30-1234567891234567890");
  RadioSend(); 
  strcpy(radioHeader, "@T");   strcpy(radioMessage, "40-12345678912345678901234567890");
  RadioSend(); 
  strcpy(radioHeader, "@T");   strcpy(radioMessage, "50-123456789123456789012345678901234567890");
  RadioSend(); 
  strcpy(radioHeader, "@T");   strcpy(radioMessage, "60-1234567891234567890123456789012345678901234567890");
  RadioSend(); 
  strcpy(radioHeader, "@T");   strcpy(radioMessage, "70-12345678912345678901234567890123456789012345678901234567890");
  RadioSend(); 
  strcpy(radioHeader, "@T");   strcpy(radioMessage, "80-123456789123456789012345678901234567890123456789012345678901234567890");
  RadioSend(); 
  strcpy(radioHeader, "@T");   strcpy(radioMessage, "100-12345678912345678901234567890123456789012345678901234567890123456789012345678901234567890");
  RadioSend(); 
   
}

void cameraToggle() {

   if(millis() > working.cameraLockout) {
     strcpy(radioHeader, "@M");   strcpy(radioMessage, "Got Camera Request");
     RadioSend(); 
     threads.yield();
     digitalWrite(relayCamera, HIGH);
     if(serialDebug) Serial.println("Camera push button");
     threads.delay(2000);
     digitalWrite(relayCamera, LOW);
     if(working.camera == 0) {
       working.camera = 1;
       strcpy(working.eventString, "Camera remotely turned ON");
       eventLog();
       strcpy(radioHeader, "@M");   strcpy(radioMessage, "Camera is now ON");
       RadioSend(); 
       
     } else {
       working.camera = 0;
       strcpy(working.eventString, "Camera remotely turned OFF");
       eventLog();
       strcpy(radioHeader, "@M");   strcpy(radioMessage, "Camera is now OFF");
       RadioSend();       
     }
     working.cameraLockout = millis() + 20000; //sometimes camera takes 15 seconds to start
   } else {
       strcpy(radioHeader, "@M");   strcpy(radioMessage, "Camera lockout time... try again");
       RadioSend();        
   }
   threads.yield();
}



void DoBoom() {  
}



//***********************************************************        OTHER AND MISC     ************************************************************
//***********************************************************        OTHER AND MISC     ************************************************************

void errorCheck() {
  // Primary error evaluation - decide what to send
  // @I,sensors,logging,ADXL,LSM,MS56,GPS,flash,voltage,temp,I2C,message
  int errorCount = 0;
  char multiMessage[200];

  strcpy(multiMessage,"MULTIPLE ERRORS: ");
  if(errors.sampleLoop == 1) {
    errorCount++;
    strcpy(errors.message,"ERROR: Sample Loop Stopped");
    strcat(multiMessage,"Samples ");
  }
  if(errors.loggingLoop == 1) {
    errorCount++;
    strcpy(errors.message,"ERROR: Logging Loop Stopped");
    strcat(multiMessage,"Logging ");
  }  
  if(errors.ADXL == 1) {
    errorCount++;
    strcpy(errors.message,"ERROR: ADXL Failed to Initialize");
    strcat(multiMessage,"ADXL ");
  }    

  if(errors.LSM == 1) {
    errorCount++;
    strcpy(errors.message,"ERROR: LSM6DS Failed to Initialize");
    strcat(multiMessage,"LSM6DS ");
  } 
  if(errors.MS56 == 1) {
    errorCount++;
    strcpy(errors.message,"ERROR: Barometer Failed to Initialize");
    strcat(multiMessage,"Baro ");
  } 
  if(errors.GPS == 1) {
    errorCount++;
    strcpy(errors.message,"ERROR: GPS Hardware Timeout");
    strcat(multiMessage,"GPS ");
  } 
  if(errors.flash == 1) {
    errorCount++;
    strcpy(errors.message,"ERROR: Flash memory write error");
    strcat(multiMessage,"Flash ");
  } 
  if(errors.voltage == 1) {
    errorCount++;
    strcpy(errors.message,"ERROR: Low Rocket Voltage");
    strcat(multiMessage,"Voltage ");
  } 
  if(errors.CPUtemp == 1) {
    errorCount++;
    strcpy(errors.message,"ERROR: High CPU Temperature");
    strcat(multiMessage,"Temp ");
  } 
  if(errors.I2C == 1) {
    errorCount++;
    strcpy(errors.message,"ERROR: I2C Bus Communication");
    strcat(multiMessage,"I2C ");
  }   
  if(errors.crash == 1) {
    errorCount++;
    strcpy(errors.message,"ERROR: CPU Crash and Reset");
    strcat(multiMessage,"Crash ");
  }   
  if(errors.radio == 1) {
    errorCount++;
    strcpy(errors.message,"ERROR: Radio Hardware Failure");
    strcat(multiMessage,"Radio ");
  }     

  if(errorCount > 1) strcpy(errors.message, multiMessage);
  errors.count = errorCount;
  if(errorCount == 0) strcpy(errors.message, "Good Health");

  if(errorCount > 0 || errors.sendIt == 1) {
    digitalWrite(led3_pin, HIGH);
    errors.count = errorCount;
    errors.sendIt = 0;
    //Send @I=issue over radio
    char tMessage[200];
    char str_int[10];
    char comma[5] = ",";
    int tempi = 0;
    strcpy(tMessage,"");
    // @I,sensors,logging,ADXL,LSM,MS56,GPS,flash,voltage,temp,I2C,radio,crash restart,count,message
    
    tempi = errors.sampleLoop;strcpy(str_int,"");iToStr(tempi, str_int, 0);strcat(tMessage,str_int);strcat(tMessage,comma);
    tempi = errors.loggingLoop;strcpy(str_int,"");iToStr(tempi, str_int, 0);strcat(tMessage,str_int);strcat(tMessage,comma);
    tempi = errors.ADXL;strcpy(str_int,"");iToStr(tempi, str_int, 0);strcat(tMessage,str_int);strcat(tMessage,comma);
    tempi = errors.LSM;strcpy(str_int,"");iToStr(tempi, str_int, 0);strcat(tMessage,str_int);strcat(tMessage,comma);
    tempi = errors.MS56;strcpy(str_int,"");iToStr(tempi, str_int, 0);strcat(tMessage,str_int);strcat(tMessage,comma);
    tempi = errors.GPS;strcpy(str_int,"");iToStr(tempi, str_int, 0);strcat(tMessage,str_int);strcat(tMessage,comma);
    tempi = errors.flash;strcpy(str_int,"");iToStr(tempi, str_int, 0);strcat(tMessage,str_int);strcat(tMessage,comma);
    tempi = errors.voltage;strcpy(str_int,"");iToStr(tempi, str_int, 0);strcat(tMessage,str_int);strcat(tMessage,comma);
    tempi = errors.CPUtemp;strcpy(str_int,"");iToStr(tempi, str_int, 0);strcat(tMessage,str_int);strcat(tMessage,comma);
    tempi = errors.I2C;strcpy(str_int,"");iToStr(tempi, str_int, 0);strcat(tMessage,str_int);strcat(tMessage,comma);
    tempi = errors.radio;strcpy(str_int,"");iToStr(tempi, str_int, 0);strcat(tMessage,str_int);strcat(tMessage,comma);
    tempi = errors.crash;strcpy(str_int,"");iToStr(tempi, str_int, 0);strcat(tMessage,str_int);strcat(tMessage,comma);
    tempi = errors.count;strcpy(str_int,"");iToStr(tempi, str_int, 0);strcat(tMessage,str_int);strcat(tMessage,comma);
    strcat(tMessage, errors.message);
    strcpy(radioHeader, "@I");   strcpy(radioMessage, tMessage);
    if(serialDebug) Serial.print("ERRORS:  ");if(serialDebug) Serial.println(tMessage);
    RadioSend();
  } else {
    digitalWrite(led3_pin, LOW);
  }
}


void getCPUtemp() {

 // Converts C to F  --  Error if over 165. It will panic at 170?
 working.CPUtemp  = (tempmonGetTemp() * 9.0f / 5.0f) + 32.0f;
 if(working.CPUtemp > working.CPUtempMax) working.CPUtempMax = working.CPUtemp;
 if(working.CPUtemp >= 170.0f) {
  errors.CPUtemp = 1; 
 } else {
  errors.CPUtemp = 2;
 }
}



void readVoltage(){
      // New 5.0 hardware (using 330k ohm / 100k ohm):  8.15 = 580, 7.0v = 497, 6.9v = 490
      int value = 0;
      float tempf = 0.0;
      float vin = 0.0;
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
      
     if(working.voltage <= 10.0f) {
      errors.voltage = 1; 
     } else {
      errors.voltage = 2;
     }
      
}


void EjectEventCheck() {

// Event State:   0 = closed (together), 1 = open (separation)

  if (events.EventFore == 0) {
     if (digitalRead(fore_pin) == HIGH) { 
          getTimeNow();
          strcpy(events.foreTime, working.timeHeader); //record fore event time
          strcpy(radioHeader, "@E");   strcpy(radioMessage, "FORE");
          RadioSend();  // ack=0,times=1,rand=0  
          strncpy(working.eventString,"FORE Separation Event",30);
          eventLog();
          events.EventFore = 1;
          if(serialDebug) Serial.println("FORE Separation");
     } 
  } //removed else. Can only have one event. Prevents multiple reports for bad contacts 
  
  if (events.EventAft == 0) {
     if (digitalRead(aft_pin) == HIGH) { 
          getTimeNow();
          strcpy(events.aftTime, working.timeHeader); //record aft event time
          strcpy(radioHeader, "@E");   strcpy(radioMessage, "AFT");
          RadioSend();  // ack=0,times=1,rand=0  
          strncpy(working.eventString,"AFT Separation Event",30);
          eventLog();
          events.EventAft = 1;
          if(serialDebug) Serial.println("AFT Separation");
     }   
  } 

  if (events.EventSep == 0) {

    if ((digitalRead(sep_pin) == HIGH && configs.invertSep == false) || (digitalRead(sep_pin) == LOW && configs.invertSep == true)) { 
          getTimeNow();
          strcpy(events.sepTime, working.timeHeader); //record aft event time
          strcpy(radioHeader, "@E");   strcpy(radioMessage, "STAGE");
          RadioSend();  // ack=0,times=1,rand=0  
          strncpy(working.eventString,"STAGING Separation Event",30);
          eventLog();
          events.EventSep = 1;
          if(serialDebug) Serial.println("STAGING Separation");
     }   
  } 
  
  if (events.EventFore == 1 && digitalRead(fore_pin) == LOW) events.EventFore = 2;  // capture the error state for reporting
  if (events.EventAft == 1 && digitalRead(aft_pin) == LOW) events.EventAft = 2;  // capture the error state for reporting (won't matter for inverted)  
  if (events.EventSep == 1 && digitalRead(sep_pin) == LOW) events.EventSep = 2;  // capture the error state for reporting  (won't matter for inverted) 

}

void getTimeNow(){
  //return is in working.timeHeader
  strcpy(working.timeHeader, "");
  if(events.clockSet == 1) {
    const char *colon = ":"; 
    char tempS[20];
    int tempI = 0;
    tempI = hour();
    strcpy(tempS,"");iToStr(tempI, tempS, 2);
    strcat(working.timeHeader, tempS);
    strcat(working.timeHeader, colon); 
    tempI = minute();
    strcpy(tempS,"");iToStr(tempI, tempS, 2);
    strcat(working.timeHeader, tempS);
    strcat(working.timeHeader, colon);
    tempI = second();
    strcpy(tempS,"");iToStr(tempI, tempS, 2);
    strcat(working.timeHeader, tempS);  
        
    } else {
       //just use the full milliseconds
       char buf[16];
       ltoa(millis(),buf,10);
       strcpy(working.timeHeader, "ms");
       strcat(working.timeHeader, buf);
       //strcpy(working.timeHeader, "no time  ");
    }
}


//***********************************************************        SAMPLE THREAD    ************************************************************
//***********************************************************        SAMPLE THREAD     ************************************************************
//***********************************************************        SAMPLE THREAD     ************************************************************

// Dedicated thread for sampling the I2C sensors (BME280 Baro, ADXL375 Accel, LSM6DSO32 6dof).

void sample_thread() {

  if (serialDebug) Serial.println("Init Sampling Thread...");
  threads.delay(10);
    
  Wire.begin();
  
  threads.delay(300);
  Wire.setClock(400000); // 400kHz I2C clock.  (can only be 100k, 400k, 1000k)
  threads.delay(400);
  errors.I2C = 2; // set to healthy

  elapsedMillis checkIn = 0;

  // ======          CONFIGURATION                          ============
    int baroRate = 0;    //millis rate - set to 1ms with new baro "six passes" approach

  // ====== MS5607 Barometer setup code and local variables ============

    baro.Baseline = 0.0; // baseline altitude to subtract
    float oldAltitude = 0.0;
    unsigned long baroReadTimer = 0;
    float altFilter = 0.0;
    unsigned long oldTime = 0.0;
    int altFilterCount = 0;
    float avgFilter = 0.0;
    int fSpeed = 0;
    float fSpeedSample[10] = {0.0};
  

    unsigned long pConv = 0;
    unsigned long pMeasure = millis() + 10000;
    unsigned long pDV = millis() + 15000;
    unsigned long tConv = millis() + 10000;
    unsigned long tMeasure = millis() + 10000;
    unsigned long tDV = millis() + 15000;  
    short CONV_D1_MB = 0x48;          // corresponding temp conv. command for OSR
    short CONV_D2_MB = 0x58;          // corresponding pressure conv. command for OSR  
    unsigned long MDP, MDT;
    int baroReady = 0;
    float T_val,H_val;
    int baselineCount = 0;
    float baselineAvg = 0.0;
    float baselineBias = 0.0;
    
    if(configs.baro) {
      if(!P_Sens.begin()){
        if(serialDebug) Serial.println("Error Communicating with MS5607, check your connections!");
        errors.MS56 = 1;
        digitalWrite(led3_pin,HIGH);
      }else{
        if(serialDebug) Serial.println("MS5607 initialization successful!");
        errors.MS56 = 2; //healthy
        //P_Sens.setOSR(2048); 
        P_Sens.setOSR(4096); 
        threads.delay(500);
        // first run
        P_Sens.startConversionMB(CONV_D1_MB); threads.delay(10);
        P_Sens.startMeasurmentMB(); threads.delay(10);
        P_Sens.getDigitalValueMB(MDP); threads.delay(10);
        P_Sens.startConversionMB(CONV_D2_MB); threads.delay(10);
        P_Sens.startMeasurmentMB(); threads.delay(10);
        P_Sens.getDigitalValueMB(MDT);
        //get first baseline
        P_Sens.setValuesMB(MDT, MDP);    
        T_val = P_Sens.getTemperature();
        H_val = P_Sens.getAltitude();  
        baro.TempF = (T_val * (float) 1.8) + (float) 32.0; //convert to F
        baro.tempAltitude = H_val * (float) 3.28084;
        baroReadTimer = millis();
      }
    }

     // ====== ADXL375  setup code and local variables ============
    int16_t x, y, z;
    int adxlBurnoutCount = 0;
    if(configs.ADXL) {
      Accel.settings.commInterface = I2C_MODE;
      //Accel.settings.I2CAddress = 0x53;
      Accel.settings.I2CAddress = 0x1D;
      if (Accel.beginI2C() == false) { //Begin communication over I2C 
        if(serialDebug) Serial.println("The ADXL sensor did not respond. Please check wiring.");
        errors.ADXL = 1;
        digitalWrite(led3_pin,HIGH);
      } else {
         if(serialDebug) Serial.println("Sensor:  Good ADXL375 sensor detected");
         errors.ADXL = 2; //healthy
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
        threads.delay(100);
      }
     } // end if configs.ADXL
      //timers 
      float ADXLxOffset = -362.26;
      float ADXLyOffset = -497.97;
      float ADXLzOffset = -59.12;



    // ====== LSM6DSO32 setup code and local variables ============
    
       // set up the master quaternion using the accelerometer ground values before launching
      int16_t gyroRead[3];       
      float Gx, Gy, Gz = 0.0;     
      float GxOffset = -.37400;
      float GyOffset = -.16113;
      float GzOffset = 0.01303;   
      //added for auto-biasing on the pad
      float GxSum = 0.0;
      float GySum = 0.0;
      float GzSum = 0.0;
      float GxOffBias = 0.0;
      float GyOffBias = 0.0;
      float GzOffBias = 0.0;
      unsigned long gyroBiasTimer = millis() + 15000;
      int gyroCounter = 0;

      unsigned long td = 0;
      elapsedMicros lastMicros; //for Gyro shift
      unsigned long gyroReset = 0;
      float dt = 0.0;
      int LSM6DBurnoutCount = 0;

      int16_t accelRead[3]; 
      float Ax, Ay, Az;      
      float AxOffset = -2.95;
      float AyOffset = -16.55;
      float AzOffset = -11.85;
         
    if(configs.LSM) {
      //basic health check
      uint8_t theID;
      theID = LSM6.whoAmI();
      if(theID == 108) { // HEX 6C :)
        errors.LSM = 2; // healthy
        //LSM settings
        LSM6.setAccelRateScale(B01100100); // 01100100 :  0110 = 416hz, 01 = 32g, 0 = no LPF, 0 = default 
        threads.delay(10);
        LSM6.setGyroRateScale(B01101100); // 01101100 :  0110 = 416hz, 11 = 2000 dps, 0 = FS, 0 = default 
        //LSM6.setAnyRegister(mLSM6DS_INT1_CTRL,B00000010); //set int1 to new gyro values
        threads.delay(10);
        LSM6.setAnyRegister(mLSM6DS_INT1_CTRL,B00000000); //set int1 to new gyro values
        threads.delay(10);
        LSM6.setAnyRegister(mLSM6DS_INT2_CTRL,B00000001); //set int2 to new gyro values
  
        gyroPrelaunch();  
        gyroReset = millis(); //start with an immediate reset the clear drift every 2 seconds
      } else {
        errors.LSM = 1;
        if(serialDebug) Serial.println("Error in Communicating with the LSM6DS Accel/Gyro");
      }

    }

     // Ready to rock...  
      
      threads.delay(200);
      Wire.setClock(400000); // 400kHz I2C clock.  (can only be 100k, 400k, 1000k) reset after initializing all the sensors
      threads.delay(300);
      if(serialDebug) Serial.println(">>> Starting Sampling  Loop...");
      lastMicros = 0;
      LSM6D.last = 0;
      adxl.last = 0;

  while(1) {  //infinite loop handles sampling for the I2C sensors  ==================================================================

    

  // =================== ADXL375  code execution  ========================
  // Important note:  Mounted with x- up, so LinZ = (x + 1000) * -1 
   if(configs.ADXL and errors.ADXL ==2) {
    if(digitalRead(intADXL) == HIGH) {  //ADXL interput pin high      
      if(debugMode) debugVars.adxlStart = micros();
      //*** Now do the ADXL375 Accelerometer tasks
        uint8_t buffer[6];
        Accel.readRegisterRegion(buffer, 0x32, 6);        
        x = (int16_t) (((int16_t)buffer[1] << 8) | buffer[0]);
        y = (int16_t) (((int16_t)buffer[3] << 8) | buffer[2]);
        z = (int16_t) (((int16_t)buffer[5] << 8) | buffer[4]);
        if(debugMode) debugVars.adxlTime += micros() - debugVars.adxlStart;  

        adxl.x = (float)x * (float)49.3;
        adxl.y = (float)y * (float)49.3;
        adxl.z = (float)z * (float)49.3;
        adxl.x += ADXLxOffset;
        adxl.y += ADXLyOffset;
        adxl.z += ADXLzOffset;             
        if(debugMode) debugVars.adxlCount++;
        
        //-------- PRE-LAUNCH BIAS CORRECTION -------------//
          if (events.launch == 0 && adxl.x > -1700 && adxl.x < -300) {  //sample for bias correction about every 10 sec within .7 range
            adxl.biasCount++;
            adxl.tempbias += (adxl.x + 1000);
            if (adxl.biasCount == 4000) {
               adxl.bias = (int) (adxl.tempbias / 4000);
               adxl.bias = adxl.bias * -1;
               adxl.tempbias = 0;
               adxl.biasCount = 0;  
               adxl.maxZ = 0;
               adxl.minZ = 0;
            }
          }
        //-------- CALCULATE LINEAR Z         -------------//  
          adxl.linZ = (float)(adxl.x + 1000.0 + adxl.bias) * (float)-1.0; 
          adxl.tempAvgZ += adxl.linZ;
          adxl.count++; if(adxl.count > 30000) adxl.count = 1;
        //-------- CALCULATE MIN MAX Z        -------------//
          if (adxl.maxZ < adxl.linZ) adxl.maxZ = adxl.linZ;
          if (adxl.minZ > adxl.linZ) adxl.minZ = adxl.linZ;
        //-------- VELOCITY CALCULATIONS      -------------//  
          if(adxl.last > 0 && micros() < adxl.last) adxl.last = 0; //trap micros roll-over
          if (events.launch == 0 && adxl.linZ > 500) { //pre launch integration (only .5g for the big guy)
            float rateD = 0.0;
            if (adxl.last == 0) {
              rateD = 0.002;
              adxl.last = micros();
            } else {
              rateD = (float)(micros() - adxl.last) / (float)1000000.0;
              adxl.last = micros();
            }
            adxl.velocity += adxl.linZ * rateD;
          }
          if (events.launch == 0 && adxl.velocity > 0 && adxl.linZ < 500) {
            adxl.velocity = 0; //clear any false starts
            adxl.last = 0;
          }
          if (events.launch == 1) { // integrate velocity
             float rateD = 0.0;
            if (adxl.last == 0) {
              rateD = (float) 1.0 / (float) 400.0;
              adxl.last = micros();
            } else {
              rateD = (float)(micros() - adxl.last) / (float)1000000.0;
              adxl.last = micros();
            }
            adxl.velocity+= adxl.linZ * rateD;       
           //-------- VELOCITY APOGEE DETECTION   -------------// 
            if (adxl.velocity < 0 && adxl.apogee == 0) {
               adxl.apogee = 1;
            } 
          } //end launch = 1
          //-------- BURNOUT CALCULATIONS      -------------//   
          if (events.launch == 1 && adxl.linZ > configs.burnoutArm && adxl.burnoutArm == 0) {
            adxl.burnoutArm = 1; // must hit 3G during launch before allowing burn-out flag
          }
          if (adxl.burnoutArm == 1 && adxl.burnout == 0) {  //watch for burnout
            if(adxl.velocity < adxl.velocityLast) adxlBurnoutCount++;
            if (adxlBurnoutCount == 10) {
              adxl.burnout = 1;
              if(events.burnout == 0) {
                events.burnoutSeconds = (float)(millis() - events.launchClock) / (float)1000.00;
                events.burnoutClock = millis();
                events.burnout = 1;
              }
            }
            adxl.velocityLast = adxl.velocity;          
          }
        threads.yield(); 
     } //end adxl interupt pin
   } // end if configs.ADXL
     
    
    // =================== LSM6DSO32  code execution  ========================

    if(configs.LSM and errors.LSM == 2) {
      if(digitalRead(int2LSM) == HIGH) {  //get an interupt from the gyro at 500hz 
        if(debugMode) debugVars.LSMstart = micros();
  
        //-------- GET ACCELEROMETER RAW DATA -------------//
        //LSM Accelerometer Functions       
        LSM6.readAccelData(accelRead); //populate accelRead[3] array with raw sample data from accelerometer
        Ax = accelRead[0] + AxOffset;
        Ay = accelRead[1] + AyOffset;
        Az = accelRead[2] + AzOffset;
         
        // Now we'll calculate the accleration value into actual g's
        Ax = Ax * (float) 0.976;  // .976 for 32G, .488 16G, .244 8G, .122 4G, .061 2G (datasheet pg 25)
        Ay = Ay * (float) 0.976;
        Az = Az * (float) 0.976;
        LSM6D.x = Ax;
        LSM6D.y = Ay;
        LSM6D.z = Az;
        if(debugMode) debugVars.LSM6DCount++;
          //-------- PRE-LAUNCH BIAS CORRECTION -------------//
            if (events.launch == 0 && LSM6D.x < 1500) {  //sample for bias correction about every 10 sec
              LSM6D.biasCount++;
              LSM6D.tempbias += LSM6D.x - 1000;
              if (LSM6D.biasCount == 8000) {
                 LSM6D.bias = (int) (LSM6D.tempbias / 8000);
                 LSM6D.bias = LSM6D.bias * -1;
                 LSM6D.tempbias = 0;
                 LSM6D.biasCount = 0;  
              }
            }
          //-------- CALCULATE LINEAR Z         -------------//  
            LSM6D.linZ = (LSM6D.x - 1000.0 + LSM6D.bias); 
            LSM6D.tempAvgZ += LSM6D.linZ;
            LSM6D.count++; if(LSM6D.count > 30000) LSM6D.count = 1;
          //-------- CALCULATE MIN MAX Z        -------------//
            if (LSM6D.maxZ < LSM6D.linZ) LSM6D.maxZ = LSM6D.linZ;
            if (LSM6D.minZ > LSM6D.linZ) LSM6D.minZ = LSM6D.linZ;
          //-------- VELOCITY CALCULATIONS      -------------//  
          if(LSM6D.last > 0 && micros() < LSM6D.last) LSM6D.last = 0; //trap micros roll-over
            if (events.launch == 0 && LSM6D.linZ > 500) { //pre launch integration (only .5g for the big guy)
              float rateD = 0.0;
              if (LSM6D.last == 0) {
                rateD = 0.002;
                LSM6D.last = micros();
              } else {
                rateD = (float)(micros() - LSM6D.last) / (float)1000000.0;
              LSM6D.last = micros();
              }
              LSM6D.velocity += LSM6D.linZ * rateD;
            }
            if (events.launch == 0 && LSM6D.velocity > 0 && LSM6D.linZ < 500) {
              LSM6D.velocity = 0; //clear any false starts
              LSM6D.last = 0;
            }
            if (events.launch == 1) { // integrate velocity
               float rateD = 0.0;
              if (LSM6D.last == 0) {
                rateD = 1/500;
                LSM6D.last = micros();
              } else {
                rateD = (float)(micros() - LSM6D.last) / (float)1000000.0;
                LSM6D.last = micros();
              }
              LSM6D.velocity+= LSM6D.linZ * rateD;       
             //-------- VELOCITY APOGEE DETECTION   -------------// 
              if (LSM6D.velocity < 0 && LSM6D.apogee == 0) {
                 LSM6D.apogee = 1;
              } 
            } //end launch = 1
            //-------- BURNOUT CALCULATIONS      -------------//   
            if (events.launch == 1 && LSM6D.linZ > configs.burnoutArm && LSM6D.burnoutArm == 0) { // launch and 3G
              LSM6D.burnoutArm = 1; // must hit 3G during launch before allowing burn-out flag
            }
            if (LSM6D.burnoutArm == 1 && LSM6D.burnout == 0) {  //watch for burnout
              if(LSM6D.linZ < (float)0.0) LSM6DBurnoutCount++;
              if (LSM6DBurnoutCount == 25) { //get a lot of samples before calling burnout.
                LSM6D.burnout = 1;
                if(events.burnout == 0) {
                  events.burnoutSeconds = (float)((float)(millis() - events.launchClock)) / (float)1000.00;
                  events.burnoutClock = millis();
                  events.burnout = 1;
                }
              }
              LSM6D.velocityLast = LSM6D.velocity;     
                  
            }
  
        //-------- GYRO RAW DATA READ      -------------// 
       
       if (strcmp(working.phase, "DESCENT") != 0) {  // do not do gyro on descent
        LSM6.readGyroData(gyroRead);   
        td = lastMicros;
        lastMicros = 0;
        dt = (float) td / (float) 1000000.0;  // this is the time interval in microseconds
        //now convert the gyro readings into degrees per second, based on the lsb per degree from the datasheet 
        Gx = (float)gyroRead[0] * (float) .070;  // 4000=140, 2000=70, 1000=35, 500=17.5, 250=8.75, 125 = 4.375 (datasheet pg 25)
        Gy = (float)gyroRead[1] * (float) .070;
        Gz = (float)gyroRead[2] * (float) .070;
        //adjust for still calibration offsets
        Gx += GxOffset;
        Gy += GyOffset;
        Gz += GzOffset;
        //auto-biasing logic
        GxSum += Gx;
        GySum += Gy;
        GzSum += Gz;
        gyroCounter++; if(gyroCounter > 30000) gyroCounter = 1;
        if((millis() > gyroBiasTimer) && (events.launch == 0))  {
          GxOffBias = (float) (GxSum / (float) gyroCounter * (float) -1.0);
          GyOffBias = (float) (GySum / (float) gyroCounter * (float) -1.0);
          GzOffBias = (float) (GzSum / (float) gyroCounter * (float) -1.0);
          GxOffset += GxOffBias;
          GyOffset += GyOffBias;
          GzOffset += GzOffBias;
          GxOffBias = 0;GyOffBias = 0;GzOffBias = 0;
          gyroCounter = 0;
          gyroBiasTimer = millis() + 15000; // correct every 15 seconds on the pad
        }
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
        
        
        if((events.launch == 0) && (millis() > gyroReset)) {
          gyroReset = millis() + 2000;
          gyroPrelaunch(); //reset drift every 2 seconds on the pad
        }
       if(debugMode) debugVars.LSMtime += (micros() - debugVars.LSMstart);
       threads.yield();
      } // end LSM6D interupt
    } // end if configs.LSM

    // =================== MS5607  Altimeter code execution  ========================
    
    // Barometer code - altitude, speed, temp (35 Hz)
    if(configs.baro && errors.MS56 == 2) {
      if (millis() > baroReadTimer) {  // set to 1ms and throttle with other settings
          // Six steps to get a good read...
          if(millis() > pConv) {  // step 1
            P_Sens.startConversionMB(CONV_D1_MB);
            pConv = millis() + 10000;  //punt until ready
            pMeasure = millis() + 9; //wait 10ms for sample
          }
          if(millis() > pMeasure) {  //step 2
            P_Sens.startMeasurmentMB();
            pMeasure = millis() + 11000; //punt until ready
            pDV = millis() + 1; //wait 2ms for measurement
          }
          if(millis() > pDV) { //step 3
            P_Sens.getDigitalValueMB(MDP);
            pDV = millis() + 12000;
            tConv = millis(); 
          }
          if(millis() > tConv) { //step 4
            P_Sens.startConversionMB(CONV_D2_MB);
            tConv = millis() + 10000;  //punt until ready
            tMeasure = millis() + 9; //wait 10ms for sample
          }
          if(millis() > tMeasure) { //step 5
            P_Sens.startMeasurmentMB();
            tMeasure = millis() + 10000; //punt until ready
            tDV = millis() + 1; //wait 2ms for measurement
          }
          if(millis() > tDV) { //step 6
            P_Sens.getDigitalValueMB(MDT);
            tDV = millis() + 10000;
            pConv = millis() + 0; //throttle rate with this spacer
            baroReady = 1;
          }
          // -------  WE HAVE A GOOD NEW READ -------------
          // ----------------------------------------------
  
          if(baroReady == 1) {
            P_Sens.setValuesMB(MDT, MDP);    
            T_val = P_Sens.getTemperature();
            baro.Tempraw = T_val;
            baro.Pressure = P_Sens.getPressure();
            H_val = P_Sens.getAltitude();   
            baro.Altraw = H_val;
            baroReady = 0;
  
             baro.TempF = (T_val * (float) 1.8) + (float) 32.0; //convert to F
             if (baro.TempF > baro.TempMax) baro.TempMax = baro.TempF;
           
             baro.tempAltitude = H_val * (float) 3.28084;
             
             if (baro.Baseline == 0.00) baro.Baseline = baro.tempAltitude; 
             float altitudeDifference;
             altitudeDifference = baro.tempAltitude - baro.Baseline;  //true AGL
  
             //correct baseline every ten seconds pre-launch
             if (events.launch == 0) {
              baselineCount++;
              baselineAvg += altitudeDifference;
              if(baselineCount == 400) {
                baselineBias = (float) ((float) baselineAvg / (float) baselineCount);
                baro.Baseline += baselineBias;
                baselineAvg = 0;
                baselineCount = 0;
              }
             }
             
             if (altitudeDifference < 5.0 && altitudeDifference > -5.0) {  // zero out altitude for baro noise
              altitudeDifference = 0;
              baro.Altitude = 0;  
             } else {
              baro.Altitude = (baro.tempAltitude - baro.Baseline);  // true AGL
             }  
             if (baro.Altitude > baro.AltitudeMax) baro.AltitudeMax = baro.Altitude;  // record max baro altitude  (AGL)
             
             //baro speed calculations here - two filters. 1) every 19 samples (.5 sec) and 2) every four seconds lagging (for Descent phase).
             altFilterCount++;
             altFilter += baro.Altitude;
             if(altFilterCount == 19) {  // filter altitude over half a second
               //half second filter
               avgFilter = altFilter / (float) 19.0;
               float tSpeed = (float) 0.0;
               tSpeed = avgFilter - oldAltitude;
               if(millis() != oldTime) { 
                  tSpeed = tSpeed * ((float) 1000.0 / (float) (millis() - oldTime));
               } else {
                  tSpeed = -9999.0;
               }
               // four second speed filter starts here
               fSpeed++;
               if(fSpeed == 9) fSpeed = 1;
               fSpeedSample[fSpeed] = tSpeed;
               float fSpeedSum = 0.0;
               for(int i = 1; i < 9; i++) { 
                fSpeedSum += fSpeedSample[i];
                }
               float fSpeedResult = 0.0;
               fSpeedResult = fSpeedSum / (float) 8.0;
               // set the speed
               if (strcmp(working.phase, "DESCENT") == 0 && millis() > (events.apogeeClock + 2000)) { // switch to four second filter for descent + 2 sec.
                baro.Speed = fSpeedResult;
               } else {
                baro.Speed = tSpeed;
               }
               // end four second buffer
               if (baro.Speed > baro.SpeedMax) baro.SpeedMax = baro.Speed;   // set max speed
               oldAltitude = avgFilter;
               oldTime = millis();
               altFilterCount = 0;
               altFilter = (float) 0.0;
               avgFilter = (float) 0.0;
             }
             
           if(debugMode) debugVars.baroCount++;
         } // end good baro read
       
       baroReadTimer = millis() + baroRate; //set to 50 hz
     } // end if millis baro timer
    } // end if configs.baro


    if (checkIn > 250) { 
          working.sampleClock = millis(); //heartbeat clock to make sure thread is alive
          checkIn = 0;
    }
   threads.yield(); 
 } // end while(1)
} // end sample_thread


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
