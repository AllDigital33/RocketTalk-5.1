
/*   ***********  Rocket Talk Booster Code - copyright 2018 @Pad33space  ***********
 */


  bool serialDebug = true;  // set to true to get Serial output for debugging
  bool debugMode = true;

  #define voltage_pinA A0
  #define RXD1 0
  #define TXD1 1
  #define RXD2 7
  #define TXD2 8  
  #define continuityMainPin 5
  #define continuityDroguePin 4
  #define relayMain 2
  #define relayDrogue 3
  #define ledpin 13

//Teensy
  #include <TeensyThreads.h>
  #include "Arduino.h"
  #include <TimeLib.h>  // for logging time

  // For BME280
  #include "SparkFunBME280.h"
  BME280 mySensor;
  unsigned long BME280timer;


  struct configStruct {
   int machLockout = 7000;  // seconds to lock-out baro for mach
   int altitudeMin = 500;  // 500 feet minimum before pyro events
   float apogeeSpeed = 5.0; //apogee below 5fps speed
   float apogeeVelocity = 500.0; //apogee below integrated velocity of 500 on adxl
   int stagingMaxTime = 25000; // 25 seconds post launch max for staging
   float stagingSpeed = 500.0; // need to be > 500fps for staging
   int burnoutPlus = 2000; // separation two seconds after burnout
   int burnoutArm = 3000; // 3G - must hit three G before allowing burnout
   int sustainerTime = 2000; // ignite sustainer two seconds after separation
   float stagingTilt = 35.0; // 35 degree lock-out for firing sustainer
   float mainAltitude = 1500.0; // deploy main at 1500 feet
   int pyroDuration = 2000; // 2 second hold for pyros
   
  };
  configStruct configs;



  struct workingStruct {  // working variables
    char phase[25];   // Normally should be set to STARTUP
    char callsign[10];  //Radio requires Amatuer Radio Licence
    // Testing Phase variables
    char FlightLogMaster[500];
    char RadioLogMaster[300];
    char FlightSummaryMaster[400];
    char eventString[200];
    byte logFastMode = 0; //used to keep log files open during ascent
    byte flightLogOpen = 0;
    byte gpsLogOpen = 0;
    byte radioLogOpen = 0;
    char sendFlight[150];
    char sendSummary1[150];
    char sendSummary2[150];
    char sendSummary3[150];
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
    unsigned long gpsTimer = 0;
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
    unsigned long pyroSafetyTimer = 0;
    int gpsInterval = 5000;
    int sendFlightInterval = 4000;
    int voltageInterval = 1000;
    int eventsInterval = 500;
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
    unsigned long launchDetectDelay;
    byte apogee;  
    int count;  
    byte launch;
  };
  baroStruct baro;

  // GPS
  struct gpsDataStruct {  // GPS variables
    char vGNRMC[200]; 
    char vGNGGA[200]; 
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
    float baseline;
    float maxAGLfeet;
    char gpsLogLine[300];
    char gpsSendLine[150];
    int gpsStatus;
  };
  gpsDataStruct gpsData;
  char aGNRMC[20][20]={0x0}; //for parsing
  char aGNGGA[20][20]={0x0};
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
    int baroCount;
    int count1;
    unsigned long debugTimer;
  };
  debugStruct debugVars;


//***********************************************************        SETUP    ************************************************************
//***********************************************************        SETUP     ************************************************************


void setup() {

// Pin setup

  pinMode(relayMain, OUTPUT);  digitalWrite(relayMain, LOW);
  pinMode(relayDrogue, OUTPUT);  digitalWrite(relayDrogue, LOW);
  pinMode(ledpin, OUTPUT);  digitalWrite(ledpin, LOW);
  pinMode(voltage_pinA, INPUT);
  pinMode(continuityMainPin, INPUT_PULLUP);   
  pinMode(continuityDroguePin, INPUT_PULLUP);   

  Serial.begin(115200);
  if(serialDebug) Serial.println("Starting up....");
  delay(2000);

   // Initialize GPS
    delay(1000);
    Serial2.begin(9600); // for M8N 
    delay(100); // Little delay before flushing
    Serial2.flush();
    delay(100);
    changeBaudrate();  
    delay(100);
    changeFrequency();  // Increase frequency to 100 ms
    delay(100);
    filterNMEA();  // Filtering out all NMEA codes but two
    delay(100);
    changeGPSDynamicModel();  //Change the GPS Dynamic Model

  // Initialize Radio  (Radio at 435.92 Mhz / 4800b radio-radio, but 19.2b cpu-radio)
  Serial1.begin(19200);

  //create separate thread loop for sampling
  threads.addThread(sample_thread);

  digitalWrite(ledpin,HIGH);delay(250);
  digitalWrite(ledpin,LOW);delay(250);
  digitalWrite(ledpin,HIGH);delay(250);
  digitalWrite(ledpin,LOW);

  readVoltage();
  strcpy(working.callsign, "KK6EVH");
  //check continuity before starting up
  pyroContinuity(); 
  if(serialDebug) Serial.println(">>> Starting Main Loop...");
  PhaseStartupInit();
  delay(3000);

}

//***********************************************************        LOOP    ************************************************************
//***********************************************************        LOOP     ************************************************************

void loop() {


  //=================  HOUSEKEEPING ==================== //

  if(millis() > timers.gpsTimer) {
    CheckGPS();
    if (gpsData.gpsStatus == 2) {
      strcpy(radioHeader, "@#G"); strcpy(radioMessage, gpsData.gpsSendLine);
      RadioSend();             
    }        
    timers.gpsTimer = millis() + timers.gpsInterval;
    threads.yield();
  }
  if(millis() > timers.sendFlightTimer) {
    flightRadioString();
    strcpy(radioHeader, "@#F"); strcpy(radioMessage, working.sendFlight);
    RadioSend(); 
    timers.sendFlightTimer = millis() + timers.sendFlightInterval;
    threads.yield();
  }  
  if(millis() > timers.voltageTimer) {
    readVoltage();
    timers.voltageTimer = millis() + timers.voltageInterval;
    threads.yield();
  }
  if(millis() > timers.radioRXTimer) {
    checkRadio();
    timers.radioRXTimer = millis() + timers.radioRXInterval;
    threads.yield();
  }
  if(millis() > timers.radioTXTimer && rSendCount > 0) {
    RadioSendQueue();
    timers.radioTXTimer = millis() + timers.radioTXInterval; //1200 default spacing
    threads.yield();
  }
  if(millis() > timers.pyroOffTimer) {
    pyroTimers();
    timers.pyroOffTimer = millis() + 200;  //check pyros for off every .2 sec
    threads.yield();
  }

  if(millis() > timers.pyroContinuityTimer) {
    pyroContinuity();
    timers.pyroContinuityTimer = millis() + 2000;  //check pyros for off every 2 sec
    threads.yield();
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

//***********************************************************        PHASES    ************************************************************
//***********************************************************        PHASES    ************************************************************


// ==========================  PHASE STARTUP ===========================
void PhaseStartupInit() {
  
  timers.sendFlightInterval = 15000; // should be >= log interval 
  timers.gpsInterval = 15000;
  timers.voltageInterval = 1000;
  timers.continuityInterval = 4000;
  timers.radioRXInterval = 500;
  timers.radioTXInterval = 2000;
  strcpy(working.phase, "STARTUP");
  strcpy(radioHeader, "@M"); strcpy(radioMessage, "Booster reporting for duty");
  RadioSend();   
  timers.PhaseTimerA = millis() + 10000;  //transmit errors during startup
  timers.PhaseTimerB = millis();  //15 sec for Radio and Logging
  working.retries = 0;
}

void PhaseStartup() {

  if(millis() > timers.PhaseTimerA && (working.error)) {
      strcpy(radioHeader, "@M"); strcpy(radioMessage, working.errorMessage);
      RadioSend(); 
      timers.PhaseTimerA = millis() + 10000;
  }
  if(millis() > timers.PhaseTimerB) {
      if (gpsData.gpsStatus == 2) {   // ready to rock. Move to waiting phase. 
        PhaseWaitingInit();
        return;
      } else {  // try for ten seconds more 15 times
        working.retries++;
        timers.PhaseTimerB = millis() + 15000; 
      }
      // check for GPS timeout and abort to Waiting phase
      if (working.retries == 10) {
        PhaseWaitingInit();
        return;    
      }
  }  
  threads.yield();
}

// ==========================  PHASE WAITING ===========================

void PhaseWaitingInit() {

    timers.sendFlightInterval = 15000; // should be >= log interval 
    timers.gpsInterval = 15000;
    strcpy(working.phase, "WAITING");
    strcpy(radioHeader, "@M"); strcpy(radioMessage, "Booster Waiting for launch");
    RadioSend();            
    timers.sendFlightTimer = millis(); //force radio flight update
    if(serialDebug) Serial.println(F("Phase:  Waiting for launch mode"));
    //reset variables after warm-up 
    baro.Baseline = baro.tempAltitude; //forces a reset of baseline to altitude  
    baro.AltitudeMax = 0;
    baro.SpeedMax = 0;
    gpsData.maxAGLfeet = 0;
    gpsData.baseline = 0.0;
    working.triggerCount = 0;
    working.triggerCount2 = 0;
    threads.yield();
}

void PhaseWaiting() {

    // *** LAUNCH TEST TWO - BAROMETER ***  (conservative backup)
    // three samples of >50 ft in a row -- use 100ms delay to prevent radio interference 
    if (baro.Altitude > 50.0 && millis() > baro.launchDetectDelay) {  // use 50' for now as absolute launch detect 
      working.triggerCount2++;
      baro.launchDetectDelay = millis() + 100; //100ms to prevent false positive
      if(working.triggerCount2 == 3) {
        baro.launch = 1;
        PhaseLaunchInit();
        return;
      }
    }
     if (working.triggerCount2 > 0 && baro.Altitude < 25.0) {
        working.triggerCount2 = 0;  // reset counter if there was a false alarm   
    }  
    threads.yield();
}



// ==========================  PHASE LAUNCH - PARTY ON! ===========================

void PhaseLaunchInit() {

    events.launchClock = millis();
    timers.sendFlightInterval = 10000; // every 10 seconds on ascent
    timers.gpsInterval = 10000; // check GPS every 6 seconds on ascent
    strcpy(working.phase, "LAUNCH");
    timers.flightLogTimer = millis();
    threads.yield();
    strcpy(radioHeader, "@M"); strcpy(radioMessage, "Booster Launch");
    RadioSend();            
    if(serialDebug) Serial.println(F("LAUNCH DETECT!"));
    events.launch = 1;
    getTimeNow();
    strcpy(events.launchTime, working.timeHeader); //record launch time in string
    baro.lastAltitude = baro.Altitude; 
    working.triggerCount = 0; //reset for next phase
    working.triggerCount2 = 0; 
    timers.PhaseTimerB = millis() + 100;  //used for launch max tracking 
    threads.yield();  
}

void PhaseLaunch() {

    threads.yield();  
    if(millis() > timers.PhaseTimerB) {  // check for events every 20ms

       if (baro.AltitudeMax > configs.altitudeMin && millis() > (events.launchClock + configs.machLockout)) {  // wait until >500' and 5 sec to check everything
  
          //-------------  BARO  Apogee Logic  ------------------------ 
           if(baro.Altitude < (baro.AltitudeMax - 30) && baro.Altitude <= baro.lastAltitude) { //sampling faster than baro
              working.triggerCount++;       
              threads.yield(); 
           } else {
              working.triggerCount = 0;
           }
           if(working.triggerCount == 4) {  // We have apogee!  (4 = 300ms ) 
            PhaseDescentInit();
            return;
           }
       } //end Max>500
      baro.lastAltitude = baro.Altitude;

      timers.PhaseTimerB = millis() + 100;
    } // end phase timerB event loop
    threads.yield();  
} // End Launch Phase


// ==========================  PHASE DESCENT ===========================

void PhaseDescentInit() {

    pyroDrogue(); //Fire Drogue at Apogee 
    events.apogeeClock = millis();
    events.ascentSeconds = (millis() - events.launchClock) / 1000;
    getTimeNow();
    strcpy(events.apogeeTime, working.timeHeader); //record apogee time
    events.apogee = 1;
    strcpy(working.phase,"DESCENT");           
    strcpy(radioHeader, "@M"); strcpy(radioMessage, "Booster Apogee Detect");
    RadioSend();  
    timers.sendFlightTimer = millis(); //force radio flight update
    timers.sendFlightInterval = 5000; // every 5 seconds on descent
    timers.gpsInterval = 5000; // check GPS every 5 seconds on descent
    baro.lastAltitude = baro.Altitude;   
    if(serialDebug) Serial.println(F("APOGEE DETECT!"));    
    working.triggerCount = 0; //reset for next phase
    working.triggerCount2 = 0; 

    timers.PhaseTimerB = millis() + 200;  // check for main deploy every 200ms
    timers.PhaseTimerC = millis() + 5000; // check for landed
    threads.yield();  
}

void PhaseDescent() {


    //-------------  Main Deploy Logic  ------------------------ 
    if(millis() > timers.PhaseTimerB && events.pyroMain == 0) {
      if(baro.Altitude < configs.mainAltitude && millis() > (events.launchClock + configs.machLockout)) {
        working.triggerCount++;
        threads.yield();
      } else {
        working.triggerCount = 0;
      }
      if(working.triggerCount == 3) {
        events.mainClock = millis();
        pyroMain();
      }
      timers.PhaseTimerB = millis() + 200;  
    }
    
    threads.yield(); 

    //-------------  Landed Detect Logic  ------------------------    
    if(millis() > timers.PhaseTimerC) {

        if (baro.Altitude < (baro.lastAltitude + 10) && baro.Altitude > (baro.lastAltitude - 10)) {   // LANDED detect based on standing still barometer 
            working.triggerCount2++; 
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
   threads.yield(); 
}

// ==========================  PHASE LANDED ===========================

void PhaseLandedInit() {

    events.landedClock = millis();
    events.descentSeconds = (millis() - events.apogeeClock) / 1000;
    getTimeNow();
    events.landed = 1;
    strcpy(working.phase,"XLANDED");           
    strcpy(radioHeader, "@M"); strcpy(radioMessage, "Booster Landed");
    RadioSend();  
    timers.sendFlightTimer = millis(); //force radio flight update
    timers.flightLogInterval = 15000;  // log every 40ms / >20hz  // keep logging fast for a while
    timers.sendFlightInterval = 15000; // every 4 seconds on descent
    timers.gpsInterval = 10000; // check GPS every 5 seconds on ascent 
    if(serialDebug) Serial.println(F("LANDED DETECT!"));    
    timers.PhaseTimerA = millis() + 15000;  //used for sending summary
    // log summary
    flightSummaryString();
    working.summaryWrite = 1;
    threads.yield();  
}

void PhaseLanded() {

      if (millis() > timers.PhaseTimerA) {  // send the summary 
        summaryRepeater();
        timers.PhaseTimerA = millis() + 15000;  
      }     
      
      threads.yield(); 
}

// ==========================  PHASE BASIC ===========================

void PhaseBasicInit() {


   strcpy(working.phase, "BASIC");
   strcpy(radioHeader, "@M");   strcpy(radioMessage, "Booster in Basic Mode...");
   RadioSend(); 
   timers.sendFlightInterval = 10000; // every 5 seconds in basic mode
   timers.gpsInterval = 10000; // check GPS every 5 seconds in basic mode
   threads.yield(); 
}

void PhaseBasic() {

  // nothing to do but hang out and send data back...
  threads.yield(); 
  
}


//***********************************************************        PYRO EVENTS (BOOM!)      ************************************************************


void pyroDrogue() {
  
  if(events.pyroDrogue != 1) {
    timers.pyroDrogueTimer = millis() + configs.pyroDuration;
    digitalWrite(relayDrogue, HIGH);  // fire away!
    events.pyroDrogue = 1;
    strcpy(radioHeader, "@M");   strcpy(radioMessage, "Booster Pyro Drogue FIRED!");
    RadioSend();    
  }
  threads.yield();
}

void pyroMain() {
    if(events.pyroMain != 1) {
    timers.pyroMainTimer = millis() + configs.pyroDuration;
    digitalWrite(relayMain, HIGH);  // fire away!
    events.pyroMain = 1;
    strcpy(radioHeader, "@M");   strcpy(radioMessage, "Booster Pyro Main FIRED!");
    RadioSend();    
  }
  threads.yield();
}

void pyroBoom() {
  // DANGER!!  EMERGENCY DETONATE OF ALL EVENT SEPARATION PYROS!
  // Main, Separation, Droge in that order
  pyroDrogue();
  delay(500);
  pyroMain();
  strcpy(radioHeader, "@M");   strcpy(radioMessage, "Booster BOOM Pyro Events");
  RadioSend();   
}

void pyroTimers() {

  if(timers.pyroMainTimer != 0 && millis() > timers.pyroMainTimer) { //shut off main pyro
    digitalWrite(relayMain, LOW);
    timers.pyroMainTimer = 0;
  }
   if(timers.pyroDrogueTimer != 0 && millis() > timers.pyroDrogueTimer) { //shut off drogue pyro
    digitalWrite(relayDrogue, LOW);
    timers.pyroDrogueTimer = 0;
  }
}

void pyroContinuity() {

    
    if(digitalRead(continuityMainPin) == LOW && events.continuityMain == 0) {
      events.continuityMain = 1;
    }
    if(digitalRead(continuityMainPin) == HIGH && events.continuityMain == 1) {
      events.continuityMain = 0;
    }  

    if(digitalRead(continuityDroguePin) == LOW && events.continuityDrogue == 0) {
      events.continuityDrogue = 1;
    }
    if(digitalRead(continuityDroguePin) == HIGH && events.continuityDrogue == 1) {
      events.continuityDrogue = 0;
    }       
}




//***********************************************************        GPS LOGIC    ************************************************************
//***********************************************************        GPS LOGIC    ************************************************************


void changeBaudrate()    // currently set to 19200 as test
{  



//    byte packet[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0x4B, 0x00, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x51,};  //19200
    byte packet[] = {0xB5, 0x62, 0x06, 0x00, 0x14, 0x00, 0x01, 0x00, 0x00, 0x00, 0xD0, 0x08, 0x00, 0x00, 0x00, 0xC2, 0x01, 0x00, 0x07, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0xBF, 0x78,};  //115200    
    sendPacket(packet, sizeof(packet));

    delay(100); // Little delay before flushing
    Serial2.flush();
    Serial2.begin(115200);
}

void changeFrequency()
{
    // CFG-RATE packet   
    //  10 Hz = B5 62 06 08 06 00 64 00 01 00 01 00 7A 12 
    //   1 Hz = B5 62 06 08 06 00 E8 03 01 00 01 00 01 39
    //  .2 Hz = B5 62 06 08 06 00 88 13 01 00 01 00 B1 49   
     byte packet[] = {0xB5,0x62,0x06,0x08,0x06,0x00,0x64,0x00,0x01,0x00,0x01,0x00,0x7A,0x12,}; //10 Hz M8N
     
    //byte packet[] = {0xB5,0x62,0x06,0x8A,0x0A,0x00,0x00,0x01,0x00,0x00,0x01,0x00,0x21,0x30,0x37,0x00,0x24,0x5F,}; //19Hz 55ms M9N
    sendPacket(packet, sizeof(packet));
}

void filterNMEA()
{   
    // remove GSV B5 62 06 01 08 00 F0 03 00 00 00 00 00 00 02 38
    byte packet[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38,};
    sendPacket(packet, sizeof(packet));

    // remove VTG B5 62 06 01 08 00 F0 05 00 00 00 00 00 00 04 46
    byte packet2[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46,};
    sendPacket(packet2, sizeof(packet2));

    // remove GSA B5 62 06 01 08 00 F0 02 00 00 00 00 00 00 01 31
    byte packet3[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31,};
    sendPacket(packet3, sizeof(packet3));

    // remove GLL B5 62 06 01 08 00 F0 01 00 00 00 00 00 00 00 2A
    byte packet4[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A,};
    sendPacket(packet4, sizeof(packet4));   
}

void changeGPSDynamicModel()  // Dynamic Model for different GPS environments
{
    // CFG-NAV5 packet
    // Basic Portable = B5 62 06 24 24 00 FF FF 00 03 00 00 00 00 10 27 00 00 05 00 FA 00 FA 00 64 00 2C 01 00 00 00 00 10 27 00 00 00 00 00 00 00 00 47 0F
    // Airborne 4g =    B5 62 06 24 24 00 FF FF 08 03 00 00 00 00 10 27 00 00 05 00 FA 00 FA 00 64 00 2C 01 00 00 00 00 10 27 00 00 00 00 00 00 00 00 4F 1F
    // Stationary  =    B5 62 06 24 24 00 FF FF 02 03 00 00 00 00 10 27 00 00 05 00 FA 00 FA 00 64 00 2C 01 00 00 00 00 10 27 00 00 00 00 00 00 00 00 49 53
//    byte packet[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x47, 0x0F,}; // Basic Portable 
    byte packet[] = {0xB5, 0x62, 0x06, 0x24, 0x24, 0x00, 0xFF, 0xFF, 0x08, 0x03, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x05, 0x00, 0xFA, 0x00, 0xFA, 0x00, 0x64, 0x00, 0x2C, 0x01, 0x00, 0x00, 0x00, 0x00, 0x10, 0x27, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F, 0x1F,}; // Airborne 4g M8N
//    byte packet[] = {0xB5, 0x62, 0x06, 0x8A, 0x09, 0x00, 0x00, 0x01, 0x00, 0x00, 0x21, 0x00, 0x11, 0x20, 0x08, 0xF4, 0x51,}; // Airborne 4g for M9N GPS

    sendPacket(packet, sizeof(packet));
}

void sendPacket(byte *packet, byte len)
{
    for (byte i = 0; i < len; i++)
     {  Serial2.write(packet[i]);  }
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
  int i;
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



void CheckGPS() {    //*********************************  Check the GPS code ********************************************

  digitalWrite(ledpin, HIGH);
  int vRMC = false;
  int vGGA = false;
  int vDone = false;
  strcpy(gpsData.vGNRMC, "");
  strcpy(gpsData.vGNGGA, ""); 
  int UTCoffset = -7;
  char ReadString[100] = "";
  int vCheck = false;
  unsigned long startTime;
  startTime = millis() + 500; // 500ms timeout for reading GPS
  //Serial.println(millis());

  // *** Get new serial data from the GPS ***
  while(Serial2.available()){  //flush the serial buffer so you don't get an old reading
    Serial2.read();} 
  while (vCheck == false) {
   int size = Serial2.readBytesUntil(13,ReadString,99);
   //if(serialDebug) Serial.println(ReadString);
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
    strcpy(ReadString, "");
    if (millis() > startTime) { // timeout
      gpsData.gpsStatus = 0;
      if(serialDebug) Serial.println("GPS timeout");
    //  Serial.println(millis());
      strcpy(gpsData.gpsLogLine, "Error on GPS - Timeout");
      return;
    }
    threads.yield(); 
   }
  // *** success, got new data from GPS without timeout ***

   char tempGNRMC[200];
   char tempGNGGA[200];
   strcpy(tempGNRMC,gpsData.vGNRMC);
   strcpy(tempGNGGA, gpsData.vGNGGA);
   parseit(tempGNRMC,aGNRMC);
   parseit(tempGNGGA,aGNGGA); 
  // Serial.println(gpsData.vGNRMC);
  // Serial.println(gpsData.vGNGGA);
   
    if(strcmp(aGNRMC[2],"A") != 0) {  // no fix - abort
      strcpy(working.gpsString, "No GPS fix:  ");
      strcat(working.gpsString,gpsData.vGNRMC);
      if(serialDebug) Serial.println(working.gpsString);
      gpsData.gpsStatus = 1;
      digitalWrite(ledpin, LOW);
      return;
     }
  //Serial.println(millis());
  // Success - Good GPS data and has fix, so proceed to load data

   // *******************  Populate the GPS Data from NMEA data  ************************

    

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
    
    gpsLogString();
    strcpy(working.gpsString, gpsData.gpsLogLine); 

    gpsRadioString();
    
    if (gpsData.gpsStatus < 2) {
      if(events.clockSet == 0) {
        events.clockSet = 1;
        setTheTime();
        strcpy(working.eventString, "GPS Active - Acquired Time from GPS "); 
        strcpy(radioHeader, "@M");   strcpy(radioMessage, "Rocket Got Initial GPS Lock");
      } else {
        strcpy(working.eventString, "GPS Regained GPS Lock");
        strcpy(radioHeader, "@M");   strcpy(radioMessage, "Rocket Regained GPS Lock");
      }

 //zzz     RadioSend();      
    }
    gpsData.gpsStatus = 2;
    digitalWrite(ledpin, LOW);
        
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
   char str_feet[16];
   strcpy(str_feet, "");sprintf (str_feet, "%1.0f" , gpsData.maxAGLfeet);  
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
   char str_feet[16];
   strcpy(str_feet, "");sprintf (str_feet, "%1.0f" , gpsData.maxAGLfeet);  
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

void setTheTime() {  // set the time using the GPS - note the UTC offset

    char target[20]="";
    int vHour;
    int vMinute;
    int vSecond;
    int vYear;
    int vMonth;
    int vDay;
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



void RadioSend() {  //New and improved
  //populate global radioHeader (e.g. @M) and radioMessage first
    char str_int[16];
    strcpy(radioMessageS, "");
    strcat(radioMessageS, radioHeader);
    strcat(radioMessageS, ",");    
    strcpy(str_int, "");
    sprintf (str_int, "%d" , working.LaunchNumber); //Launch number
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


void RadioSendQueue () {

  // this is designed to cut down on send congestion. Buffer packets 1000ms apart so they don't mangle together
  // Globals:  String rSend[10];  byte rSendCount = 0;  rSendLast = 0; rSendPos = 0; RadioSendQueueRate = 1000
   if(rSendCount > 0) {
     digitalWrite(ledpin, HIGH); //blink onboard light
     rSendLast ++;
     if (rSendLast == 11) rSendLast = 1;
     Serial1.print(rSend[rSendLast]);
     Serial1.flush(); //waits for outgoing transmission to complete
     rSendCount = rSendCount - 1; 
     digitalWrite(ledpin, LOW);
   }
}



void checkRadio() {  // **********  CHECK RADIO  *****************
   newWord = 0;
   int vtimeout = 0;
   int junk;
   char receivedChar;
   if (Serial1.available() > 0) {
  //  Serial.println("Rx...");
    unsigned long testTime = millis() + 2000;
    
    strcpy(theWord, "");
    if(serialDebug) Serial.println("radio inbound detect");

    while (newWord != 1) {
       if(Serial1.available()) {

         receivedChar = Serial1.read();
         if(receivedChar == 33) {  // look for ! to end
          newWord = 1;
          append(theWord, receivedChar);
          Serial1.read();  // read the extra end char
         } else {
           append(theWord, receivedChar);
         }
         if(receivedChar < 32 || receivedChar > 126) {  //noise or garbage on the radio - empty and abort to not waste cycles
            while(Serial.available()) {
              receivedChar = Serial1.read();
              if(millis() > testTime) return;
            }
            return;
            if(serialDebug) Serial.print("**** Radio noise ignored ****");
         }
       }
       if(millis() > testTime) {  // exit and evaluate if it takes too long
          newWord =1;
          vtimeout = 1;
          if(serialDebug) Serial.println("timeout exit error");
          if(serialDebug) Serial.println(theWord);
          exit;
       }
       delay(1);
     }
     if(serialDebug) {Serial.print("Radio RX: ");Serial.println(theWord);}
     ProcessRadio();
     
   }
}


void ProcessRadio() {   // **********  PROCESS RADIO COMMAND  *****************

      int chkNew = 0;
      char theID[20];
      if(theWord[0] == 64 && theWord[strlen(theWord)-1] == 33) {   // proper sentence recieved from radio @ !          
          chkNew = 1;

      } else {  // error
          // write the SD card
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
      //  @O,BOOM = BOOM
      //  @O,MAIN = main pyro, @O,DROGUE = pyro drogue, @O,SEP = separate, @O,SUS = fire sustainer
      
      if (chkNew == 1) {  // Real processing goes here            ********************************  RADIO PROCESSING  ******************************

          if (strncmp("@#S,",theWord,4) == 0) { 
            if(serialDebug) Serial.println(F("Got radio action STATUS"));
            strcpy(radioHeader, "@M");
            strcpy(radioMessage, "Hello from Rocket");
            RadioSend();           
          }
          
          if (strncmp("@#A,",theWord,4) == 0) {  // used to terminate waiting phase and close logs for retention
             //working.phase = "ABORT";  // this will shut down processing
             strcpy(working.phase, "ABORT");

             delay(500);
             strcpy(radioHeader, "@M");   strcpy(radioMessage, "ABORT Processed - OK to Reset");
             RadioSend();   
             if(serialDebug) Serial.println(F("Completed radio action ABORT"));
             //Beep(4);
          }

          if (strncmp("@#L,",theWord,4) == 0 && strcmp("LAUNCH",working.phase) != 0) {  // used to manually transition from waiting to launch if no launch detect

             strcpy(radioHeader, "@M");   strcpy(radioMessage, "Manual Launch Initiation");
             RadioSend();   
             PhaseLaunchInit();
          }
          if (strncmp("@#D,",theWord,4) == 0 && strcmp("DESCENT",working.phase) != 0) {  // used to manually transition to descent mode (primarily for testing)

             strcpy(radioHeader, "@M");   strcpy(radioMessage, "Descent Mode Initiation");
             RadioSend();   
           PhaseDescentInit();
          }
          if (strncmp("@#T,",theWord,4) == 0) {  // used to transmit a radio test set of strings 
             radioTest();
          }    
     
          
          //Do Reset
          if (strncmp("@#R,",theWord,4) == 0 && strcmp("ABORT",working.phase) == 0) {  // used to reset on the stand if false launch detect
             strcpy(radioHeader, "@M");   strcpy(radioMessage, "Rocket Reset Processed");
             RadioSend();
   //zzz          ResetLaunch();
          }
          if (strncmp("@#B,",theWord,4) == 0) {  // used to put rocket into basic mode after in air reset

             strcpy(radioHeader, "@M");   strcpy(radioMessage, "Basic Mode Initiated");
             RadioSend();    
            PhaseBasicInit();
          }          


          if (strncmp("@#O,BOOM",theWord,8) == 0) {  // This is boom under lock and key. Not sure what purpose yet. Fire ejection manually.
             //hope to never use this!!
             events.pyroSeparation = 0;events.pyroMain = 0;events.pyroDrogue = 0;
             strcpy(radioHeader, "@M");   strcpy(radioMessage, "BOOM ALL! Received by rocket");
             RadioSend(); 
             pyroBoom();
          }
          if (strncmp("@#O,MAIN",theWord,8) == 0) {  // Pyro ignite Main
             events.pyroMain = 0; //reset if second fire
             strcpy(radioHeader, "@M");   strcpy(radioMessage, "BOOM MAIN! Received by rocket");
             RadioSend(); 
             pyroMain();
          }
          if (strncmp("@#O,DROGUE",theWord,10) == 0) {  // Pyro ignite Drogue
             events.pyroDrogue = 0; //reset if second fire
             strcpy(radioHeader, "@M");   strcpy(radioMessage, "BOOM DROGUE! Received by rocket");
             RadioSend(); 
             pyroDrogue();
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

/* zzz
void ResetLaunch() {
   getLaunchNum(); // reset the launch number
    // Initiate Startup
   startLogs();
   strcpy(working.phase, "STARTUP");
   PhaseStartupInit();    
  //say hello on radio
   strcpy(radioHeader, "@M");   strcpy(radioMessage, "Hello. Initiating Reset Startup...");
   RadioSend(); 
}
*/


void radioTest() {
  delay(500); // make RX radio is clear
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




void flightRadioString() {

//Send 4.0 version:  phase(1char), alt, alt max, speed, speed max, temp, max temp, voltage,
//                   pyroMain,pyroDrogue,continuityMain,continuityDrogue

  char comma[5] = ",";
  char str_int[16];
  char tempstr[25];
  
  strcpy(working.sendFlight, "");  //zero out the string
  strcpy(tempstr, working.phase);
  tempstr[1] = '\0'; //truncate the phase (poorman style)
  strcat(working.sendFlight, tempstr);strcat(working.sendFlight, comma); //only use the first letter
  strcpy(str_int, "");       
  sprintf (str_int, "%1.0f" , baro.Altitude);
  strcat(working.sendFlight, str_int);strcat(working.sendFlight, comma);
  strcpy(str_int, "");       
  sprintf (str_int, "%1.0f" , baro.AltitudeMax);
  strcat(working.sendFlight, str_int);strcat(working.sendFlight, comma);  
  strcpy(str_int, "");       
  sprintf (str_int, "%1.0f" , baro.Speed);
  strcat(working.sendFlight, str_int);strcat(working.sendFlight, comma);   
  strcpy(str_int, "");       
  sprintf (str_int, "%1.0f" , baro.SpeedMax);
  strcat(working.sendFlight, str_int);strcat(working.sendFlight, comma); 
  sprintf (str_int, "%1.0f" , baro.TempF); ltrim(str_int);
  strcat(working.sendFlight, str_int);strcat(working.sendFlight, comma); 
  strcpy(str_int, "");       
  sprintf (str_int, "%1.0f" , baro.TempMax); ltrim(str_int);
  strcat(working.sendFlight, str_int);strcat(working.sendFlight, comma); 
  strcpy(str_int, "");       
  sprintf (str_int, "%1.0f" , working.voltage);
  strcat(working.sendFlight, str_int);strcat(working.sendFlight, comma); 
  strcpy(str_int, "");sprintf (str_int, "%d" , events.pyroMain);
  strcat(working.sendFlight, str_int);strcat(working.sendFlight, comma); 
  strcpy(str_int, "");sprintf (str_int, "%d" , events.pyroDrogue);
  strcat(working.sendFlight, str_int);strcat(working.sendFlight, comma); 
  strcpy(str_int, "");sprintf (str_int, "%d" , events.continuityMain);
  strcat(working.sendFlight, str_int);strcat(working.sendFlight, comma); 
  strcpy(str_int, "");sprintf (str_int, "%d" , events.continuityDrogue);
  strcat(working.sendFlight, str_int);
  
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

//***********************************************************       OTHER    ************************************************************

void readVoltage(){
      // New Teensy:  8.5=717,8.4=708,8.0=675,7.9=667,7.7=650,7.5=632,7.2=608,7.0-594,6.8=572
      working.voltage = 0.0;
      int value = 0;
      float tempf = 0.0;
      float vin = 0.0;
      // get the samples
      for (int i=1; i <= 10; i++){  //ten samples
          value = value + analogRead(voltage_pinA);
       }
      value = value / 10;
      if (value < 589) vin = 0.0f;
      if (value > 683) vin = 100.0f;
      
      
      if (value <= 682 && value >= 589) {
        tempf = (float)value - (float)589.0;
        tempf /= (float) 8.529; // in this range it is 42 points per .1v
        tempf *= (float) .1; 
        vin = tempf / (float)1.1;
        vin = vin * 100;
        if (vin > 100) vin = 100;
      }
      working.voltage = vin;            
}

void getTimeNow(){
  //return is in working.timeHeader
  strcpy(working.timeHeader, "");
  if(events.clockSet == 1) {
    char str_int[30] = "";
    const char *slash = "/"; const char *dash = "-"; const char *colon = ":"; const char *zero="0"; 
    strcpy(str_int, "");       
    sprintf (str_int, "%lu" , hour());
    if (strlen(str_int) == 1) strcat(working.timeHeader, zero);
    strcat(working.timeHeader, str_int);
    strcat(working.timeHeader, colon);
    strcpy(str_int, "");       
    sprintf (str_int, "%lu" , minute());
    if (strlen(str_int) == 1) strcat(working.timeHeader, zero);
    strcat(working.timeHeader, str_int); 
    strcat(working.timeHeader, colon);
    strcpy(str_int, "");       
    sprintf (str_int, "%lu" , second());
    if (strlen(str_int) == 1) strcat(working.timeHeader, zero);   
    strcat(working.timeHeader, str_int);    

        
    } else {
       //just use the full milliseconds
       char buf[16];
       ltoa(millis(),buf,10);
       strcpy(working.timeHeader, "ms");
       strcat(working.timeHeader, buf);
       //strcpy(working.timeHeader, "no time  ");
    }
}

void flightSummaryString() {

//launch number, launch time, apogee time, landed time, aft event time, fore event time, ascent seconds, descent seconds, burnout seconds, AGL Baro, AGL GPS, AGL Accel, Launch Accel Max, Max temp, Max tilt, Batt voltage, Speed baro fps, mps, mph, SpeedA mps, Descent rate fps, landed rate fps, Landed GPS coord

  char comma[5] = ",";
  int n;
  char str_float[16];
  char str_int[16];
  float tempcalc = 0.0;
  //Now do the radio sentences 
  
   //S1:  @S1,#,launch number, launch time, apogee time, landed time, aft event time, fore event time, ascent seconds, descent seconds, 
   //S2:  @S2,#,burnout seconds, AGL Baro, AGL GPS, AGL Accel, Launch Accel Max, Max temp, Max tilt, Batt voltage, 
   //S3:  @S3,#,Speed baro fps, mps, mph, SpeedA mps, Descent rate fps, landed rate fps, Landed GPS coord

  strcpy(working.sendSummary1, "");  //zero out the string
  strcat(working.sendSummary1, working.LaunchNumberStr);strcat(working.sendSummary1, comma);  //launch number
  strcat(working.sendSummary1, events.launchTime);strcat(working.sendSummary1, comma);  //launch time  
  strcat(working.sendSummary1, events.apogeeTime);strcat(working.sendSummary1, comma);  //apogee time    
  strcat(working.sendSummary1, events.landedTime);strcat(working.sendSummary1, comma);  //landed time  
  strcpy(str_int, "");sprintf (str_int, "%d" , events.ascentSeconds);
  strcat(working.sendSummary1, str_int);strcat(working.sendSummary1, comma);  //ascent seconds
  strcpy(str_int, "");sprintf (str_int, "%d" , events.descentSeconds);
  strcat(working.sendSummary1, str_int);  //descent seconds  

  strcpy(str_int, "");sprintf (str_int, "%1.0f" , baro.AltitudeMax);
  strcat(working.sendSummary2, str_int);strcat(working.sendSummary2, comma);  //baro max altitude
  strcpy(str_int, "");sprintf (str_int, "%1.0f" , gpsData.maxAGLfeet);
  strcat(working.sendSummary2, str_int);strcat(working.sendSummary2, comma);  //GPS max altitude (AGL)
  strcpy(str_int, ""); sprintf (str_int, "%1.0f" , baro.TempMax); ltrim(str_int);    
  strcat(working.sendSummary2, str_int);strcat(working.sendSummary2, comma); // max temp
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
      strcpy(radioHeader, "@#S3");
      strcpy(radioMessage, working.sendSummary3);  
      RadioSend();  
      working.summaryRepeat = 1;
    }
    if(working.summaryRepeat == 2) {
      strcpy(radioHeader, "@#S2");
      strcpy(radioMessage, working.sendSummary2);  
      RadioSend();  
      working.summaryRepeat = 3;
    }
    if(working.summaryRepeat == 0) working.summaryRepeat = 1;
    if(working.summaryRepeat == 1) {
      strcpy(radioHeader, "@#S1");
      strcpy(radioMessage, working.sendSummary1);  
      RadioSend();  
      working.summaryRepeat = 2;
    }
}







//***********************************************************        SAMPLE THREAD    ************************************************************
//***********************************************************        SAMPLE THREAD     ************************************************************
//***********************************************************        SAMPLE THREAD     ************************************************************

// Dedicated thread for sampling the I2C sensors (BME280 Baro, ADXL375 Accel, LSM6DSO32 6dof).

void sample_thread() {
  
  if(serialDebug) Serial.println("Init Sampling Thread...");
    
  Wire.begin();
  delay(100);
  Wire.setClock(400000); // 400kHz I2C clock.  (can only be 100k, 400k, 1000k)
  delay(1000);
   // if(serialDebug) i2c_0.I2Cscan(); // which I2C device are on the bus?
  delay(300);    

  // ======          CONFIGURATION                          ============
    int baroRate = 20;    //millis rate (e.g. 25 = 40 hz)

  // ====== BME280 Barometer setup code and local variables ============

    float baroHumidity = 0.0;
    baro.Baseline = 0.0; // baseline altitude to subtract
    float oldAltitude = 0.0;
    unsigned long speedtimer = 0;
    
    mySensor.settings.commInterface = I2C_MODE;
    mySensor.settings.I2CAddress = 0x76;
    if (mySensor.beginI2C() == false) //Begin communication over I2C
    {
      if(serialDebug) Serial.println("ERROR:  The BME280 sensor did not respond. Please check sensor.");
      working.error = 1;
      digitalWrite(ledpin,HIGH);
      strcpy(working.errorMessage,"BME280 Sensor Error");  
    } else {
      if(serialDebug) Serial.println("Sensor:  Good BME280 sensor detected");
    }
    mySensor.setPressureOverSample(16);
    mySensor.settings.filter = 0; //Filter off
    mySensor.settings.tempOverSample = 1;
    mySensor.settings.tempCorrection = -3.3;
    delay(500);
    baroHumidity = mySensor.readFloatHumidity();
    baro.TempF = mySensor.readTempF();
    baro.tempAltitude = mySensor.readFloatAltitudeFeet();
    BME280timer = millis()+baroRate;

     if(serialDebug) Serial.println("Here we go on the sample loop...");

  while(1) {  //infinite loop handles sampling for the I2C sensors 

    // =================== BME280  code execution  ========================
    // Barometer code - altitude, speed, temp, humidity (40 Hz)
    if (millis() > BME280timer) {  

         baro.TempF = mySensor.readTempF();
         if (baro.TempF > baro.TempMax) baro.TempMax = baro.TempF;
         //baroHumidity = mySensor.readFloatHumidity();
       
         baro.tempAltitude = mySensor.readFloatAltitudeFeet();
         if(debugMode) debugVars.baroCount++;
         
         if (baro.Baseline == 0.00) baro.Baseline = baro.tempAltitude; 
         float altitudeDifference;
         altitudeDifference = baro.tempAltitude - baro.Baseline;  //true AGL
         
         if (altitudeDifference < 5.0 && altitudeDifference > -5.0) {  // zero out altitude for baro noise
          altitudeDifference = 0;
          baro.Altitude = 0;  
         } else {
          baro.Altitude = (baro.tempAltitude - baro.Baseline);  // true AGL
         }  
         
         if (baro.Altitude > baro.AltitudeMax) baro.AltitudeMax = baro.Altitude;  // record max baro altitude  (AGL)
         //baro speed calculations here - calculate every second to give FPS
         if (millis() > speedtimer) { 
            baro.Speed = (baro.Altitude - oldAltitude) * 2; //gives FPS
            oldAltitude = baro.Altitude;
            if (baro.Speed > baro.SpeedMax) baro.SpeedMax = baro.Speed;   // set max speed
            speedtimer = millis() + 500; //now updates 2X per second 
         }

       BME280timer = millis() + baroRate; //set to 50 hz
       //baro.count++;
    } //end BME280
   threads.yield(); 
 } // end while(1)
} // end sample_thread
