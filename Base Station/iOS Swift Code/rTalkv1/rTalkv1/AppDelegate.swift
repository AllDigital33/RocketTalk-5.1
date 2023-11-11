
/*

 Created by Mike and Preston
 2017 to 2023
 @PAD33space / @AllDigital
 
 */


import UIKit
import Foundation


@main
class AppDelegate: UIResponder, UIApplicationDelegate {



    func application(_ application: UIApplication, didFinishLaunchingWithOptions launchOptions: [UIApplication.LaunchOptionsKey: Any]?) -> Bool {
        // Override point for customization after application launch.
        resetAll()
        return true
    }

    // MARK: UISceneSession Lifecycle

    func application(_ application: UIApplication, configurationForConnecting connectingSceneSession: UISceneSession, options: UIScene.ConnectionOptions) -> UISceneConfiguration {
        // Called when a new scene session is being created.
        // Use this method to select a configuration to create the new scene with.
        
        return UISceneConfiguration(name: "Default Configuration", sessionRole: connectingSceneSession.role)
    }

    func application(_ application: UIApplication, didDiscardSceneSessions sceneSessions: Set<UISceneSession>) {
        // Called when the user discards a scene session.
        // If any sessions were discarded while the application was not running, this will be called shortly after application:didFinishLaunchingWithOptions.
        // Use this method to release any resources that were specific to the discarded scenes, as they will not return.
    }


}

class rocketGPSStruct {
    var time: String?
    var fix: String?
    var latString: String?
    var lat: Float?
    var longString: String?
    var long: Float?
    var altitude: Int32?
    var speed: String?
    var sat: String?
    var maxAGL: Int32?
    var updateTime: Date?
    var timeout: Date?
    var goodData: Bool?
    var newData: Bool?
    var altitudeMeters: Int32?
}

class flightDataStruct {
    var launchNumber: String?
    var phase: String?
    var altitude: Int32? //feet
    var altitudeMeters: Int32?
    var altitudeMax: Int32?
    var speed: Int?
    var speedMax: Int?
    var accelZ: Float?
    var tilt: Int?
    var tiltMax: Int?
    var temp: Int?
    var battery: Int?
    var maxTemp: Int?
    var maxAccel: Float?
    var fore: Int?
    var aft: Int?
    var sep: Int?
    var eventLaunch: Bool?
    var eventApogee: Bool?
    var eventAft: Bool?
    var eventFore: Bool?
    var eventLanded: Bool?
    var updateTime: Date?
    var timeout: Date?
    var goodData: Bool?
    var eventSeparation: Bool?
    var pyroMain: Bool?
    var pyroDrogue: Bool?
    var pyroSustainer: Bool?
    var pyroSeparation: Bool?
    var continuityMain: Bool?
    var continuityDrogue: Bool?
    var continuitySustainer: Bool?
    var continuitySeparation: Bool?
    var temp2: Int?
    
}

class errorsStruct {
    var sampleLoop: Int?
    var loggingLoop: Int?
    var ADXL: Int?
    var LSM: Int?
    var MS56: Int?
    var GPS: Int?
    var flash: Int?
    var voltage: Int?
    var CPUtemp: Int?
    var I2C: Int?
    var radio: Int?
    var count: Int?
    var crash: Int?
    var message: String?
}


class baseDataStruct {
    var altitude: Int32?
    var lat: Float?
    var long: Float?
    var heading: Float?
    var rocketBearing: Float?
    var boosterBearing: Float?
    var horizontalAccuracy: Double?
    var verticalAccuracy: Double?
    var distance: Float?
    var angle: Float?
    var updateTime: Date?
    
}

class configDataStruct {
    var mute: Bool = false
    var voice: Bool = true
    var statusTime: Int = 20  // how long to show status messages
    var radioLossTime: Int = 25  // how long before assuming radio loss for flight/gps data
    var alertRadioRX: Bool = true
    var alertEvents: Bool = true
    var alertMessages: Bool = true
    var alarmTemp: Int = 1 // 0 off, 1 on, 2 triggered already
    var alarmTilt: Int = 1
    var alarmNoRadio: Int = 1
    var alarmSpeed: Int = 1
    var alarmDistance: Int = 1
    var alarmBattery: Int = 1

}

class workingDataStruct {
    var status: String?
    var statusTimeout: Date?
    var phase: String?
    var TXarrowTimeout: Date?
    var RXarrowTimeout: Date?
    var eventType: String?
    var launchTime: Date?
    var radioRXarray = [String]()
    var flightLog: String?
    var gpsLog: String?
    var radioLog: String?
    var summaryFile: String?
    var radioSend: Bool? //used as send queue across views
    var radioSendMessage: String?
    var internetUp: Bool?
    var lastRadio: Bool?
    var lastRadioString: String?
    var camera: Bool?
    var speedTestStart: Date?
    var speedTestTime: Int?
    var sustainerI2Cerror: Bool?
    var boosterI2Cerror: Bool?
    
}

class boosterWorkingDataStruct {
    var phase: String?
    var TXarrowTimeout: Date?
    var RXarrowTimeout: Date?
    var boosterLog: String?
    var newData: Bool?
    var active: Bool?
}

class summaryDataStruct {
    //S1:  @S1,#,launch number, launch time, apogee time, landed time, aft event time, fore event time, ascent seconds, descent seconds,
    //S2:  @S2,#,burnout seconds, AGL Baro, AGL GPS, AGL Accel, Launch Accel Max, Max temp, Max tilt, Batt voltage,
    //S3:  @S3,#,Speed baro fps, mps, mph, SpeedA mps, Descent rate fps, landed rate fps, Landed GPS coord
    var radio1: Bool?
    var radio2: Bool?
    var radio3: Bool?
    var radioAll: Bool?
    //data
    var launchNumber: String?
    var launchTime: String?
    var apogeeTime: String?
    var landedTime: String?
    var aftTime: String?
    var foreTime: String?
    var ascentSeconds: String?
    var descentSeconds: String?
    var burnoutSeconds: String?
    var aglBaro: String?
    var aglGPS: String?
    var aglAccel: String?
    var accelMax: String?
    var tempMax: String?
    var tiltMax: String?
    var battery: String?
    var speedBaroFPS: String?
    var speedBaroMPS: String?
    var speedBaroMPH: String?
    var speedAccelMPS: String?
    var descentRate: String? //fps
    var landedRate: String? //fps
    var landedGPScoord: String?
    // capture base variables
    var baseLaunchTime: Date?
    var baseApogeeTime: Date?
    var baseForeTime: Date?
    var baseAftTime: Date?
    var baseLandedTime: Date?
    var baseApogeeDistance: String?
    var baseLandedDistance: String?
    //ideas:  add apogee distance, landed distance,

}



let flightData = flightDataStruct()
let rocketGPS = rocketGPSStruct()
let baseData = baseDataStruct()
let configData = configDataStruct()
let workingData = workingDataStruct()
let summaryData = summaryDataStruct()
let boosterGPS = rocketGPSStruct()
let boosterData = flightDataStruct()
let boosterWorkingData = boosterWorkingDataStruct()
let errors = errorsStruct()



func resetAll() {
    
    rocketGPS.time = " "
    rocketGPS.fix = " "
    rocketGPS.latString = " "
    rocketGPS.lat = 0.0
    rocketGPS.longString = " "
    rocketGPS.long = 0.0
    rocketGPS.altitude = 0
    rocketGPS.speed = " "
    rocketGPS.sat = " "
    rocketGPS.maxAGL = 0
    rocketGPS.updateTime = Date()
    rocketGPS.timeout = Date()
    rocketGPS.goodData = false
    rocketGPS.newData = false
    rocketGPS.altitudeMeters = 0
    flightData.launchNumber = "00"
    flightData.phase = "STARTUP"
    flightData.altitude = 0
    flightData.altitudeMeters = 0
    flightData.altitudeMax = 0
    flightData.speed = 0
    flightData.speedMax = 0
    flightData.accelZ = 0.0
    flightData.tilt = 0
    flightData.tiltMax = 0
    flightData.maxAccel = 0.0
    flightData.temp = 0
    flightData.temp2 = 0
    flightData.battery = 0
    flightData.maxTemp = 0
    flightData.fore = 0
    flightData.aft = 0
    flightData.sep = 0
    flightData.eventLaunch = false
    flightData.eventApogee = false
    flightData.eventAft = false
    flightData.eventFore = false
    flightData.eventLanded = false
    flightData.updateTime = Date()
    flightData.timeout = Date()
    flightData.goodData = false
    flightData.eventSeparation = false
    flightData.pyroMain = false
    flightData.pyroDrogue = false
    flightData.pyroSustainer = false
    flightData.pyroSeparation = false
    flightData.continuityMain = false
    flightData.continuityDrogue = false
    flightData.continuitySustainer = false
    flightData.continuitySeparation = false
    baseData.altitude = 0
    baseData.lat = 0.0
    baseData.long = 0.0
    baseData.heading = 0.0
    baseData.rocketBearing = 0.0
    baseData.boosterBearing = 0.0
    baseData.horizontalAccuracy = 0.0
    baseData.verticalAccuracy = 0.0
    baseData.distance = 0.0
    baseData.angle = 0.0
    baseData.updateTime = Date()
    workingData.status = "Waiting for first communication..."
    workingData.statusTimeout = Date() + 3000
    workingData.TXarrowTimeout = Date()
    workingData.RXarrowTimeout = Date()
    workingData.launchTime = Date()
    workingData.phase = "STARTUP"
    workingData.eventType = " "
    workingData.radioLog = ""
    workingData.gpsLog = ""
    workingData.flightLog = ""
    workingData.summaryFile = ""
    workingData.radioSend = false
    workingData.radioSendMessage = ""
    workingData.internetUp = false
    workingData.lastRadio = false
    workingData.lastRadioString = ""
    workingData.camera = false
    workingData.speedTestTime = 0
    workingData.speedTestStart = Date()
    workingData.sustainerI2Cerror = false
    workingData.boosterI2Cerror = false
    configData.mute = false
    configData.voice = true
    configData.alertRadioRX = true
    configData.alertEvents = true
    configData.alertMessages = true
    configData.alarmTemp = 1 // 0 off, 1 on, 2 triggered already
    configData.alarmTilt = 1
    configData.alarmNoRadio = 1
    configData.alarmSpeed = 1
    configData.alarmDistance = 1
    configData.alarmBattery = 1
    let now = Date()
    let formatter = DateFormatter()
    formatter.dateStyle = .short
    formatter.timeStyle = .medium
    let datetime = formatter.string(from: now)
    workingData.radioRXarray.append(String(datetime + "  radio log array started"))
    //zero the summary data
    summaryData.radio1 = false
    summaryData.radio2 = false
    summaryData.radio3 = false
    summaryData.radioAll = false
    //data
    summaryData.launchNumber = ""
    summaryData.launchTime = ""
    summaryData.apogeeTime = ""
    summaryData.landedTime = ""
    summaryData.aftTime = ""
    summaryData.foreTime = ""
    summaryData.ascentSeconds = ""
    summaryData.descentSeconds = ""
    summaryData.burnoutSeconds = ""
    summaryData.aglBaro = ""
    summaryData.aglGPS = ""
    summaryData.aglAccel = ""
    summaryData.accelMax = ""
    summaryData.tempMax = ""
    summaryData.tiltMax = ""
    summaryData.battery = ""
    summaryData.speedBaroFPS = ""
    summaryData.speedBaroMPS = ""
    summaryData.speedBaroMPH = ""
    summaryData.speedAccelMPS = ""
    summaryData.descentRate = ""
    summaryData.landedRate = ""
    summaryData.landedGPScoord = ""
    // capture base variables
    summaryData.baseLaunchTime = Date().zeroTime()
    summaryData.baseApogeeTime = Date().zeroTime()
    summaryData.baseForeTime = Date().zeroTime()
    summaryData.baseAftTime = Date().zeroTime()
    summaryData.baseLandedTime = Date().zeroTime()
    summaryData.baseApogeeDistance = ""
    summaryData.baseLandedDistance = ""
    
    boosterGPS.time = " "
    boosterGPS.fix = " "
    boosterGPS.latString = " "
    boosterGPS.lat = 0.0
    boosterGPS.longString = " "
    boosterGPS.long = 0.0
    boosterGPS.altitude = 0
    boosterGPS.speed = " "
    boosterGPS.sat = " "
    boosterGPS.maxAGL = 0
    boosterGPS.updateTime = Date()
    boosterGPS.timeout = Date()
    boosterGPS.goodData = false
    boosterGPS.newData = false
    boosterGPS.altitudeMeters = 0
    boosterData.launchNumber = "00"
    boosterData.phase = "STARTUP"
    boosterData.altitude = 0
    boosterData.altitudeMeters = 0
    boosterData.altitudeMax = 0
    boosterData.speed = 0
    boosterData.speedMax = 0
    boosterData.accelZ = 0.0
    boosterData.tilt = 0
    boosterData.tiltMax = 0
    boosterData.maxAccel = 0.0
    boosterData.temp = 0
    boosterData.temp2 = 0
    boosterData.battery = 0
    boosterData.maxTemp = 0
    boosterData.fore = 0
    boosterData.aft = 0
    boosterData.sep = 0
    boosterData.eventLaunch = false
    boosterData.eventApogee = false
    boosterData.eventAft = false
    boosterData.eventFore = false
    boosterData.eventLanded = false
    boosterData.updateTime = Date()
    boosterData.timeout = Date()
    boosterData.goodData = false
    boosterData.eventSeparation = false
    boosterData.pyroMain = false
    boosterData.pyroDrogue = false
    boosterData.pyroSustainer = false
    boosterData.pyroSeparation = false
    boosterData.continuityMain = false
    boosterData.continuityDrogue = false
    boosterData.continuitySustainer = false
    boosterData.continuitySeparation = false
    boosterWorkingData.phase = "STARTUP"
    boosterWorkingData.TXarrowTimeout = Date()
    boosterWorkingData.RXarrowTimeout = Date()
    boosterWorkingData.boosterLog = "Booster.txt"
    boosterWorkingData.newData = false;
    boosterWorkingData.active = false;
    errors.sampleLoop = 0
    errors.sampleLoop = 0
    errors.loggingLoop = 0
    errors.ADXL = 0
    errors.LSM = 0
    errors.MS56 = 0
    errors.GPS = 0
    errors.flash = 0
    errors.voltage = 0
    errors.CPUtemp = 0
    errors.I2C = 0
    errors.count = 0
    errors.crash = 0
    errors.radio = 0
    errors.message = ""

}

extension Date {
    func zeroTime() -> Date?
    {
        let calendar = Calendar.current
        var components = calendar.dateComponents([.year, .month, .day, .hour, .minute, .second], from: self)
        components.minute = 0
        components.second = 0
        components.hour = 0
        return calendar.date(from: components)
    }
}

func getBearing(rLat: Float, rLong: Float) -> Float {
    
    //Get the bearing between base and rocket
    
    let teta1: Float = Float(degreesToRadians(CGFloat(baseData.lat!)))
    let teta2: Float = Float(degreesToRadians(CGFloat(rLat)))
    //let delta1: Float = Float(degreesToRadians(CGFloat(rocketGPS.lat! - baseData.lat!)))
    let delta2: Float = Float(degreesToRadians(CGFloat(rLong - baseData.long!)))
    
    //==================Heading Formula Calculation================//

    let y: Float = sin(delta2) * cos(teta2)
    let x: Float = cos(teta1) * sin(teta2) - sin(teta1) * cos(teta2) * cos(delta2)
    var brng: Float = atan2(y,x)
    brng = brng * 180 / Float.pi
    brng = Float(((Int(brng) + 360) % 360))
    print("bearing:")
    print(brng)
    return brng;
    
    
}

func getGPSDistance(rLat: Float, rLong: Float) -> Float {
    
    var lat1f: Float = 0.0
    var lon1f: Float = 0.0
    var lat2f: Float = 0.0
    var lon2f: Float = 0.0
    var dist_calc: Float = 0.0
    var dist_calc2: Float = 0.0
    var diflat: Float = 0.0
    var diflon: Float = 0.0
    let toRad = Float.pi / 180.0
    
    lat1f = rLat
    lon1f = rLong
    lat2f = Float(baseData.lat!)
    lon2f = Float(baseData.long!)
    
    diflat = (lat2f-lat1f) * toRad
    lat1f = lat1f * toRad
    lat2f = lat2f * toRad
    diflon = (lon2f - lon1f) * toRad
    
    dist_calc = (sin(diflat/2.0) * sin(diflat/2.0));
    dist_calc2 = cos(lat1f);
    dist_calc2 *= cos(lat2f);
    dist_calc2 *= sin(diflon/2.0);
    dist_calc2 *= sin(diflon/2.0);
    dist_calc += dist_calc2;
    
    dist_calc = (2*atan2(sqrt(dist_calc),sqrt(1.0-dist_calc)));
    
    dist_calc *= 6371000.0; //Converting to meters
    print("Distance is ")
    print(dist_calc)
    return dist_calc;
    
    }

// ****************************************************  RADIO PROCESSING *****************************************

func processRadio(theData: String) -> Int {
    
    var malformed: Bool = false
    
    //now check for length
    if(theData.count > 120 || theData.count < 10) {
        malformed = true
        workingData.radioRXarray.append("MALFORMED! (size - previous radio sentence")
        let theData:String = "MALFORMED SENTENCE"
        logging(message: theData, filename: workingData.radioLog!)
        return 0
    }
    if(malformed == false) {
        //now convert to array
        let tempArray = theData.components(separatedBy: ",")
        //now process message type (check length first)
        if(tempArray[0] == "@F") { //flight data
            if(tempArray.count == 26) {
            processFlightData(theArr: tempArray)
            logging(message: theData, filename: workingData.flightLog!)
                return 1 } else {
                  return 0
                }
        }
        if(tempArray[0] == "@G") { //GPS data

            if(tempArray.count == 11) {
                processGPSData(theArr: tempArray)
                logging(message: theData, filename: workingData.gpsLog!)
                return 2} else {
                  return 0
                }
        }
        if(tempArray[0] == "@M") { //message
            processMessage(theArr: tempArray)
            return 3
        }
        if(tempArray[0] == "@E") { //event
            processEvent(theArr: tempArray)
            return 4
        }
        if(tempArray[0] == "@I") { //event
            processErrors(theArr: tempArray)
            return 4
        }
        if(tempArray[0] == "@C") { //callsign ignore
            //ignore -- just a required FCC call sign check in
            return 9
        }
        if(tempArray[0] == "@S1" && summaryData.radio1 == false) {
            workingData.status = "Received Summary 1"
            workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
            processSummary1(theArr: tempArray)
            return 5
        }
        if(tempArray[0] == "@S2" && summaryData.radio2 == false) {
            workingData.status = "Received Summary 2"
            workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
            processSummary2(theArr: tempArray)
            return 5
        }
        if(tempArray[0] == "@S3" && summaryData.radio3 == false) {
            workingData.status = "Received Summary 3"
            workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
            processSummary3(theArr: tempArray)
            return 5
        }
        if(tempArray[0] == "@#F") {  // Booster flight data
            if(tempArray.count == 19) {
            processBoosterFlightData(theArr: tempArray)
                logging(message: theData, filename: boosterWorkingData.boosterLog!)
                return 11 } else {
                  return 0
                }
        }
        if(tempArray[0] == "@#G") { //Booster GPS data
            if(tempArray.count == 11) {
                processBoosterGPSData(theArr: tempArray)
                logging(message: theData, filename: boosterWorkingData.boosterLog!)
                return 12} else {
                  return 0
                }
        }
        if(tempArray[0] == "@#S1" || tempArray[0] == "@#S2" || tempArray[0] == "@#S3") { //Booster GPS data
                //just log summary info to booster log for now
                logging(message: theData, filename: boosterWorkingData.boosterLog!)
                return 13
        }
        if(tempArray[0] == "@W2") { //speed test
            workingData.speedTestTime = (Int) (Date().timeIntervalSince(workingData.speedTestStart!) * 1000)
            workingData.status = "Speed Test Result (ms) = " + String(workingData.speedTestTime!)
            workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
            return 9
        }
        
    }
    return 99 //not malformed but nothing to do
}

func openNewLogFiles(launchNumber: String!) {
    let formatter = DateFormatter()
    formatter.dateFormat = "MMdd"
    let timestamp = formatter.string(from: Date())
    let filePrefix = launchNumber + "-" + "\(timestamp)"
    let formatter2 = DateFormatter()
    formatter2.dateFormat = "MM-dd-yyyy hh:mm:ss"
    let timestamp2 = formatter2.string(from: Date())
    
    var theFile = filePrefix + "-flight.txt"
    var theM = "New flight log opened for launch " + launchNumber + " at \(timestamp2)"
    logging(message: theM, filename: theFile)
    theM = "KEY = date-time, @type, launch#, altitude, max altitude, speed, max speed, accel-Z, tilt, max tilt, temp-F, max temp, fore, aft, battery,!"
    logging(message: theM, filename: theFile)
    workingData.flightLog = theFile
    
    theFile = filePrefix + "-gps.txt"
    theM = "New GPS log opened for launch " + launchNumber + " at \(timestamp2)"
    logging(message: theM, filename: theFile)
    theM = "KEY = date-time, @type, launch#, time, fix, latitude, longitude, Altitude Sea Level, speed, satellites, max AGL feet,!"
    logging(message: theM, filename: theFile)
    workingData.gpsLog = theFile
    
    theFile = filePrefix + "-radio.txt"
    theM = "New radio log opened for launch " + launchNumber + " at \(timestamp2)"
    logging(message: theM, filename: theFile)
    workingData.radioLog = theFile
 
    theFile = filePrefix + "-summary.txt"
    workingData.summaryFile = theFile
    
    theFile = boosterWorkingData.boosterLog!
    theM = "New Booster log opened at \(timestamp2)"
    logging(message: theM, filename: theFile)
        
}

func openNewBoosterFile() {
    let formatter = DateFormatter()
    formatter.dateFormat = "MMdd"
    let formatter2 = DateFormatter()
    formatter2.dateFormat = "MM-dd-yyyy hh:mm:ss"
    let timestamp2 = formatter2.string(from: Date())
    
    var theFile = "filename"
    var theM = "message"
    theFile = boosterWorkingData.boosterLog!
    theM = "New Booster log opened at \(timestamp2)"
    logging(message: theM, filename: theFile)
        
}




func processFlightData(theArr: [String]) { //@F = 1
    //@F,45,D,1358,2139,-68,280,-116,15058,156,180,111,111,2,2,1,76,1,1,1,1,1,1,1,1,!
    
    
    if(flightData.launchNumber == "00") {
        flightData.launchNumber = theArr[1]
        workingData.phase = flightData.phase
        workingData.status = "Hello! Got first communication from rocket"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        //set the log files
        openNewLogFiles(launchNumber: flightData.launchNumber)
        }
    if(flightData.launchNumber != theArr[1]) {
        workingData.status = "ERROR - Rocket Launch Number Change!"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
    }
    flightData.launchNumber = theArr[1]
    flightData.phase = thePhase(theKey: theArr[2])
    //Events are implied if phase progresses on rocket, but no event trigger
    if(flightData.phase == "LAUNCH" && flightData.eventLaunch == false) {
        flightData.eventLaunch = true
        workingData.launchTime = Date()
    }
    if(flightData.phase == "DESCENT" && flightData.eventApogee == false) { flightData.eventApogee = true}
    if(flightData.phase == "LANDED" && flightData.eventLanded == false) { flightData.eventLanded = true}
    flightData.altitude = (Int32(theArr[3]) ?? 9999)
    flightData.altitudeMeters = Int32(Double(Double(flightData.altitude!) * 0.3048))
    flightData.altitudeMax = (Int32(theArr[4]) ?? 9999)
    flightData.speed = (Int(theArr[5]) ?? 9999)
    flightData.speedMax = (Int(theArr[6]) ?? 9999)
    flightData.accelZ = (Float(theArr[7]) ?? 9999) / 1000.0
    flightData.tilt = (Int(theArr[9]) ?? 9999)
    flightData.tiltMax = (Int(theArr[10]) ?? 9999)
    flightData.maxAccel = (Float(theArr[8]) ?? 9999) / 1000.0
    flightData.temp = (Int(theArr[11]) ?? 9999)

    flightData.maxTemp = (Int(theArr[12]) ?? 9999)
    // Fore and Aft sensors will adjust in real-time, so if there is an intermittent connection will resolve
    flightData.fore = (Int(theArr[13]) ?? 9999)
    if(flightData.fore! > 0) {
        flightData.eventFore = true
    } else {
        flightData.eventFore = false
    }
    flightData.aft = (Int(theArr[14]) ?? 9999)
    if(flightData.aft! > 0) {
        flightData.eventAft = true
    } else {
        flightData.eventAft = false
    }
    flightData.sep = (Int(theArr[15]) ?? 9999)
    if(flightData.sep! > 0) {
        flightData.eventSeparation = true
    } else {
        flightData.eventSeparation = false
    }
    flightData.battery = (Int(theArr[16]) ?? 9999)
    // pyros
    if((Int(theArr[17]) ?? 9999) == 1) {
      flightData.pyroMain = true
    } else {
      flightData.pyroMain = false
    }
    if((Int(theArr[18]) ?? 9999) == 1) {
      flightData.pyroDrogue = true
    } else {
      flightData.pyroDrogue = false
    }
    if((Int(theArr[19]) ?? 9999) == 1) {
      flightData.pyroSustainer = true
    } else {
      flightData.pyroSustainer = false
    }
    if((Int(theArr[20]) ?? 9999) == 1) {
      flightData.pyroSeparation = true
    } else {
      flightData.pyroSeparation = false
    }
    //continuity
    if((Int(theArr[21]) ?? 9999) == 1) {
      flightData.continuityMain = true
    } else {
      flightData.continuityMain = false
    }
    if((Int(theArr[22]) ?? 9999) == 1) {
      flightData.continuityDrogue = true
    } else {
      flightData.continuityDrogue = false
    }
    if((Int(theArr[23]) ?? 9999) == 1) {
      flightData.continuitySustainer = true
    } else {
      flightData.continuitySustainer = false
    }
    if((Int(theArr[24]) ?? 9999) == 1) {
      flightData.continuitySeparation = true
    } else {
      flightData.continuitySeparation = false
    }
    
    flightData.updateTime = Date()
    flightData.timeout = Date() + TimeInterval(configData.radioLossTime)
    flightData.goodData = true
  }

func thePhase(theKey: String) -> String {
    if(theKey == "S") {return "STARTUP"}
    if(theKey == "W") {return "WAITING"}
    if(theKey == "L") {return "LAUNCH"}
    if(theKey == "D") {return "DESCENT"}
    if(theKey == "X") {return "LANDED"}
    if(theKey == "B") {return "BASIC"}
    return "ERROR"
}

func processGPSData(theArr: [String]) {
    //@G,45,58,A,3520.83023N,11748.72041W,3616,11.299,10,2248,!
    
    if(flightData.launchNumber == "00") {
        flightData.launchNumber = theArr[1]
        workingData.phase = flightData.phase
        workingData.status = "Hello! Got first communication from rocket"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        //set the log files
        openNewLogFiles(launchNumber: flightData.launchNumber)
    }
    rocketGPS.fix = theArr[2]
    rocketGPS.fix = theArr[3]
    rocketGPS.latString = theArr[4]
    rocketGPS.lat = Float((rocketGPS.latString ?? "9999")) ?? 9999
    rocketGPS.longString = theArr[5]
    rocketGPS.long = Float((rocketGPS.longString ?? "9999")) ?? 9999
    rocketGPS.altitude = (Int32(theArr[6]) ?? 9999)  //this is actual sea level in feet
    rocketGPS.altitudeMeters = Int32(Double(Double(rocketGPS.altitude!) * 0.3048))
    rocketGPS.speed = theArr[7]
    rocketGPS.sat = theArr[8]
    rocketGPS.maxAGL = (Int32(theArr[9]) ?? 9999) //this is actual max to AGL in feet (adjusted using a baseline)
    rocketGPS.updateTime = Date()
    rocketGPS.timeout = Date() + TimeInterval(configData.radioLossTime)
    rocketGPS.goodData = true
    rocketGPS.newData = true

    //log last good GPS
    
    let lastG: String = rocketGPS.latString! + "," + rocketGPS.longString!
    logLastGPS(message: lastG, filename: "lastGPS")
    
}

func getSubString(theString: String, thePre: Int, thePost: Int) -> String {
    
    let start = theString.index(theString.startIndex, offsetBy: thePre)
    let end = theString.index(theString.endIndex, offsetBy: -thePost)
    let range = start..<end
    let mySubstring = theString[range]  // play
    let myString = String(mySubstring)
    return myString
}

func processMessage(theArr: [String]) {
    workingData.status = theArr[2]
    workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
    let theData:String = "MESSAGE Received: " + theArr[2]
    logging(message: theData, filename: workingData.flightLog!)
    //check for Camera on/off to toggle icon
    if(theArr[2] == "Camera is now ON") {workingData.camera = true}
    if(theArr[2] == "Camera is now OFF") {workingData.camera = false}
    
}

func processErrors(theArr: [String]) {   //@I
    
    errors.sampleLoop = (Int(theArr[2]) ?? 0)
    errors.loggingLoop = (Int(theArr[3]) ?? 0)
    errors.ADXL = (Int(theArr[4]) ?? 0)
    errors.LSM = (Int(theArr[5]) ?? 0)
    errors.MS56 = (Int(theArr[6]) ?? 0)
    errors.GPS = (Int(theArr[7]) ?? 0)
    errors.flash = (Int(theArr[8]) ?? 0)
    errors.voltage = (Int(theArr[9]) ?? 0)
    errors.CPUtemp = (Int(theArr[10]) ?? 0)
    errors.I2C = (Int(theArr[11]) ?? 0)
    errors.radio = (Int(theArr[12]) ?? 0)
    errors.crash = (Int(theArr[13]) ?? 0)
    errors.count = (Int(theArr[14]) ?? 0)
    errors.message = theArr[15]
    
    if(errors.count ?? 0 > 0) {
        workingData.status = errors.message
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        let theData:String = "ERRORS: " + (errors.message ?? "")
        logging(message: theData, filename: workingData.flightLog!)
    }
}

func processEvent(theArr: [String]) {
    let theEvent: String = theArr[2]
    if(theEvent == "FORE") {
        workingData.status = "FORE Separation Detected"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        flightData.eventFore = true
        workingData.eventType = "FORE"
        summaryData.baseForeTime = Date()
        let theData:String = "EVENT: Forward Separation Detected"
        logging(message: theData, filename: workingData.flightLog!)
    }
    if(theEvent == "AFT") {
        workingData.status = "AFT Separation Detected"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        flightData.eventAft = true
        workingData.eventType = "AFT"
        summaryData.baseAftTime = Date()
        let theData:String = "EVENT: AFT Separation Detected"
        logging(message: theData, filename: workingData.flightLog!)
    }
    if(theEvent == "STAGE") {
        workingData.status = "Staging Separation Detected"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        flightData.eventAft = true
        workingData.eventType = "STAGE"
        summaryData.baseAftTime = Date()
        let theData:String = "EVENT: Staging Separation Detected"
        logging(message: theData, filename: workingData.flightLog!)
    }
    if(theEvent == "LAUNCH") {
        workingData.launchTime = Date()
        workingData.status = "Launch Detected"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        flightData.eventLaunch = true
        workingData.eventType = "LAUNCH"
        summaryData.baseLaunchTime = Date()
        let theData:String = "EVENT: LAUNCH Detected"
        logging(message: theData, filename: workingData.flightLog!)
    }
    if(theEvent == "APOGEE") {
        workingData.status = "Apogee Detected"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        flightData.eventApogee = true
        workingData.eventType = "APOGEE"
        summaryData.baseApogeeTime = Date()
        let theData:String = "EVENT: APOGEE Detected"
        logging(message: theData, filename: workingData.flightLog!)
    }
    if(theEvent == "LANDED") {
        workingData.status = "Landing Detected"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        flightData.eventLanded = true
        workingData.eventType = "LANDED"
        summaryData.baseLandedDistance = "\(String(format: "%.0f", baseData.distance!))" + " M"
        summaryData.baseLandedTime = Date()
        let theData:String = "EVENT: Landed Detected"
        logging(message: theData, filename: workingData.flightLog!)
    }
    
}
func processSummary1(theArr: [String]) {
    //S1:  @S1,#,launch number, launch time, apogee time, landed time, aft event time, fore event time, ascent seconds, descent seconds,
    //S2:  @S2,#,burnout seconds, AGL Baro, AGL GPS, AGL Accel, Launch Accel Max, Max temp, Max tilt, Batt voltage,
    //S3:  @S3,#,Speed baro fps, mps, mph, SpeedA mps, Descent rate fps, landed rate fps, Landed GPS coord
    //@S1,45,45,13:09:14B,13:16:47B,13:18:02,13:16:45,13:16:45,452,75,,!
    //@S2,45,n/a,2139,2248,n/a,15058,111,37,76.74,,!
    //@S3,45,280,85,191,n/a,29,2,35 20.741,-117 48.740',3520.74187,11748.74066,!
    
    summaryData.radio1 = true
    summaryData.launchNumber = theArr[1]
    summaryData.launchTime = theArr[3]
    summaryData.apogeeTime = theArr[4]
    summaryData.landedTime = theArr[5]
    summaryData.aftTime = theArr[6]
    summaryData.foreTime = theArr[7]
    summaryData.ascentSeconds = theArr[8]
    summaryData.descentSeconds = theArr[9]
    
    if (summaryData.radio1! && summaryData.radio2! && summaryData.radio3!) {
        summaryData.radioAll = true
        recordSummary()
        workingData.status = "Received Complete Summary"
    }
}
func processSummary2(theArr: [String]) {
    
    summaryData.radio2 = true
    summaryData.burnoutSeconds = theArr[2]
    summaryData.aglBaro = theArr[3]
    summaryData.aglGPS = theArr[4]
    summaryData.aglAccel = theArr[5]
    summaryData.accelMax = theArr[6]
    summaryData.tempMax = theArr[7]
    summaryData.tiltMax = theArr[8]
    summaryData.battery = theArr[9]
    if (summaryData.radio1! && summaryData.radio2! && summaryData.radio3!) {
        summaryData.radioAll = true
        recordSummary()
        workingData.status = "Received Complete Summary"
    }
    
}
func processSummary3(theArr: [String]) {
    summaryData.radio3 = true
    summaryData.speedBaroFPS = theArr[2]
    summaryData.speedBaroMPS = theArr[3]
    summaryData.speedBaroMPH = theArr[4]
    summaryData.speedAccelMPS = theArr[5]
    summaryData.descentRate = theArr[6]
    summaryData.landedRate = theArr[7]
    summaryData.landedGPScoord = theArr[8]
    if (summaryData.radio1! && summaryData.radio2! && summaryData.radio3!) {
        summaryData.radioAll = true
        recordSummary()
        workingData.status = "Received Complete Summary"
    }
    
    
}

func recordSummary() {
    
    var theM: String = ""
    let formatter2 = DateFormatter()
    formatter2.dateFormat = "MM-dd-yyyy hh:mm:ss"
    var timestamp2 = formatter2.string(from: Date())
    
    theM = "New Summary file created for launch " + flightData.launchNumber! + " at \(timestamp2)"
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "  "
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "    Launch Time: " + summaryData.launchTime!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "    Apogee Time: " + summaryData.apogeeTime!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "    Landed Time: " + summaryData.landedTime!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "       Aft Time: " + summaryData.aftTime!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "      Fore Time: " + summaryData.foreTime!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "   "
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = " Ascent Seconds: " + summaryData.ascentSeconds!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "Descent Seconds: " + summaryData.descentSeconds!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "Burnout Seconds: " + summaryData.burnoutSeconds!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "  "
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "  Baro AGL feet: " + summaryData.aglBaro!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "   GPS AGL feet: " + summaryData.aglGPS!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = " Accel AGL feet: " + summaryData.aglAccel!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "  "
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "      Accel Max: " + summaryData.accelMax!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "       Temp Max: " + summaryData.tempMax!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "       Tilt Max: " + summaryData.tiltMax!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "       Battery%: " + summaryData.battery!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "  "
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "   Speed in FPS: " + summaryData.speedBaroFPS!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "   Speed in MPS: " + summaryData.speedBaroMPS!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "   Speed in MPH: " + summaryData.speedBaroMPH!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "  "
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "   Descent Rate: " + summaryData.descentRate!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "FPS Landed Rate: " + summaryData.landedRate!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "     Landed GPS: " + summaryData.landedGPScoord!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "  "
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "======================================================================"
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "BASE STATION COLLECTED DATA"
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "   "
        writeSummary(message: theM, filename: workingData.summaryFile!)
    formatter2.dateFormat = "hh:mm:ss"
    timestamp2 = formatter2.string(from: summaryData.baseLaunchTime!)
    theM = "    Launch Time: " + timestamp2
        writeSummary(message: theM, filename: workingData.summaryFile!)
    timestamp2 = formatter2.string(from: summaryData.baseApogeeTime!)
    theM = "    Apogee Time: " + timestamp2
        writeSummary(message: theM, filename: workingData.summaryFile!)
    timestamp2 = formatter2.string(from: summaryData.baseAftTime!)
    theM = "       Aft Time: " + timestamp2
        writeSummary(message: theM, filename: workingData.summaryFile!)
    timestamp2 = formatter2.string(from: summaryData.baseForeTime!)
    theM = "      Fore Time: " + timestamp2
        writeSummary(message: theM, filename: workingData.summaryFile!)
    timestamp2 = formatter2.string(from: summaryData.baseLandedTime!)
    theM = "    Landed Time: " + timestamp2
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "Apogee Distance: " + summaryData.baseApogeeDistance!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "Landed Distance: " + summaryData.baseLandedDistance!
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "    "
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "======================================================================"
        writeSummary(message: theM, filename: workingData.summaryFile!)
    theM = "Have a nice day!"
        writeSummary(message: theM, filename: workingData.summaryFile!)
      
}

func processBoosterFlightData(theArr: [String]) { //@#F = 1
    //@#F,launch, phase, alt, alt max, speed, speed max, temp, temp max, battery, pyro main, pyro drogue, cont main, cont drogue, !
    
    if(boosterWorkingData.active == false) {
        boosterWorkingData.active = true
        openNewBoosterFile()
    }
    boosterData.phase = thePhase(theKey: theArr[2])
    //Events are implied if phase progresses on rocket, but no event trigger
    if(boosterData.phase == "LAUNCH" && boosterData.eventLaunch == false) {
        boosterData.eventLaunch = true
    }
    if(boosterData.phase == "DESCENT" && boosterData.eventApogee == false) { boosterData.eventApogee = true}
    if(boosterData.phase == "LANDED" && boosterData.eventLanded == false) { boosterData.eventLanded = true}
    var tempS: String
    tempS = errChkI(theString: theArr[3])
    boosterData.altitude = (Int32(tempS) ?? 9999)
    boosterData.altitudeMeters = Int32(Double(Double(flightData.altitude!) * 0.3048))
    tempS = errChkI(theString: theArr[4])
    boosterData.altitudeMax = (Int32(tempS) ?? 9999)
    tempS = errChkI(theString: theArr[5])
    boosterData.speed = (Int(tempS) ?? 9999)
    tempS = errChkI(theString: theArr[6])
    boosterData.speedMax = (Int(tempS) ?? 9999)
    boosterData.temp = (Int(theArr[7]) ?? 999)
    boosterData.maxTemp = (Int(theArr[8]) ?? 999)
    boosterData.battery = (Int(theArr[9]) ?? 999)
    // pyros
    if(Int(theArr[10]) == 1) {
        boosterData.pyroMain = true
    } else {
        boosterData.pyroMain = false
    }
    if(Int(theArr[11]) == 1) {
        boosterData.pyroDrogue = true
    } else {
        boosterData.pyroDrogue = false
    }
    //continuity
    if(Int(theArr[12]) == 1) {
        boosterData.continuityMain = true
    } else {
        boosterData.continuityMain = false
    }
    if(Int(theArr[13]) == 1) {
        boosterData.continuityDrogue = true
    } else {
        boosterData.continuityDrogue = false
    }

    tempS = errChkF(theString: theArr[14])
    boosterData.accelZ = (Float(tempS) ?? 9999) / 1000.0
    tempS = errChkI(theString: theArr[16])
    boosterData.tilt = (Int(tempS) ?? 9999)
    tempS = errChkF(theString: theArr[15])
    boosterData.maxAccel = (Float(tempS) ?? 9999) / 1000.0
    boosterData.temp2 = (Int(theArr[17]) ?? 999)
    
    boosterData.updateTime = Date()
    boosterData.timeout = Date() + TimeInterval(46)
    boosterData.goodData = true
    boosterWorkingData.newData = true
  }

func errChkI(theString: String) -> String {
    var theChk: String
    theChk = String((Int(theString) ?? 9999))
    return theChk
}
func errChkF(theString: String) -> String {
    var theChk: String
    theChk = String((Float(theString) ?? 9999))
    return theChk
}


func processBoosterGPSData(theArr: [String]) {  //same as rocket but different variables
    //@G,45,58,A,3520.83023N,11748.72041W,3616,11.299,10,2248,!
    
    boosterGPS.fix = theArr[2]
    boosterGPS.fix = theArr[3]
    boosterGPS.latString = theArr[4]
    boosterGPS.lat = Float(boosterGPS.latString ?? "9999")
    boosterGPS.longString = theArr[5]
    boosterGPS.long = Float(boosterGPS.longString ?? "9999")
    boosterGPS.altitude = (Int32(theArr[6]) ?? 9999)  //this is actual sea level in feet
    boosterGPS.altitudeMeters = Int32(Double(Double(boosterGPS.altitude ?? 9999) * 0.3048))
    boosterGPS.speed = theArr[7]
    boosterGPS.sat = theArr[8]
    boosterGPS.maxAGL = (Int32(theArr[9]) ?? 9999) //this is actual max to AGL in feet (adjusted using a baseline)
    boosterGPS.updateTime = Date()
    boosterGPS.timeout = Date() + TimeInterval(45)
    boosterGPS.goodData = true
    boosterGPS.newData = true
    
}



//*********************************  File Logging Functions **********************************

// ##-MMdd-flight.txt
// ##-MMdd-gps.txt
// ##-MMdd-radio.txt


func logFileURL(fileN: String) -> URL? {
    guard let documentsDirectory = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first else { return nil }
    return documentsDirectory.appendingPathComponent(fileN)
    }

func logging(message: String, filename: String) {
      
      let logFile = logFileURL(fileN: filename)
    
      let formatter = DateFormatter()
      formatter.dateFormat = "HH:mm:ss "
      let timestamp = formatter.string(from: Date())
    
     // guard let data = ("[\(timestamp)]  [\(className ?? "")] \(functionName)\(line): \(message)" + "\n").data(using: String.Encoding.utf8) else { return }
      guard let data = ("\(timestamp) \(message)" + "\n").data(using: String.Encoding.utf8) else { return }

    if FileManager.default.fileExists(atPath: logFile!.path) {
        if let fileHandle = try? FileHandle(forWritingTo: logFile!)          {
                fileHandle.seekToEndOfFile()
                fileHandle.write(data)
                fileHandle.closeFile()
         }
        } else {
            try? data.write(to: logFile!, options: .atomicWrite)
        }
}

func logLastGPS(message: String, filename: String) {
      
    let logFile = logFileURL(fileN: filename)

    guard let data = ("\(message)").data(using: String.Encoding.utf8) else { return }

    if FileManager.default.fileExists(atPath: logFile!.path) {
        if let fileHandle = try? FileHandle(forWritingTo: logFile!)          {
    //            fileHandle.seekToEndOfFile()
                fileHandle.write(data)
                fileHandle.closeFile()
         }
        } else {
            try? data.write(to: logFile!, options: .atomicWrite)
        }
}

func writeSummary(message: String, filename: String) {
      
      let logFile = logFileURL(fileN: filename)
    
      guard let data = ("\(message)" + "\n").data(using: String.Encoding.utf8) else { return }

    if FileManager.default.fileExists(atPath: logFile!.path) {
        if let fileHandle = try? FileHandle(forWritingTo: logFile!)          {
                fileHandle.seekToEndOfFile()
                fileHandle.write(data)
                fileHandle.closeFile()
         }
        } else {
            try? data.write(to: logFile!, options: .atomicWrite)
        }
}
