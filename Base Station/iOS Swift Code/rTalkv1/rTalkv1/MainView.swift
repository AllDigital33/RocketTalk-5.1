//
//  MainView.swift
//
//

import Foundation
import AVFoundation
import UIKit
import CoreLocation
import RedSerial


class MainView: UIViewController, CLLocationManagerDelegate, RedSerialPortDelegate, RedSerialDeviceManagerDelegate {

    @IBOutlet weak var labelLaunchNumber: UILabel!
    @IBOutlet weak var labelPhase: UILabel!
    @IBOutlet weak var labelLaunchEvent: UILabel!
    @IBOutlet weak var labelApogeeEvent: UILabel!
    @IBOutlet weak var labelLandedEvent: UILabel!
    @IBOutlet weak var labelAftEvent: UILabel!
    @IBOutlet weak var labelSeparationEvent: UILabel!
    @IBOutlet weak var labelForeEvent: UILabel!
    @IBOutlet weak var labelStatus: UILabel!
    @IBOutlet weak var labelFlightData: UILabel!
    @IBOutlet weak var labelGPSdata: UILabel!
    @IBOutlet weak var labelFlightAltitude: UILabel!
    @IBOutlet weak var labelSpeed: UILabel!
    @IBOutlet weak var labelTilt: UILabel!
    @IBOutlet weak var labelAccel: UILabel!
    @IBOutlet weak var labelMaxAltitude: UILabel!
    @IBOutlet weak var labelMaxSpeed: UILabel!
    @IBOutlet weak var labelMaxTilt: UILabel!
    @IBOutlet weak var labelMaxAccel: UILabel!
    @IBOutlet weak var labelMaxTemp: UILabel!
    @IBOutlet weak var labelGPSaltitude: UILabel!
    @IBOutlet weak var labelDistance: UILabel!
    @IBOutlet weak var labelAngle: UILabel!
    @IBOutlet weak var labelHeading: UILabel!
    @IBOutlet weak var labelGPSlatLong: UILabel!
    @IBOutlet weak var labelGPSMaxAGL: UILabel!
    @IBOutlet weak var labelGPSsatQ: UILabel!
    @IBOutlet weak var labelBattery: UILabel!
    @IBOutlet weak var labelTemp: UILabel!
    @IBOutlet weak var labelClock: UILabel!
    @IBOutlet weak var switchMute: UISwitch!
    @IBOutlet weak var switchVoice: UISwitch!
    @IBOutlet weak var switchFTS: UISwitch!
    @IBOutlet weak var imagePointer: UIImageView!
    @IBOutlet weak var imageRadio: UIImageView!
    @IBOutlet weak var imageTXarrow: UIImageView!
    @IBOutlet weak var imageRXarrow: UIImageView!
    @IBOutlet weak var imageCamera: UIImageView!
    @IBOutlet weak var continuitySeparation: UILabel!
    @IBOutlet weak var continuitySustainer: UILabel!
    @IBOutlet weak var continuityDrogue: UILabel!
    @IBOutlet weak var continuityMain: UILabel!
    @IBOutlet weak var pyroSeparation: UILabel!
    @IBOutlet weak var pyroSustainer: UILabel!
    @IBOutlet weak var pyroDrogue: UILabel!
    @IBOutlet weak var pyroMain: UILabel!
    @IBOutlet weak var buttonFTS: UIButton!
    @IBOutlet weak var labelMT: UILabel!
    @IBOutlet weak var labelErrors: UILabel!
    
    
    var locationManager: CLLocationManager = CLLocationManager()
    var myPort: RedSerialPort!
    var player: AVAudioPlayer?
    var timerOne: Timer?
    var easter1: Bool = false
    var theRadio: Bool = false

    override func viewDidLoad() {
        super.viewDidLoad()
        
        print("Main View Did Load start")
        //set tab bar fonts
        let systemFontAttributes = [NSAttributedString.Key.font: UIFont.systemFont(ofSize: 16.0)]
        UITabBarItem.appearance().setTitleTextAttributes(systemFontAttributes, for: .normal)
        

        UIreset()
        flightDataRefresh()
        zeroGPS()
        RedSerialDeviceManager.shared().delegate = self;
        RedSerialDeviceManager.shared().startDiscovery();
        imageRadio.image = UIImage(named: "bad")!
        theRadio = false
        
        
        //compass
        locationManager = CLLocationManager()
        locationManager.delegate = self
        locationManager.requestAlwaysAuthorization()
        
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.startUpdatingLocation()
        locationManager.startUpdatingHeading()
        
        //housekeeping timer
        timerOne = Timer.scheduledTimer(timeInterval: 0.5, target: self, selector: #selector(MainView.housekeepingTimer), userInfo: nil, repeats: true)

    }
    
    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(false)
        print("Main View did appear again")
        if(configData.mute) {
            switchMute.isOn = true
        } else {
            switchMute.isOn = false
        }
        if(configData.voice) {
            switchVoice.isOn = true
        } else {
            switchVoice.isOn = false
        }
        

    }

    func newGPSdata() {
        
        UIView.animate(withDuration: 0.0, animations: {
            var angle: CGFloat = 0.0
            if (baseData.heading! < baseData.rocketBearing!) {
                angle = degreesToRadians(CGFloat(baseData.rocketBearing! - baseData.heading!))
                print(baseData.rocketBearing! - baseData.heading!)
            } else {
                angle = degreesToRadians(CGFloat(360.0 - (baseData.heading! - baseData.rocketBearing!)))
                print(360.0 - (baseData.heading! + baseData.rocketBearing!))
            }
            self.imagePointer.transform = CGAffineTransform(rotationAngle: angle)
        })
        updateDistanceAngle()
        rocketGPS.newData = false
        
        // Log local base GPS calcs with every new GPS signal
        var localGPS: String
        localGPS = ""
        localGPS = "Base Data, " + "\(String(format: "%.5f", baseData.lat!))"+", " + "\(String(format: "%.5f", baseData.long!))"+", " + "\(String(format: "%.5f", baseData.altitude!))"+", " + "\(String(format: "%.5f", baseData.angle!))"+", " + "\(String(format: "%.5f", baseData.rocketBearing!))"+", " + "\(String(format: "%.5f", baseData.distance!))"+", "
        logging(message: localGPS, filename: workingData.gpsLog!)
        
    }
    
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {

        locations.forEach { (location) in
            //update base globals
            baseData.lat = Float(location.coordinate.latitude)
            baseData.long = Float(location.coordinate.longitude)
            baseData.altitude = Int32(location.altitude)
            updateDistanceAngle()
            //updateBearing()
        }
    }

    
    func locationManager(_ manager: CLLocationManager, didUpdateHeading newHeading: CLHeading) {
        
        if(rocketGPS.goodData!) {
            // Update big pointer
            baseData.heading = Float(newHeading.trueHeading) + 90.00 //adjust for landscape mode
            if (baseData.heading! > 360.0) {
                baseData.heading = baseData.heading! - 360.0
            }
            
            UIView.animate(withDuration: 0.0, animations: {
                var angle: CGFloat = 0.0
                if (baseData.heading! < baseData.rocketBearing!) {
                    angle = degreesToRadians(CGFloat(baseData.rocketBearing! - baseData.heading!))
                    print(baseData.rocketBearing! - baseData.heading!)
                } else {
                    angle = degreesToRadians(CGFloat(360.0 - (baseData.heading! - baseData.rocketBearing!)))
                    print(360.0 - (baseData.heading! + baseData.rocketBearing!))
                }
                self.imagePointer.transform = CGAffineTransform(rotationAngle: angle)
            })
        }
    }
    
    @IBAction func switchMute(_ sender: Any) {
    }
    
    @IBAction func switchVoice(_ sender: Any) {
        
        if(switchVoice.isOn) {
            configData.voice = true
        } else {
            configData.voice = false
        }
    }
    

    
    func UIreset() {
        labelLaunchEvent.backgroundColor = .white
        labelApogeeEvent.backgroundColor = .white
        labelLandedEvent.backgroundColor = .white
        labelAftEvent.backgroundColor = .white
        labelForeEvent.backgroundColor = .white
        labelFlightData.textColor = .white
        labelGPSdata.textColor = .white
        labelSeparationEvent.backgroundColor = .white
        continuitySeparation.backgroundColor = .lightGray
        continuitySustainer.backgroundColor = .lightGray
        continuityDrogue.backgroundColor = .lightGray
        continuityMain.backgroundColor = .lightGray
        pyroSeparation.backgroundColor = .lightGray
        pyroSustainer.backgroundColor = .lightGray
        pyroDrogue.backgroundColor = .lightGray
        pyroMain.backgroundColor = .lightGray
        
        //mute
        //voice
        //clock
    }
    
    func flightDataRefresh() {
        labelLaunchNumber.text = flightData.launchNumber
        if(flightData.launchNumber! == "0") { // SD Card error
            labelLaunchNumber.textColor = .red
        } else {
            labelLaunchNumber.textColor = .black
        }
        labelPhase.text = flightData.phase
        
        if(flightData.eventLaunch!) {
            labelLaunchEvent.backgroundColor = .green
        } else {
            labelLaunchEvent.backgroundColor = .white
        }
        if(flightData.eventApogee!) {
            labelApogeeEvent.backgroundColor = .green
        } else {
            labelApogeeEvent.backgroundColor = .white
        }
        if(flightData.eventLanded!) {
            labelLandedEvent.backgroundColor = .green
        } else {
            labelLandedEvent.backgroundColor = .white
        }
        if(flightData.eventAft!) {
            labelAftEvent.backgroundColor = .green
        } else {
            labelAftEvent.backgroundColor = .white
        }
        if(flightData.eventFore!) {
            labelForeEvent.backgroundColor = .green
        } else {
            labelForeEvent.backgroundColor = .white
        }
        if(flightData.eventSeparation!) {
            labelSeparationEvent.backgroundColor = .green
        } else {
            labelSeparationEvent.backgroundColor = .white
        }
        if(flightData.continuitySeparation!) {
            continuitySeparation.backgroundColor = .green
        } else {
            continuitySeparation.backgroundColor = .lightGray
        }
        if(flightData.continuitySustainer!) {
            continuitySustainer.backgroundColor = .green
        } else {
            continuitySustainer.backgroundColor = .lightGray
        }
        if(flightData.continuityDrogue!) {
            continuityDrogue.backgroundColor = .green
        } else {
            continuityDrogue.backgroundColor = .lightGray
        }
        if(flightData.continuityMain!) {
            continuityMain.backgroundColor = .green
        } else {
            continuityMain.backgroundColor = .lightGray
        }
        if(flightData.pyroSeparation!) {
            pyroSeparation.backgroundColor = .red
            pyroSeparation.textColor = .white
        } else {
            pyroSeparation.backgroundColor = .lightGray
            pyroSeparation.textColor = .black
        }
        if(flightData.pyroSustainer!) {
            pyroSustainer.backgroundColor = .red
            pyroSustainer.textColor = .white
        } else {
            pyroSustainer.backgroundColor = .lightGray
            pyroSustainer.textColor = .black
        }
        if(flightData.pyroDrogue!) {
            pyroDrogue.backgroundColor = .red
            pyroDrogue.textColor = .white
        } else {
            pyroDrogue.backgroundColor = .lightGray
            pyroDrogue.textColor = .black
        }
        if(flightData.pyroMain!) {
            pyroMain.backgroundColor = .red
            pyroMain.textColor = .white
        } else {
            pyroMain.backgroundColor = .lightGray
            pyroMain.textColor = .black
        }

        if(workingData.statusTimeout! > Date()) {
            labelStatus.text = workingData.status
        } else {
            labelStatus.text = ""
        }
        //clock ?
      
        
        let numberFormatter = NumberFormatter()
        numberFormatter.numberStyle = .decimal
        //let formattedNumber = numberFormatter.string(from: NSNumber(value:largeNumber))
        
        labelFlightAltitude.text = numberFormatter.string(from: NSNumber(value:flightData.altitude!))! + "'"
        labelSpeed.text = "\(String(flightData.speed!))" + " fps"
        labelTilt.text = "\(String(flightData.tilt!))" + "°"
        labelAccel.text = "\(String(format: "%.2f",flightData.accelZ!))" + " G"
        labelMaxAltitude.text = "\(String(flightData.altitudeMax!))" + "'"
        labelMaxSpeed.text = "\(String(flightData.speedMax!))" + " fps"
        labelMaxTilt.text = "\(String(flightData.tiltMax!))" + "°"
        labelMaxAccel.text = "\(String(format: "%.2f",flightData.maxAccel!))" + " G"
        labelMaxTemp.text = "\(String(flightData.maxTemp!))" + "°"
        labelBattery.text = "\(String(flightData.battery!))" + "%"
        labelTemp.text = "\(String(flightData.temp!))" + "°"
        
        if(errors.count ?? 0 > 0) {
            labelMT.isHidden = true
            labelMaxTemp.isHidden = true
            labelErrors.isHidden = false
        } else {
            labelMT.isHidden = false
            labelMaxTemp.isHidden = false
            labelErrors.isHidden = true
        }
        
    }
    

    
    func refreshGPS() {
       

            labelGPSlatLong.text = "\(String(format: "%.5f", rocketGPS.lat!))"+", "+"\(String(format: "%.5f", rocketGPS.long!))"
            labelGPSaltitude.text = "\(String(rocketGPS.altitude!))" + "'"
            labelGPSMaxAGL.text = "\(String(rocketGPS.maxAGL!))" + "'"
            labelGPSsatQ.text = "\(String(rocketGPS.sat!))"
            //update distance, angle, and bearing
            updateDistanceAngle ()
        let theBearing: Float = getBearing(rLat: rocketGPS.lat!, rLong: rocketGPS.long!)
            baseData.rocketBearing = theBearing
            labelHeading.text = "\(String(format: "%.0f", theBearing))" + "°"
            imagePointer.image = UIImage(named: "Arrow")!
            
    }
    
    func refreshStatus() {
        labelStatus.text = workingData.status
    }
    
    func updateDistanceAngle () {
        
        if (rocketGPS.goodData == false) { return }
        //Distance
        var theDistance: Float
        theDistance = getGPSDistance(rLat: rocketGPS.lat!, rLong: rocketGPS.long!)
        baseData.distance = theDistance
        print(theDistance)
        var labelTemp: String
        if (theDistance > 999) {
            labelTemp = "\(String(format: "%.2f", theDistance/1000))" + " Km"
        } else {
            labelTemp = "\(String(format: "%.0f", theDistance))" + " M"
        }
        if (theDistance < 20) { // the red zone
            labelDistance.textColor = #colorLiteral(red: 0.8078431487, green: 0.02745098062, blue: 0.3333333433, alpha: 1)
        } else {
            labelDistance.textColor = #colorLiteral(red: 0, green: 0, blue: 0, alpha: 1)
        }
        labelDistance.text = labelTemp
        // Elevation Angle
        var theAngle: Float = 0.0
        if(flightData.timeout! < Date()) { //GPS altitude is more reliable - auto switch
            theAngle = atan(Float(rocketGPS.altitudeMeters!)/theDistance) * 180.0 / Float.pi
        } else {
            theAngle = atan(Float(flightData.altitudeMeters!)/theDistance) * 180.0 / Float.pi
        }
        labelAngle.text = "\(String(format: "%.0f", theAngle))" + "°"
        baseData.angle = theAngle


    }

    func zeroGPS() {
        labelGPSaltitude.text = "0'"
        labelDistance.text = "0"
        labelAngle.text = "0"
        labelHeading.text = "0"
        labelGPSlatLong.text = "N/A"
        labelGPSMaxAGL.text = "0"
        labelGPSsatQ.text = "0"
        imagePointer.image = UIImage(named: "arrowoff")!
    }
    // *********** Radio Stuff below here ***************
    
    func deviceDetected(_ thePort: RedSerialPort) {
        
        imageRadio.image = UIImage(named: "good")!
        theRadio = true
        myPort = thePort
        myPort.delegate = self
        myPort.baudRate = 19200;
        myPort.dataConfiguration = kDataConfig_8N1;
        theRadio = true
        
        
        self.doReceive()
    }
    
    func deviceDisconnected(_ thePort: RedSerialPort) {
        imageRadio.image = UIImage(named: "bad")!
        theRadio = false
        myPort = nil
    }
    
    //************************************************  RADIO HANDLER HERE ************************************************
    
    func doReceive()  // Radio Handler
    {
        
        myPort.recvData { [self] (data: Data) in
            
            // Completion handler is called when bytes received
            // Put your code here to process incoming data from radio
            var goodSentence: Bool = false
            var malformed: Bool = false
            self.imageRXarrow.isHidden = false
            workingData.RXarrowTimeout = Date() + 2
            var dataString = String(data: data, encoding: String.Encoding.utf8)!
            
            //New logic to verify integrity and concatenate multiple sentences
            //first check for @ and ! characters
            let tempLeft: String = String(dataString.prefix(1))
            let tempRight: String = String(dataString.suffix(1))
            if (tempLeft == "@" && tempRight=="!") {
                //Got a good sentence
                if(workingData.lastRadio == true) { //clear previous partial sentence
                    malformed = true
                    workingData.lastRadio = false
                }
                goodSentence = true
            } else { // Process not regular sentence
                if (!(tempLeft == "@" || tempRight=="!")) { //garbage clear previous
                    workingData.lastRadio = false
                    malformed = true
                }
                if (tempLeft == "@") {  //Potentially Valid first half
                    if(workingData.lastRadio == true) { //clear previous malformed first half
                        malformed = true
                        workingData.lastRadio = false
                    } //valid first half
                    workingData.lastRadio = true
                    workingData.lastRadioString = dataString
                    dataString = dataString + " (PARTIAL)"
                }
                if (tempRight == "!") {  //Potentially Valid second half
                    if(workingData.lastRadio == true) { //got a match!
                        dataString = workingData.lastRadioString! + dataString
                        goodSentence = true
                        workingData.lastRadio = false
                        workingData.radioRXarray.append(String("(concatenated)"))
                        logging(message: String("(concatenated)"), filename: workingData.radioLog!)
                    } else {
                        malformed = true
                    }
                } //end valid second half
                if (malformed == true) {
                    dataString = dataString + " (MALFORMED!)"
                }
            } //end integrity check
            
            //Now Log the radio line
            let now = Date()
            let formatter = DateFormatter()
            formatter.dateStyle = .short
            formatter.timeStyle = .medium
            let datetime = formatter.string(from: now)
            //log it to memory and disk
            workingData.radioRXarray.append(String(datetime + "  " + dataString))
            logging(message: dataString, filename: workingData.radioLog!)
    
            // ***************************  PROCESS GOOD RADIO SENTANCES HERE **********************************

           if(goodSentence == true) {
            let radioProcess: Int = processRadio(theData: dataString)
            //self.labelStatus.text = "Radio Process = " + "\(String(radioProcess))"
            if (radioProcess == 1) { //received flight data
                self.flightDataRefresh()
                if(configData.alertRadioRX) {self.playSound(theFile: "radioRX")}
                
            }
            if (radioProcess == 2) { //received good GPS data
                self.refreshGPS()
                //check for apogee location
                if(flightData.phase == "DESCENT" && summaryData.baseApogeeDistance == "") {
                    summaryData.baseApogeeDistance = "\(String(format: "%.0f", baseData.distance!))" + " M"
                }
                if(configData.alertRadioRX) {self.playSound(theFile: "radioRX")}
                
            }
            if (radioProcess == 3) { //received good message
                self.refreshStatus()
                if(configData.alertMessages) {self.playSound(theFile: "message")}
                
            }
            if (radioProcess == 4) { //received good events
                self.flightDataRefresh()
                if(configData.alertEvents) {self.playSound(theFile: "event")}
                if(workingData.eventType == "LAUNCH" && configData.voice == true) {
                    speakIt(theMessage: "Launch Detected")
                }
                if(workingData.eventType == "STAGE" && configData.voice == true) {
                    speakIt(theMessage: "Staging Separation Detected")
                }
                if(workingData.eventType == "FORE" && configData.voice == true) {
                    speakIt(theMessage: "Forward Separation Detected")
                }
                if(workingData.eventType == "AFT" && configData.voice == true) {
                    speakIt(theMessage: "Aft Separation Detected")
                }
                if(workingData.eventType == "LANDED" && configData.voice == true) {
                    speakIt(theMessage: "Landing Detected")
                }
                if(workingData.eventType == "APOGEE" && configData.voice == true) {
                    speakIt(theMessage: "Apogee Detected")
                }
            }
            if (radioProcess == 5) { //summary 1
                self.refreshStatus()
                if(configData.alertMessages) {self.playSound(theFile: "message")}
            }
            if (radioProcess == 11) { //received booster flight data
                if(configData.alertRadioRX) {self.playSound(theFile: "booster2")}
            }
            if (radioProcess == 12) { //received good booster GPS data
                if(configData.alertRadioRX) {self.playSound(theFile: "booster2")}
            }
            if (radioProcess == 13) { //received booster summary data
                if(configData.alertRadioRX) {self.playSound(theFile: "booster2")}
            }
            
            
          } // end good sentence
            
            self.doReceive()
        }
    }
    
    func speakIt(theMessage: String) {
        let utterance = AVSpeechUtterance(string: theMessage)
        utterance.voice = AVSpeechSynthesisVoice(language: "en-US")
        //utterance.rate = 0.1
        let synthesizer = AVSpeechSynthesizer()
        synthesizer.speak(utterance)
    }
    
    func playSound(theFile: String) {
        if(configData.mute == false) {
            guard let url = Bundle.main.url(forResource: theFile, withExtension: "mp3") else { return }

            do {
                try AVAudioSession.sharedInstance().setCategory(.playback, mode: .default)
                try AVAudioSession.sharedInstance().setActive(true)
                player = try AVAudioPlayer(contentsOf: url, fileTypeHint: AVFileType.mp3.rawValue)
                guard let player = player else { return }
                player.play()
            } catch let error {
                print(error.localizedDescription)
            }
        }
    }

    @IBAction func switchFTSchange(_ sender: Any) {
        if(switchFTS.isOn) {
            buttonFTS.isEnabled = true
            buttonFTS.setImage(UIImage(named: "FTSred.png"), for: .normal)
        } else {
            buttonFTS.isEnabled = false
            buttonFTS.setImage(UIImage(named: "FTSgray.png"), for: .normal)
        }
    }
    
    @IBAction func switchMuteChange(_ sender: Any) {
        if(switchMute.isOn) {
            configData.mute = true
        } else {
            configData.mute = false
        }
    }
    
    @IBAction func buttonFTSsend(_ sender: Any) {
        if(switchFTS.isOn) {
          workingData.radioSendMessage = "@O,BOOM,033,!"
          workingData.radioSend = true
          workingData.status = "TERMINATION Command Sent"
          workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
          switchFTS.isOn = false
          buttonFTS.isEnabled = false
          buttonFTS.setImage(UIImage(named: "FTSgray.png"), for: .normal)
        }
    }
    @IBAction func buttonStatusRequest(_ sender: Any) {
        
        workingData.radioSendMessage = "@S,033,!"
        workingData.radioSend = true
        workingData.status = "Sent Status Request..."
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        
    }
/*
    @IBAction func buttonTempSpeedTest(_ sender: Any) {
        //zzz delete this later
        workingData.speedTestStart = Date()
        radioSend(theM: "@W,1234567890,!")
        workingData.status = "Sent speed test"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
    }
*/
    
    func radioSend(theM: String)
    {
        if(theRadio) {
            let textToSend: String! = theM
            let data = textToSend.data(using: String.Encoding.utf8);
            imageTXarrow.isHidden = false
            workingData.TXarrowTimeout = Date() + 2
            myPort.send(data!) {
                let theData:String = "SENT Request to rocket " + theM
                logging(message: theData, filename: workingData.flightLog!)
            }
            self.playSound(theFile: "event")
        } else {
            workingData.status = "Error: No radio connection"
            workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        }
    }
    
    
    @objc func housekeepingTimer() {
        
       // clear status
        if(workingData.statusTimeout! < Date() && labelStatus.text != "") {
            labelStatus.text = ""
        }
        //update status
        if(workingData.statusTimeout! > Date()) {
            labelStatus.text = workingData.status
        }
        // radio light
        if(workingData.RXarrowTimeout! < Date() && imageRXarrow.isHidden == false) {
            imageRXarrow.isHidden = true
        }
        if(workingData.TXarrowTimeout! < Date() && imageTXarrow.isHidden == false) {
            imageTXarrow.isHidden = true
        }
        // flight data timeout and gps data timeout
        if(flightData.timeout! < Date()) {
            labelFlightData.textColor = .red
            if(configData.alarmNoRadio == 1 && flightData.phase != "STARTUP") {
                speakIt(theMessage: "Lost Radio Transmissions")
                configData.alarmNoRadio = 2
            }
        } else {
            labelFlightData.textColor = .white
        }
        if(rocketGPS.timeout! < Date()) {
            labelGPSdata.textColor = .red
        } else {
            labelGPSdata.textColor = .white
        }
        //new GPS data
        if(rocketGPS.newData == true) {
           newGPSdata()
        }
        // Launch clock
        if(flightData.eventLaunch == true) {
            let theDiff: Int = getDateDiff(start: workingData.launchTime!, end: Date())
            let (h,m,s) = secondsToHoursMinutesSeconds(seconds: theDiff)
            let displayClock: String = "\(String(format: "%01d", h))" + ":" + "\(String(format: "%02d", m))" + ":" + "\(String(format: "%02d", s))"
            labelClock.text = displayClock
        }
        // process anything on the send queue
        if(workingData.radioSend!) {
            radioSend(theM: workingData.radioSendMessage!)
            workingData.radioSend = false
        }
        // camera icon
        if(workingData.camera! == true) {
            imageCamera.isHidden = false
        } else {
            imageCamera.isHidden = true
        }
        // check for major alarms
        // rocket temperature
        if(configData.alarmTemp == 1 && flightData.temp! > 150) {
            speakIt(theMessage: "Warning. Rocket High Temperature")
            configData.alarmTemp = 2
            workingData.status = "Warning. Rocket High Temperature"
            workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
            logging(message: "Alarm: Warning. Rocket High Temperature", filename: workingData.flightLog!)
        }
        // rocket battery
        if(configData.alarmBattery == 1 && flightData.battery! < 20 && flightData.goodData == true) {
            speakIt(theMessage: "Warning. Rocket Battery Low")
            configData.alarmBattery = 2
            workingData.status = "Warning. Rocket Battery Low"
            workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
            logging(message: "Alarm: Warning. Rocket Battery Low", filename: workingData.flightLog!)
        }
        // rocket descent speed  (greater than 120 fps falling)
        if(configData.alarmSpeed == 1 && flightData.phase == "DESCENT" && flightData.speed! < -100) {
            speakIt(theMessage: "Danger. High Rocket Descent Speed.")
            configData.alarmSpeed = 2
            workingData.status = "Danger. High Rocket Descent Speed."
            workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
            logging(message: "Alarm: High Rocket Descent Speed.", filename: workingData.flightLog!)
        }
        // rocket descent distance - less than 50M warning
        if(configData.alarmDistance == 1 && flightData.phase == "DESCENT" && baseData.distance! < 30) {
            if(Date() < (rocketGPS.timeout! - 20)) { //ensure a recent GPS update
                speakIt(theMessage: "Danger. Rocket Directly Overhead.")
                configData.alarmDistance = 2
                workingData.status = "Danger. Rocket Directly Overhead."
                workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
                logging(message: "Alarm: Danger. Rocket Directly Overhead.", filename: workingData.flightLog!)
            }
        }
        // rocket tilt warning
        if(configData.alarmTilt == 1 && flightData.tilt! > 45 && flightData.phase == "LAUNCH") {
                speakIt(theMessage: "Rocket Tilt Warning")
                configData.alarmTilt = 2
            workingData.status = "Rocket Tilt Warning"
            workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
            logging(message: "Alarm: Rocket Tilt Warning", filename: workingData.flightLog!)
        }
    }
    
    func getDateDiff(start: Date, end: Date) -> Int  {
        let calendar = Calendar.current
        let dateComponents = calendar.dateComponents([Calendar.Component.second], from: start, to: end)

        let seconds = dateComponents.second
        return Int(seconds!)
    }
    func secondsToHoursMinutesSeconds (seconds : Int) -> (Int, Int, Int) {
      return (seconds / 3600, (seconds % 3600) / 60, (seconds % 3600) % 60)
    }
    
    @IBAction func buttonHidden1(_ sender: Any) {  //easter egg
        easter1 = true
    }
    @IBAction func buttonHidden2(_ sender: Any) {  //easter egg
        if (easter1 == true && configData.voice == true) {
            easter1 = false
            speakIt(theMessage: "Preston. You must make the rocket gods happy")
        }
        
    }

    @IBAction func buttonHidden3(_ sender: Any) {
        
        if (easter1 == true && configData.voice == true) {
            let monte: String = "And the Lord spoke, saying, First shalt thou take out the Holy Pin. Then shalt thou count to three, no more, no less. Three shall be the number thou shalt count, and the number of the counting shall be three. Four shalt thou not count, neither count thou two, excepting that thou then proceed to three. Five is right out! Once the number three, being the third number, be reached, then launchith thou thy Holy Rocket of Antioch towards thy sky and pray it does not cato."
            
            let utterance = AVSpeechUtterance(string: monte)
            utterance.voice = AVSpeechSynthesisVoice(language: "en-GB")
            //utterance.rate = 0.1
            let synthesizer = AVSpeechSynthesizer()
            synthesizer.speak(utterance)
        }
    }
    

    
} // end view







