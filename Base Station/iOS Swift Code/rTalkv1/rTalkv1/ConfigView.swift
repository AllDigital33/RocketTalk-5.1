//
//  ConfigView.swift
//

import UIKit
import AVFoundation


class ConfigView: UIViewController {

    
    @IBOutlet weak var switchMute: UISwitch!
    @IBOutlet weak var switchVoice: UISwitch!
    @IBOutlet weak var switchRadio: UISwitch!
    @IBOutlet weak var switchEvents: UISwitch!
    @IBOutlet weak var switchMessages: UISwitch!
    @IBOutlet weak var switchTemp: UISwitch!
    @IBOutlet weak var switchTilt: UISwitch!
    @IBOutlet weak var switchNoRadio: UISwitch!
    @IBOutlet weak var switchSpeed: UISwitch!
    @IBOutlet weak var switchDistance: UISwitch!
    @IBOutlet weak var switchUnlockReset: UISwitch!
    @IBOutlet weak var switchBattery: UISwitch!
    @IBOutlet weak var labelStatus: UILabel!
    @IBOutlet weak var labelErrSample: UILabel!
    @IBOutlet weak var labelErrLogging: UILabel!
    @IBOutlet weak var labelErrADXL: UILabel!
    @IBOutlet weak var labelErrLSM: UILabel!
    @IBOutlet weak var labelErrMS56: UILabel!
    @IBOutlet weak var labelErrGPS: UILabel!
    @IBOutlet weak var labelErrFlash: UILabel!
    @IBOutlet weak var labelErrVoltage: UILabel!
    @IBOutlet weak var labelErrTemp: UILabel!
    @IBOutlet weak var labelErrI2C: UILabel!
    @IBOutlet weak var labelErrCrash: UILabel!
    @IBOutlet weak var labelErrRadio: UILabel!
    
    
    var timerOne: Timer?
    
    var player: AVAudioPlayer?
    
    
    override func viewDidLoad() {
        super.viewDidLoad()
        timerOne = Timer.scheduledTimer(timeInterval: 1.0, target: self, selector: #selector(ConfigView.housekeepingTimer3), userInfo: nil, repeats: true)
        refreshConfig()
        
    }
    
    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(false)
       
        print("View did appear again")
        refreshConfig()
    }
    
    @objc func housekeepingTimer3() {
        
       // clear status
        if(workingData.statusTimeout! < Date() && labelStatus.text != "") {
            labelStatus.text = ""
        }
        //update status
        if(workingData.statusTimeout! > Date()) {
            labelStatus.text = workingData.status
        }
    }
    func refreshConfig() {
        labelStatus.text = ""
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
        if(configData.alertRadioRX) {
            switchRadio.isOn = true
        } else {
            switchRadio.isOn = false
        }
        if(configData.alertEvents) {
            switchEvents.isOn = true
        } else {
            switchEvents.isOn = false
        }
        if(configData.alertEvents) {
            switchMessages.isOn = true
        } else {
            switchMessages.isOn = false
        }
        if(configData.alarmTemp > 0) {
            switchTemp.isOn = true
        } else {
            switchTemp.isOn = false
        }
        if(configData.alarmTilt > 0) {
            switchTilt.isOn = true
        } else {
            switchTilt.isOn = false
        }
        if(configData.alarmNoRadio > 0) {
            switchNoRadio.isOn = true
        } else {
            switchNoRadio.isOn = false
        }
        if(configData.alarmSpeed > 0) {
            switchSpeed.isOn = true
        } else {
            switchSpeed.isOn = false
        }
        if(configData.alarmDistance > 0) {
            switchDistance.isOn = true
        } else {
            switchDistance.isOn = false
        }
        if(configData.alarmBattery > 0) {
            switchBattery.isOn = true
        } else {
            switchBattery.isOn = false
        }
        switchUnlockReset.isOn = false
        
        // Health Check and Errors
        
        if(errors.sampleLoop == 1) {
            labelErrSample.backgroundColor = .red
        } else if(errors.sampleLoop == 2) {
            labelErrSample.backgroundColor = .green
        } else {
            labelErrSample.backgroundColor = .lightGray
        }
        if(errors.loggingLoop == 1) {
            labelErrLogging.backgroundColor = .red
        } else if(errors.loggingLoop == 2) {
            labelErrLogging.backgroundColor = .green
        } else {
            labelErrLogging.backgroundColor = .lightGray
        }
        if(errors.ADXL == 1) {
            labelErrADXL.backgroundColor = .red
        } else if(errors.ADXL == 2) {
            labelErrADXL.backgroundColor = .green
        } else {
            labelErrADXL.backgroundColor = .lightGray
        }
        if(errors.LSM == 1) {
            labelErrLSM.backgroundColor = .red
        } else if(errors.LSM == 2) {
            labelErrLSM.backgroundColor = .green
        } else {
            labelErrLSM.backgroundColor = .lightGray
        }
        if(errors.MS56 == 1) {
            labelErrMS56.backgroundColor = .red
        } else if(errors.MS56 == 2) {
            labelErrMS56.backgroundColor = .green
        } else {
            labelErrMS56.backgroundColor = .lightGray
        }
        if(errors.GPS == 1) {
            labelErrGPS.backgroundColor = .red
        } else if(errors.GPS == 2) {
            labelErrGPS.backgroundColor = .green
        } else {
            labelErrGPS.backgroundColor = .lightGray
        }
        if(errors.flash == 1) {
            labelErrFlash.backgroundColor = .red
        } else if(errors.flash == 2) {
            labelErrFlash.backgroundColor = .green
        } else {
            labelErrFlash.backgroundColor = .lightGray
        }
        if(errors.voltage == 1) {
            labelErrVoltage.backgroundColor = .red
        } else if(errors.voltage == 2) {
            labelErrVoltage.backgroundColor = .green
        } else {
            labelErrVoltage.backgroundColor = .lightGray
        }
        if(errors.CPUtemp == 1) {
            labelErrTemp.backgroundColor = .red
        } else if(errors.CPUtemp == 2) {
            labelErrTemp.backgroundColor = .green
        } else {
            labelErrTemp.backgroundColor = .lightGray
        }
        if(errors.I2C == 1) {
            labelErrI2C.backgroundColor = .red
        } else if(errors.I2C == 2) {
            labelErrI2C.backgroundColor = .green
        } else {
            labelErrI2C.backgroundColor = .lightGray
        }
        if(errors.crash == 1) {
            labelErrCrash.backgroundColor = .red
        } else if(errors.crash == 2) {
            labelErrCrash.backgroundColor = .green
        } else {
            labelErrCrash.backgroundColor = .lightGray
        }
        if(errors.radio == 1) {
            labelErrRadio.backgroundColor = .red
        } else if(errors.radio == 2) {
            labelErrRadio.backgroundColor = .green
        } else {
            labelErrRadio.backgroundColor = .lightGray
        }
        
    }
    
    
    @IBAction func changedBattery(_ sender: Any) {
        if(switchBattery.isOn) {configData.alarmBattery = 1} else {configData.alarmBattery = 0}
    }
    @IBAction func changedMute(_ sender: Any) {
        if(switchMute.isOn) {configData.mute = true} else {configData.mute = false}
    }
    @IBAction func changedVoice(_ sender: Any) {
        if(switchVoice.isOn) {configData.voice = true} else {configData.voice = false}
    }
    @IBAction func changedRadioRX(_ sender: Any) {
        if(switchRadio.isOn) {configData.alertRadioRX = true} else {configData.alertRadioRX = false}
    }
    @IBAction func changedEvents(_ sender: Any) {
        if(switchEvents.isOn) {configData.alertEvents = true} else {configData.alertEvents = false}
    }
    @IBAction func changedMessages(_ sender: Any) {
        if(switchMessages.isOn) {configData.alertMessages = true} else {configData.alertMessages = false}
    }
    @IBAction func changedTemp(_ sender: Any) {
        if(switchTemp.isOn) {configData.alarmTemp = 1} else {configData.alarmTemp = 0}
    }
    @IBAction func changedTilt(_ sender: Any) {
        if(switchTilt.isOn) {configData.alarmTilt = 1} else {configData.alarmTilt = 0}
    }
    @IBAction func changedNoRadio(_ sender: Any) {
        if(switchNoRadio.isOn) {configData.alarmNoRadio = 1} else {configData.alarmNoRadio = 0}
    }
    @IBAction func changedSpeed(_ sender: Any) {
        if(switchSpeed.isOn) {configData.alarmSpeed = 1} else {configData.alarmSpeed = 0}
    }
    @IBAction func changedDistance(_ sender: Any) {
        if(switchDistance.isOn) {configData.alarmDistance = 1} else {configData.alarmDistance = 0}
    }
    @IBAction func buttonResetAll(_ sender: Any) {
        if(switchUnlockReset.isOn) {
            resetAll()
            switchUnlockReset.isOn = false
            self.playSound(theFile: "event")
            
        }
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
    
}
