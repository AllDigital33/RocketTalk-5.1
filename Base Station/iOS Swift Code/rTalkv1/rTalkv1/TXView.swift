//
//  TXView.swift
//

import UIKit


class TXView: UIViewController {

    
    @IBOutlet weak var buttonStatus: UIButton!
    @IBOutlet weak var buttonMarco: UIButton!
    @IBOutlet weak var buttonLaunchMode: UIButton!
    @IBOutlet weak var buttonBasicMode: UIButton!

    @IBOutlet weak var buttonReset: UIButton!
    @IBOutlet weak var buttonBoom: UIButton!
    @IBOutlet weak var buttonMain: UIButton!
    @IBOutlet weak var buttonDrogue: UIButton!
    @IBOutlet weak var buttonSustainer: UIButton!
    @IBOutlet weak var buttonSeparation: UIButton!
    @IBOutlet weak var switchLaunch: UISwitch!
    @IBOutlet weak var switchBasic: UISwitch!
    @IBOutlet weak var switchAbort: UISwitch!
    @IBOutlet weak var switchReset: UISwitch!
    @IBOutlet weak var switchBoom: UISwitch!
    @IBOutlet weak var labelStatus: UILabel!
    @IBOutlet weak var switchDescent: UISwitch!
    @IBOutlet weak var switchRadio: UISwitch!
    @IBOutlet weak var switchVideo: UISwitch!
    @IBOutlet weak var switchMain: UISwitch!
    @IBOutlet weak var switchDrogue: UISwitch!
    @IBOutlet weak var switchSustainer: UISwitch!
    @IBOutlet weak var switchSeparation: UISwitch!
    @IBOutlet weak var switchRadioOff: UISwitch!
    @IBOutlet weak var switchBoosterRadio: UISwitch!
    
    @IBOutlet weak var switchBoosterReset: UISwitch!
    
    @IBOutlet weak var switchEraseFlash: UISwitch!
    @IBOutlet weak var switchBoosterBasic: UISwitch!
    
    var timerOne: Timer?
    
    
    override func viewDidLoad() {
        super.viewDidLoad()

        resetView()
    }
    
    override func viewDidAppear(_ animated: Bool) { //refresh everything
        super.viewDidAppear(false)
        
        resetView()
        //housekeeping timer
        timerOne = Timer.scheduledTimer(timeInterval: 1.0, target: self, selector: #selector(TXView.housekeepingTimer2), userInfo: nil, repeats: true)
        
    }
    
    
    func resetView() {
        
        switchLaunch.isOn = false
        switchBasic.isOn = false
        switchBoosterBasic.isOn = false
        switchReset.isOn = false
        switchBoosterReset.isOn = false
        switchBoom.isOn = false
        labelStatus.text = ""
    
    }

    @objc func housekeepingTimer2() {
        
       // clear status
        if(workingData.statusTimeout! < Date() && labelStatus.text != "") {
            labelStatus.text = ""
        }
        //update status
        if(workingData.statusTimeout! > Date()) {
            labelStatus.text = workingData.status
        }
    }

    
    @IBAction func buttonStatus(_ sender: Any) {
        
        workingData.radioSendMessage = "@S,033,!"
        workingData.radioSend = true
        workingData.status = "Sent Summary Request..."
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))

    }
    
    @IBAction func buttonMarco(_ sender: Any) {
        
        workingData.radioSendMessage = "@P,033,!"
        workingData.radioSend = true
        workingData.status = "Marco"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        
    }
    
    @IBAction func buttonLaunchMode(_ sender: Any) {
        if(switchLaunch.isOn) {
        workingData.radioSendMessage = "@L,033,!"
        workingData.radioSend = true
        workingData.status = "Sent Launch Mode Request..."
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        switchLaunch.isOn = false
        }
    }
    
    @IBAction func buttonBasicMode(_ sender: Any) {
        if(switchBasic.isOn) {
        workingData.radioSendMessage = "@B,033,!"
        workingData.radioSend = true
        workingData.status = "Sent Basic Mode Request..."
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        switchBasic.isOn = false
        }
    }
    
   
    @IBAction func buttonReset(_ sender: Any) {
        if(switchReset.isOn) {
        workingData.radioSendMessage = "@R,033,!"
        workingData.radioSend = true
        workingData.status = "Sent Reset Command"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        switchReset.isOn = false
        }
    }
    
    @IBAction func buttonBoosterReset(_ sender: Any) {
        if(switchBoosterReset.isOn) {
            workingData.radioSendMessage = "@#R,033,!"
            workingData.radioSend = true
            workingData.status = "Sent Booster Reset Command"
            workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
            switchBoosterReset.isOn = false
        }
    }
    
    @IBAction func buttonBoosterBasic(_ sender: Any) {
        if(switchBoosterBasic.isOn) {
        workingData.radioSendMessage = "@#B,033,!"
        workingData.radioSend = true
        workingData.status = "Sent Booster Basic Mode Request..."
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        switchBoosterBasic.isOn = false
        }
    }
    
    
    
    
    @IBAction func buttonBoosterBoom(_ sender: Any) {
        if(switchBoom.isOn) {
        workingData.radioSendMessage = "@#O,BOOM,033,!"
        workingData.radioSend = true
        workingData.status = "BOOSTER BOOM! Sent"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        }
    }
    
    @IBAction func buttonBoom(_ sender: Any) {
        if(switchBoom.isOn) {
        workingData.radioSendMessage = "@O,BOOM,033,!"
        workingData.radioSend = true
        workingData.status = "BOOM! Sent"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        }
    }
    
    @IBAction func buttonMain(_ sender: Any) {
        if(switchMain.isOn) {
        workingData.radioSendMessage = "@O,MAIN,033,!"
        workingData.radioSend = true
        workingData.status = "Pyro Fire MAIN sent!"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        }
    }
    @IBAction func buttonDrogue(_ sender: Any) {
        if(switchDrogue.isOn) {
        workingData.radioSendMessage = "@O,DROGUE,033,!"
        workingData.radioSend = true
        workingData.status = "Pyro Fire DROGUE Sent!"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        }
    }
    
    @IBAction func buttonBoosterMain(_ sender: Any) {
        if(switchMain.isOn) {
        workingData.radioSendMessage = "@#O,MAIN,033,!"
        workingData.radioSend = true
        workingData.status = "Pyro Fire BOOSTER MAIN sent!"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        }
    }
    
    @IBAction func buttonBoosterDrogue(_ sender: Any) {
        if(switchDrogue.isOn) {
        workingData.radioSendMessage = "@#O,DROGUE,033,!"
        workingData.radioSend = true
        workingData.status = "Pyro Fire BOOSTER DROGUE Sent!"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        }
    }

    @IBAction func buttonEraseFlash(_ sender: Any) {
        if(switchEraseFlash.isOn) {
        workingData.radioSendMessage = "@9,033,!"
        workingData.radioSend = true
        workingData.status = "Erase Flash Request Sent!"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        }
    }
    
    @IBAction func buttonSustainer(_ sender: Any) {
        if(switchSustainer.isOn) {
        workingData.radioSendMessage = "@O,SUSTAINER,033,!"
        workingData.radioSend = true
        workingData.status = "Pyro Fire SUSTAINER Sent!"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        }
    }
    @IBAction func buttonSeparation(_ sender: Any) {
        if(switchSeparation.isOn) {
        workingData.radioSendMessage = "@O,SEPARATION,033,!"
        workingData.radioSend = true
        workingData.status = "Pyro Fire STAGING SEPARATION Sent!"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        }
    }
    
    @IBAction func buttonDescent(_ sender: Any) {
        if(switchDescent.isOn) {
        workingData.radioSendMessage = "@D,033,!"
        workingData.radioSend = true
        workingData.status = "Sent Descent Mode Request..."
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        }
    }
    @IBAction func buttonRadio(_ sender: Any) {
        if(switchRadio.isOn) {
        workingData.radioSendMessage = "@T,033,!"
        workingData.radioSend = true
        workingData.status = "Sent Radio Test..."
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        }
    }
    
    @IBAction func buttonSpeedTest(_ sender: Any) {
        
        if(switchRadio.isOn) {
        workingData.speedTestStart = Date()
        workingData.radioSendMessage = "@W,1234567890,!"
        workingData.radioSend = true
        workingData.status = "Sent Speed Test..."
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        }
    }
    
    @IBAction func buttonRadioOff(_ sender: Any) {
        if(switchRadioOff.isOn) {
            workingData.radioSendMessage = "@0R,033,!"
            workingData.radioSend = true
            workingData.status = "Sent Radio OFF..."
            workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
            switchRadioOff.isOn = false
        }
    }
    
    @IBAction func buttonRadioOn(_ sender: Any) {
        if(switchRadioOff.isOn) {
            workingData.radioSendMessage = "@1R,033,!"
            workingData.radioSend = true
            workingData.status = "Sent Radio ON..."
            workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
            switchRadioOff.isOn = false
        }
        
        
    }
    
    @IBAction func buttonBoosterRadioOff(_ sender: Any) {
        if(switchBoosterRadio.isOn) {
            workingData.radioSendMessage = "@#0R,033,!"
            workingData.radioSend = true
            workingData.status = "Sent Booster Radio OFF..."
            workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
            switchBoosterRadio.isOn = false
        }
    }
    
    @IBAction func buttonBoosterRadioOn(_ sender: Any) {
        if(switchBoosterRadio.isOn) {
            workingData.radioSendMessage = "@#1R,033,!"
            workingData.radioSend = true
            workingData.status = "Sent Booster Radio ON..."
            workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
            switchBoosterRadio.isOn = false
        }
    }
    
    
    
    
    @IBAction func buttonCamera(_ sender: Any) {
        if(switchVideo.isOn) {
        workingData.radioSendMessage = "@V,033,!"
        workingData.radioSend = true
        workingData.status = "Sent Request"
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        }
    }
    
}



    //  @S = get status
    //  @A = Abort
    //  @L = Launch
    //  @B = Basic
    //  @R = Reset (must use abort first)
    //  @M = BOOM
    //  @P = Marco/Polo
    //  @T = Radio Test
    //  @D = Force Descent Mode
    //  @V = Toggle Video Camera On/Off
