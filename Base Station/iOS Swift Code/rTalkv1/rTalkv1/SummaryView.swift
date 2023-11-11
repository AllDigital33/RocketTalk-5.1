//
//  SummaryView.swift
//

import UIKit


class SummaryView: UIViewController{
    

    @IBOutlet weak var labelSummary1: UILabel!
    @IBOutlet weak var labelSummary2: UILabel!
    @IBOutlet weak var labelSummary3: UILabel!
    @IBOutlet weak var labelLaunchTime: UILabel!
    @IBOutlet weak var labelApogeeTime: UILabel!
    @IBOutlet weak var labelAftTime: UILabel!
    @IBOutlet weak var labelForeTime: UILabel!
    @IBOutlet weak var labelLandedTime: UILabel!
    @IBOutlet weak var labelAscendSeconds: UILabel!
    @IBOutlet weak var labelDescendSeconds: UILabel!
    @IBOutlet weak var labelBurnoutSeconds: UILabel!
    @IBOutlet weak var labelBaroMaxAGL: UILabel!
    @IBOutlet weak var labelGPSmaxAGL: UILabel!
    @IBOutlet weak var labelAccelMax: UILabel!
    @IBOutlet weak var labelTempMax: UILabel!
    @IBOutlet weak var labelTiltMax: UILabel!
    @IBOutlet weak var labelBattery: UILabel!
    @IBOutlet weak var labelSpeedFPS: UILabel!
    @IBOutlet weak var labelSpeedMPS: UILabel!
    @IBOutlet weak var labelSpeedMPH: UILabel!
    @IBOutlet weak var labelDescentRate: UILabel!
    @IBOutlet weak var labelLandedRate: UILabel!
    @IBOutlet weak var labelGPS: UILabel!
    @IBOutlet weak var labelBaseLaunch: UILabel!
    @IBOutlet weak var labelBaseApogee: UILabel!
    @IBOutlet weak var labelBaseAft: UILabel!
    @IBOutlet weak var labelBaseFore: UILabel!
    @IBOutlet weak var labelBaseLanded: UILabel!
    @IBOutlet weak var labelBaseApogeeDistance: UILabel!
    @IBOutlet weak var labelBaseLandedDistance: UILabel!
    @IBOutlet weak var labelStatus: UILabel!
    
    var timerOne: Timer?
    
    override func viewDidLoad() {
        super.viewDidLoad()
        print("Summary View Did Load")
        
        refreshSummary()
        
    }
    
    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(false)
       
        print("View did appear again")
        refreshSummary()
        
        timerOne = Timer.scheduledTimer(timeInterval: 1.0, target: self, selector: #selector(SummaryView.housekeepingTimer4), userInfo: nil, repeats: true)
    }
    
    @objc func housekeepingTimer4() {
        
       // clear status
        if(workingData.statusTimeout! < Date() && labelStatus.text != "") {
            labelStatus.text = ""
        }
        //update status
        if(workingData.statusTimeout! > Date()) {
            labelStatus.text = workingData.status
        }
    }
    
    
    

    func refreshSummary() {
        
        labelStatus.text = ""
        if(summaryData.radio1!) {
            labelSummary1.text = "YES"
            labelSummary1.textColor = .green
        } else {
            labelSummary1.text = "NO"
            labelSummary1.textColor = .red
        }
        if(summaryData.radio2!) {
            labelSummary2.text = "YES"
            labelSummary2.textColor = .green
        } else {
            labelSummary2.text = "NO"
            labelSummary2.textColor = .red
        }
        if(summaryData.radio3!) {
            labelSummary3.text = "YES"
            labelSummary3.textColor = .green
        } else {
            labelSummary3.text = "NO"
            labelSummary3.textColor = .red
        }
        
        labelLaunchTime.text = summaryData.launchTime
        labelApogeeTime.text = summaryData.apogeeTime
        labelAftTime.text = summaryData.aftTime
        labelForeTime.text = summaryData.foreTime
        labelLandedTime.text = summaryData.landedTime
        labelAscendSeconds.text = summaryData.ascentSeconds
        labelDescendSeconds.text = summaryData.descentSeconds
        labelBurnoutSeconds.text = summaryData.burnoutSeconds
        labelBaroMaxAGL.text = summaryData.aglBaro
        labelGPSmaxAGL.text = summaryData.aglGPS
        labelAccelMax.text = summaryData.accelMax
        labelTempMax.text = summaryData.tempMax
        labelTiltMax.text = summaryData.tiltMax
        labelBattery.text = summaryData.battery
        labelSpeedFPS.text = summaryData.speedBaroFPS
        labelSpeedMPS.text = summaryData.speedBaroMPS
        labelSpeedMPH.text = summaryData.speedBaroMPH
        labelDescentRate.text = summaryData.descentRate
        labelLandedRate.text = summaryData.landedRate
        labelGPS.text = summaryData.landedGPScoord
        
        let formatter = DateFormatter()
        formatter.dateFormat = "hh:mm:ss"
        var timestamp = formatter.string(from: summaryData.baseLaunchTime!)
        labelBaseLaunch.text = timestamp
        timestamp = formatter.string(from: summaryData.baseApogeeTime!)
        labelBaseApogee.text = timestamp
        timestamp = formatter.string(from: summaryData.baseAftTime!)
        labelBaseAft.text = timestamp
        timestamp = formatter.string(from: summaryData.baseForeTime!)
        labelBaseFore.text = timestamp
        timestamp = formatter.string(from: summaryData.baseLandedTime!)
        labelBaseLanded.text = timestamp
        
        labelBaseApogeeDistance.text = summaryData.baseApogeeDistance
        labelBaseLandedDistance.text = summaryData.baseLandedDistance
        
    }
    
    @IBAction func buttonReqSummary(_ sender: Any) {
        
        workingData.radioSendMessage = "@Z,033,!"
        workingData.radioSend = true
        workingData.status = "Sent Summary Request..."
        workingData.statusTimeout = (Date() + TimeInterval(configData.statusTime))
        

    }
    
    
    
    
    
    
    
}


//S1:  @S1,#,launch number, launch time, apogee time, landed time, aft event time, fore event time, ascent seconds, descent seconds,
//S2:  @S2,#,burnout seconds, AGL Baro, AGL GPS, AGL Accel, Launch Accel Max, Max temp, Max tilt, Batt voltage,
//S3:  @S3,#,Speed baro fps, mps, mph, SpeedA mps, Descent rate fps, landed rate fps, Landed GPS coord
//@S1,45,45,13:09:14B,13:16:47B,13:18:02,13:16:45,13:16:45,452,75,,!
//@S2,45,n/a,2139,2248,n/a,15058,111,37,76.74,,!
//@S3,45,280,85,191,n/a,29,2,35 20.741,-117 48.740',3520.74187,11748.74066,!
