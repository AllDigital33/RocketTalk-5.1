//
//  RadioView.swift
//

import UIKit

class RadioView: UIViewController {

    @IBOutlet weak var textBox: UITextView!
    @IBOutlet weak var switchRefreshLive: UISwitch!
    var currentCount: Int = 0
    var timerRadio: Timer?
    
    
    override func viewDidLoad() {
        super.viewDidLoad()
        

        for item in workingData.radioRXarray {
            textBox.text += item
            textBox.text += "\n"
        }
        currentCount = workingData.radioRXarray.count

        //housekeeping timer
        timerRadio = Timer.scheduledTimer(timeInterval: 2.0, target: self, selector: #selector(RadioView.radioTimer), userInfo: nil, repeats: true)

    }
    
    @objc func radioTimer() {
        
        if(switchRefreshLive.isOn) {
            if(workingData.radioRXarray.count > currentCount) {
                let theStart: Int = currentCount
                let theEnd: Int = workingData.radioRXarray.count - 1
                for n in theStart...theEnd {
                    textBox.text += workingData.radioRXarray[n]
                    textBox.text += "\n"
                }
                currentCount = workingData.radioRXarray.count
            }
        }
    }


}

/*
 
 

 
 
 
 
 
 */
