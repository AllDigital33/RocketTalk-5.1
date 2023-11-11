//
//  FilesView.swift
//

import UIKit
import Foundation
import AVFoundation
import MessageUI

class FilesView: UIViewController, UITableViewDelegate, UITableViewDataSource, MFMailComposeViewControllerDelegate {

    @IBOutlet weak var textFileContents: UITextView!
    @IBOutlet weak var tableView: UITableView!
    @IBOutlet weak var buttonDelete: UIButton!
    @IBOutlet weak var switchDelete: UISwitch!

    let cellReuseIdentifier = "cell"
    let cellSpacingHeight: CGFloat = 1
    var theFileList = [String]()
    var theSelected: String? = ""
    
    var player: AVAudioPlayer?

    
    override func viewDidLoad() {
        super.viewDidLoad()
        textFileContents.layer.borderWidth = 2
        textFileContents.layer.borderColor = UIColor.black.cgColor
        tableView.layer.borderWidth = 2
        tableView.layer.borderColor = UIColor.black.cgColor
        buttonDelete.layer.borderColor = UIColor.black.cgColor
        
        
        let filemgr = FileManager.default
        let dirPaths = filemgr.urls(for: .documentDirectory, in: .userDomainMask)
        let docsDir = dirPaths[0].path

        do {
            //let filelist = try filemgr.contentsOfDirectory(atPath: "/")
            let filelistR = try filemgr.contentsOfDirectory(atPath: docsDir)
            let filelist = filelistR.sorted()
            for filename in filelist {
                print(filename)
                theFileList.append("\(filename)")
            }
        } catch let error {
            print("Error: \(error.localizedDescription)")
        }
 
        // Register the table view cell class and its reuse id
        self.tableView.register(UITableViewCell.self, forCellReuseIdentifier: cellReuseIdentifier)

        // This view controller itself will provide the delegate methods and row data for the table view.
        tableView.delegate = self
        tableView.dataSource = self
        
        
    }
    
    
    
    
    
    override func viewDidAppear(_ animated: Bool) { //refresh everything
        super.viewDidAppear(false)
       
        // now clear the table
        textFileContents.text = ""
        
        theFileList.removeAll()
        tableView.reloadData()
        
        
        let filemgr = FileManager.default
        let dirPaths = filemgr.urls(for: .documentDirectory, in: .userDomainMask)
        let docsDir = dirPaths[0].path

        do {
            //let filelist = try filemgr.contentsOfDirectory(atPath: "/")
            let filelistR = try filemgr.contentsOfDirectory(atPath: docsDir)
            let filelist = filelistR.sorted()
            for filename in filelist {
                print(filename)
                theFileList.append("\(filename)")
            }
        } catch let error {
            print("Error: \(error.localizedDescription)")
        }
        tableView.reloadData()
    
    }

    // number of rows in table view
    func tableView(_ tableView: UITableView, numberOfRowsInSection section: Int) -> Int {
        //return self.animals.count
        return self.theFileList.count
    }
    
    // create a cell for each table view row
    func tableView(_ tableView: UITableView, cellForRowAt indexPath: IndexPath) -> UITableViewCell {
        
        // create a new cell if needed or reuse an old one
        let cell:UITableViewCell = (self.tableView.dequeueReusableCell(withIdentifier: cellReuseIdentifier) as UITableViewCell?)!
        
        // set the text from the data model
        //cell.textLabel?.text = self.animals[indexPath.row]
        cell.textLabel?.text = self.theFileList[indexPath.row]
        
        return cell
    }
    
    // Set the spacing between sections
    func tableView(_ tableView: UITableView, heightForHeaderInSection section: Int) -> CGFloat {
        return cellSpacingHeight
    }
    
    // method to run when table view cell is tapped
    func tableView(_ tableView: UITableView, didSelectRowAt indexPath: IndexPath) {
        print("You tapped cell number \(indexPath.row).")
        theSelected = theFileList[indexPath.row]
        readSelectedFile()
        
    }
    
    
    
    @IBAction func buttonShare(_ sender: Any) {
        
        if(theSelected != "") {
            if MFMailComposeViewController.canSendMail() {
               
               let mail = MFMailComposeViewController()
               mail.setToRecipients(["mikebrinker@cox.net"])
               mail.setSubject("RocketTalk File")
               mail.setMessageBody("Attached is a file", isHTML: true)
               mail.mailComposeDelegate = self
               //add attachment
                
                let directoryURL = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
                //let url = URL(fileURLWithPath: "71-1019-gps.txt", relativeTo: directoryURL)
                let url = URL(fileURLWithPath: theSelected!, relativeTo: directoryURL)
                
                
                do {
                let attachmentData = try Data(contentsOf: url)
                    mail.addAttachmentData(attachmentData, mimeType: "text/txt", fileName: theSelected!)

                } catch let error {
                    print("We have encountered error \(error.localizedDescription)")
                }
               present(mail, animated: true)
               }
            else {
               print("Email cannot be sent")
            }
        }

    }
    
    func mailComposeController(_ controller: MFMailComposeViewController, didFinishWith result: MFMailComposeResult, error: Error?) {
       if let _ = error {
          self.dismiss(animated: true, completion: nil)
       }
       switch result {
          case .cancelled:
          print("Cancelled")
          break
          case .sent:
          print("Mail sent successfully")
          break
          case .failed:
          print("Sending mail failed")
          break
          default:
          break
       }
       controller.dismiss(animated: true, completion: nil)
    }
    

    
    func readSelectedFile() {
        
        textFileContents.text = ""
        let directoryURL = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
       // let fileURL = URL(fileURLWithPath: "myFile", relativeTo: directoryURL)
        
        
       // let fileURL = URL(fileURLWithPath: theSelected!, relativeTo: directoryURL).appendingPathExtension("txt")
        let fileURL = URL(fileURLWithPath: theSelected!, relativeTo: directoryURL)
        
        do {
         // Get the saved data
         let savedData = try Data(contentsOf: fileURL)
         // Convert the data back into a string
         if let savedString = String(data: savedData, encoding: .utf8) {
            print(savedString)
            textFileContents.text += savedString
         }
        } catch {
         // Catch any errors
            textFileContents.text = "Error reading the file"
         print("Unable to read the file")
        }
        
        
    }

    
    
    
    @IBAction func butteonDeletePress(_ sender: Any) {  // delete all files in the Documents directory
        
        if(switchDelete.isOn) {
            
            let documentsUrl =  FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!

            do {
                let fileURLs = try FileManager.default.contentsOfDirectory(at: documentsUrl,
                                                                           includingPropertiesForKeys: nil,
                                                                           options: .skipsHiddenFiles)
                for fileURL in fileURLs {
                    if fileURL.pathExtension == "txt" {
                        try FileManager.default.removeItem(at: fileURL)
                    }
                }
            } catch  { print(error) }
        
        // now clear the table
        textFileContents.text = ""
        
        theFileList.removeAll()
        tableView.reloadData()
        
        
        let filemgr = FileManager.default
        let dirPaths = filemgr.urls(for: .documentDirectory, in: .userDomainMask)
        let docsDir = dirPaths[0].path

        do {
            //let filelist = try filemgr.contentsOfDirectory(atPath: "/")
            let filelist = try filemgr.contentsOfDirectory(atPath: docsDir)
            for filename in filelist {
                print(filename)
                theFileList.append("\(filename)")
            }
        } catch let error {
            print("Error: \(error.localizedDescription)")
        }
        tableView.reloadData()
        
        if(configData.mute == false) {
           self.playSound(theFile: "message")
        }
            switchDelete.isOn = false
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


