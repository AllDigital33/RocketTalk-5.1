//
//  TrackerView.swift
//

import AVFoundation
import CoreLocation
import Foundation
import UIKit
import MapKit

private extension MKMapView {
    func centerToLocation(
        _ location: CLLocation,
        regionRadius: CLLocationDistance = ((Double(baseData.distance!) * 3))
        ) {
        let coordinateRegion = MKCoordinateRegion(
            center: location.coordinate,
            latitudinalMeters: regionRadius,
            longitudinalMeters: regionRadius)
        setRegion(coordinateRegion, animated: true)
        print("set region")
        print((Double(baseData.distance!) + 1000.0))
    }
}

var trackMapOn: Bool = false

let degreesToRadians: (CGFloat) -> CGFloat = {
    return $0 / 180.0 * CGFloat(Double.pi)
}



class TrackerView: UIViewController, CLLocationManagerDelegate, UITableViewDelegate, UITableViewDataSource, MKMapViewDelegate  {
    
    
    @IBOutlet weak var trackerSegmentControl: UISegmentedControl!
    @IBOutlet weak var buttonGoTracker: UIButton!
    @IBOutlet weak var textLong: UITextField!
    @IBOutlet weak var textLat: UITextField!
    @IBOutlet weak var labelManualLat: UILabel!
    @IBOutlet weak var labelManualLong: UILabel!
    @IBOutlet weak var trackerMapView: MKMapView!
    @IBOutlet weak var labelBaseGPS: UILabel!
    @IBOutlet weak var labelHorz: UILabel!
    @IBOutlet weak var labelVert: UILabel!
    @IBOutlet weak var labelAzm: UILabel!
    @IBOutlet weak var labelCompass: UILabel!
    @IBOutlet weak var imageTrackerPointer: UIImageView!
    @IBOutlet weak var labelRocketGPS: UILabel!
    @IBOutlet weak var labelAltitude: UILabel!
    @IBOutlet weak var labelDistance: UILabel!
    @IBOutlet weak var switchCenter: UISwitch!
    @IBOutlet weak var labelBaseHeading: UILabel!
    @IBOutlet weak var labelLastGPS: UILabel!
    @IBOutlet weak var labelLastGPSTime: UILabel!
    @IBOutlet weak var labelNoInternet: UILabel!
    @IBOutlet weak var tableView: UITableView!
    
    var locationManager: CLLocationManager = CLLocationManager()
    var startLocation: CLLocation!
    var timerTwo: Timer?
    var trackReady: Bool?
    var trackLat: Float?
    var trackLong: Float?
    var firstLat: Float?
    var firstLong: Float?
    var trackMode: Int?
    
    let cellReuseIdentifier = "cell"
    let cellSpacingHeight: CGFloat = 1
    var theFileList = [String]()
    var theSelected: String? = ""
    // for file reading
    var tempFileLat: Float? = 0.0
    var tempFileLong: Float? = 0.0
    var coordinates: [CLLocationCoordinate2D] = []
    var fileCount: Int? = 0
    var bigMap: Bool? = false

      override func viewDidLoad() {
        
        super.viewDidLoad()
        //tempLoad()
        //format UI
        checkInternet()
        labelNoInternet.isHidden = true
        labelManualLat.isHidden = true
        labelManualLong.isHidden = true
        textLat.isHidden = true
        textLong.isHidden = true
        tableView.isHidden = true
        imageTrackerPointer.image = UIImage(named: "arrowoff")!
        
        //location and map stuff
        locationManager = CLLocationManager()
        locationManager.delegate = self
        locationManager.requestWhenInUseAuthorization()
        trackerMapView.isZoomEnabled = true
        trackerMapView.isScrollEnabled = true
        trackerMapView.mapType = MKMapType.hybrid
        locationManager.desiredAccuracy = kCLLocationAccuracyBest
        locationManager.distanceFilter = kCLDistanceFilterNone
        locationManager.startUpdatingLocation()
        locationManager.startUpdatingHeading()
        trackerMapView.showsUserLocation = true
        trackMode = 0
        trackReady = false
        
        //housekeeping timer
        timerTwo = Timer.scheduledTimer(timeInterval: 3.0, target: self, selector: #selector(TrackerView.trackerTimer), userInfo: nil, repeats: true)
        
        //file list loading here
          tableView.layer.borderWidth = 2
          tableView.layer.borderColor = UIColor.black.cgColor
          
          
          let filemgr = FileManager.default
          let dirPaths = filemgr.urls(for: .documentDirectory, in: .userDomainMask)
          let docsDir = dirPaths[0].path
          var theCount: Int? = 0

          do {
              //let filelist = try filemgr.contentsOfDirectory(atPath: "/")
              let filelistR = try filemgr.contentsOfDirectory(atPath: docsDir)
              let filelist = filelistR.sorted()
              for filename in filelist {
                  print(filename)
                  let theTest = filename.contains("gps")
                  if(theTest) {
                      theFileList.append("\(filename)")
                      theCount! += 1
                  }
              }
              if(theCount == 0) {theFileList.append("NO GPS FILES!")}
          } catch let error {
              print("Error: \(error.localizedDescription)")
          }
          
   
          // Register the table view cell class and its reuse id
          self.tableView.register(UITableViewCell.self, forCellReuseIdentifier: cellReuseIdentifier)

          // This view controller itself will provide the delegate methods and row data for the table view.
          tableView.delegate = self
          tableView.dataSource = self
          
          trackerMapView.delegate = self
          
          let tap = UITapGestureRecognizer(target: self, action: #selector(doubleTapped))
          tap.numberOfTapsRequired = 2
          view.addGestureRecognizer(tap)
      
          
        print("Tracker View Did Load")

    }
    
    override func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(false)
        
        print("View did appear again")
    }
    
    @objc func doubleTapped() {
        if(bigMap!) {
            bigMap = false
            UIView.animate(withDuration: 2, animations: {
                self.trackerMapView.frame = CGRect(x: 2, y: 160, width: 560, height: 557)
                self.trackerMapView.layoutIfNeeded()
            })

        } else {
            bigMap = true
            UIView.animate(withDuration: 2, animations: {
                self.trackerMapView.frame = CGRect(x: 2, y: 160, width: 1018, height: 557)
                self.trackerMapView.layoutIfNeeded()
            })
        }
    }
    
    @objc func trackerTimer() {
        
        if(rocketGPS.goodData! && trackerSegmentControl.selectedSegmentIndex == 0) {
            updateGPS()
        }
        if(workingData.internetUp == false) {
            labelNoInternet.isHidden = false
            checkInternet()
        } else {
            labelNoInternet.isHidden = true
        }
    }
 
    @IBAction func buttonGoNow(_ sender: Any) {
        
        if(trackMode == 2) {
            let latTest: Float = Float(String(textLat.text!))!
            let longTest: Float = Float(String(textLong.text!))!
            var testGood: Bool = true
            if (latTest < 10.00 || latTest > 80.00) {
                testGood = false
                textLat.text! += " Error"
            }
            if (longTest < -170.00 || longTest > -30.00) {
                testGood = false
                textLong.text! += " Error"
            }
            if (testGood) {
                trackLat = latTest
                trackLong = longTest
                trackReady = true
                updateGPS()
            } else {
                trackReady = false
            }
        }
        if(trackMode == 1 && trackReady == true) {
                updateGPS()
        }

    }
    
    func refreshUI() {
     
        
        labelBaseGPS.text = "\(String(format: "%.5f", baseData.lat!))"+", "+"\(String(format: "%.5f", baseData.long!))"
        labelHorz.text = "\(String(format: "%.0f",baseData.horizontalAccuracy!))"
        labelVert.text = "\(String(format: "%.0f",baseData.verticalAccuracy!))"
        labelAltitude.text = "\(String(flightData.altitude!))" + "'"
    }
    
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {

        locations.forEach { (location) in
            //update base globals
            baseData.horizontalAccuracy = location.horizontalAccuracy
            baseData.verticalAccuracy = location.verticalAccuracy
            baseData.lat = Float(location.coordinate.latitude)
            baseData.long = Float(location.coordinate.longitude)
            baseData.altitude = Int32(location.altitude)
            //check map
            if(trackMapOn == false) {
                trackMapOn = true
                let initialLocation = CLLocation(latitude: Double((baseData.lat)!), longitude: Double((baseData.long)!))
                trackerMapView.centerToLocation(initialLocation)
                updateGPS()
            }
            refreshUI()
            updateDistanceAngle()
            //updateBearing()
        
        }
    }
    
    func updateDistanceAngle () {
        
        if(trackReady!) {
            //Distance
            var theDistance: Float
            theDistance = getGPSDistance(rLat: trackLat!, rLong: trackLong!)
            baseData.distance = theDistance
            print(theDistance)
            var labelTemp: String
            if (theDistance > 999) {
                labelTemp = "\(String(format: "%.2f", theDistance/1000))" + " Km"
            } else {
                labelTemp = "\(String(format: "%.0f", theDistance))" + " M"
            }
            if (theDistance < 15) { // the red zone
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
            labelAzm.text = "\(String(format: "%.0f", theAngle))" + "°"
            print("Angle: ")
            print(theAngle)
        } else {
            labelDistance.text = "0"
            labelAzm.text = "0"
            labelCompass.text = "0"
            labelRocketGPS.text = "N/A"
        }
        

    }
    
    func locationManager(_ manager: CLLocationManager, didUpdateHeading newHeading: CLHeading) {
        
        if(trackReady == true) {
            // Update big pointer
            baseData.heading = Float(newHeading.trueHeading) + 90.00 //adjust for landscape mode
            if (baseData.heading! > 360.0) {
                baseData.heading = baseData.heading! - 360.0
            }
            labelBaseHeading.text = "\(String(format: "%.0f", baseData.heading!))" + "°"
            
            UIView.animate(withDuration: 0.0, animations: {
                var angle: CGFloat = 0.0
                if (baseData.heading! < baseData.rocketBearing!) {
                    angle = degreesToRadians(CGFloat(baseData.rocketBearing! - baseData.heading!))
                    print(baseData.rocketBearing! - baseData.heading!)
                } else {
                    angle = degreesToRadians(CGFloat(360.0 - (baseData.heading! - baseData.rocketBearing!)))
                    print(360.0 - (baseData.heading! + baseData.rocketBearing!))
                }
                self.imageTrackerPointer.transform = CGAffineTransform(rotationAngle: angle)
            })
        }
        
        
        
    }
    
    func updateGPS() {
        
        if(rocketGPS.goodData == true && trackMode == 0) {
            imageTrackerPointer.image = UIImage(named: "Arrow")!
            print("Updating GPS")
            let formatter = DateFormatter()
            formatter.locale = Locale(identifier: "en_US")
            formatter.setLocalizedDateFormatFromTemplate("HH:mm:ss")
            let datetime = formatter.string(from: Date())
            labelLastGPSTime.text = datetime
            trackLat = rocketGPS.lat
            trackLong = rocketGPS.long
            trackReady = true
            refreshGPS()
        }
        if(trackMode == 1 && trackReady == true) {
            imageTrackerPointer.image = UIImage(named: "Arrow")!
            refreshGPS()
            
        }
        if(trackMode == 2 && trackReady == true) {
            imageTrackerPointer.image = UIImage(named: "Arrow")!
            refreshGPS()
            
        }
        if(trackMode == 3 && trackReady == true) {
            imageTrackerPointer.image = UIImage(named: "Arrow")!
            refreshGPS()
            
        }
    }
    func refreshGPS() {
        
        trackerMapView.removeAnnotations(trackerMapView.annotations)
        let coords = CLLocationCoordinate2DMake(Double(trackLat!), Double(trackLong!))
        
        let annotation = MKPointAnnotation()
        annotation.coordinate = coords
        annotation.title = "rocket"
        trackerMapView.addAnnotation(annotation)
    
        //let place = MKPlacemark(coordinate: coords, addressDictionary: nil)
        //trackerMapView.addAnnotation(place)

        labelRocketGPS.text = "\(String(format: "%.5f", trackLat!))"+", "+"\(String(format: "%.5f", trackLong!))"
        updateDistanceAngle ()
        //re-center the map
        if(switchCenter.isOn) {
            let currentLocation = CLLocation(latitude: Double((baseData.lat)!), longitude: Double((baseData.long)!))
            trackerMapView.centerToLocation(currentLocation)
            print("Update location on map")
        }
        let theBearing: Float = getBearing(rLat: trackLat!, rLong: trackLong!)
        baseData.rocketBearing = theBearing
        labelCompass.text = "\(String(format: "%.0f", theBearing))" + "°"
        // update altitude
        labelAltitude.text = "\(String(flightData.altitude!))" + "'"
        
    }
    
    
    
    
    @IBAction func sliderSwitched(_ sender: Any) {
        
        
        if(trackerSegmentControl.selectedSegmentIndex == 0) {
            imageTrackerPointer.image = UIImage(named: "arrowoff")!
            switchCenter.isOn = true
            trackMode = 0
            labelLastGPS.isHidden = false
            labelLastGPSTime.isHidden = false
            labelLastGPS.text = "Last GPS:"
            textLat.isHidden = true
            textLong.isHidden = true
            labelManualLat.isHidden = true
            labelManualLong.isHidden = true
            trackReady = false
            tableView.isHidden = true
            trackerMapView.removeAnnotations(trackerMapView.annotations)
            self.trackerMapView.removeOverlays(self.trackerMapView.overlays)
            updateGPS()
        }
        if(trackerSegmentControl.selectedSegmentIndex == 1) {  //last file (must hit go)
            imageTrackerPointer.image = UIImage(named: "arrowoff")!
            trackMode = 1
            labelLastGPS.isHidden = false
            labelLastGPSTime.isHidden = false
            labelLastGPS.text = "File GPS:"
            textLat.isHidden = true
            textLong.isHidden = true
            labelManualLat.isHidden = true
            labelManualLong.isHidden = true
            trackReady = false
            tableView.isHidden = true
            trackerMapView.removeAnnotations(trackerMapView.annotations)
            self.trackerMapView.removeOverlays(self.trackerMapView.overlays)
            getLastGPSfile()
          
        }
        
        if(trackerSegmentControl.selectedSegmentIndex == 2) {  //manual (must hit go)
            trackMode = 2
            imageTrackerPointer.image = UIImage(named: "arrowoff")!
            labelLastGPS.isHidden = true
            labelLastGPSTime.isHidden = true
            textLat.isHidden = false
            textLong.isHidden = false
            labelManualLat.isHidden = false
            labelManualLong.isHidden = false
            trackReady = false
            tableView.isHidden = true
            trackerMapView.removeAnnotations(trackerMapView.annotations)
            self.trackerMapView.removeOverlays(self.trackerMapView.overlays)
       
        }
        if(trackerSegmentControl.selectedSegmentIndex == 3) {  //from a gps file
            trackMode = 3
            switchCenter.isOn = false
            imageTrackerPointer.image = UIImage(named: "arrowoff")!
            labelLastGPS.isHidden = true
            labelLastGPSTime.isHidden = true
            textLat.isHidden = true
            textLong.isHidden = true
            labelManualLat.isHidden = true
            labelManualLong.isHidden = true
            trackReady = false
            tableView.isHidden = false
       
        }
        
        
        

    }
    
    //================================  READ FROM FILE and DRAW LINES ============================================
    
    func getLastGPSfile() {
              
        let directoryURL = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
        let fileURL = URL(fileURLWithPath: "lastGPS", relativeTo: directoryURL)
        
        do {
         // Get the saved data
         let savedData = try Data(contentsOf: fileURL)
         // Convert the data back into a string
         if let savedString = String(data: savedData, encoding: .utf8) {
            print(savedString)
            let tempArray = savedString.components(separatedBy: ",")
            
            let latTest: Float = Float(String(tempArray[0]))!
            let longTest: Float = Float(String(tempArray[1]))!
            var testGood: Bool = true
            if (latTest < 10.00 || latTest > 80.00) {
                testGood = false
                labelLastGPSTime.text = "File Data Error"
            }
            if (longTest < -170.00 || longTest > -30.00) {
                testGood = false
                labelLastGPSTime.text = "File Data Error"
            }
            if (testGood) {
                trackLat = latTest
                trackLong = longTest
                trackReady = true
                labelLastGPSTime.text = "\(String(format: "%.5f", trackLat!))" + ", " + "\(String(format: "%.5f", trackLong!))"
            } else {
                trackReady = false
            }
         }
        } catch {
         // Catch any errors
            labelLastGPSTime.text = "Unable to read file"
         print("Unable to read the file")
        }

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
    
    func readSelectedFile() {
        
        let directoryURL = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask)[0]
        let fileURL = URL(fileURLWithPath: theSelected!, relativeTo: directoryURL)
        coordinates.removeAll()
        fileCount = 0

        do {
            try String(contentsOf: fileURL, encoding: .utf8)
                .split(separator: "\n")
                .forEach { line in
                    let n = Int(line)
                    if n != nil {
                        //number = n!
                    } else if(line.contains("@G")){
                       
                        let tempArray = line.components(separatedBy: ",")
                        let latTest: Double = Double(String(tempArray[4])) ?? 0
                        let longTest: Double = Double(String(tempArray[5])) ?? 0
                        //change this test for other regions of the world
                        var testGood: Bool = true
                        if (latTest < 10.00 || latTest > 80.00) {
                            testGood = false
                        }
                        if (longTest < -170.00 || longTest > -30.00) {
                            testGood = false
                        }
                        if (testGood) {
                            tempFileLat = Float(latTest)
                            tempFileLong = Float(longTest)
                            coordinates.append(CLLocationCoordinate2D(latitude: latTest, longitude: longTest))
                            if(fileCount == 0) { // save for pin drop
                                firstLat = tempFileLat
                                firstLong = tempFileLong
                            }
                            fileCount! += 1
                            //labelLastGPSTime.text = "\(String(format: "%.5f", trackLat!))" + ", " + "\(String(format: "%.5f", trackLong!))"
                        } // end good line
                    } // end @G line
                } // end for each line
        } catch let error as NSError {
            print("An error took place: \(error)")
        }
        
        if(fileCount ?? 0 > 0) {

            trackLat = tempFileLat
            trackLong = tempFileLong
            trackReady = true
            // done reading them all
            print("File Count is \(fileCount!)")
            print("Last lat is \(trackLat!)")
            print("Last long is \(trackLong!)")
            mapTrack()
            //expand the map
            bigMap = true
            UIView.animate(withDuration: 2, animations: {
                self.trackerMapView.frame = CGRect(x: 2, y: 160, width: 1018, height: 557)
                self.trackerMapView.layoutIfNeeded()
            })
            updateGPS()
        } else {
            //trackReady = false
            print("No lines were processed")
        }
    }
    
    func mapTrack() {
        
        trackerMapView.removeAnnotations(trackerMapView.annotations)
        self.trackerMapView.removeOverlays(self.trackerMapView.overlays)
        
        tableView.isHidden = true
        trackerMapView.addOverlay(MKPolyline(coordinates: coordinates, count: fileCount!))
        
        let coords1 = CLLocationCoordinate2DMake(Double(firstLat!), Double(firstLong!))
        let annotation1 = MKPointAnnotation()
        annotation1.coordinate = coords1
        annotation1.title = "pad"
        trackerMapView.addAnnotation(annotation1)
        
        let coords = CLLocationCoordinate2DMake(Double(trackLat!), Double(trackLong!))
        let annotation = MKPointAnnotation()
        annotation.coordinate = coords
        annotation.title = "rocket"
        trackerMapView.addAnnotation(annotation)
        
        
        
        
        
    }
    
    func mapView(_ mapView: MKMapView, rendererFor overlay: MKOverlay) -> MKOverlayRenderer {
      let polylineRenderer = MKPolylineRenderer(polyline: overlay as! MKPolyline)
      polylineRenderer.strokeColor = UIColor.yellow
      polylineRenderer.lineWidth = 4.0
      return polylineRenderer
    }

        
    func mapView(_ mapView: MKMapView, viewFor annotation: MKAnnotation) -> MKAnnotationView? {
        let annotationView = MKMarkerAnnotationView(annotation: annotation, reuseIdentifier: "MyMarker")
        //annotationView.markerTintColor = UIColor.blue
        
        annotationView.displayPriority = .defaultHigh
        annotationView.displayPriority = .required
        

        switch annotation.title!! {
            case "rocket":
                annotationView.markerTintColor = UIColor(red: (255/255), green: (0/255), blue: (0/255), alpha: 1.0)
                annotationView.glyphImage = UIImage(named: "rocketiconsmall")
            case "pad":
                annotationView.markerTintColor = UIColor(red: (0/255), green: (255/255), blue: (0/255), alpha: 1.0)
                annotationView.glyphImage = UIImage(named: "greenlaunchpin")
            case "me":
                annotationView.markerTintColor = UIColor(red: (0/255), green: (0/255), blue: (255/255), alpha: 1.0)
                annotationView.glyphImage = UIImage(named: "me")
            default:
                annotationView.markerTintColor = UIColor.blue
                annotationView.glyphImage = UIImage(named: "me")
        }
        
        
        return annotationView
    }
 

    
    func checkInternet() {
        if let url = URL(string: "https://apple.com") {
          var request = URLRequest(url: url)
          request.httpMethod = "HEAD"
          URLSession(configuration: .default)
            .dataTask(with: request) { (_, response, error) -> Void in
              guard error == nil else {
                print("Error:", error ?? "")
                workingData.internetUp = false
                return
              }
              guard (response as? HTTPURLResponse)?
                .statusCode == 200 else {
                  print("internet down")
                  workingData.internetUp = false
                  return
              }
              print("internet up")
                workingData.internetUp = true
                return
            }
            .resume()
          } else {
            workingData.internetUp = false
        }
    }
    
}


