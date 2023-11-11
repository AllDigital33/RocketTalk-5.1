
//  Booster.swift
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




class BoosterView: UIViewController, CLLocationManagerDelegate {

    @IBOutlet weak var labelPhase: UILabel!
    @IBOutlet weak var labelLaunch: UILabel!
    @IBOutlet weak var labelApogee: UILabel!
    @IBOutlet weak var labelLanded: UILabel!
    @IBOutlet weak var labelDrogue: UILabel!
    @IBOutlet weak var labelMain: UILabel!
    @IBOutlet weak var labelStatus: UILabel!
    @IBOutlet weak var labelRadioTimeout: UILabel!
    @IBOutlet weak var labelGPSTimeout: UILabel!
    @IBOutlet weak var labelAltitude: UILabel!
    @IBOutlet weak var labelSpeed: UILabel!
    @IBOutlet weak var labelMaxAltitude: UILabel!
    @IBOutlet weak var labelMaxSpeed: UILabel!
    @IBOutlet weak var labelGPSAltitude: UILabel!
    @IBOutlet weak var labelDistance: UILabel!
    @IBOutlet weak var labelAzm: UILabel!
    @IBOutlet weak var labelBearing: UILabel!
    @IBOutlet weak var labelLatLong: UILabel!
    @IBOutlet weak var trackerMapView: MKMapView!
    @IBOutlet weak var labelNoInternet: UILabel!
    @IBOutlet weak var imageTrackerPointer: UIImageView!
    @IBOutlet weak var labelBattery: UILabel!
    @IBOutlet weak var labelTemp: UILabel!
    @IBOutlet weak var continuityDrogue: UILabel!
    @IBOutlet weak var continuityMain: UILabel!
    @IBOutlet weak var labelMaxGPSAltitude: UILabel!
    @IBOutlet weak var labelCPUtemp: UILabel!
    @IBOutlet weak var labelAccel: UILabel!
    @IBOutlet weak var labelAccelMax: UILabel!

    @IBOutlet weak var labelSats: UILabel!
    
    var locationManager: CLLocationManager = CLLocationManager()
    var startLocation: CLLocation!
    var timerTwo: Timer?
    var timerOne: Timer?
    var trackReady: Bool?
    var trackLat: Float?
    var trackLong: Float?
    var trackMode: Int?
    
    override func viewDidLoad() {
        super.viewDidLoad()
        checkInternet()
        labelNoInternet.isHidden = true
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
        
        labelPhase.text = "STARTUP"
        continuityDrogue.backgroundColor = .white
        continuityDrogue.text = ""
        continuityMain.backgroundColor = .white
        continuityMain.text = ""
        labelAltitude.text = "n/a"
        labelSpeed.text = "n/a"
        labelMaxAltitude.text = "n/a"
        labelMaxSpeed.text = "n/a"
        labelBattery.text = "n/a"
        labelTemp.text = "n/a"
        labelDistance.text = "n/a"
        labelAzm.text = "n/a"
        labelSats.text = "0"
        
        labelLatLong.text = "n/a"
        labelBearing.text = "n/a"
        labelGPSAltitude.text = "n/a"
        labelMaxGPSAltitude.text = "n/a"
        
        //housekeeping timer
        timerOne = Timer.scheduledTimer(timeInterval: 0.5, target: self, selector: #selector(BoosterView.boosterTimer), userInfo: nil, repeats: true)
        timerTwo = Timer.scheduledTimer(timeInterval: 3.0, target: self, selector: #selector(BoosterView.boosterTimer2), userInfo: nil, repeats: true)
        print("Booster View Did Load")

    }
    
    override func viewDidAppear(_ animated: Bool) { //refresh everything
        super.viewDidAppear(false)
       
    }

    @objc func boosterTimer() { //fast timer for data refresh
      
        
        if(boosterGPS.newData!) {
            boosterGPS.newData = false
            updateBoosterGPS()
        }
        if(boosterWorkingData.newData!) {
            updateBoosterFlightData()
        }
        // clear status
         if(workingData.statusTimeout! < Date() && labelStatus.text != "") {
             labelStatus.text = ""
         }
         //update status
         if(workingData.statusTimeout! > Date()) {
             labelStatus.text = workingData.status
         } else {
             labelStatus.text = ""
         }
 
    }

    @objc func boosterTimer2() { // slow timer for location refresh
      
        if(workingData.internetUp == false) {
            labelNoInternet.isHidden = false
            checkInternet()
        } else {
            labelNoInternet.isHidden = true
        }
        if(boosterData.timeout! < Date()) {
            labelRadioTimeout.isHidden = false
        } else {
            labelRadioTimeout.isHidden = true
        }
        if(boosterGPS.timeout! < Date()) {
            labelGPSTimeout.isHidden = false
        } else {
            labelGPSTimeout.isHidden = true
        }
 
    }
    func updateBoosterFlightData() {
        boosterWorkingData.newData = false
        labelRadioTimeout.isHidden = true

        labelPhase.text = boosterData.phase
        
        if(boosterData.eventLaunch!) {
            labelLaunch.backgroundColor = .green
        } else {
            labelLaunch.backgroundColor = .white
        }
        if(boosterData.eventApogee!) {
            labelApogee.backgroundColor = .green
        } else {
            labelApogee.backgroundColor = .white
        }
        if(boosterData.eventLanded!) {
            labelLanded.backgroundColor = .green
        } else {
            labelLanded.backgroundColor = .white
        }
        if(boosterData.continuityDrogue!) {
            continuityDrogue.backgroundColor = .green
            continuityDrogue.text = "Y"
        } else {
            continuityDrogue.backgroundColor = .red
            continuityDrogue.text = "X"
        }
        if(boosterData.continuityMain!) {
            continuityMain.backgroundColor = .green
            continuityMain.text = "Y"
        } else {
            continuityMain.backgroundColor = .red
            continuityMain.text = "X"
        }
        if(boosterData.pyroDrogue!) {
            labelDrogue.backgroundColor = .green
        } else {
            labelDrogue.backgroundColor = .lightGray
        }
        if(boosterData.pyroMain!) {
            labelMain.backgroundColor = .green
        } else {
            labelMain.backgroundColor = .lightGray
        }
        
        let numberFormatter = NumberFormatter()
        numberFormatter.numberStyle = .decimal
        //let formattedNumber = numberFormatter.string(from: NSNumber(value:largeNumber))
        
        labelAltitude.text = numberFormatter.string(from: NSNumber(value:boosterData.altitude!))! + "'"
        labelSpeed.text = "\(String(boosterData.speed!))" + " fps"
        labelMaxAltitude.text = "\(String(boosterData.altitudeMax!))" + "'"
        labelMaxSpeed.text = "\(String(boosterData.speedMax!))" + " fps"
        labelBattery.text = "\(String(boosterData.battery!))" + "%"
        labelTemp.text = "\(String(boosterData.temp!))" + "°"
        labelCPUtemp.text = "\(String(boosterData.temp2!))" + "°"
        labelAccel.text =  "\(String(format: "%.0f",boosterData.accelZ!))" + "G"
        labelAccelMax.text =  "\(String(format: "%.0f",boosterData.maxAccel!))" + "G"

        
    }
    
    func updateBoosterGPS() {
        
        if(boosterGPS.goodData!) {
            imageTrackerPointer.image = UIImage(named: "Arrow")!
            
            let formatter = DateFormatter()
            formatter.locale = Locale(identifier: "en_US")
            formatter.setLocalizedDateFormatFromTemplate("HH:mm:ss")
            trackLat = boosterGPS.lat
            trackLong = boosterGPS.long
            trackReady = true
            refreshGPS2()
        }
        
    }
    func refreshGPS2() {
        trackerMapView.removeAnnotations(trackerMapView.annotations)
        let coords = CLLocationCoordinate2DMake(Double(trackLat!), Double(trackLong!))
        let place = MKPlacemark(coordinate: coords, addressDictionary: nil)
        trackerMapView.addAnnotation(place)
        labelLatLong.text = "\(String(format: "%.5f", trackLat!))"+", "+"\(String(format: "%.5f", trackLong!))"
        updateDistanceAngle ()
        //re-center the map

        let currentLocation = CLLocation(latitude: Double((baseData.lat)!), longitude: Double((baseData.long)!))
        trackerMapView.centerToLocation(currentLocation)
        print("Update location on map")
        let theBearing: Float = getBearing(rLat: trackLat!, rLong: trackLong!)
        baseData.boosterBearing = theBearing
        labelBearing.text = "\(String(format: "%.0f", theBearing))" + "°"
        // update altitude
        labelGPSAltitude.text = "\(String(boosterGPS.altitude!))" + "'"
        labelMaxGPSAltitude.text = "\(String(boosterGPS.maxAGL!))" + "'"
        labelSats.text = "\(String(boosterGPS.sat!))"
    }
    
    
    func updateDistanceAngle () {
        
            //Distance
        if(trackReady!) {
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
            theAngle = atan(Float(boosterData.altitudeMeters!)/theDistance) * 180.0 / Float.pi
            labelAzm.text = "\(String(format: "%.0f", theAngle))" + "°"
            print("Angle: ")
            print(theAngle)
        }

    }
    
    func locationManager(_ manager: CLLocationManager, didUpdateHeading newHeading: CLHeading) {
        
            // Update big pointer
        if(trackReady!) {
            baseData.heading = Float(newHeading.trueHeading) + 90.00 //adjust for landscape mode
            if (baseData.heading! > 360.0) {
                baseData.heading = baseData.heading! - 360.0
            }

            UIView.animate(withDuration: 0.0, animations: {
                var angle: CGFloat = 0.0
                if (baseData.heading! < baseData.boosterBearing!) {
                    angle = degreesToRadians(CGFloat(baseData.boosterBearing! - baseData.heading!))
                    print(baseData.boosterBearing! - baseData.heading!)
                } else {
                    angle = degreesToRadians(CGFloat(360.0 - (baseData.heading! - baseData.boosterBearing!)))
                    print(360.0 - (baseData.heading! + baseData.boosterBearing!))
                }
                self.imageTrackerPointer.transform = CGAffineTransform(rotationAngle: angle)
            })
        }
        
    }
    func locationManager(_ manager: CLLocationManager, didUpdateLocations locations: [CLLocation]) {

        if(trackReady!) {
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
                updateBoosterGPS()
            }
            updateDistanceAngle()
            //updateBearing()
        
          }
        }
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

} // end view
