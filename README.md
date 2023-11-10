# Rocket Talk v5.1

This is a full-featured rocket flight computer for amateur and experimental rocketry that incorporates radio telemetry, GPS tracking, staging, pyro event control, FTS, camera control, and more.

Please read the Overview.PDF file attached for a detailed overview of the hardware and software solution.

There is also a quick video overview here:  https://vimeo.com/489179754/c7f04c6ab9

The code in the repository is split between the vehicle code (Arduino / Teensy) and the base station code (iPad / iOS Swift). 

# Disclaimer

We started this radio telemetry project in 2016 to better track our rockets in the skies over the Mojave desert. It has gone through a dozen hardware iterations and over fifty test flights. We always intended to share it and open source it, but projects like these are never complete. There is lots of room for improvement.

We’ve decided to take a snapshot and share our progress, so student groups and other amateurs can learn, borrow, or reuse some of our ideas and our sloppy code.

We owe a lot to the rocketry community, the TRF board, and student groups out at FAR that have shared their ideas, code, and practices to make our project and launches better and safer. 

This was never meant to be or become a commercial project, so please don't ask. The components and approach are way more expensive than any other trackers on the market, so you are not saving money by building your own. The EggTimer, TeleMega, and many more solutions are reliable and battle tested solutions.

This is a passion project to create bi-directional telemetry, long-range tracking, and an easy to use iPad based field solution.  


# Summary

**VEHICLE BOARD:**
* Teensy 4.1 MCU (ARM M7)
* GPS
* Barometer
* Two Accelerometers
* Gyroscope
* Pyro relays x 4
* Camera power control
* 64Mb of flash storage + SD transfer
* Temperature monitoring
* Voltage monitoring
* Bi-directional radio comms / 1W LoRa 433 Mhz 

**IPAD HANDHELD BASE STATION:**
* Simple touch-based UI for data and tracking
* Voice (siri like) event and alert reporting
* Local base GPS 
* Magnetometer / bearing
* Satellite mapping (Apple Maps)
* Local logging and file storage 
* Wifi/Internet
* Bi-directional radio comms / 1W LoRa 433 Mhz


# Vehicle Board Current Technology Choices

* **Teensy 4.1:**  The T4.1 with its Arm Cortex-M7 is just insanely fast for an MCU and it is loaded with a lot of versatile GPIO. We started our project on other Arduino based MCUs. Our last one was the ESP32. In benchmarks, an ESP32 can increment an integer about 700K times a second. That seems fast until you see the T4.1 (or 4.0) increment an integer 242 million times a second (wowzers!). This provides for almost unlimited CPU cycles for anything you can dream up during flight. The real limitation becomes the I2C bus, communication speeds, serial speeds, etc.  
* **Flash Memory:** The T4.1 has a built-in SD card that we used for years for logging data, but ultimately we had too many SD errors during flight that crashed the logging thread, so we switched to a 64 Mb flash chip (W25Q512JV) or 256mb (W25N02KVZEIR). The flash chip solders to the Teensy 4.1 and we're using the native LittleFS file system that uses the flash memory similar to a SD card. For easy transfer of data to SD there is a utility sketch that can be run after the flight. 
* **GPS:**  We are using the Ublox Max-M10S with serial communication. The M10S has proven to be an exceptional GPS, especially when combined with a Maxtena Helicore antenna. We run the GPS serial ttl instead of I2C, so we can save our I2C bus for sensor sampling. These GPS units are still under ITAR regulations, so they have limitations above 100K (more research needed).
* **GPS Antenna** The Maxtena Helicore M1575HCT-22P-SMA antenna is a passive GPS antenna capable of locking in 24+ satelites before launch.  
* **Barometer:**  We are using the TE MS5607 barometer on I2C sampling at 40Hz. We were using the BME280, but found it very susceptible to RF interference. The MS5607 has been reliable (no RFI), tracks up to 110K feet in the baro chamber, but it has a noisier signal, especially on descent. We also have to do some code gymnastics to sample at 40 hz and allow the sampling thread (and I2C bus) to stay free.
* **Accelerometers:** We have an Analog Devices ADXL375 200G accelerometer and a ST LSM6DSO32 32G accelerometer. The 200G is good to catch all the really big stuff and we often exceed the 32G’s of the LSM6DS, but the LSB resolution is not as good in a 200G setting, so the ST provides better resolution and accuracy below 32G’s. We are sampling the accelerometers and the gyro at 400 hz on the I2C bus.
* **Gyro:** The ST LSM6DSO32 also has a built-in gyroscope. We are using that, along with quaternions to calculate the rocket attitude (tilt).
* **Radios:**  We have tried a number of different radios, but we are very happy with our 1 watt LoRa radios on 433 Mhz (==FCC amateur radio license required==). You can get similar radios in 900 Mhz that don’t require a license. But, you are in this deep, so take the time to get a license. We use the LoRa 6100Pro serial TTL radios from NiceRF in China. The Lora technology is great, as we can create a mesh network and add a third radio on a tower to act as a repeater. These have proven to be reliable up to 15 miles “peak to peak” on the ground (mountain test) and I’m certain they will go to 100K+ feet in the air with the right antennas.
* **Downlink Antenna:**  The choice of downlink antennas on the vehicle is almost more important than the radio. We’ve tried dozens of antennas. The best antenna for us is a simple homemade dipole antenna running vertical in the airframe. We’ve tried whip, inverted V, fractal, and more, but the dipole performs the best. We used to make them by hand, but now we print our diploe antennas onto a long thin custom PCBs and trim to tune them (about $2/each)
* **5V Regulator:**  We use two L4941BDT-TR 5v 1A regulators to power the Teensy and the Radio. We run a separate regulator for the radio to minimize RFI and RF blowback, as well, provide the radio with a full one amp of power. All of the pyro events are triggered directly off of the Lipo battery and do not go through the regulators. 
* **Pyro Drivers:** We are using Rohm BV1HD090FJ-CE2 automotive switching drivers for all of our pyro events. These are rated at 5.5A and provide over-current protection and they have a convenient status pin that provides continuity check back to the MCU. There are other drivers that can switch up to 10A, but these have served us well for hundreds of pyro events using commercial MJG e-matches.
* **Terminals:**  We use JST board connectors for the low voltage connections and screw terminals for the hazard connections (power, switch, pyros). 


# Vehicle Board Code Overview

The Teensy code is built from a single C sketch in the Arduino (Teensyduino) IDE. It utilizes some external libraries for sensors, but is otherwise self-contained in a single code file. 

The code is partitioned into three Teensy threads, mimicking three separate infinite looping cores. The three threads share global variables, but are otherwise completely autonomous. Given the CPU speed, there is very little noticeable blocking between threads. Individual timers are used to trigger checks and activity within each loop.

**MAIN LOOP**
* Phase Logic (for each stage of flight)
* GPS check and transmit
* Transmit flight telemetry data
* Check voltage
* Check for separation events
* Check continuity
* Fire pyros and staging logic
* Radio receive processing 

**SAMPLE LOOP**
* Sample ADXL375 accelerometer at 400 Hz
* Sample LSM6DSO32 Accelerometer at 400 Hz
* Sample LSM6DSO32 Gyro at 400 Hz
* Compute Quaternion Tilt at 400 Hz
* Sample Barometer at 40 Hz

**LOGGING LOOP**
* Continuous logging to flash file system at various rates
* Logging flight and event data at 20 Hz
* Logging GPS data
* Logging radio TX/RX data
* Logging flight summary data


**PHASES:**

**STARTUP:** Power up sequence. Wait for GPS lock. Warm-up and characterize the Barometer. Bias adjust the Accelerometers.
**WAITING:**  Waiting for launch. Calibrate the Barometer. Bias adjust the Accelerometers.
**LAUNCH:**   Increase sample logging rate. Watch for and trigger staging lock-out and pyro events. Watch for Apogee and fire drogue pyro. 
**DESCENT:**  Reduce logging after 20 seconds. Watch for main altitude and fire main pyro. Watch for landing using barometer.
**LANDED:**   Log and transmit flight summary. Idle logging and transmitting.
**BASIC:**    Exception safe mode (activated by base) with no event triggering, just sampling, logging, and transmitting flight/gps data.

# iPad Base Station Technology Choices

* **iPad Mini:**  In earlier versions, we built the handheld base station from an Arduino and an LCD. This worked, but integrating the GPS and a magnetometer for tracking proved messy at best (field calibrating the magnetometer for an accurate bearing was not fun). So, we decided to switch over to an iPad mini, using Apple’s Swift development code to build the application and UI.  After a few days of “hello world” building, Swift was a dream compared to building hardware. Pulling base coordinates to compare to vehicle coordinates only required one line of code. Magnetic bearing was only one line of code. Voice event announcements only one line of code. Maps drop right in. Everything was just so much easier. That said, this does require a cellular iPad mini (no plan required), as those are the only iPads with actual GPS hardware. Also, we needed to use a proprietary lightening to RS232 serial cable (Redpark serial cable) to interface with the radio. And the radio needs its own little companion box with a battery. 
* **Redpark Serial Cable:**  This cable and the iOS Swift libraries is required to interface the iPad to the LoRa serial radio. It is RS232 serial, so a small RS232 to TTL converter is also used before connecting to the radio. There are not many ways to interface an Apple iOS device directly to serial communications. This one works great. A note:  You cannot distribute apps in the Apple App Store that use the RedPark cable. It can only be used with developer apps or enterprise app distribution.
* **Bluetooth Serial Option:**  We use the same iPad-Radio approach for other projects, including a liquid GSE project. In those projects we have switched away from using the Redpark cable and instead use a ESP32 as a "bluetooth bridge" between the iPad and the Serial LoRa radio. We have found it to be as good if not more reliable and allows for easier bench testing. Eventually, we will move the code over to this RocketTalk project to support that bridge.
* **Radios:**  We use the exact same radio as the vehicle radio. The LoRa 6100Pro serial TTL radios from NiceRF in China. We have tested these point-to-point at 14 miles on the ground.
* **Antenna:**  We are using an Arrow II handheld Yagi antenna (440-3) for good directional reception.
* **Additional mounting components:** To make a great one-handed tracker, we mount the iPad onto a UZOPI Mavic Mini Air Pro Platinum Spark Accessories Tablet Holder and we bolted the antenna and a camera pistol grip to the holder. The radio, battery, and cable are in a small project box attached to the holder plate.


# iPad Application Overview

The iPad application is written in xCode using Swift, Apple's native app development language. In the main storyboard file you will see it is entirely in a tab view controller with eight different tabs or views. The global variables and functions are in the AppDelegate file, but all other code is in the view controllers for each tab screen. The redPark serial communication callback for the radio is in the first "main" tab view, since it always appears first. The screen views are as follows:

* **Main:**  This is the primary screen for viewing flight data, event data, continuity status, GPS location/distance/bearing/azimuth, and a dozen other indicators. Flight data and GPS data is updated independently, based on what is received over the radio.
* **Tracking:**  This view has the same directional arrow, distance, bearing, azimuth, etc., but also includes an Apple Maps satellite location view and allows for tracking from a) live radio data 2) the local file log or 3) from a manual entry. Both Main and tracking use the iPad GPS data and compass to calculate distance and bearing. 
* **Send:** This view allows you to send commands to the rocket in flight to fire pyros, get additional status, force it into phases or modes, and FTS.
* **Booster:**  The application is setup, so you can track both a sustainer and booster using the same radio. Booster data (flight data and GPS) come in to the booster view, along with event (pyro/separation/continuity) data. The Booster uses a mini version of the vehicle board that still includes a barometer, accelerometer, gyro, GPS, and radio.
* **Summary:** The summary screen gets populated after a successful landing with additional data sent from the vehicle. The flight summary data includes speed, acceleration, flight time, distance, max tilt, and dozens of other flight measurements. 
* **Setup:**  This view allows you to configure alarms and alerts. The application provides voice alerts for things like high rocket descent speed, rocket descending directly overhead, high temperature, battery warnings, tilt warnings on launch, etc.
* **Files:**  This view allows you to manage and view base station log files for flight data, GPS data, radio data, and flight summary information. Log files can also be exported or emailed from this screen. 
* **Radio:**  This view provides a real-time monitor for radio traffic. Radio transmissions are in ascii, comma delimited sentences with specific start and stop characters and other checksum conditions. Real-time radio monitoring is usually just for diagnostic or troubleshooting purposes.

# Other

**CUSTOM PCB**
We started the first versions of this four years ago on breadboards, but quickly realized breadboards don't hold up to the extremes of rockets very well. We found that using EasyEDA to create the schematic and then generate the board design was a good way to make the design accessable to everyone without expensive PCB software licenses. EasyEDA and JCLPCB make it really easy to design a circuit, trace the PCB, and print it for cheap. The source EasyEDA files are included in the hardware folders. 

**CUSTOM 3D PRINTED SLEDs**
We have made dozens of different sleds for 3", 4", and 8" rockets using the online version of Sketchup. We've included the 3" vehicle sled files in the vehicle folder for reference. We also included the base station radio project box files for reference, but those are probably better replaced with a simple off the shelf project box.  


Thank you to everyone that has contributed and offered advice along the way. 

May your rockets fly straight and high and may you always know exactly where they are.

Mike & Preston

