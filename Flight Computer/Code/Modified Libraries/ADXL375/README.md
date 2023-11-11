# shockduino

The idea is to create a small device that you can include in a package/parcel sent via the post. 
 
The device will be capable of detecting shock/drop events. Before signing for the parcel at the other end you can see if it's been dropped or not.

## Hardware

Initially using an [RFduino](http://www.rfduino.com/) and an [ADXL375](http://www.analog.com/en/products/mems/mems-accelerometers/adxl375.html) accelerometer.

### Prototype

The prototype design is using an [RFduino dev board](http://www.rfduino.com/product/rfd22102-rfduino-dip/index.html), [RFduino USB shield](http://www.rfduino.com/product/rfd22121-usb-shield-for-rfduino/index.html) and an [ADXL375Z eval board](http://www.analog.com/en/design-center/evaluation-hardware-and-software/evaluation-boards-kits/EVAL-ADXL375.html) all slung together on a prototyping breadboard.

Connections between the RFduino and ADXL375:
- Ground is common
- 3.3v from the RFduino is connected to VIO and VS 
- SCL is connected between GPIO pins 3,4,5,6 to SDO,SCL,SDA,CS

## Project Plan

### Prototype Hardware

- [x] Connect up hardware and verify it works

### Prototype Software

- [x] Pull meaningful data from the ADXL375
- [x] Separate out ADXL375 functionality into a library
- [x] Implement shock detection on the ADXL375
- [x] Record detailed accelleration data after a shock event is triggered
- [x] Return maximum shock amplitude after a shock has occurred
- [x] Make configurable shock detection interface simple
- [x] Get iBeacon reporting of shocks working
- [ ] Get RFduino sleep mode and interrupt on shock event working
- [ ] Get INACT/ACT detection working with ADXL375
- [ ] Configure iBeacon to only advertise when the ADXL375 is active
- [ ] Allow RFduino to detect issues and power cycle ADXL375
