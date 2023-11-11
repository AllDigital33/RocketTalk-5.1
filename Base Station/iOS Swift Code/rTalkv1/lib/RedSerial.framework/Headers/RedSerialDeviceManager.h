//
//  RedSerialDeviceManager.h
//  RedSerial
//
//  Created by Jeremy Bramson on 9/30/19.
//  Copyright © 2019 Redpark Product Development. All rights reserved.
//

#import <Foundation/Foundation.h>
#import <RedSerial/RedSerialPort.h>

NS_ASSUME_NONNULL_BEGIN

@protocol RedSerialDeviceManagerDelegate  <NSObject>

//
// called when a device is discovered
//
- (void) deviceDetected:(RedSerialPort *)thePort;

//
// called when device connection is broken, object is now orphaned
//
- (void) deviceDisconnected:(RedSerialPort *)thePort;
@end


@interface RedSerialDeviceManager : NSObject
{
    
}

@property (nonatomic, weak) id <RedSerialDeviceManagerDelegate> delegate;

// return singleton RedSerialDeviceManager
+ (RedSerialDeviceManager *)sharedManager;

// Call to enumerate connected device or register for notifications
// app should call this once at app launch
- (void) startDiscovery;

// application must call this when re-entering foreground to resume communication
// session with accessory
-(void) resume;

// called to clean up existing communication sessions with device
// app must call when going into background
- (void) stop;


@end

NS_ASSUME_NONNULL_END
