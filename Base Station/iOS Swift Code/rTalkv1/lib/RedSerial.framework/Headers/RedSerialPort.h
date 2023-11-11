//
//  RedSerialPort.h
//  RedSerial
//
//  Created by Jeremy Bramson on 9/30/19.
//  Copyright © 2019 Redpark Product Development. All rights reserved.
//

#import <Foundation/Foundation.h>

NS_ASSUME_NONNULL_BEGIN


// DEFAULT VALUES
// baudRate == 9600
// dataConfiguration == kDataConfig_8N1
// enableRtsCtsFlowControl == NO
// enableDtrDsrFlowControl == NO
// enableSoftwareFlowControl == NO
// dtr == FALSE
// rts == FALSE

enum SerialPortDataConfiguration
{
    kDataConfig_8N1,  // 8bit, no parity, one stop bit
    kDataConfig_8O1,  // 8bit, odd parity, one stop bit
    kDataConfig_8E1,  // 8bit, even parity, one stop bit
    kDataConfig_7O1,  // 7bit, odd parity, one stop bit
    kDataConfig_7E1,  // 7bit, even parity, one stop bit
    kDataConfig_8N2,  // 8bit, no parity, two stop bits
    kDataConfig_8O2,  // 8bit, odd parity, two stop bits
    kDataConfig_8E2,  // 8bit, even parity, two stop bits

};

@class RedSerialPort;

@protocol RedSerialPortDelegate  <NSObject>


@optional
// called when serial port's transmit FIFO is empty
-(void) transmitFifoIsEmpty:(RedSerialPort *)thePort;

// called when any of the rx modem signals change state (CTS, DSR, DCD or RI)
// user can check value of each modem signal property to determine current state
// optionally, user can set an observer for the modem signal property interested in
- (void) modemSignalChange:(RedSerialPort *)thePort;

// called when TX or RX Flow Control state changes
- (void) flowControlStateChange:(RedSerialPort *)thePort flowControlIsHalted:(BOOL)isFlowHalted;


@end

@interface RedSerialPort : NSObject
{

}

@property (nonatomic, weak) id <RedSerialPortDelegate> delegate;
@property (readonly) BOOL cts;
@property (readonly) BOOL dsr;
@property (readonly) BOOL dcd;
@property (readonly) BOOL ri;
@property (nonatomic) BOOL dtr;
@property (nonatomic) BOOL rts;
@property (readonly) BOOL isFlowConbtrolHalted;
@property (nonatomic) int baudRate;
@property (nonatomic) enum SerialPortDataConfiguration dataConfiguration;
@property (nonatomic) BOOL enableRtsCtsFlowControl;
@property (nonatomic) BOOL enableDtrDsrFlowControl;
@property (nonatomic) BOOL enableSoftwareFlowControl;

// send NSData bytes over serial connection
// calls sendCompleteBlock when all bytes in the NSData are transferred from UART FIFO
-(BOOL)sendData:(NSData *)data sendDataComplete:(void (^)(void))sendCompleteBlock;

// post a read to the serial port
// recvCompleBlock is called when bytes are received
-(BOOL)recvData:(void (^)(NSData *))recvCompleteBlock;



/* =============================================================*/
enum
{
    kFwUpdataComplete = 0,
    kFwUpdataInProgress   = 1,
    kFwUpdataErasingSectors       = 2,
    kFwUpdataRebootingDevice      = 3,
    kFwUpdataBadAddress   = 0x81,
    kFwUpdataVerifyFailed = 0x82,
    kFwUpdataWriteError   = 0x83,
    kFwUpdataBadLength    = 0x84,
   kFwUpdataFlashTimeout = 0x85
    
};

// Check if connected serial device has latest firmware
-(BOOL)isFirmwareCurrent;

// update serial device firmware to latest version
// updateProgressBlock will be called with firmare download progress
// progress is a value 0-100 indicating the percent of file downloaded.
// state = kFwUpdataInProgress - download is in progress.
// state = kFwUpdataComplete - download is complete and was successful
// state = kFwUpdataRebootingDevice - normal expected state during update
// state = kFwUpdataErasingSectors - normal expected state during update
// state = 0x8n - an error occurred during download and was stopped.  Cable was not updated.
-(void)updateFirmware:(void (^)(int progress, uint8_t state))updateProgressBlock;

@end

NS_ASSUME_NONNULL_END
