/*
 This file is part of the Structure SDK.
 Copyright © 2016 Occipital, Inc. All rights reserved.
 http://structure.io
 */

#import "ViewController.h"
#import "ViewController+Camera.h"
#import "ViewController+Sensor.h"
#import "ViewController+SLAM.h"
#import "ViewController+OpenGL.h"

#import <Structure/Structure.h>
#import <Structure/StructureSLAM.h>

#include <iostream>
using namespace std;

@implementation ViewController (Sensor)

#pragma mark -  Structure Sensor delegates

- (void)setupStructureSensor
{
    // Get the sensor controller singleton.
    _sensorController = [STSensorController sharedController];
    
    // Set ourself as the delegate to receive sensor data.
    _sensorController.delegate = self;
}

- (BOOL)isStructureConnectedAndCharged
{
    return [_sensorController isConnected] && ![_sensorController isLowPower];
}

- (void)sensorDidConnect
{
    NSLog(@"[Structure] Sensor connected!");
    
    if ([self currentStateNeedsSensor])
        [self connectToStructureSensorAndStartStreaming];
}

- (void)sensorDidLeaveLowPowerMode
{
    _appStatus.sensorStatus = AppStatus::SensorStatusNeedsUserToConnect;
    [self updateAppStatusMessage];
}

- (void)sensorBatteryNeedsCharging
{
    // Notify the user that the sensor needs to be charged.
    _appStatus.sensorStatus = AppStatus::SensorStatusNeedsUserToCharge;
    [self updateAppStatusMessage];
}

- (void)sensorDidStopStreaming:(STSensorControllerDidStopStreamingReason)reason
{
    if (reason == STSensorControllerDidStopStreamingReasonAppWillResignActive)
    {
        [self stopColorCamera];
        NSLog(@"[Structure] Stopped streaming because the app will resign its active state.");
    }
    else
    {
        NSLog(@"[Structure] Stopped streaming for an unknown reason.");
    }
}

- (void)sensorDidDisconnect
{
    // If we receive the message while in background, do nothing. We'll check the status when we
    // become active again.
    if ([[UIApplication sharedApplication] applicationState] != UIApplicationStateActive)
        return;
    
    // Reset the scan on disconnect, since we won't be able to recover afterwards.
    if (_slamState.roomCaptureState == RoomCaptureStateScanning)
    {
        [self resetButtonPressed:self];
    }
    
    [self stopColorCamera];
    
    NSLog(@"[Structure] Sensor disconnected!");
    // We only show the app status when we need sensor
    if ([self currentStateNeedsSensor])
    {
        _appStatus.sensorStatus = AppStatus::SensorStatusNeedsUserToConnect;
        [self updateAppStatusMessage];
    }
    
    if (_calibrationOverlay)
        _calibrationOverlay.hidden = true;
    
    [self updateIdleTimer];
}

- (STSensorControllerInitStatus)connectToStructureSensorAndStartStreaming
{
    
    // Try connecting to a Structure Sensor.
    STSensorControllerInitStatus result = [_sensorController initializeSensorConnection];
    
    if (result == STSensorControllerInitStatusSuccess || result == STSensorControllerInitStatusAlreadyInitialized)
    {
        // We are connected, so get rid of potential previous messages being displayed.
        _appStatus.sensorStatus = AppStatus::SensorStatusOk;
        [self updateAppStatusMessage];
        
        // Start streaming depth data.
        [self startStructureSensorStreaming];
    }
    else
    {
        switch (result)
        {
            case STSensorControllerInitStatusSensorNotFound:     NSLog(@"[Structure] No sensor found."); break;
            case STSensorControllerInitStatusOpenFailed:         NSLog(@"[Structure] Error: open failed."); break;
            case STSensorControllerInitStatusSensorIsWakingUp:   NSLog(@"[Structure] Sensor is waking up."); break;
            default: {}
        }
        
        _appStatus.sensorStatus = AppStatus::SensorStatusNeedsUserToConnect;
        [self updateAppStatusMessage];
    }
    
    [self updateIdleTimer];
    
    return result;
}

- (void)startStructureSensorStreaming
{
    if (![self isStructureConnectedAndCharged])
    {
        NSLog(@"Error: Structure Sensor not connected or not charged.");
        return;
    }
    
    // Tell the driver to start streaming.
    // We are also using the color camera, so make sure the depth gets synchronized with it.
    NSError *error = nil;
        
    // We will use color for tracking and rendering, so let's use registered depth.
    STStreamConfig structureStreamConfig = _options.useHardwareRegisteredDepth ? STStreamConfigRegisteredDepth320x240 : STStreamConfigDepth320x240;
    
    BOOL optionsAreValid = [_sensorController startStreamingWithOptions:@{
                                                                          kSTStreamConfigKey : @(structureStreamConfig),
                                                                          kSTFrameSyncConfigKey : @(STFrameSyncDepthAndRgb),
                                                                          kSTColorCameraFixedLensPositionKey: @(_options.colorCameraLensPosition)
                                                                          }
                                                                  error:nil];
    
    if (!optionsAreValid)
    {
        NSLog(@"Error during streaming start: %s", [[error localizedDescription] UTF8String]);
        return;
    }
    
    NSLog(@"[Structure] Streaming started.");
    
    
    // We'll only turn on the color camera if we have at least an approximate calibration
    STCalibrationType calibrationType = [_sensorController calibrationType];
    if(calibrationType == STCalibrationTypeApproximate || calibrationType == STCalibrationTypeDeviceSpecific)
    {
        _appStatus.colorCameraIsCalibrated = true;
        [self updateAppStatusMessage];
        
        [self startColorCamera];
        
    }
    else
    {
        NSLog(@"This device does not have a calibration between color and depth.");
        
        _appStatus.colorCameraIsCalibrated = false;
        [self updateAppStatusMessage];
    }
    
    
    // Notify and initialize streaming dependent objects.
    [self onStructureSensorStartedStreaming];
}

- (void)onStructureSensorStartedStreaming
{
    STCalibrationType calibrationType = [_sensorController calibrationType];
    
    // The Calibrator app will be updated to support future iPads, and additional attachment brackets will be released as well.
    const bool deviceIsLikelySupportedByCalibratorApp = (UI_USER_INTERFACE_IDIOM() == UIUserInterfaceIdiomPad);
    
    // Only present the option to switch over to the Calibrator app if the sensor doesn't already have a device specific
    // calibration and the app knows how to calibrate this iOS device.
    if (calibrationType != STCalibrationTypeDeviceSpecific && deviceIsLikelySupportedByCalibratorApp)
    {
        if (_calibrationOverlay)
            _calibrationOverlay.hidden = false;
        else
            _calibrationOverlay = [CalibrationOverlay calibrationOverlaySubviewOf:self.view atOrigin:CGPointMake(16, 16)];
    }
    else
    {
        if (_calibrationOverlay)
            _calibrationOverlay.hidden = true;
    }
}

- (void)sensorDidOutputSynchronizedDepthFrame:(STDepthFrame *)depthFrame
                                   colorFrame:(STColorFrame*)colorFrame
{
    if (_slamState.initialized)
    {
        [self processDepthFrame:depthFrame colorFrame:colorFrame];
        // Scene rendering is triggered by new frames to avoid rendering the same view several times.
        [self renderSceneWithDepthFrame:depthFrame colorFrame:colorFrame];
        
        // update curPoint and curScreenPoint
        [self updatePotentialPoint: depthFrame];
//        switch (_measure.mstatus) {
//            case Measurements::MeasureNoPoint:
//                break;
//                
//            case Measurements::MeasureOnePoint:
//                if (_measure.pt1NeedsConvert){
//                    _measure.pt1 = [self screenPtsTo3DPts:_measure.pt1 fromDepth:depthFrame];
//                    _measure.pt1NeedsConvert = false;
//                }
//                break;
//                
//            case Measurements::MeasureTwoPoints:
//                if (_measure.pt1NeedsConvert){
//                    _measure.pt1 = [self screenPtsTo3DPts:_measure.pt1 fromDepth:depthFrame];
//                    _measure.pt1NeedsConvert = false;
//                }
//                if (_measure.pt2NeedsConvert){
//                    _measure.pt2 = [self screenPtsTo3DPts:_measure.pt2 fromDepth:depthFrame];
//                    _measure.pt2NeedsConvert = false;
//                }
//                
//                // Calculate distance
//                _measure.distance = GLKVector3Length(GLKVector3Subtract(_measure.pt2, _measure.pt1));
//                [self.distanceLabel setText:[NSString stringWithFormat:@"%f m", _measure.distance*0.001] ];
//                cout << "distance: " << _measure.distance << endl;
//                _measure.mstatus = Measurements::MeasureNoPoint;
//                break;
//            default:{}
//        }
    }
}

- (void) updatePotentialPoint: (STDepthFrame*) depthFrame {
    // check every depth in the center 11*11
    float minDepth = NAN;
    for (int ic = -5; ic <= 5; ic++){
        for (int ir = -5; ir <= 5; ir++){
            GLKVector3 spt = GLKVector3Make(self.view.frame.size.width/2+ic*3.2, self.view.frame.size.height/2+ir*3.2, NAN);
            GLKVector3 pt3d = [self screenPtsTo3DPts:spt fromDepth:depthFrame];
            cout << "ic:" << ic << " ir:" << ir << " pt3d v[2]:" << pt3d.v[2] << endl;
            if (minDepth != minDepth || pt3d.v[2] < minDepth){
                minDepth = pt3d.v[2];
                _measure.curPoint = pt3d;
                _measure.curPointScreen = CGPointMake(spt.v[0], spt.v[1]);
            }
        }
    }
    cout << "updatePotentialPoint minDepth:" << minDepth << " screenPt:(" << _measure.curPointScreen.x << ", " << _measure.curPointScreen.y << ")" <<  endl;
}

// Assume the following intrinsics, from the Structure SDK docs
// K_RGB_QVGA       = [305.73, 0, 159.69; 0, 305.62, 119.86; 0, 0, 1]
#define QVGA_COLS 320
#define QVGA_ROWS 240
#define QVGA_F_X 305.73
#define QVGA_F_Y 305.62
#define QVGA_C_X 159.69
#define QVGA_C_Y 119.86
- (GLKVector3)screenPtsTo3DPts: (GLKVector3) screenPt fromDepth: (STDepthFrame *)depthFrame {
    float _fx = QVGA_F_X/QVGA_COLS*depthFrame.width;
    float _fy = QVGA_F_Y/QVGA_ROWS*depthFrame.height;
    float _cx = QVGA_C_X/QVGA_COLS*depthFrame.width;
    float _cy = QVGA_C_Y/QVGA_ROWS*depthFrame.height;
    
    int r = screenPt.v[1]/3.2, c = screenPt.v[0]/3.2;
    int pointIndex = r*depthFrame.width + c;
    
    float depth=depthFrame.depthInMillimeters[pointIndex];
    float xc = depth * (c - _cx) / _fx;
    float yc = depth * (r - _cy) / _fy;
    float zc = depth;
    
    GLKVector3 ptInCam = GLKVector3Make(xc, yc, zc);
    GLKMatrix4 camPose = [_slamState.tracker lastFrameCameraPose];
    GLKVector3 ptInWorld = GLKMatrix4MultiplyVector3WithTranslation(camPose, ptInCam);
    
    
//    cout << "screen point (" << screenPt.v[0] << "," << screenPt.v[1] << ") to 3d point ("
//        << xc << "," << yc << "," << zc << ") at r:" << r << ",c:" << c << ",depth:" << depth << endl;
//    cout << "_fx:" << _fx << " _fy:" << _fy << " _cx:" << _cx << " _cy:" << _cy << endl;
    return ptInWorld;
}

//- (GLKVector3)screenPtsTo3DPtsByMesh: (GLKVector3)

@end
