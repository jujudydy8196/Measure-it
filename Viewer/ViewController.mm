/*
  This file is part of the Structure SDK.
  Copyright © 2016 Occipital, Inc. All rights reserved.
  http://structure.io
*/

#import "ViewController.h"
#import <Structure/StructureSLAM.h>
#import <AVFoundation/AVFoundation.h>
#import <Accelerate/Accelerate.h>
#import <algorithm>
#include <iostream>

//------------------------------------------------------------------------------

namespace {

bool
convertYCbCrToBGRA (
    size_t width,
    size_t height,
    const uint8_t* yData,
    const uint8_t* cbcrData,
    uint8_t* rgbaData,
    uint8_t alpha,
    size_t yBytesPerRow,
    size_t cbCrBytesPerRow,
    size_t rgbaBytesPerRow)
{
    assert(width <= rgbaBytesPerRow);
    
    // Input RGBA buffer:
    
    vImage_Buffer rgbaBuffer
    {
        .data = (void*)rgbaData,
        .width = (size_t)width,
        .height = (size_t)height,
        .rowBytes = rgbaBytesPerRow
    };
    
    // Destination Y, CbCr buffers:
    
    vImage_Buffer cbCrBuffer
    {
        .data = (void*)cbcrData,
        .width = (size_t)width/2,
        .height = (size_t)height/2,
        .rowBytes = (size_t)cbCrBytesPerRow // 2 bytes per pixel (Cb+Cr)
    };
    
    vImage_Buffer yBuffer
    {
        .data = (void*)yData,
        .width = (size_t)width,
        .height = (size_t)height,
        .rowBytes = (size_t)yBytesPerRow
    };

    vImage_Error error = kvImageNoError;
    
    // Conversion information:
    
    static vImage_YpCbCrToARGB info;
    {
        static bool infoGenerated = false;
    
        if(!infoGenerated)
        {
            vImage_Flags flags = kvImageNoFlags;
            
            vImage_YpCbCrPixelRange pixelRange
            {
                .Yp_bias =      0,
                .CbCr_bias =    128,
                .YpRangeMax =   255,
                .CbCrRangeMax = 255,
                .YpMax =        255,
                .YpMin =        0,
                .CbCrMax=       255,
                .CbCrMin =      1
            };

            error = vImageConvert_YpCbCrToARGB_GenerateConversion(
                kvImage_YpCbCrToARGBMatrix_ITU_R_601_4,
                &pixelRange,
                &info,
                kvImage420Yp8_CbCr8, kvImageARGB8888,
                flags
            );
            
            if (kvImageNoError != error)
                return false;

            infoGenerated = true;
        }
    }

    static const uint8_t permuteMapBGRA [4] { 3, 2, 1, 0 };

    error = vImageConvert_420Yp8_CbCr8ToARGB8888(
        &yBuffer,
        &cbCrBuffer,
        &rgbaBuffer,
        &info,
        permuteMapBGRA,
        255,
        kvImageNoFlags | kvImageDoNotTile // Disable multithreading.
    );

    return kvImageNoError == error;
}

} // namespace

//------------------------------------------------------------------------------

struct AppStatus
{
    NSString* const   pleaseConnectSensorMessage = @"Please connect Structure Sensor.";
    NSString* const    pleaseChargeSensorMessage = @"Please charge Structure Sensor.";
    NSString* const needColorCameraAccessMessage = @"This app requires camera access to capture color.\nAllow access by going to Settings → Privacy → Camera.";
    
    enum SensorStatus
    {
        SensorStatusOk,
        SensorStatusNeedsUserToConnect,
        SensorStatusNeedsUserToCharge,
    };
    
    // Structure Sensor status.
    SensorStatus sensorStatus = SensorStatusOk;
    
    // Whether iOS camera access was granted by the user.
    bool colorCameraIsAuthorized = true;
    
    // Whether there is currently a message to show.
    bool needsDisplayOfStatusMessage = false;
    
    // Flag to disable entirely status message display.
    bool statusMessageDisabled = false;
};

//------------------------------------------------------------------------------

@interface ViewController () <AVCaptureVideoDataOutputSampleBufferDelegate> {
    
    STSensorController *_sensorController;
    
    AVCaptureSession *_avCaptureSession;
    AVCaptureDevice *_videoDevice;

    UIImageView *_depthImageView;
//    UIImageView *_normalsImageView;
//    UIImageView *_colorImageView;
    
    uint16_t *_linearizeBuffer;
    uint8_t *_coloredDepthBuffer;
//    uint8_t *_normalsBuffer;
//    uint8_t *_colorImageBuffer;

//    STNormalEstimator *_normalsEstimator;
    
    UILabel* _statusLabel;
    
    
    AppStatus _appStatus;
    
    double measureCoords[4]; // [x1, y1, x2, y2]
    
    UITextView *coordView_; // For deubgging

}

- (BOOL)connectAndStartStreaming;
- (void)renderDepthFrame:(STDepthFrame*)depthFrame;
- (void)renderNormalsFrame:(STDepthFrame*)normalsFrame;
- (void)renderColorFrame:(CMSampleBufferRef)sampleBuffer;
- (void)setupColorCamera;
- (void)startColorCamera;
- (void)stopColorCamera;

@end

//------------------------------------------------------------------------------

@implementation ViewController

- (void)viewDidLoad
{
    [super viewDidLoad];
    
    _sensorController = [STSensorController sharedController];
    _sensorController.delegate = self;
    
        coordView_ = [[UITextView alloc] initWithFrame:CGRectMake(0,15,self.view.frame.size.width,35)];
        [coordView_ setOpaque:false]; // Set to be Opaque
        [coordView_ setBackgroundColor:[UIColor clearColor]]; // Set background color to be clear
        [coordView_ setTextColor:[UIColor redColor]]; // Set text to be RED
        [coordView_ setFont:[UIFont systemFontOfSize:18]]; // Set the Font size
        [self.view addSubview:coordView_];


    // Create three image views where we will render our frames
    
    CGRect depthFrame = self.view.frame;
//    depthFrame.size.height /= 2;
//    depthFrame.origin.y = self.view.frame.size.height/2;
    depthFrame.origin.y = 1;
    depthFrame.origin.x = 1;
//    depthFrame.origin.x = -self.view.frame.size.width * 0.25;
    
//    CGRect normalsFrame = self.view.frame;
//    normalsFrame.size.height /= 2;
//    normalsFrame.origin.y = self.view.frame.size.height/2;
//    normalsFrame.origin.x = 1;
//    normalsFrame.origin.x = self.view.frame.size.width * 0.25;
//    
//    CGRect colorFrame = self.view.frame;
//    colorFrame.size.height /= 2;
    
    _linearizeBuffer = NULL;
//    _coloredDepthBuffer = NULL;
//    _normalsBuffer = NULL;
//    _colorImageBuffer = NULL;

    _depthImageView = [[UIImageView alloc] initWithFrame:depthFrame];
    _depthImageView.contentMode = UIViewContentModeScaleAspectFit;
    [self.view addSubview:_depthImageView];
    
//    _normalsImageView = [[UIImageView alloc] initWithFrame:normalsFrame];
//    _normalsImageView.contentMode = UIViewContentModeScaleAspectFit;
//    [self.view addSubview:_normalsImageView];
//    
//    _colorImageView = [[UIImageView alloc] initWithFrame:colorFrame];
//    _colorImageView.contentMode = UIViewContentModeScaleAspectFit;
//    [self.view addSubview:_colorImageView];

//    [self setupColorCamera];
    measureCoords[0] = 1;
    measureCoords[1] = 1;
    measureCoords[2] = depthFrame.size.width-1;
    measureCoords[3] = depthFrame.size.height-1;
    
    std::cout << "measureCoords: (" << measureCoords[0] << "," << measureCoords[1] << ") to ("
    << measureCoords[2] << "," << measureCoords[3] << ")" << std::endl;
    
}

- (void)dealloc
{
    free(_linearizeBuffer);
    free(_coloredDepthBuffer);
//    free(_normalsBuffer);
//    free(_colorImageBuffer);
}

- (void)viewDidAppear:(BOOL)animated
{
    [super viewDidAppear:animated];
    
    static BOOL fromLaunch = YES;
    
    if(!fromLaunch)
        return;

    // Create a UILabel in the center of our view to display status messages.

    if (!_statusLabel)
    {
        // We do this here instead of in viewDidLoad so that we get the correctly size/rotation view bounds.
        
        _statusLabel = [[UILabel alloc] initWithFrame:self.view.bounds];
        _statusLabel.backgroundColor = [[UIColor blackColor] colorWithAlphaComponent:0.7];
        _statusLabel.textAlignment = NSTextAlignmentCenter;
        _statusLabel.font = [UIFont systemFontOfSize:35.0];
        _statusLabel.numberOfLines = 2;
        _statusLabel.textColor = [UIColor whiteColor];

        [self updateAppStatusMessage];
        
        [self.view addSubview: _statusLabel];
    }

    [self connectAndStartStreaming];

    
    fromLaunch = NO;

    // From now on, make sure we get notified when the app becomes active to restore the sensor state if necessary.

    [[NSNotificationCenter defaultCenter]
        addObserver:self
        selector:@selector(appDidBecomeActive)
        name:UIApplicationDidBecomeActiveNotification
        object:nil
    ];
}

- (void)appDidBecomeActive
{
    [self connectAndStartStreaming];
}

- (void)didReceiveMemoryWarning
{
    [super didReceiveMemoryWarning];
}

- (BOOL)connectAndStartStreaming
{
    std::cout << "connectAndStartStreaming" << std::endl;
    STSensorControllerInitStatus result = [_sensorController initializeSensorConnection];

    std::cout << "connectAndStartStreaming result" << result << std::endl;
    
    BOOL didSucceed =
        result == STSensorControllerInitStatusSuccess
     || result == STSensorControllerInitStatusAlreadyInitialized;
    
    std::cout << "connectAndStartStreaming didSucceed " << didSucceed << std::endl;

    if (didSucceed)
    {
        // There's no status about the sensor that we need to display anymore.

        [self updateAppStatusMessageWithSensorStatus:AppStatus::SensorStatusOk];
        
        // Start the color camera, setup if needed.

        [self startColorCamera];
        
        // Set sensor stream quality.

        STStreamConfig streamConfig = STStreamConfigDepth320x240;

        // Request that we receive depth frames with synchronized color pairs.
        // After this call, we will start to receive frames through the delegate methods.

        NSError* error = nil;

        BOOL optionsAreValid = [_sensorController
            startStreamingWithOptions:@{
                kSTStreamConfigKey : @(streamConfig),
                kSTFrameSyncConfigKey:@(STFrameSyncDepthAndRgb),
                kSTHoleFilterEnabledKey:@TRUE // Depth looks better without holes.
            }
            error:&error
        ];
        
        if (!optionsAreValid)
        {
            NSLog(@"Error during streaming start: %s", [[error localizedDescription] UTF8String]);
            return false;
        }
        
        // Allocate the depth to surface normals converter class.
//        _normalsEstimator = [[STNormalEstimator alloc] init];
    }
    else
    {
        if (result == STSensorControllerInitStatusSensorNotFound)
            NSLog(@"[Debug] No Structure Sensor found!");
        else if (result == STSensorControllerInitStatusOpenFailed)
            NSLog(@"[Error] Structure Sensor open failed.");
        else if (result == STSensorControllerInitStatusSensorIsWakingUp)
            NSLog(@"[Debug] Structure Sensor is waking from low power.");
        else if (result != STSensorControllerInitStatusSuccess)
            NSLog(@"[Debug] Structure Sensor failed to init with status %d.", (int)result);
        
        [self updateAppStatusMessageWithSensorStatus:AppStatus::SensorStatusNeedsUserToConnect];
    }
    
    return didSucceed;
}

- (void)showAppStatusMessage:(NSString *)msg
{
    _appStatus.needsDisplayOfStatusMessage = true;
    
    [self.view.layer removeAllAnimations];
    
    [_statusLabel setText:msg];
    [_statusLabel setHidden:NO];
    
    // Progressively show the message label.

    [self.view setUserInteractionEnabled:false];
    [UIView
        animateWithDuration:0.5f
        animations:^{
            _statusLabel.alpha = 1.0f;
        }
        completion:nil
    ];
}

- (void)hideAppStatusMessage
{
    _appStatus.needsDisplayOfStatusMessage = false;

    [self.view.layer removeAllAnimations];
    
    [UIView
        animateWithDuration:0.5f
        animations:^{
            _statusLabel.alpha = 0.0f;
        }
        completion:^(BOOL finished) {

            // If nobody called showAppStatusMessage before the end of the animation, do not hide it.

            if (!_appStatus.needsDisplayOfStatusMessage)
            {
                [_statusLabel setHidden:YES];
                [self.view setUserInteractionEnabled:true];
            }
        }
    ];
}

-(void)updateAppStatusMessage
{
    // Skip everything if we should not show app status messages (e.g. in viewing state).

    if (_appStatus.statusMessageDisabled)
    {
        [self hideAppStatusMessage];

        return;
    }
    
    // First show sensor issues, if any.

    switch (_appStatus.sensorStatus)
    {
        case AppStatus::SensorStatusOk:
        {
            break;
        }
            
        case AppStatus::SensorStatusNeedsUserToConnect:
        {
            [self showAppStatusMessage:_appStatus.pleaseConnectSensorMessage];
//
//            return;
            // hack Yi
            break;
        }
            
        case AppStatus::SensorStatusNeedsUserToCharge:
        {
            [self showAppStatusMessage:_appStatus.pleaseChargeSensorMessage];

            return;
        }
    }
    
    // Then show color camera permission issues, if any.

    if (!_appStatus.colorCameraIsAuthorized)
    {
        [self showAppStatusMessage:_appStatus.needColorCameraAccessMessage];

        return;
    }
    
    // If we reach this point, no status to show.

    [self hideAppStatusMessage];
}

-(void)updateAppStatusMessageWithColorCameraAuthorization:(bool)colorCameraIsAuthorized
{
    _appStatus.colorCameraIsAuthorized = colorCameraIsAuthorized;
    
    [self updateAppStatusMessage];
}

-(void)updateAppStatusMessageWithSensorStatus:(AppStatus::SensorStatus)sensorStatus
{
    _appStatus.sensorStatus = sensorStatus;
    
    [self updateAppStatusMessage];
}

-(bool) isConnectedAndCharged
{
    return [_sensorController isConnected] && ![_sensorController isLowPower];
}

//------------------------------------------------------------------------------

#pragma mark -
#pragma mark Structure SDK Delegate Methods

- (void)sensorDidDisconnect
{
    NSLog(@"Structure Sensor disconnected!");

    [self updateAppStatusMessageWithSensorStatus:AppStatus::SensorStatusNeedsUserToConnect];
    
    // Stop the color camera when there isn't a connected Structure Sensor.

    [self stopColorCamera];
}

- (void)sensorDidConnect
{
    NSLog(@"Structure Sensor connected!");

    [self connectAndStartStreaming];
}

- (void)sensorDidLeaveLowPowerMode
{
    // Notify the user that the sensor needs to be connected.

    [self updateAppStatusMessageWithSensorStatus:AppStatus::SensorStatusNeedsUserToConnect];
}


- (void)sensorBatteryNeedsCharging
{
    // Notify the user that the sensor needs to be charged.

    [self updateAppStatusMessageWithSensorStatus:AppStatus::SensorStatusNeedsUserToCharge];
}

- (void)sensorDidStopStreaming:(STSensorControllerDidStopStreamingReason)reason
{
    // If needed, change any UI elements to account for the stopped stream.

    // Stop the color camera when we're not streaming from the Structure Sensor.

    [self stopColorCamera];
}

- (void)sensorDidOutputDepthFrame:(STDepthFrame *)depthFrame
{
    [self renderDepthFrame:depthFrame];
//    [self renderNormalsFrame:depthFrame];
}

// This synchronized API will only be called when two frames match.
// Typically, timestamps are within 1ms of each other.
// Two important things have to happen for this method to be called:
// Tell the SDK we want framesync with options @{ kSTFrameSyncConfigKey : @(STFrameSyncDepthAndRgb) } in [STSensorController startStreamingWithOptions:error:]
// Give the SDK color frames as they come in: [_ocSensorController frameSyncNewColorBuffer:sampleBuffer]
- (void)sensorDidOutputSynchronizedDepthFrame:(STDepthFrame *)depthFrame
                                   colorFrame:(STColorFrame *)colorFrame
{
    [self renderDepthFrame:depthFrame];
//    [self renderNormalsFrame:depthFrame];
//    [self renderColorFrame:colorFrame.sampleBuffer];
}

//------------------------------------------------------------------------------

#pragma mark -
#pragma mark Rendering

const uint16_t maxShiftValue = 2048;

- (void)populateLinearizeBuffer
{
    _linearizeBuffer = (uint16_t*)malloc((maxShiftValue + 1) * sizeof(uint16_t));
    
    for (int i=0; i <= maxShiftValue; i++)
    {
        float v = i / (float)maxShiftValue;
        v = powf(v, 3)* 6;
        _linearizeBuffer[i] = v*6*256;
    }
}

// Conversion of 16-bit non-linear shift depth values to 32-bit RGBA
// Adapted from: https://github.com/OpenKinect/libfreenect/blob/master/examples/glview.c
// This function is equivalent to calling [STDepthAsRgba convertDepthFrameToRgba] with the STDepthToRgbaStrategyRedToBlueGradient strategy.
// Not using the SDK here for didactic purposes.
- (void)convertShiftToRGBA:(const uint16_t*)shiftValues depthValuesCount:(size_t)depthValuesCount
{
    
//    std::cout << depthValuesCount << std::endl;
    
    for (size_t i = 0; i < depthValuesCount; i++)
    {
        
        // We should not get higher values than maxShiftValue, but let's stay on the safe side.
        uint16_t boundedShift = std::min (shiftValues[i], maxShiftValue);
        // Use a lookup table to make the non-linear input values vary more linearly with metric depth
        int linearizedDepth = _linearizeBuffer[boundedShift];

        // Use the upper byte of the linearized shift value to choose a base color
        // Base colors range from: (closest) White, Red, Orange, Yellow, Green, Cyan, Blue, Black (farthest)
        int lowerByte = (linearizedDepth & 0xff);
        
        // Use the lower byte to scale between the base colors
        int upperByte = (linearizedDepth >> 8);
        
        switch (upperByte)
        {
            case 0:
                _coloredDepthBuffer[4 * i + 0] = 255;
                _coloredDepthBuffer[4 * i + 1] = 255 - lowerByte;
                _coloredDepthBuffer[4 * i + 2] = 255 - lowerByte;
                _coloredDepthBuffer[4 * i + 3] = 255;
                break;
            case 1:
                _coloredDepthBuffer[4 * i + 0] = 255;
                _coloredDepthBuffer[4 * i + 1] = lowerByte;
                _coloredDepthBuffer[4 * i + 2] = 0;
                break;
            case 2:
                _coloredDepthBuffer[4 * i + 0] = 255 - lowerByte;
                _coloredDepthBuffer[4 * i + 1] = 255;
                _coloredDepthBuffer[4 * i + 2] = 0;
                break;
            case 3:
                _coloredDepthBuffer[4 * i + 0] = 0;
                _coloredDepthBuffer[4 * i + 1] = 255;
                _coloredDepthBuffer[4 * i + 2] = lowerByte;
                break;
            case 4:
                _coloredDepthBuffer[4 * i + 0] = 0;
                _coloredDepthBuffer[4 * i + 1] = 255 - lowerByte;
                _coloredDepthBuffer[4 * i + 2] = 255;
                break;
            case 5:
                _coloredDepthBuffer[4 * i + 0] = 0;
                _coloredDepthBuffer[4 * i + 1] = 0;
                _coloredDepthBuffer[4 * i + 2] = 255 - lowerByte;
                break;
            default:
                _coloredDepthBuffer[4 * i + 0] = 0;
                _coloredDepthBuffer[4 * i + 1] = 0;
                _coloredDepthBuffer[4 * i + 2] = 0;
                break;
        }
    }
}

- (void)renderDepthFrame:(STDepthFrame *)depthFrame
{
    size_t cols = depthFrame.width;
    size_t rows = depthFrame.height;
//    std::cout << "cols : " << cols << " rows: " << rows << std::endl;
    
    if (_linearizeBuffer == NULL) //|| _normalsBuffer == NULL)
    {
        [self populateLinearizeBuffer];
        _coloredDepthBuffer = (uint8_t*)malloc(cols * rows * 4);
    }
    
    [self convertShiftToRGBA:depthFrame.shiftData depthValuesCount:cols * rows];
//    [self distance:depthFrame.shiftData depthValuesCount:cols * rows];

    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    
    CGBitmapInfo bitmapInfo;
    bitmapInfo = (CGBitmapInfo)kCGImageAlphaNoneSkipLast;
    bitmapInfo |= kCGBitmapByteOrder32Big;
    
    NSData *data = [NSData dataWithBytes:_coloredDepthBuffer length:cols * rows * 4];
    CGDataProviderRef provider = CGDataProviderCreateWithCFData((CFDataRef)data); //toll-free ARC bridging
    
    CGImageRef imageRef = CGImageCreate(cols,                       //width
                                       rows,                        //height
                                       8,                           //bits per component
                                       8 * 4,                       //bits per pixel
                                       cols * 4,                    //bytes per row
                                       colorSpace,                  //Quartz color space
                                       bitmapInfo,                  //Bitmap info (alpha channel?, order, etc)
                                       provider,                    //Source of data for bitmap
                                       NULL,                        //decode
                                       false,                       //pixel interpolation
                                       kCGRenderingIntentDefault);  //rendering intent
    
    // Assign CGImage to UIImage
    _depthImageView.image = [UIImage imageWithCGImage:imageRef];

    
    CGImageRelease(imageRef);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(colorSpace);
    
}

//- (void) renderNormalsFrame: (STDepthFrame*) depthFrame
//{
//    // Estimate surface normal direction from depth float values
//    STNormalFrame *normalsFrame = [_normalsEstimator calculateNormalsWithDepthFrame:depthFrame];
//    
//    size_t cols = normalsFrame.width;
//    size_t rows = normalsFrame.height;
//    
//    // Convert normal unit vectors (ranging from -1 to 1) to RGB (ranging from 0 to 255)
//    // Z can be slightly positive in some cases too!
//    if (_normalsBuffer == NULL)
//    {
//        _normalsBuffer = (uint8_t*)malloc(cols * rows * 4);
//    }
//    for (size_t i = 0; i < cols * rows; i++)
//    {
//        _normalsBuffer[4*i+0] = (uint8_t)( ( ( normalsFrame.normals[i].x / 2 ) + 0.5 ) * 255);
//        _normalsBuffer[4*i+1] = (uint8_t)( ( ( normalsFrame.normals[i].y / 2 ) + 0.5 ) * 255);
//        _normalsBuffer[4*i+2] = (uint8_t)( ( ( normalsFrame.normals[i].z / 2 ) + 0.5 ) * 255);
//    }
//    
//    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
//    
//    CGBitmapInfo bitmapInfo;
//    bitmapInfo = (CGBitmapInfo)kCGImageAlphaNoneSkipFirst;
//    bitmapInfo |= kCGBitmapByteOrder32Little;
//    
//    NSData *data = [NSData dataWithBytes:_normalsBuffer length:cols * rows * 4];
//    CGDataProviderRef provider = CGDataProviderCreateWithCFData((CFDataRef)data);
//    
//    CGImageRef imageRef = CGImageCreate(cols,
//                                        rows,
//                                        8,
//                                        8 * 4,
//                                        cols * 4,
//                                        colorSpace,
//                                        bitmapInfo,
//                                        provider,
//                                        NULL,
//                                        false,
//                                        kCGRenderingIntentDefault);
//    
//    _normalsImageView.image = [[UIImage alloc] initWithCGImage:imageRef];
//    
//    CGImageRelease(imageRef);
//    CGDataProviderRelease(provider);
//    CGColorSpaceRelease(colorSpace);
//}

//- (void)renderColorFrame:(CMSampleBufferRef)yCbCrSampleBuffer
//{
//    
//    CVImageBufferRef pixelBuffer = CMSampleBufferGetImageBuffer(yCbCrSampleBuffer);
//    
//    // get image size
//    size_t cols = CVPixelBufferGetWidth(pixelBuffer);
//    size_t rows = CVPixelBufferGetHeight(pixelBuffer);
//    
//    // allocate memory for RGBA image for the first time
//    if(_colorImageBuffer==NULL)
//        _colorImageBuffer = (uint8_t*)malloc(cols * rows * 4);
//    
//    // color space
//    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
//    
//    CVPixelBufferLockBaseAddress(pixelBuffer, 0);
//    
//    // get y plane
//    const uint8_t* yData = reinterpret_cast<uint8_t*> (CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0));
//    
//    // get cbCr plane
//    const uint8_t* cbCrData = reinterpret_cast<uint8_t*> (CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1));
//    
//    size_t yBytePerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0);
//    size_t cbcrBytePerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1);
//    assert( yBytePerRow==cbcrBytePerRow );
//
//    uint8_t* bgra = _colorImageBuffer;
//    
//    bool ok = convertYCbCrToBGRA(cols, rows, yData, cbCrData, bgra, 0xff, yBytePerRow, cbcrBytePerRow, 4 * cols);
//
//    if (!ok)
//    {
//        NSLog(@"YCbCr to BGRA conversion failed.");
//        return;
//    }
//
//    NSData *data = [[NSData alloc] initWithBytes:_colorImageBuffer length:rows*cols*4];
//    
//    CVPixelBufferUnlockBaseAddress(pixelBuffer, 0);
//    
//    CGBitmapInfo bitmapInfo;
//    bitmapInfo = (CGBitmapInfo)kCGImageAlphaNoneSkipFirst;
//    bitmapInfo |= kCGBitmapByteOrder32Little;
//    
//    CGDataProviderRef provider = CGDataProviderCreateWithCFData((CFDataRef)data);
//    
//    CGImageRef imageRef = CGImageCreate(
//        cols,
//        rows,
//        8,
//        8 * 4,
//        cols*4,
//        colorSpace,
//        bitmapInfo,
//        provider,
//        NULL,
//        false,
//        kCGRenderingIntentDefault
//    );
//    
//    _colorImageView.image = [[UIImage alloc] initWithCGImage:imageRef];
//    
//    CGImageRelease(imageRef);
//    CGDataProviderRelease(provider);
//    CGColorSpaceRelease(colorSpace);
//}
//
//------------------------------------------------------------------------------

#pragma mark -  AVFoundation

- (bool)queryCameraAuthorizationStatusAndNotifyUserIfNotGranted
{
    const NSUInteger numCameras = [[AVCaptureDevice devicesWithMediaType:AVMediaTypeVideo] count];
    
    if (0 == numCameras)
        return false; // This can happen even on devices that include a camera, when camera access is restricted globally.

    AVAuthorizationStatus authStatus = [AVCaptureDevice authorizationStatusForMediaType:AVMediaTypeVideo];
    
    if (authStatus != AVAuthorizationStatusAuthorized)
    {
        NSLog(@"Not authorized to use the camera!");
        
        [AVCaptureDevice requestAccessForMediaType:AVMediaTypeVideo
            completionHandler:^(BOOL granted) {
                // This block fires on a separate thread, so we need to ensure any actions here
                // are sent to the right place.

                // If the request is granted, let's try again to start an AVFoundation session. Otherwise, alert
                // the user that things won't go well.
                if (granted)
                {

                    dispatch_async(dispatch_get_main_queue(), ^(void) {

                        [self startColorCamera];

                        [self updateAppStatusMessageWithColorCameraAuthorization:true];

                    });
                }
            }
        ];
        
        return false;
    }

    return true;
    
}

- (void)setupColorCamera
{
    // Early-return if the capture session was already setup.

    if (_avCaptureSession)
        return;

    // Ensure that camera access was properly granted.
    
    bool cameraAccessAuthorized = [self queryCameraAuthorizationStatusAndNotifyUserIfNotGranted];
    
    if (!cameraAccessAuthorized)
    {
        [self updateAppStatusMessageWithColorCameraAuthorization:false];

        return;
    }

    // Set up the capture session.

    _avCaptureSession = [[AVCaptureSession alloc] init];

    [_avCaptureSession beginConfiguration];
    
    // Set preset session size.

    // Capture color frames at VGA resolution.

    [_avCaptureSession setSessionPreset:AVCaptureSessionPreset640x480];
    
    // Create a video device.

    _videoDevice = [AVCaptureDevice defaultDeviceWithMediaType:AVMediaTypeVideo];

    assert(_videoDevice != nil);

    NSError *error = nil;
    
    // Use auto-exposure, and auto-white balance and set the focus to infinity.

    if([_videoDevice lockForConfiguration:&error])
    {
        // Allow exposure to change
        if ([_videoDevice isExposureModeSupported:AVCaptureExposureModeContinuousAutoExposure])
            [_videoDevice setExposureMode:AVCaptureExposureModeContinuousAutoExposure];
        
        // Allow white balance to change
        if ([_videoDevice isWhiteBalanceModeSupported:AVCaptureWhiteBalanceModeContinuousAutoWhiteBalance])
            [_videoDevice setWhiteBalanceMode:AVCaptureWhiteBalanceModeContinuousAutoWhiteBalance];
        
        // Set focus at the maximum position allowable (e.g. "near-infinity") to get the
        // best color/depth alignment.
        [_videoDevice setFocusModeLockedWithLensPosition:1.0f completionHandler:nil];
        
        [_videoDevice unlockForConfiguration];
    }
    
    // Create the video capture device input.

    AVCaptureDeviceInput *input = [AVCaptureDeviceInput deviceInputWithDevice:_videoDevice error:&error];

    if (error)
    {
        NSLog(@"Cannot initialize AVCaptureDeviceInput");
        assert(0);
    }
    
    // Add the input to the capture session.

    [_avCaptureSession addInput:input];

    //  Create the video data output.

    AVCaptureVideoDataOutput* dataOutput = [[AVCaptureVideoDataOutput alloc] init];
    
    // We don't want to process late frames.

    [dataOutput setAlwaysDiscardsLateVideoFrames:YES];
    
    // Use kCVPixelFormatType_420YpCbCr8BiPlanarFullRange format.

    [dataOutput setVideoSettings:@{ (NSString*)kCVPixelBufferPixelFormatTypeKey:@(kCVPixelFormatType_420YpCbCr8BiPlanarFullRange) }];

    // Dispatch the capture callbacks on the main thread, where OpenGL calls can be made synchronously.

    [dataOutput setSampleBufferDelegate:self queue:dispatch_get_main_queue()];
    
    // Add the output to the capture session.

    [_avCaptureSession addOutput:dataOutput];
    
    // Enforce 30 FPS capture rate.
    
    if([_videoDevice lockForConfiguration:&error])
    {
        [_videoDevice setActiveVideoMaxFrameDuration:CMTimeMake(1, 30)];
        [_videoDevice setActiveVideoMinFrameDuration:CMTimeMake(1, 30)];
        [_videoDevice unlockForConfiguration];
    }
    
    [_avCaptureSession commitConfiguration];
}

- (void)startColorCamera
{
    if (_avCaptureSession && [_avCaptureSession isRunning])
        return;
    
    // Re-setup so focus is lock even when back from background.

    if (_avCaptureSession == nil)
        [self setupColorCamera];
    
    // Start streaming color sample buffers.

    [_avCaptureSession startRunning];
}

- (void)stopColorCamera
{
    if ([_avCaptureSession isRunning])
    {
        // Stop the session.

        [_avCaptureSession stopRunning];
    }
    
    _avCaptureSession = nil;
    _videoDevice = nil;
}

- (void)captureOutput:(AVCaptureOutput *)captureOutput didOutputSampleBuffer:(CMSampleBufferRef)sampleBuffer fromConnection:(AVCaptureConnection *)connection
{
    // Pass the sample buffer to the driver.
    // It will be returned later, paired with a synchronized depth or IR frame.

    [_sensorController frameSyncNewColorBuffer:sampleBuffer];
}

- (void) touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event {
    UITouch *touch = [touches anyObject];
    CGPoint currentPoint = [touch locationInView:self.view];
    std::cout << "touches began " << currentPoint.x << ", " << currentPoint.y << std::endl;
    NSString *coord_NSStr = [NSString stringWithFormat:@"touches began (%2.2f, %2.2f)",
                             currentPoint.x, currentPoint.y];
    coordView_.text = coord_NSStr;
    // TODO Yi: draw square around current selections
}

- (void) touchesMoved:(NSSet *)touches withEvent:(UIEvent *)event {
    UITouch *touch = [touches anyObject];
    CGPoint currentPoint = [touch locationInView:self.view];
    std::cout << "touches moved " << currentPoint.x << ", " << currentPoint.y << std::endl;
    NSString *coord_NSStr = [NSString stringWithFormat:@"touches moved (%2.2f, %2.2f)",
                             currentPoint.x, currentPoint.y];
    coordView_.text = coord_NSStr;
    // TODO Yi: draw square around current selections
    
//    UIGraphicsBeginImageContext(self.view.frame.size);
//    [self.tempDrawImage.image drawInRect:CGRectMake(0, 0, self.view.frame.size.width, self.view.frame.size.height)];
//    CGContextMoveToPoint(UIGraphicsGetCurrentContext(), lastPoint.x, lastPoint.y);
//    CGContextAddLineToPoint(UIGraphicsGetCurrentContext(), currentPoint.x, currentPoint.y);
//    CGContextSetLineCap(UIGraphicsGetCurrentContext(), kCGLineCapRound);
//    CGContextSetLineWidth(UIGraphicsGetCurrentContext(), brush);
//    CGContextSetRGBStrokeColor(UIGraphicsGetCurrentContext(), red, green, blue, 1.0);
//    CGContextSetBlendMode(UIGraphicsGetCurrentContext(), kCGBlendModeNormal);
//    
//    CGContextStrokePath(UIGraphicsGetCurrentContext());
//    self.tempDrawImage.image = UIGraphicsGetImageFromCurrentImageContext();
//    [self.tempDrawImage setAlpha:opacity];
//    UIGraphicsEndImageContext();
}

- (void) touchesEnded:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event {
    UITouch *touch = [touches anyObject];
    CGPoint currentPoint = [touch locationInView:self.view];
    std::cout << "touches ended " << currentPoint.x << ", " << currentPoint.y << std::endl;
    NSString *coord_NSStr = [NSString stringWithFormat:@"touches ended (%2.2f, %2.2f)",
                             currentPoint.x, currentPoint.y];
    coordView_.text = coord_NSStr;
    // TODO Yi: Alternatively update selected points

//    if (!mouseSwiped) {
//        UIGraphicsBeginImageContext(self.view.frame.size);
//        [self.tempDrawImage.image drawInRect:CGRectMake(0, 0, self.view.frame.size.width, self.view.frame.size.height)];
//        CGContextSetLineCap(UIGraphicsGetCurrentContext(), kCGLineCapRound);
//        CGContextSetLineWidth(UIGraphicsGetCurrentContext(), brush);
//        CGContextSetRGBStrokeColor(UIGraphicsGetCurrentContext(), red, green, blue, opacity);
//        CGContextMoveToPoint(UIGraphicsGetCurrentContext(), lastPoint.x, lastPoint.y);
//        CGContextAddLineToPoint(UIGraphicsGetCurrentContext(), lastPoint.x, lastPoint.y);
//        CGContextStrokePath(UIGraphicsGetCurrentContext());
//        CGContextFlush(UIGraphicsGetCurrentContext());
//        self.tempDrawImage.image = UIGraphicsGetImageFromCurrentImageContext();
//        UIGraphicsEndImageContext();
//    }
//    
//    UIGraphicsBeginImageContext(self.mainImage.frame.size);
//    [self.mainImage.image drawInRect:CGRectMake(0, 0, self.view.frame.size.width, self.view.frame.size.height) blendMode:kCGBlendModeNormal alpha:1.0];
//    [self.tempDrawImage.image drawInRect:CGRectMake(0, 0, self.view.frame.size.width, self.view.frame.size.height) blendMode:kCGBlendModeNormal alpha:opacity];
//    self.mainImage.image = UIGraphicsGetImageFromCurrentImageContext();
//    self.tempDrawImage.image = nil;
//    UIGraphicsEndImageContext();
//    NSLog(@"height: %f", self.view.frame.size.height);
    
}
- (void) distance:(const uint16_t*)shiftValues depthValuesCount:(size_t)depthValuesCount{
//    measureCoords[0]=2;
//    measureCoords[1]=10;
//    measureCoords[2]=2;
//    measureCoords[3]=20;
    
    uint16_t lefttop = std::min (shiftValues[0], maxShiftValue);
    // Use a lookup table to make the non-linear input values vary more linearly with metric depth
    int leftTop_linearizedDepth = _linearizeBuffer[lefttop];
    
    uint16_t rightbot = std::min (shiftValues[100], maxShiftValue);
    // Use a lookup table to make the non-linear input values vary more linearly with metric depth
    int rightBot_linearizedDepth = _linearizeBuffer[rightbot];
    std::cout << "leftTop : " << leftTop_linearizedDepth << " rightBot: " << rightBot_linearizedDepth << std::endl;
}

@end
