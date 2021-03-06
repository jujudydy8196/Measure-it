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
#include "opencv2/imgproc/imgproc.hpp"

//------------------------------------------------------------------------------
using namespace std;
namespace {
    
    int NEIGHBOUR_THRES = 20;

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
    UIImageView *_colorImageView;
    
    uint16_t *_linearizeBuffer;
    uint8_t *_coloredDepthBuffer;
//    uint8_t *_normalsBuffer;
    uint8_t *_colorImageBuffer;

//    STNormalEstimator *_normalsEstimator;
    
    UILabel* _statusLabel;
    
    
    AppStatus _appStatus;
    
    float measureCoords[4]; // [x1, y1, x2, y2]
    uint measurePtCursor;
    int depthFrameWidth;
    int depthFrameHeight;
    UIImage *interestPoints; // binary image that masks geometrically intersting points
    
    UIImageView *_selectionView; // show selected points for measurements
    int halfSquare;
    UITextView *coordView_; // For deubgging
    UITextView *distanceView_;
    
    
    vector<cv::Vec4i> lineSeg; //For houghLinesP

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
    
    // Create three image views where we will render our frames
    
    CGRect depthFrame = self.view.frame;
    depthFrame.origin.y = 1;
    depthFrame.origin.x = 1;
    
    _linearizeBuffer = NULL;
//    _coloredDepthBuffer = NULL;
//    _normalsBuffer = NULL;
    _colorImageBuffer = NULL;

    _depthImageView = [[UIImageView alloc] initWithFrame:depthFrame];
    _depthImageView.contentMode = UIViewContentModeScaleAspectFit;
    [self.view addSubview:_depthImageView];
    
//    [self setupColorCamera];

    measureCoords[0] = 0;
    measureCoords[1] = 0;
    measureCoords[2] = depthFrame.size.width-1;
    measureCoords[3] = depthFrame.size.height-1;
    measurePtCursor = 0;
    
    std::cout << "measureCoords: (" << measureCoords[0] << "," << measureCoords[1] << ") to ("
    << measureCoords[2] << "," << measureCoords[3] << ")" << std::endl;
    
    _selectionView = [[UIImageView alloc] initWithFrame:depthFrame];
    [_selectionView setOpaque:false];
    [self.view addSubview:_selectionView];
    halfSquare = 15;

    coordView_ = [[UITextView alloc] initWithFrame:CGRectMake(0,15,self.view.frame.size.width,35)];
    [coordView_ setOpaque:false];
    [coordView_ setBackgroundColor:[UIColor clearColor]]; // Set background color to be clear
    [coordView_ setTextColor:[UIColor lightGrayColor]]; // Set text to be RED
    [coordView_ setFont:[UIFont systemFontOfSize:18]]; // Set the Font size
    [self.view addSubview:coordView_];
    [self.view bringSubviewToFront:coordView_] ;
    
    distanceView_ = [[UITextView alloc] initWithFrame:CGRectMake(0,50,self.view.frame.size.width,35)];
    [distanceView_ setOpaque:false];
    [distanceView_ setBackgroundColor:[UIColor clearColor]]; // Set background color to be clear
    [distanceView_ setTextColor:[UIColor lightGrayColor]]; // Set text to be RED
    [distanceView_ setFont:[UIFont systemFontOfSize:18]]; // Set the Font size
    [distanceView_ setHidden:true];
    [self.view addSubview:distanceView_];
    [self.view bringSubviewToFront:distanceView_] ;

}

- (void)dealloc
{
    free(_linearizeBuffer);
    free(_coloredDepthBuffer);
//    free(_normalsBuffer);
    free(_colorImageBuffer);
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
    if (depthFrameWidth == 0 || depthFrameHeight == 0){
        depthFrameWidth = depthFrame.width;
        depthFrameHeight = depthFrame.height;
    }
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
    [self renderColorFrame:colorFrame.sampleBuffer];
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
    [self distance:depthFrame];
    
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
//    _depthImageView.image = [UIImage imageWithCGImage:imageRef];
    
    // Find geometrically interesting points
//    interestPoints = [self findInterstPoints:[UIImage imageWithCGImage:imageRef]];

//    interestPoints = [self findInterestEdges:[UIImage imageWithCGImage:imageRef]];
    
    // TODO: Judy
    interestPoints = [self findInterestEdgesSeg:[UIImage imageWithCGImage:imageRef]];

    _depthImageView.image = interestPoints;
    
    
    
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

- (void)renderColorFrame:(CMSampleBufferRef)yCbCrSampleBuffer
{
    
    CVImageBufferRef pixelBuffer = CMSampleBufferGetImageBuffer(yCbCrSampleBuffer);
    
    // get image size
    size_t cols = CVPixelBufferGetWidth(pixelBuffer);
    size_t rows = CVPixelBufferGetHeight(pixelBuffer);
    
    // allocate memory for RGBA image for the first time
    if(_colorImageBuffer==NULL)
        _colorImageBuffer = (uint8_t*)malloc(cols * rows * 4);
    
    // color space
    CGColorSpaceRef colorSpace = CGColorSpaceCreateDeviceRGB();
    
    CVPixelBufferLockBaseAddress(pixelBuffer, 0);
    
    // get y plane
    const uint8_t* yData = reinterpret_cast<uint8_t*> (CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0));
    
    // get cbCr plane
    const uint8_t* cbCrData = reinterpret_cast<uint8_t*> (CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1));
    
    size_t yBytePerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0);
    size_t cbcrBytePerRow = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1);
    assert( yBytePerRow==cbcrBytePerRow );

    uint8_t* bgra = _colorImageBuffer;
    
    bool ok = convertYCbCrToBGRA(cols, rows, yData, cbCrData, bgra, 0xff, yBytePerRow, cbcrBytePerRow, 4 * cols);

    if (!ok)
    {
        NSLog(@"YCbCr to BGRA conversion failed.");
        return;
    }

    NSData *data = [[NSData alloc] initWithBytes:_colorImageBuffer length:rows*cols*4];
    
    CVPixelBufferUnlockBaseAddress(pixelBuffer, 0);
    
    CGBitmapInfo bitmapInfo;
    bitmapInfo = (CGBitmapInfo)kCGImageAlphaNoneSkipFirst;
    bitmapInfo |= kCGBitmapByteOrder32Little;
    
    CGDataProviderRef provider = CGDataProviderCreateWithCFData((CFDataRef)data);
    
    CGImageRef imageRef = CGImageCreate(
        cols,
        rows,
        8,
        8 * 4,
        cols*4,
        colorSpace,
        bitmapInfo,
        provider,
        NULL,
        false,
        kCGRenderingIntentDefault
    );
    
//    _colorImageView.image = [[UIImage alloc] initWithCGImage:imageRef];
//    _depthImageView.image = [[UIImage alloc] initWithCGImage:imageRef];
    // Find geometrically interesting points
//    _depthImageView.image = [self findInterstPoints:[UIImage imageWithCGImage:imageRef]];

//    std::cout <<"rendering color to depthImageView" << std::endl;
  
    CGImageRelease(imageRef);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(colorSpace);
}

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

- (void) updateNextPoint: (CGPoint) p {
    // Update the measurement point specified by cursor with given CGPoint and update cursor.
    if (depthFrameWidth == 0 || depthFrameHeight == 0){
        std::cout << "Depth Frame size not found" << std::endl;
        measureCoords[measurePtCursor*2] = p.x;
        measureCoords[measurePtCursor*2+1] = p.y;
    } else {
        // normalize to depth frame resolution
        measureCoords[measurePtCursor*2] = p.x * (depthFrameWidth / self.view.frame.size.width);
        measureCoords[measurePtCursor*2+1] = p.y * (depthFrameHeight / self.view.frame.size.height);
    }
    
    measurePtCursor = (measurePtCursor + 1) % 2;
    
    std::cout << "(" << measureCoords[0] << ", " << measureCoords[1] << ") to ("
    << measureCoords[2] << "," << measureCoords[3] << ")" << std::endl;
}

- (CGPoint) findInterstPointNear: (CGPoint)selectedPoint within: (int)nearThresh{
    // reads interestPoint
    // nearThresh defines the size of neighborhood to search for an interest point
    
    // TODO Yi: find nearest interest point within neighborhood threshold
    std::cout << "TODO Yi: find nearest interest point within neighborhood threshold" << std::endl;
    return selectedPoint;
}

- (void) touchesBegan:(NSSet *)touches withEvent:(UIEvent *)event {
    UITouch *touch = [touches anyObject];
    CGPoint currentPoint = [touch locationInView:self.view];
//    currentPoint = [self findInterstPointNearEdge:currentPoint within: NEIGHBOUR_THRES];
//    int lineIdx  = [self findInterstPointNearEdge:currentPoint within: NEIGHBOUR_THRES];

    std::cout << "touches began " << currentPoint.x << ", " << currentPoint.y << std::endl;
    NSString *coord_NSStr = [NSString stringWithFormat:@"touches began (%2.2f, %2.2f)",
                             currentPoint.x, currentPoint.y];
    coordView_.text = coord_NSStr;
    

    
    
    // Yi: draw square around current selections
    UIGraphicsBeginImageContextWithOptions(_selectionView.frame.size, false, 1.0f);
    CGContextRef context = UIGraphicsGetCurrentContext();
    CGRect selectedRect = CGRectMake(currentPoint.x - halfSquare, currentPoint.y - halfSquare,
                                     halfSquare*2, halfSquare*2);
    CGContextSetRGBStrokeColor(context, 1.0, 1.0, 0.0, 1);
    CGContextSetLineWidth(context, 2.0);
    CGContextSetRGBFillColor(context, 1.0, 1.0, 0.0, 1);
    CGContextFillEllipseInRect(context, selectedRect);
    CGContextFillPath(context);
    
//    CGContextStrokeRect(context, selectedRect);
    
    if (!measurePtCursor) {
        std::cout << "add another in touch begin" << std::endl;
        CGRect prevRect = CGRectMake(measureCoords[0] - halfSquare, measureCoords[1] - halfSquare,
                                     halfSquare*2, halfSquare*2);
        CGContextSetRGBStrokeColor(context, 1.0, 0.0, 0.0, 1);
        CGContextSetLineWidth(context, 2.0);
        CGContextSetRGBFillColor(context, 1.0, 1.0, 0.0, 1);
        CGContextFillEllipseInRect(context, prevRect);
        CGContextFillPath(context);
//        CGContextStrokeRect(context, prevRect);
    }
    
//    CGContextSetLineWidth(context,3.0f);
//    /* Start the line at this point */
//    CGContextMoveToPoint(context,lineSeg[lineIdx][0], lineSeg[lineIdx][1]);
//    /* And end it at this point */
//    CGContextAddLineToPoint(context,lineSeg[lineIdx][2], lineSeg[lineIdx][3]);
//    /* Use the context's current color to draw the line */
//    CGContextSetRGBStrokeColor(context, 1.0, 0.0, 0.0, 1);
//    CGContextStrokePath(context);
    
    _selectionView.image = UIGraphicsGetImageFromCurrentImageContext();
    UIGraphicsEndImageContext();
    
}

- (void) touchesMoved:(NSSet *)touches withEvent:(UIEvent *)event {
    UITouch *touch = [touches anyObject];
    CGPoint currentPoint = [touch locationInView:self.view];
//    currentPoint = [self findInterstPointNearEdge:currentPoint within: NEIGHBOUR_THRES];
//    int lineIdx  = [self findInterstPointNearEdge:currentPoint within: NEIGHBOUR_THRES];

    std::cout << "touches moved " << currentPoint.x << ", " << currentPoint.y << std::endl;
    NSString *coord_NSStr = [NSString stringWithFormat:@"touches moved (%2.2f, %2.2f)",
                             currentPoint.x, currentPoint.y];
    coordView_.text = coord_NSStr;
    
    // Draw square around current selections
    UIGraphicsBeginImageContextWithOptions(_selectionView.frame.size, false, 1.0f);
    CGContextRef context = UIGraphicsGetCurrentContext();
    CGRect selectedRect = CGRectMake(currentPoint.x - halfSquare, currentPoint.y - halfSquare,
                                    halfSquare*2, halfSquare*2);
    CGContextSetRGBStrokeColor(context, 1.0, 1.0, 0.0, 1);
    CGContextSetLineWidth(context, 2.0);
    CGContextSetRGBFillColor(context, 1.0, 1.0, 0.0, 1);
    CGContextFillEllipseInRect(context, selectedRect);
    CGContextFillPath(context);
//    CGContextStrokeRect(context, selectedRect);
    if (measurePtCursor) {
        std::cout << "add another in touch move" << std::endl;
        CGRect prevRect = CGRectMake(measureCoords[0] - halfSquare, measureCoords[1] - halfSquare,
                                     halfSquare*2, halfSquare*2);
        CGContextSetRGBStrokeColor(context, 1.0, 0.0, 0.0, 1);
        CGContextSetLineWidth(context, 2.0);
        CGContextSetRGBFillColor(context, 1.0, 1.0, 0.0, 1);
        CGContextFillEllipseInRect(context, prevRect);
        CGContextFillPath(context);
//        CGContextStrokeRect(context, prevRect);
        CGContextSetLineWidth(context,3.0f);
        /* Start the line at this point */
        CGContextMoveToPoint(context,measureCoords[0], measureCoords[1]);
        /* And end it at this point */
        CGContextAddLineToPoint(context,currentPoint.x, currentPoint.y);
        /* Use the context's current color to draw the line */
        CGContextStrokePath(context);
    }
    
//    CGContextSetLineWidth(context,3.0f);
//    /* Start the line at this point */
//    CGContextMoveToPoint(context,lineSeg[lineIdx][0], lineSeg[lineIdx][1]);
//    /* And end it at this point */
//    CGContextAddLineToPoint(context,lineSeg[lineIdx][2], lineSeg[lineIdx][3]);
//    /* Use the context's current color to draw the line */
//    CGContextSetRGBStrokeColor(context, 1.0, 0.0, 0.0, 1);
//    CGContextStrokePath(context);
    
    _selectionView.image = UIGraphicsGetImageFromCurrentImageContext();
    UIGraphicsEndImageContext();
}

- (void) touchesEnded:(NSSet<UITouch *> *)touches withEvent:(UIEvent *)event {
    UITouch *touch = [touches anyObject];
    CGPoint currentPoint = [touch locationInView:self.view];
    std::cout << "before snap: (" << currentPoint.x << "," << currentPoint.y << ")" << std::endl;
    currentPoint = [self findInterstPointNearEdge:currentPoint within: NEIGHBOUR_THRES];
    std::cout << "after snap: (" << currentPoint.x << "," << currentPoint.y << ")" << std::endl;

    
//    int lineIdx = [self findInterstPointNearEdge:currentPoint within: NEIGHBOUR_THRES];

    
    // Alternatively update selected points
    [self updateNextPoint:currentPoint];
    std::cout << "touches ended (" << measureCoords[0] << ", " << measureCoords[1] << ") to ("
                << measureCoords[2] << "," << measureCoords[3] << ")" << std::endl;
    NSString *coord_NSStr = [NSString stringWithFormat:@"measuring from (%2.2f, %2.2f) to (%2.2f, %2.2f)",
                             measureCoords[0], measureCoords[1], measureCoords[2], measureCoords[3]];
    coordView_.text = coord_NSStr;
    
    // Draw square around current selections
    UIGraphicsBeginImageContextWithOptions(_selectionView.frame.size, false, 1.0f);
    CGContextRef context = UIGraphicsGetCurrentContext();
    CGRect selectedRect = CGRectMake(currentPoint.x - halfSquare, currentPoint.y - halfSquare,
                                     halfSquare*2, halfSquare*2);
    CGContextSetRGBStrokeColor(context, 1.0, 1.0, 0.0, 1);
    CGContextSetLineWidth(context, 2.0);
    CGContextSetRGBFillColor(context, 1.0, 1.0, 0.0, 1);
    CGContextFillEllipseInRect(context, selectedRect);
    CGContextFillPath(context);
//    CGContextStrokeRect(context, selectedRect);
    
    if (!measurePtCursor) {
        std::cout << "add another in touch end" << std::endl;
        CGRect prevRect = CGRectMake(measureCoords[0] - halfSquare, measureCoords[1] - halfSquare,
                                     halfSquare*2, halfSquare*2);
        CGContextSetRGBStrokeColor(context, 1.0, 0.0, 0.0, 1);
        CGContextSetLineWidth(context, 2.0);
        CGContextSetRGBFillColor(context, 1.0, 1.0, 0.0, 1);
        CGContextFillEllipseInRect(context, prevRect);
        CGContextFillPath(context);
//        CGContextStrokeRect(context, prevRect);
        
        CGContextSetLineWidth(context,3.0f);
        /* Start the line at this point */
        CGContextMoveToPoint(context,measureCoords[0], measureCoords[1]);
        /* And end it at this point */
        CGContextAddLineToPoint(context,measureCoords[2], measureCoords[3]);
        /* Use the context's current color to draw the line */
        CGContextStrokePath(context);
    }
    
//    if (lineIdx != -1) {
//        CGContextSetLineWidth(context,3.0f);
//        /* Start the line at this point */
//        std::cout << "line idx: " << lineIdx << std::endl;
//        CGContextMoveToPoint(context,lineSeg[lineIdx][0]*3.2, lineSeg[lineIdx][1]*3.2);
//        /* And end it at this point */
//        CGContextAddLineToPoint(context,lineSeg[lineIdx][2]*3.2, lineSeg[lineIdx][3]*3.2);
//        /* Use the context's current color to draw the line */
//        CGContextSetRGBStrokeColor(context, 1.0, 1.0, 1.0, 2);
//        CGContextStrokePath(context);
//    }
    
    _selectionView.image = UIGraphicsGetImageFromCurrentImageContext();
    UIGraphicsEndImageContext();
}

// Assume the following intrinsics, from the Structure SDK docs
// K_RGB_QVGA       = [305.73, 0, 159.69; 0, 305.62, 119.86; 0, 0, 1]
#define QVGA_COLS 320
#define QVGA_ROWS 240
#define QVGA_F_X 305.73
#define QVGA_F_Y 305.62
#define QVGA_C_X 159.69
#define QVGA_C_Y 119.86

- (void) distance:(STDepthFrame *)depthFrame{//:(const uint16_t*)shiftValues depthValuesCount:(size_t)depthValuesCount{
    // TODO_judy: get intrinsic, get 3D points, get distance

    float _fx = QVGA_F_X/QVGA_COLS*depthFrame.width;
    float _fy = QVGA_F_X/QVGA_ROWS*depthFrame.height;
    float _cx = QVGA_C_X/QVGA_COLS*depthFrame.width;
    float _cy = QVGA_C_Y/QVGA_ROWS*depthFrame.height;
    
//    std::cout <<depthFrame.width << " " << depthFrame.height << std::endl;
    
    // center distance debug
//    int centerC=depthFrame.width/2, centerR=depthFrame.height/2;
//    int imgC=1024/2, imgR=768/2;
//    int pointIndex = centerR*depthFrame.width+centerC;
//    float depth=depthFrame.depthInMillimeters[pointIndex];
//    float x=depth * (imgC - _cx) / _fx;
//    float y=depth * (_cy - imgR) / _fy;
//    float z=depth;
//    std::cout << "3D pt1: (" << x << "," << y << "," << z << ")" << std::endl;
//    std::cout << "measure Coords: " << measureCoords[0] << " " << measureCoords[1] << " " << measureCoords[2] << " " << measureCoords[3] << std::endl;
    
    int r1=measureCoords[1]/3.2, c1=measureCoords[0]/3.2;
    int pointIndex1 = r1*depthFrame.width + c1;
//    std::cout << "point1 coord: " << c1 << " " << r1 << std::endl;

//    NSLog(@"Pt1 shift: %d depth: %f", depthFrame.shiftData[pointIndex1], depthFrame.depthInMillimeters[pointIndex1]);
    
    float depth1=depthFrame.depthInMillimeters[pointIndex1];
    float x1=depth1 * (c1 - _cx) / _fx;
    float y1=depth1 * (_cy - r1) / _fy;
    float z1=depth1;
//    std::cout << "3D pt1: (" << x1 << "," << y1<< "," << z1 << ")" << std::endl;

    
    int r2=measureCoords[3]/3.2, c2=measureCoords[2]/3.2;
    int pointIndex2 = r2*depthFrame.width + c2;
//    std::cout << "point2 coord: " << c2 << " " << r2 << std::endl;

    
    float depth2=depthFrame.depthInMillimeters[pointIndex2];
    float x2=depth2 * (c2 - _cx) / _fx;
    float y2=depth2 * (_cy - r2) / _fy;
    float z2=depth2;
//    std::cout << "3D pt2: (" << x2 << "," << y2<< "," << z2 << ")" << std::endl;
    
    float dist= sqrt(pow(x2-x1,2)+pow(y2-y1,2)+pow(z2-z1,2));
//    std::cout << "distance: " << dist/10.0 << std::endl;
    NSString *distance_NSStr = [NSString stringWithFormat:@"Distance: %2.2f", dist/10.0];
    distanceView_.text = distance_NSStr;
    [distanceView_ setHidden:false];
    
//    NSLog(@"Central shift: %d depth: %f", depthFrame.shiftData[160*340+120], depthFrame.depthInMillimeters[160*340+120]);
    

    
}

- (UIImage *) findInterstPoints: (UIImage *)depthImage {
    // Harris corner detector
    cv::Mat cvImage = [self cvMatFromUIImage:depthImage];
    cv::Mat gray; cv::cvtColor(cvImage, gray, CV_RGBA2GRAY); // Convert to grayscale
    cv::Mat dst, dst_norm, dst_norm_scaled;
    dst = cv::Mat::zeros( gray.size(), CV_32FC1 );
    
    /// Detector parameters
    int blockSize = 2;
    int apertureSize = 3;
    double k = 0.04;
    int thresh = 200;
    
    /// Detecting corners
    cv::cornerHarris( gray, dst, blockSize, apertureSize, k, cv::BORDER_DEFAULT );
    
    /// Normalizing
    cv::normalize( dst, dst_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat() );
//    cv::convertScaleAbs( dst_norm, dst_norm_scaled );
    
    /// Drawing a circle around corners
    for( int j = 0; j < dst_norm.rows ; j++ ){
        for( int i = 0; i < dst_norm.cols; i++ ){
            if( (int) dst_norm.at<float>(j,i) > thresh ){
                cv::circle( cvImage, cv::Point( i, j ), 5,  cv::Scalar(255, 0, 0), 2, 8, 0 );
            }
        }
    }
    
    return [self UIImageFromCVMat:cvImage];

}

- (CGFloat)distanceToLineSeg:(CGPoint)P toLineWithPointA:(CGPoint)A andPointB:(CGPoint)B {
    if ((P.x - A.x) * (B.x - A.x) + (P.y - A.y) * (B.y - A.y) < 0)
        return sqrt(pow((P.x - A.x), 2) + pow((P.y - A.y), 2));
    
    if ((P.x - B.x) * (A.x - B.x) + (P.y - B.y) * (A.y - B.y) < 0)
        return sqrt(pow(P.x - B.x, 2) + pow(P.y - B.y, 2));
    
    return ABS((P.x * (A.y - B.y) + P.y * (B.x - A.x) + (A.x * B.y - B.x * A.y)) / sqrt(pow(B.x - A.x, 2) + pow(B.y - A.y, 2)));
}

- (CGPoint) findInterstPointNearEdge: (CGPoint)selectedPoint within: (int)nearThresh{
    // nearThresh defines the size of neighborhood to search for an interest point
    
//     TODO Judy: find point snap to edge within circle (thres)
//    std::cout << "number of lines detected: "  << lineSeg.size() << std::endl;
    std::cout << "point need to snap " << selectedPoint.x << " " << selectedPoint.y << std::endl;
    CGFloat minDist = CGFLOAT_MAX;
    int minIdx=-1;
    for (int i=1; i<lineSeg.size(); i++) {
        CGPoint a = CGPointMake(lineSeg[i][0]*3.2, lineSeg[i][1]*3.2);
        CGPoint b = CGPointMake(lineSeg[i][2]*3.2, lineSeg[i][3]*3.2);
        std::cout << "from : (" << a.x << "," << a.y << ") to (" << b.x << "," << b.y << ")" << std::endl;
        
//        CGFloat dist = [distanceToLine]
        CGFloat dist = [self distanceToLineSeg:selectedPoint toLineWithPointA:a andPointB:b ] ;
        std::cout << "dist to line " << i << " : " << dist << std::endl;
        if (minDist>dist) {
            minDist=dist;
            minIdx=i;
        }
    }
    if (minIdx==-1)
        return selectedPoint;
    
    cv::Point2f pt1 (lineSeg[minIdx][0]*3.2, lineSeg[minIdx][1]*3.2);
    cv::Point2f pt2 (lineSeg[minIdx][2]*3.2, lineSeg[minIdx][3]*3.2);
    cv::Point2f p (selectedPoint.x, selectedPoint.y);
    cv::Point2f a = p-pt1, b = pt2-pt1;
    
//    cv::Point2f a(2.0,2.8);
//    cv::Point2f b(3.3,2.7);
    std::cout << "vec A: " << a << " vec B: " << b << " dot: " << a.ddot(b) << " norm a: " << cv::norm(a) << std::endl;
    float cos = a.ddot(b) / (cv::norm(a)*cv::norm(b));
    float bprimeLen = cv::norm(a) * cos;
    float scale = bprimeLen / cv::norm(b);
    b *= scale;
    CGPoint nearestpt = CGPointMake(pt1.x+b.x, pt1.y+b.y);
    
//    return (minDist<nearThresh) ? minIdx : -1;
    return nearestpt;
//    cv::Mat cvImage = [self cvMatFromUIImage:depthImage];
//    cv::line( cvImage, cv::Point(lineSeg[i][0], lineSeg[i][1]),
//             cv::Point(lineSeg[i][2], lineSeg[i][3]), cv::Scalar(0,0,255), 3, 8 );
//    std::cout << minIdx << std::endl;
//    return minIdx;
}

- (UIImage *)findInterestEdges: (UIImage *)depthImage {
    
    cv::Mat cvImage = [self cvMatFromUIImage:depthImage];
    cv::Mat dst, cdst;
    cv::Canny(cvImage, dst, 50, 200, 3);
    cv::cvtColor(dst, cdst, CV_GRAY2BGR);
    
    vector<cv::Vec2f> lines;
    cv::HoughLines(dst, lines, 1, CV_PI/180, 75, 0, 0 );
    
    
    
    for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0], theta = lines[i][1];
        cv::Point pt1, pt2;
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        pt1.x = cvRound(x0 + 1000*(-b));
        pt1.y = cvRound(y0 + 1000*(a));
        pt2.x = cvRound(x0 - 1000*(-b));
        pt2.y = cvRound(y0 - 1000*(a));
//        cv::line( cvImage, pt1, pt2, cv::Scalar(0,0,255), 3, CV_AA);
        cv::line( cvImage, pt1, pt2, cv::Scalar(0,0,255), 1, CV_AA);

    }
    return [self UIImageFromCVMat:cvImage];
}

- (UIImage *)findInterestEdgesSeg: (UIImage *)depthImage {
    
    cv::Mat cvImage = [self cvMatFromUIImage:depthImage];
    cv::Mat dst, cdst;
    cv::Canny(cvImage, dst, 50, 200, 3);
    
//    cv::cvtColor(dst, cdst, CV_GRAY2BGR);
    cv::HoughLinesP(dst, lineSeg, 1, CV_PI/180, 75, 10, 10 );
    
    
    
    for( size_t i = 0; i < lineSeg.size(); i++ )
    {
        cv::line( cvImage, cv::Point(lineSeg[i][0], lineSeg[i][1]),
                 cv::Point(lineSeg[i][2], lineSeg[i][3]), cv::Scalar(0,0,255), 3, 8 );
        
    }
    return [self UIImageFromCVMat:cvImage];
}

//---------------- Provided functions from class ------------------
// Member functions for converting from cvMat to UIImage
- (cv::Mat)cvMatFromUIImage:(UIImage *)image
{
    CGColorSpaceRef colorSpace = CGImageGetColorSpace(image.CGImage);
    CGFloat cols = image.size.width;
    CGFloat rows = image.size.height;
    
    cv::Mat cvMat(rows, cols, CV_8UC4); // 8 bits per component, 4 channels (color channels + alpha)
    
    CGContextRef contextRef = CGBitmapContextCreate(cvMat.data,                 // Pointer to  data
                                                    cols,                       // Width of bitmap
                                                    rows,                       // Height of bitmap
                                                    8,                          // Bits per component
                                                    cvMat.step[0],              // Bytes per row
                                                    colorSpace,                 // Colorspace
                                                    kCGImageAlphaNoneSkipLast |
                                                    kCGBitmapByteOrderDefault); // Bitmap info flags
    
    CGContextDrawImage(contextRef, CGRectMake(0, 0, cols, rows), image.CGImage);
    CGContextRelease(contextRef);
    
    return cvMat;
}
// Member functions for converting from UIImage to cvMat
-(UIImage *)UIImageFromCVMat:(cv::Mat)cvMat
{
    NSData *data = [NSData dataWithBytes:cvMat.data length:cvMat.elemSize()*cvMat.total()];
    CGColorSpaceRef colorSpace;
    
    if (cvMat.elemSize() == 1) {
        colorSpace = CGColorSpaceCreateDeviceGray();
    } else {
        colorSpace = CGColorSpaceCreateDeviceRGB();
    }
    
    CGDataProviderRef provider = CGDataProviderCreateWithCFData((__bridge CFDataRef)data);
    
    // Creating CGImage from cv::Mat
    CGImageRef imageRef = CGImageCreate(cvMat.cols,                                 //width
                                        cvMat.rows,                                 //height
                                        8,                                          //bits per component
                                        8 * cvMat.elemSize(),                       //bits per pixel
                                        cvMat.step[0],                            //bytesPerRow
                                        colorSpace,                                 //colorspace
                                        kCGImageAlphaNone|kCGBitmapByteOrderDefault,// bitmap info
                                        provider,                                   //CGDataProviderRef
                                        NULL,                                       //decode
                                        false,                                      //should interpolate
                                        kCGRenderingIntentDefault                   //intent
                                        );
    
    
    // Getting UIImage from CGImage
    UIImage *finalImage = [UIImage imageWithCGImage:imageRef];
    CGImageRelease(imageRef);
    CGDataProviderRelease(provider);
    CGColorSpaceRelease(colorSpace);
    
    return finalImage;
}




@end
