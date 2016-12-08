/*
 This file is part of the Structure SDK.
 Copyright © 2016 Occipital, Inc. All rights reserved.
 http://structure.io
 */

#import "ViewController.h"
#import "ViewController+OpenGL.h"

#include <cmath>
#include <limits>
#include <iostream>

using namespace std;

@implementation ViewController (OpenGL)

#pragma mark -  OpenGL

- (void)setupGL
{
    // Create an EAGLContext for our EAGLView.
    _display.context = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
    if (!_display.context) { NSLog(@"Failed to create ES context"); return; }
    
    [EAGLContext setCurrentContext:_display.context];
    [(EAGLView*)self.view setContext:_display.context];
    [(EAGLView*)self.view setFramebuffer];
    
    _display.yCbCrTextureShader = [[STGLTextureShaderYCbCr alloc] init];
    _display.rgbaTextureShader = [[STGLTextureShaderRGBA alloc] init];
    
    // Set up a textureCache for images output by the color camera.
    CVReturn texError = CVOpenGLESTextureCacheCreate(kCFAllocatorDefault, NULL, _display.context, NULL, &_display.videoTextureCache);
    if (texError) { NSLog(@"Error at CVOpenGLESTextureCacheCreate %d", texError); return; }
    
    // Scanning volume feedback.
    {
        // We configured the sensor for QVGA depth.
        const int w = 320, h = 240;
        
        // Create the RGBA buffer to store the feedback pixels.
        _display.scanningVolumeFeedbackBuffer.resize (w*h*4, 0);
        
        // Create the GL texture to display the feedback.
        glGenTextures(1, &_display.scanningVolumeFeedbackTexture);
        glBindTexture(GL_TEXTURE_2D, _display.scanningVolumeFeedbackTexture);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_NEAREST);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, w, h, 0, GL_RGBA, GL_UNSIGNED_BYTE, NULL);
    }
    
    // YI
    _graphicsRenderer = 0;
    _graphicsRenderer = new GraphicsRenderer(@"measurementTape.png");
    _graphicsRenderer->initializeGL();
}

- (void)setupGLViewport
{
    const float vgaAspectRatio = 640.0f/480.0f;
    
    // Helper function to handle float precision issues.
    auto nearlyEqual = [] (float a, float b) { return std::abs(a-b) < std::numeric_limits<float>::epsilon(); };
    
    CGSize frameBufferSize = [(EAGLView*)self.view getFramebufferSize];
    
    float imageAspectRatio = 1.0f;
    
    float framebufferAspectRatio = frameBufferSize.width/frameBufferSize.height;
    
    // The iPad's diplay conveniently has a 4:3 aspect ratio just like our video feed.
    // Some iOS devices need to render to only a portion of the screen so that we don't distort
    // our RGB image. Alternatively, you could enlarge the viewport (losing visual information),
    // but fill the whole screen.
    if (!nearlyEqual (framebufferAspectRatio, vgaAspectRatio))
        imageAspectRatio = 480.f/640.0f;
    
    _display.viewport[0] = 0;
    _display.viewport[1] = 0;
    _display.viewport[2] = frameBufferSize.width*imageAspectRatio;
    _display.viewport[3] = frameBufferSize.height;
}

- (void)uploadGLColorTexture:(STColorFrame*)colorFrame
{
    _display.colorCameraGLProjectionMatrix = [colorFrame glProjectionMatrix];
    
    if (!_display.videoTextureCache)
    {
        NSLog(@"Cannot upload color texture: No texture cache is present.");
        return;
    }
    
    // Clear the previous color texture.
    if (_display.lumaTexture)
    {
        CFRelease (_display.lumaTexture);
        _display.lumaTexture = NULL;
    }
    
    // Clear the previous color texture.
    if (_display.chromaTexture)
    {
        CFRelease (_display.chromaTexture);
        _display.chromaTexture = NULL;
    }
    
    // Allow the texture cache to do internal cleanup.
    CVOpenGLESTextureCacheFlush(_display.videoTextureCache, 0);
    
    CVImageBufferRef pixelBuffer = CMSampleBufferGetImageBuffer(colorFrame.sampleBuffer);
    size_t width = CVPixelBufferGetWidth(pixelBuffer);
    size_t height = CVPixelBufferGetHeight(pixelBuffer);
    
    OSType pixelFormat = CVPixelBufferGetPixelFormatType (pixelBuffer);
    NSAssert(pixelFormat == kCVPixelFormatType_420YpCbCr8BiPlanarFullRange, @"YCbCr is expected!");
    
    // Activate the default texture unit.
    glActiveTexture (GL_TEXTURE0);
    
    // Create an new Y texture from the video texture cache.
    CVReturn err = CVOpenGLESTextureCacheCreateTextureFromImage(kCFAllocatorDefault,
                                                                _display.videoTextureCache,
                                                                pixelBuffer,
                                                                NULL,
                                                                GL_TEXTURE_2D,
                                                                GL_RED_EXT,
                                                                (int)width,
                                                                (int)height,
                                                                GL_RED_EXT,
                                                                GL_UNSIGNED_BYTE,
                                                                0,
                                                                &_display.lumaTexture);
    
    if (err)
    {
        NSLog(@"Error with CVOpenGLESTextureCacheCreateTextureFromImage: %d", err);
        return;
    }
    
    // Set good rendering properties for the new texture.
    glBindTexture(CVOpenGLESTextureGetTarget(_display.lumaTexture), CVOpenGLESTextureGetName(_display.lumaTexture));
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    
    // Activate the default texture unit.
    glActiveTexture (GL_TEXTURE1);
    // Create an new CbCr texture from the video texture cache.
    err = CVOpenGLESTextureCacheCreateTextureFromImage(kCFAllocatorDefault,
                                                       _display.videoTextureCache,
                                                       pixelBuffer,
                                                       NULL,
                                                       GL_TEXTURE_2D,
                                                       GL_RG_EXT,
                                                       (int)width/2,
                                                       (int)height/2,
                                                       GL_RG_EXT,
                                                       GL_UNSIGNED_BYTE,
                                                       1,
                                                       &_display.chromaTexture);
    if (err)
    {
        NSLog(@"Error with CVOpenGLESTextureCacheCreateTextureFromImage: %d", err);
        return;
    }
    
    glBindTexture(CVOpenGLESTextureGetTarget(_display.chromaTexture), CVOpenGLESTextureGetName(_display.chromaTexture));
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
}

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


//#define halfSquare 15;
- (void)renderSceneWithDepthFrame:(STDepthFrame*)depthFrame colorFrame:(STColorFrame*)colorFrame
{
    // TODO: JUDY line seg
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
//    [self distance:depthFrame];
    
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
    UIImage* edgeSegView = [self findInterestEdgesSeg:[UIImage imageWithCGImage:imageRef]];
//    UIGraphicsBeginImageContext(self.measureView.frame.size);
//    [edgeSegView drawInRect:CGRectMake(0, 0, self.measureView.frame.size.width, self.measureView.frame.size.height)];
//    self.measureView.image = edgeSegView;
    
    
    // Activate our view framebuffer.
    [(EAGLView *)self.view setFramebuffer];
    
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);
    glClear(GL_DEPTH_BUFFER_BIT);
    
    glViewport (_display.viewport[0], _display.viewport[1], _display.viewport[2], _display.viewport[3]);
    
    switch (_slamState.roomCaptureState)
    {
        case RoomCaptureStatePoseInitialization:
        {
            // Render the background image from the color camera.
            [self renderColorImage];
            
            // Render the feedback overlay to tell us if we are inside the scanning volume.
            [self renderScanningVolumeFeedbackOverlayWithDepthFrame:depthFrame colorFrame:colorFrame];
            
            break;
        }
            
        case RoomCaptureStateScanning:
        {
            // Render the background image from the color camera.
            [self renderColorImage];
            
            GLKMatrix4 depthCameraPose = [_slamState.tracker lastFrameCameraPose];
            GLKMatrix4 cameraGLProjection = _display.colorCameraGLProjectionMatrix;
            
            // In case we are not using registered depth.
            GLKMatrix4 colorCameraPoseInDepthCoordinateSpace;
            [depthFrame colorCameraPoseInDepthCoordinateFrame:colorCameraPoseInDepthCoordinateSpace.m];
            
            // colorCameraPoseInWorld
            GLKMatrix4 cameraViewpoint = GLKMatrix4Multiply(depthCameraPose, colorCameraPoseInDepthCoordinateSpace);
            
            // Render the current mesh reconstruction using the last estimated camera pose.
            [_slamState.scene renderMeshFromViewpoint:cameraViewpoint
                                   cameraGLProjection:cameraGLProjection
                                                alpha:1.0
                             highlightOutOfRangeDepth:false
                                            wireframe:true];
            
            break;
        }
            
            // MeshViewerController handles this.
        default: {}
    };
    
    // TODO: YI render cubes for tracked points
    switch (_measure.mstatus){
        
        case Measurements::MeasureNoPoint:{
            // Render both points and line from last measurement if distance is not NAN
            UIGraphicsBeginImageContextWithOptions(self.measureView.frame.size, false, 1.0f);
            if (_measure.distance != NAN){
//                GLKMatrix4 currentModelView = GLKMatrix4Identity;
//                GLKMatrix4 currentProjection = _display.colorCameraGLProjectionMatrix;
//                _graphicsRenderer->renderLine(_measure.pt1, _measure.pt2, currentProjection, currentModelView, 1); //_circle1.frame.origin.x <_circle2.frame.origin.x)
                
                CGPoint sp1 = [self point3dToScreen:_measure.pt1];
                CGPoint sp2 = [self point3dToScreen:_measure.pt2];
                
                int halfSquare = 15;
                CGContextRef context = UIGraphicsGetCurrentContext();
                CGContextSetRGBStrokeColor(context, 1.0, 1.0, 0.0, 1);
                CGContextSetLineWidth(context, 2.0);
                CGContextSetRGBFillColor(context, 1.0, 1.0, 0.0, 1);
                
                bool sp1Valid = [self isValidScreenPoint: sp1];
                bool sp2Valid = [self isValidScreenPoint: sp2];
                
                if (sp1Valid){
                    CGRect selectedRect = CGRectMake(sp1.x - halfSquare, sp1.y - halfSquare, halfSquare*2, halfSquare*2);
                    CGContextFillEllipseInRect(context, selectedRect);
                    CGContextFillPath(context);
                }
                if (sp2Valid){
                    CGRect selectedRect = CGRectMake(sp2.x - halfSquare, sp2.y - halfSquare, halfSquare*2, halfSquare*2);
                    CGContextFillEllipseInRect(context, selectedRect);
                    CGContextFillPath(context);
                }
                if (sp1Valid && sp2Valid){
                    CGContextSetStrokeColorWithColor(context, [UIColor whiteColor].CGColor);
                    //Set the width of the pen mark
                    CGContextSetLineWidth(context, 1.0);
                    
                    // Draw a line
                    //Start at this point
                    CGContextMoveToPoint(context, sp1.x, sp1.y);
                    CGContextAddLineToPoint(context, sp2.x, sp2.y);
                    CGContextStrokePath(context);
                }
                
            }
            self.measureView.image = UIGraphicsGetImageFromCurrentImageContext();
            UIGraphicsEndImageContext();
            break;
        }
        case Measurements::MeasureOnePoint:
        case Measurements::MeasureTwoPoints:
            // If pt1 does not need to be updated, render point 1
            UIGraphicsBeginImageContextWithOptions(self.measureView.frame.size, false, 1.0f);
            if (!_measure.pt1NeedsConvert){
                CGPoint sp1 = [self point3dToScreen:_measure.pt1];
                int halfSquare = 15;
                CGContextRef context = UIGraphicsGetCurrentContext();
                CGContextSetRGBStrokeColor(context, 1.0, 1.0, 0.0, 1);
                CGContextSetLineWidth(context, 2.0);
                CGContextSetRGBFillColor(context, 1.0, 1.0, 0.0, 1);
                
                bool sp1Valid = [self isValidScreenPoint: sp1];
                if (sp1Valid){
                    CGRect selectedRect = CGRectMake(sp1.x - halfSquare, sp1.y - halfSquare, halfSquare*2, halfSquare*2);
                    CGContextFillEllipseInRect(context, selectedRect);
                    CGContextFillPath(context);
                }
//                cout << "renderSceneWithDepthFrame Draw point" << endl;
//                cout << "pt1 valid: " << sp1Valid << endl;
//                cout << "pt1 3d coord: " << _measure.pt1.v[0] << "," << _measure.pt1.v[1] << "," << _measure.pt1.v[2] << endl;
//                cout << "pt1 coord: " << sp1.x << "," << sp1.y << endl;
            }
            self.measureView.image = UIGraphicsGetImageFromCurrentImageContext();
            UIGraphicsEndImageContext();
            break;
        default:{}
    }
    
    // Check for OpenGL errors
    GLenum err = glGetError ();
    if (err != GL_NO_ERROR)
    {
        NSLog(@"glError: %d", err);
    }
    
    // Display the rendered framebuffer.
    [(EAGLView *)self.view presentFramebuffer];
}

- (void)renderColorImage
{
    if (!_display.lumaTexture || !_display.chromaTexture)
        return;
    
    glActiveTexture(GL_TEXTURE0);
    glBindTexture(CVOpenGLESTextureGetTarget(_display.lumaTexture),
                  CVOpenGLESTextureGetName(_display.lumaTexture));
    
    glActiveTexture(GL_TEXTURE1);
    glBindTexture(CVOpenGLESTextureGetTarget(_display.chromaTexture),
                  CVOpenGLESTextureGetName(_display.chromaTexture));
    
    glDisable(GL_BLEND);
    [_display.yCbCrTextureShader useShaderProgram];
    [_display.yCbCrTextureShader renderWithLumaTexture:GL_TEXTURE0 chromaTexture:GL_TEXTURE1];
    
    glUseProgram (0);
}


// If we are outside of the scanning volume we make the pixels very dark.
- (void)renderScanningVolumeFeedbackOverlayWithDepthFrame:(STDepthFrame*)depthFrame colorFrame:(STColorFrame*)colorFrame
{
    glActiveTexture(GL_TEXTURE2);
    
    glBindTexture(GL_TEXTURE_2D, _display.scanningVolumeFeedbackTexture);
    int cols = depthFrame.width, rows = depthFrame.height;
    
    // Get the list of depth pixels which lie within the scanning volume boundaries.
    std::vector<uint8_t> mask (rows*cols);
    [_slamState.cameraPoseInitializer detectInnerPixelsWithDepthFrame:[depthFrame registeredToColorFrame:colorFrame] mask:&mask[0]];
    
    // Fill the feedback RGBA buffer.
    for (int r = 0; r < rows; ++r)
        for (int c = 0; c < cols; ++c)
        {
            const int pixelIndex = r*cols + c;
            bool insideVolume = mask[pixelIndex];
            if (insideVolume)
            {
                // Set the alpha to 0, leaving the pixels already in the render buffer unchanged.
                _display.scanningVolumeFeedbackBuffer[4*pixelIndex+3] = 0;
            }
            else
            {
                // Set the alpha to a higher value, making the pixel in the render buffer darker.
                _display.scanningVolumeFeedbackBuffer[4*pixelIndex+3] = 200;
            }
        }
    
    // Upload the texture to the GPU.
    glTexSubImage2D(GL_TEXTURE_2D, 0, 0, 0, cols, rows, GL_RGBA, GL_UNSIGNED_BYTE, _display.scanningVolumeFeedbackBuffer.data());
    
    // Rendering it with blending enabled to apply the overlay on the previously rendered buffer.
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    [_display.rgbaTextureShader useShaderProgram];
    [_display.rgbaTextureShader renderTexture:GL_TEXTURE2];
}

#define QVGA_COLS 320
#define QVGA_ROWS 240
#define QVGA_F_X 305.73
#define QVGA_F_Y 305.62
#define QVGA_C_X 159.69
#define QVGA_C_Y 119.86
#define MAGIC_RATIO 3.2
- (CGPoint) point3dToScreen:(GLKVector3) pt3d {
    // input is point in world
    GLKMatrix4 camPose = [_slamState.tracker lastFrameCameraPose];
    bool isInvertible = false;
    GLKMatrix4 worldToCam = GLKMatrix4Invert(camPose, &isInvertible);
//    cout << "3d pt:" << pt3d.v[0] << "," << pt3d.v[1] << "," << pt3d.v[2] << endl;


    if (isInvertible){
        GLKVector3 ptInCam = GLKMatrix4MultiplyVector3WithTranslation(worldToCam, pt3d);
        float xs = QVGA_F_X * ptInCam.v[0] / ptInCam.v[2]+ QVGA_C_X;
        float ys = QVGA_F_Y * ptInCam.v[1] / ptInCam.v[2]+ QVGA_C_Y;
        cout << "screen pt:" << xs << "," << ys << endl;
        return CGPointMake(xs*MAGIC_RATIO, ys*MAGIC_RATIO);
    } else {
        return CGPointMake(NAN, NAN);
    }
}

- (bool) isValidScreenPoint: (CGPoint)sp {
    return sp.x != NAN && sp.y != NAN && sp.x >= 0 && sp.x <= self.measureView.frame.size.width && sp.y >= 0 && sp.y <= self.measureView.frame.size.height;
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
                 cv::Point(lineSeg[i][2], lineSeg[i][3]), cv::Scalar(255,255,255), 3, 8 );
        
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
