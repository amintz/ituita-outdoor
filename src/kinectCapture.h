//
//  kinectCapture.h
//  ituita-outdoor
//
//  Created by André Goes Mintz on 7/11/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#ifndef ituita_outdoor_kinectCapture_h
#define ituita_outdoor_kinectCapture_h

#include "ofxOpenCv.h"
#include "ofxKinect.h"

#define KIN_W 640
#define KIN_H 480
#define KIN2_INTERS_W 320
#define OUTPUT_W 960

class kinectCapture {
    
    public:
    
    kinectCapture();
    virtual ~kinectCapture();
    
    void setup(bool _bTwoKinects = false);
    void update();
    void updateThreshPar(int _iFarThreshold, int _iNearThreshold);
    void updateBlobPar(int _iMinBlobSize, int _iMaxBlobSize, int _iMaxNumBlobs);
    void drawDepth(int x, int y, int w, int h, bool kin2 = false);
    void drawThreshImg(int x, int y, int w, int h, bool kin2 = false);
    void drawContour(int x, int y, int w, int h, bool kin2 = false);
    void close();
    
    vector<ofxCvBlob> foundBlobs;
    vector<ofxCvBlob> kin1FoundBlobs;
    vector<ofxCvBlob> kin2FoundBlobs;
    vector<ofPoint> pointCloud;
    
    
    private:
    
    float normWidth(int val, bool _bTwoKinects = false);
    float normHeight(int val);
    float normDepth(int val);
    
    // MARK: KINECT AND RELATED OBJECTS DECLARATION
    
    ofxKinect kinect1;
    ofxCvGrayscaleImage cvGrayKin1ThreshNear;
    ofxCvGrayscaleImage cvGrayKin1ThreshFar;
    ofxCvGrayscaleImage cvGrayKin1;
    ofxCvContourFinder  cvContKin1;
	
    ofxKinect kinect2;
    ofxCvGrayscaleImage cvGrayKin2;
    ofxCvGrayscaleImage cvGrayKin2ThreshNear;
    ofxCvGrayscaleImage cvGrayKin2ThreshFar;
    ofxCvContourFinder  cvContKin2;
    
    // MARK: CONTROL VARIABLES
    
    bool bTwoKinects;
    
    int     iFarThreshold, iNearThreshold;
    int     iMinBlobSize, iMaxBlobSize, iMaxNumBlobs;
    
    float fKin1Angle, fKin2Angle;
    
    int   numPix;
    
    float norm960[960];
    float norm640[640];
    float norm480[480];
    float norm4000[4000];
    
};

#endif
