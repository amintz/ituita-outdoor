#pragma once

#include "ofMain.h"
//#include "ofxOpenCv.h"
//#include "ofxKinect.h"
#include "ofxSimpleGuiToo.h"
//#include "ofxOsc.h"
#include "kinectCapture.h"

// MARK: "USE TWO KINECTS" SWITCH (COMMENT TO USE JUST ONE)

#define USE_TWO_KINECTS

/*//#define KIN_W 640
//#define KIN_H 480
//#define KIN2_INTERS_W 320
//#define OUTPUT_W 960

#define HOST "localhost"
#define PORT 12345*/

// ---------------------------------------------

class testApp : public ofBaseApp{
    
	public:
    
// MARK: OF BASICS
    
		void setup();
		void update();
		void draw();
        void exit();
    
        void drawPointCloud();
		
		void keyPressed(int key);
		void keyReleased(int key);
		void mouseMoved(int x, int y);
		void mouseDragged(int x, int y, int button);
		void mousePressed(int x, int y, int button);
		void mouseReleased(int x, int y, int button);
		void windowResized(int w, int h);
		void dragEvent(ofDragInfo dragInfo);
		void gotMessage(ofMessage msg);
    
// --------------------------------------------
    
// MARK: GRAPHIC INTERFACE DECLARATION
    
   ofxSimpleGuiToo gui;

// --------------------------------------------
    
// MARK: KINECT AND RELATED OBJECTS DECLARATION
    
    kinectCapture kinect;
    
/*    
//    ofxKinect kinect1;
//    ofxCvGrayscaleImage cvGrayKin1ThreshNear;
//    ofxCvGrayscaleImage cvGrayKin1ThreshFar;
//    ofxCvGrayscaleImage cvGrayKin1;
//    ofxCvContourFinder  cvContKin1;
//    
//#ifdef USE_TWO_KINECTS
//	
//    ofxKinect kinect2;
//    ofxCvGrayscaleImage cvGrayKin2;
//    ofxCvGrayscaleImage cvGrayKin2ThreshNear;
//    ofxCvGrayscaleImage cvGrayKin2ThreshFar;
//    ofxCvContourFinder  cvContKin2;
//
//#endif
    */
// --------------------------------------------

// MARK: COMMUNICATION OBJECTS AND VARIABLES
/*
//    ofxOscSender oscSender;
//    bool bSendMessages;
    */
// --------------------------------------------
    
// MARK: INTERFACE VARIABLES

    bool    bDrawDepthMap;
    bool    bDrawThreshold;
    bool    bDrawBlobs;
    
    int     iDrawWidth, iDrawHeight;
    int     iTopMargin, iLeftMargin;
    
// --------------------------------------------

// MARK: CONTROL VARIABLES

    int     iFarThreshold, iNearThreshold;
    int     iMinBlobSize, iMaxBlobSize, iMaxNumBlobs;

    /*
//    
//    int   numPix;
//    
//    float norm960[960];
//    float norm640[640];
//    float norm480[480];
//    float norm4000[4000];
    */
};
