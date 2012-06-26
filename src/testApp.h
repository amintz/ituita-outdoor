#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxSimpleGuiToo.h"

// MARK: "USE TWO KINECTS" SWITCH (COMMENT TO USE JUST ONE)

#define USE_TWO_KINECTS

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
    
    ofxKinect kinect1;
    ofxCvGrayscaleImage cvGrayKin1;
    ofxCvContourFinder  cvContKin1;
    
#ifdef USE_TWO_KINECTS
	
    ofxKinect kinect2;
    ofxCvGrayscaleImage cvGrayKin2;
    ofxCvContourFinder  cvContKin2;

#endif
    
// --------------------------------------------
    
// MARK: INTERFACE VARIABLES

    bool    bDrawDepthMap;
    bool    bDrawThreshold;
    bool    bDrawBlobs;
    
    int     iDrawWidth, iDrawHeight;
    int     iTopMargin, iLeftMargin;
    
// --------------------------------------------

// MARK: CONTROL VARIABLES

    float fMaxDist;
    float fMinDist;
    
    int angle;
    int farThreshold;
    int nearThreshold;
    
    bool testBool;
    
};
