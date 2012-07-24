#pragma once

#include "ofMain.h"
#include "ofxSimpleGuiToo.h"
#include "kinectCapture.h"
#include "ituitaBlobTracker.h"

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
    //ituitaBlobTracker blobTracker;
    
// --------------------------------------------
   
// MARK: INTERFACE VARIABLES

    bool    bDrawDepthMap;
    bool    bDrawThreshold;
    bool    bDrawBlobs;
    
    int     iDrawWidth, iDrawHeight;
    int     iTopMargin, iLeftMargin;
    
    int     iMode;
    
// --------------------------------------------

// MARK: CONTROL VARIABLES

    int     iFarThreshold, iNearThreshold;
    int     iMinBlobSize, iMaxBlobSize, iMaxNumBlobs;

};
