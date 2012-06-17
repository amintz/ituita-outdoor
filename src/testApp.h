#pragma once

#include "ofMain.h"
//#include "ofxOpenCv.h"
#include "ofxKinect.h"

// uncomment this to read from two kinects simultaneously
//#define USE_TWO_KINECTS

class testApp : public ofBaseApp{
	public:
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
    
    ofxKinect kinect;
    
    float fMaxDist;
    float fMinDist;
    
    bool bOverlay;
    bool bKeyDPressed;
    int  iLeftCrop;
    int  iRightCrop;
	
#ifdef USE_TWO_KINECTS
	ofxKinect kinect2;
#endif
    
    // used for viewing the point cloud
	ofEasyCam easyCam;
    
    int angle;
    
};
