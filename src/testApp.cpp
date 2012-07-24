#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
    
    ofSetLogLevel(OF_LOG_VERBOSE);

// MARK: CONTROL VARIABLES SETUP
    
    iNearThreshold  = 0;
    iFarThreshold   = 255;
    iMinBlobSize    = 20;
    iMaxBlobSize    = 300000;
    iMaxNumBlobs    = 10;

// --------------------------------------------
    
#ifdef USE_TWO_KINECTS
    
    kinect.setup(true);
    
#endif
    
#ifndef USE_TWO_KINECTS
    
    kinect.setup(false);
    
#endif
    
// MARK: INTERFACE SETUP
    
    iDrawWidth  = 400;
    iDrawHeight = 300;
    
    iTopMargin  = 115;
    iLeftMargin = 250;
    
    bDrawDepthMap   = true;
    bDrawThreshold  = false;
    bDrawBlobs      = false;
    
    iMode           = 0;
    
//    gui.loadFromXML();
    gui.addSlider("Display Modes", iMode, 0, 2);
    gui.addSlider("Near Threshold", iNearThreshold, 0, 255);
    gui.addSlider("Far Threshold", iFarThreshold, 255, 0);
    gui.addSlider("Min Blob Size", iMinBlobSize, 0, 40000);
    gui.addSlider("Max Blob Size", iMaxBlobSize, 1, 307200);
    gui.addSlider("Max Num Blobs", iMaxNumBlobs, 1, 30);
    gui.show();
    
// --------------------------------------------
    
}

//--------------------------------------------------------------
void testApp::update(){
    
    ofBackground(100, 100, 100);
    
    kinect.updateThreshPar(iFarThreshold, iNearThreshold);
    kinect.updateBlobPar(iMinBlobSize, iMaxBlobSize, iMaxNumBlobs);
    kinect.update();

}

//--------------------------------------------------------------
void testApp::draw(){
    
    ofSetColor(255, 255, 255);
    ofNoFill();
    
    if(iMode == 0) {
        kinect.drawDepth(iLeftMargin, iTopMargin, iDrawWidth, iDrawHeight);
        ofRect(iLeftMargin, iTopMargin, iDrawWidth, iDrawHeight);
        ofDrawBitmapString("Kinect 1", iLeftMargin + 5, iTopMargin + 15);
        
        kinect.drawThreshImg(iLeftMargin, iTopMargin + iDrawHeight + 20, iDrawWidth, iDrawHeight);
        kinect.drawContour(iLeftMargin, iTopMargin + iDrawHeight + 20, iDrawWidth, iDrawHeight);
        ofRect(iLeftMargin, iTopMargin + iDrawHeight + 20, iDrawWidth, iDrawHeight);
        
    #ifdef USE_TWO_KINECTS
        
        kinect.drawDepth(iLeftMargin + iDrawWidth + 20, iTopMargin, iDrawWidth, iDrawHeight, true);
        ofRect(iLeftMargin + iDrawWidth + 20, iTopMargin, iDrawWidth, iDrawHeight);
        ofDrawBitmapString("Kinect 2", iLeftMargin + iDrawWidth + 25, iTopMargin + 15);
        
        kinect.drawThreshImg(iLeftMargin + iDrawWidth + 20, iTopMargin + iDrawHeight + 20, iDrawWidth, iDrawHeight, true);
        kinect.drawContour(iLeftMargin + iDrawWidth + 20, iTopMargin + iDrawHeight + 20, iDrawWidth, iDrawHeight, true);
        
        ofRect(iLeftMargin + iDrawWidth + 20, iTopMargin + iDrawHeight + 20, iDrawWidth, iDrawHeight);
    
    #endif
    
    }
    
    else if(iMode == 1) {
        kinect.drawThreshImg(iLeftMargin, iTopMargin, 640, 480);
        kinect.drawThreshImg(iLeftMargin + 320, iTopMargin, 640, 480, true);
        kinect.drawNormBlobs(iLeftMargin, iTopMargin, 960, 480);
        ofSetColor(255, 255, 255);
        ofDrawBitmapString("Blobs drawn from normalized blobs", iLeftMargin, iTopMargin + 500);
    }
    
    else if(iMode == 2) {
        kinect.drawDepthFromCloud(iLeftMargin, iTopMargin, 960, 480);
        ofSetColor(255, 255, 255);
        ofDrawBitmapString("Depth map drawn from normalized point cloud", iLeftMargin, iTopMargin + 500);
    }
    
    gui.draw();
    
}

//--------------------------------------------------------------
void testApp::exit() {

    kinect.close();

}

//--------------------------------------------------------------
void testApp::keyPressed(int key){
    
    switch (key) {

	}


}

//--------------------------------------------------------------
void testApp::keyReleased(int key){

}

//--------------------------------------------------------------
void testApp::mouseMoved(int x, int y){

}

//--------------------------------------------------------------
void testApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void testApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void testApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void testApp::dragEvent(ofDragInfo dragInfo){ 

}