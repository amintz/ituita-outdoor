#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
    
    ofSetLogLevel(OF_LOG_VERBOSE);
    
// MARK: KINECT AND RELATED OBJECTS INITIALIZATION
    
    kinect1.init(false, false);
	kinect1.open(0);
    cvGrayKin1ThreshNear.allocate(640, 480);
    cvGrayKin1ThreshFar.allocate(640, 480);
    cvGrayKin1.allocate(640, 480);
	
#ifdef USE_TWO_KINECTS
	kinect2.init(false, false);
	kinect2.open(1);
    cvGrayKin2ThreshNear.allocate(640, 480);
    cvGrayKin2ThreshFar.allocate(640, 480);
    cvGrayKin2.allocate(640, 480);
#endif
    
// --------------------------------------------

// MARK: CONTROL VARIABLES SETUP
    
    iNearThreshold  = 0;
    iFarThreshold   = 255;
    iMinBlobSize    = 20;
    fMaxBlobFraction= 4;
    iMaxNumBlobs    = 10;

// --------------------------------------------
    
// MARK: INTERFACE SETUP
    
    iDrawWidth  = 400;
    iDrawHeight = 300;
    
    iTopMargin  = 115;
    iLeftMargin = 250;
    
    bDrawDepthMap   = true;
    bDrawThreshold  = false;
    bDrawBlobs      = false;
    
    gui.loadFromXML();
    gui.addSlider("Near Threshold", iNearThreshold, 0, 255);
    gui.addSlider("Far Threshold", iFarThreshold, 255, 0);
    gui.addSlider("Min Blob Size", iMinBlobSize, 0, 40000);
    gui.addSlider("Max Blob Fraction of Img", fMaxBlobFraction, 1.0, 20.0);
    gui.addSlider("Max Num Blobs", iMaxNumBlobs, 1, 30);
    gui.show();
    
// --------------------------------------------
	

    
    // zero the tilt on startup
	angle = 0;
	kinect1.setCameraTiltAngle(angle);


    
}

//--------------------------------------------------------------
void testApp::update(){
    
    ofBackground(100, 100, 100);
	
	kinect1.update();
    
    if (kinect1.isFrameNew()) {
        cvGrayKin1.setFromPixels(kinect1.getDepthPixels(), kinect1.width, kinect1.height);
            
            cvGrayKin1ThreshNear    = cvGrayKin1;
            cvGrayKin1ThreshFar     = cvGrayKin1;
			
            cvGrayKin1ThreshNear.threshold(255 - iNearThreshold, true);
			cvGrayKin1ThreshFar.threshold(255 - iFarThreshold);
			cvAnd(cvGrayKin1ThreshNear.getCvImage(), cvGrayKin1ThreshFar.getCvImage(), cvGrayKin1.getCvImage(), NULL);
        
        cvGrayKin1.flagImageChanged();
        cvContKin1.findContours(cvGrayKin1, iMinBlobSize, (kinect1.width*kinect1.height)/fMaxBlobFraction, iMaxNumBlobs, false);
    }
    
#ifdef USE_TWO_KINECTS
	kinect2.update();
    
    if (kinect2.isFrameNew()) {
        cvGrayKin2.setFromPixels(kinect2.getDepthPixels(), kinect2.width, kinect2.height);
        
        cvGrayKin2ThreshNear    = cvGrayKin2;
        cvGrayKin2ThreshFar     = cvGrayKin2;
        
        cvGrayKin2ThreshNear.threshold(255 - iNearThreshold, true);
        cvGrayKin2ThreshFar.threshold(255 - iFarThreshold);
        cvAnd(cvGrayKin2ThreshNear.getCvImage(), cvGrayKin2ThreshFar.getCvImage(), cvGrayKin2.getCvImage(), NULL);
        
        cvGrayKin2.flagImageChanged();
        cvContKin2.findContours(cvGrayKin2, iMinBlobSize, (kinect1.width*kinect1.height)/fMaxBlobFraction, iMaxNumBlobs, false);
    }
    
#endif

}

//--------------------------------------------------------------
void testApp::draw(){
    
    ofSetColor(255, 255, 255);
    ofNoFill();
    
    kinect1.drawDepth(iLeftMargin, iTopMargin, iDrawWidth, iDrawHeight);
    ofRect(iLeftMargin, iTopMargin, iDrawWidth, iDrawHeight);
    ofDrawBitmapString("Kinect 1", iLeftMargin + 5, iTopMargin + 15);
    
    cvGrayKin1.draw(iLeftMargin, iTopMargin + iDrawHeight + 20, iDrawWidth, iDrawHeight);
    cvContKin1.draw(iLeftMargin, iTopMargin + iDrawHeight + 20, iDrawWidth, iDrawHeight);
    ofRect(iLeftMargin, iTopMargin + iDrawHeight + 20, iDrawWidth, iDrawHeight);
    
#ifdef USE_TWO_KINECTS
    kinect2.drawDepth(iLeftMargin + iDrawWidth + 20, iTopMargin, iDrawWidth, iDrawHeight);
    ofRect(iLeftMargin + iDrawWidth + 20, iTopMargin, iDrawWidth, iDrawHeight);
    ofDrawBitmapString("Kinect 2", iLeftMargin + iDrawWidth + 25, iTopMargin + 15);
    
    cvGrayKin2.draw(iLeftMargin + iDrawWidth + 20, iTopMargin + iDrawHeight + 20, iDrawWidth, iDrawHeight);
    cvContKin2.draw(iLeftMargin + iDrawWidth + 20, iTopMargin + iDrawHeight + 20, iDrawWidth, iDrawHeight);
    ofRect(iLeftMargin + iDrawWidth + 20, iTopMargin + iDrawHeight + 20, iDrawWidth, iDrawHeight);
    
#endif
    
    gui.draw();
    
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect1.setCameraTiltAngle(0); // zero the tilt on exit
	kinect1.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){
    
    switch (key) {
            

		case 'w':
			kinect1.enableDepthNearValueWhite(!kinect1.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect1.setCameraTiltAngle(angle); // go back to prev tilt
			kinect1.open();
			break;
			
		case 'c':
			kinect1.setCameraTiltAngle(0); // zero the tilt
			kinect1.close();
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect1.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect1.setCameraTiltAngle(angle);
			break;
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