#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
    
    ofSetLogLevel(OF_LOG_VERBOSE);
    
// MARK: KINECT AND RELATED OBJECTS INITIALIZATION
    
    kinect1.init(false, false);
	kinect1.open();
    cvGrayKin1.allocate(640, 480);
	
#ifdef USE_TWO_KINECTS
	kinect2.init(false, false);
	kinect2.open();
    cvGrayKin2.allocate(640, 480);
#endif
    
// --------------------------------------------
    
// MARK: INTERFACE SETUP
    
    iDrawWidth  = 400;
    iDrawHeight = 300;
    
    iTopMargin  = 115;
    iLeftMargin = 250;
    
    bDrawDepthMap   = true;
    bDrawThreshold  = false;
    bDrawBlobs      = false;
    
// --------------------------------------------
	

    
    // zero the tilt on startup
	angle = 0;
	kinect1.setCameraTiltAngle(angle);

    gui.setup();
    gui.addButton("test button", testBool);
    gui.show();
    
}

//--------------------------------------------------------------
void testApp::update(){
    
    ofBackground(100, 100, 100);
	
	kinect1.update();
    
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif

}

//--------------------------------------------------------------
void testApp::draw(){
    
    ofSetColor(255, 255, 255);
    ofNoFill();
    
    kinect1.drawDepth(iLeftMargin, iTopMargin, iDrawWidth, iDrawHeight);
    ofRect(iLeftMargin, iTopMargin, iDrawWidth, iDrawHeight);
    ofDrawBitmapString("Kinect 1", iLeftMargin + 5, iTopMargin + 15);
    
#ifdef USE_TWO_KINECTS
    kinect2.drawDepth(iLeftMargin + iDrawWidth + 20, iTopMargin, iDrawWidth, iDrawHeight);
    ofRect(iLeftMargin + iDrawWidth + 20, iTopMargin, iDrawWidth, iDrawHeight);
    ofDrawBitmapString("Kinect 2", iLeftMargin + iDrawWidth + 25, iTopMargin + 15);
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
            
		case '>':
		case '.':
			farThreshold ++;
			if (farThreshold > 255) farThreshold = 255;
			break;
			
		case '<':
		case ',':
			farThreshold --;
			if (farThreshold < 0) farThreshold = 0;
			break;
			
		case '+':
		case '=':
			nearThreshold ++;
			if (nearThreshold > 255) nearThreshold = 255;
			break;
			
		case '-':
			nearThreshold --;
			if (nearThreshold < 0) nearThreshold = 0;
			break;
            
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