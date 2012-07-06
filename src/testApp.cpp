#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
    
    ofSetLogLevel(OF_LOG_VERBOSE);
    
    for (int i = 0 ; i < 640 ; i++) {
        norm640[i] = ofNormalize(i, 0, 640);
    }
    
    for (int i = 0; i < 480; i++) {
        norm480[i] = ofNormalize(i, 0, 480);
    }
    
    for (int i = 0; i < 960; i++) {
        norm960[i] = ofNormalize(i, 0, 960);
    }
    
    for (int i = 0; i < 4000; i++) {
        norm4000[i] = ofNormalize(i, 0, 4000);
    }
    
// MARK: KINECT AND RELATED OBJECTS INITIALIZATION
    
    fKin1Angle = 0;
    
    kinect1.init(false, false);
	kinect1.open(0);
    kinect1.setCameraTiltAngle(fKin1Angle);
    cvGrayKin1ThreshNear.allocate(640, 480);
    cvGrayKin1ThreshFar.allocate(640, 480);
    cvGrayKin1.allocate(640, 480);
	
    
#ifdef USE_TWO_KINECTS
    
    fKin2Angle = 0;
    
	kinect2.init(false, false);
	kinect2.open(1);
    kinect2.setCameraTiltAngle(fKin2Angle);
    cvGrayKin2ThreshNear.allocate(640, 480);
    cvGrayKin2ThreshFar.allocate(640, 480);
    cvGrayKin2.allocate(640, 480);
    
#endif
    
    
    
// --------------------------------------------

// MARK: COMMUNICATION SETUP
    
    oscSender.setup(HOST, PORT);
    bSendMessages = false;
    
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
    gui.addToggle("Send Messages", bSendMessages);
    gui.show();
    
// --------------------------------------------
    
}

//--------------------------------------------------------------
void testApp::update(){
    
    ofBackground(100, 100, 100);
    
    int blobIdx = 0;
	
    ofxOscBundle bundle;
    
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
        
        if (cvContKin1.blobs.size() > 0 && bSendMessages) {
            
            for (int i = 0; i < cvContKin1.blobs.size(); i++) {
                
                ofxOscMessage blobMessage;
                blobMessage.setAddress("/blob");
                
                // Index
                blobMessage.addIntArg(blobIdx); // Index
                blobIdx ++;
                
                // Centroid
                blobMessage.addFloatArg(norm960[(int)cvContKin1.blobs[i].centroid.x]); // Centroid X
                blobMessage.addFloatArg(norm480[(int)cvContKin1.blobs[i].centroid.y]); // Centroid Y
                
                // TODO: Speed | Implement Blob Tracking
                blobMessage.addFloatArg(0.0f); // Speed X
                blobMessage.addFloatArg(0.0f); // Speed Y
                
                // Points
                if (cvContKin1.blobs[i].pts.size() > 0) {
                    for (int j = 0; j < cvContKin1.blobs[i].pts.size(); j++) {
                        blobMessage.addFloatArg(norm960[(int)cvContKin1.blobs[i].pts[j].x]); // X coordinate
                        blobMessage.addFloatArg(norm480[(int)cvContKin1.blobs[i].pts[j].y]); // Y coordinate
                        blobMessage.addFloatArg(0.0f); // Z coordinate - just in case;
                    }
                }
                oscSender.sendMessage(blobMessage);
                blobMessage.clear();
            }
        }
        
#ifndef USE_TWO_KINECTS
        if (bSendMessages) {
            ofxOscMessage cloudMessage;
            
            cloudMessage.setAddress("/cloud");
            
            for (int x = 0; x < KIN_W; x++) {
                for (int y = 0; y < KIN_H; y++) {
                    cloudMessage.addFloatArg(norm640[x]);
                    cloudMessage.addFloatArg(norm480[y]);
                    cloudMessage.addFloatArg(norm4000[(int)kinect1.getDistanceAt(x,y)]);
                }
            }
            oscSender.sendMessage(cloudMessage);
        }
        
#endif
        
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
        
        if (cvContKin2.blobs.size() > 0 && bSendMessages) {
            
            for (int i = 0; i < cvContKin2.blobs.size(); i++) {
                
                ofxOscMessage blobMessage;
                blobMessage.setAddress("/blob");
                
                // Index
                blobMessage.addIntArg(blobIdx);
                blobIdx++;
                
                // Centroid
                blobMessage.addFloatArg(norm960[(int)cvContKin2.blobs[i].centroid.x + KIN2_INTERS_W]);
                blobMessage.addFloatArg(norm480[(int)cvContKin2.blobs[i].centroid.y]);
                
                // TODO: Speed | Implement Blob Tracking
                blobMessage.addFloatArg(0.0f);
                blobMessage.addFloatArg(0.0f);
                
                // Points
                if (cvContKin2.blobs[i].pts.size() > 0) {
                    for (int j = 0; j < cvContKin2.blobs[i].pts.size(); j++) {
                        blobMessage.addFloatArg(norm960[(int)cvContKin2.blobs[i].pts[j].x + KIN2_INTERS_W]);
                        blobMessage.addFloatArg(norm480[(int)cvContKin2.blobs[i].pts[j].y]);
                        blobMessage.addFloatArg(0.0f);
                    }
                }
                oscSender.sendMessage(blobMessage);
                blobMessage.clear();
            }
        }
    }
    
    if (bSendMessages) {
        
        ofxOscMessage cloudMessage;
        
        cloudMessage.setAddress("/cloud");
        
        for (int x = 0; x < OUTPUT_W - 30; x+= 30) {
            for (int y = 0; y < KIN_H - 30; y += 30) {
                if (x <= KIN2_INTERS_W) {
                    cloudMessage.addFloatArg(norm960[x]);
                    cloudMessage.addFloatArg(norm480[y]);
                    cloudMessage.addFloatArg(ofNormalize(kinect1.getDistanceAt(x,y), 0, 4000));
                }
                else if (x > KIN2_INTERS_W && x <= KIN_W) {
                    cloudMessage.addFloatArg(norm960[x]);
                    cloudMessage.addFloatArg(norm480[y]);
                    int minDist = kinect1.getDistanceAt(x, y) < kinect2.getDistanceAt(x - KIN2_INTERS_W, y) ? kinect1.getDistanceAt(x, y) : kinect2.getDistanceAt(x - KIN2_INTERS_W, y);
                    cloudMessage.addFloatArg(norm4000[minDist]);
                }
                else if (x > KIN2_INTERS_W) {
                    cloudMessage.addFloatArg(norm960[x]);
                    cloudMessage.addFloatArg(norm480[y]);
                    cloudMessage.addFloatArg(norm4000[(int)kinect2.getDistanceAt(x - KIN2_INTERS_W, y)]);
                }
            }
        }
        oscSender.sendMessage(cloudMessage);
        cloudMessage.clear();
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
    
    kinect2.setCameraTiltAngle(0);
	kinect2.close();

#endif

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