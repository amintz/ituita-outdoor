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
    
    bResetData            = true;
    iMaxRandomParticles   = 20;
    iDeltaRandomParticles = 60;
    fAttractionForce      = 2.0;
    
    fDensity = 8.0;
    fBounce = 0.2;
    fFriction  = 0.5;

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
    
    gui.addPage("Particles");
    gui.addSlider("Random Max", iMaxRandomParticles, 6, 40);
    gui.addSlider("Random Delta", iDeltaRandomParticles, 0, 100);
    gui.addSlider("Density", fDensity, 0.0f, 20.0f);
    gui.addSlider("Bounce", fBounce, 0.0f, 1.0f);
    gui.addSlider("Friction", fFriction, 0.0f, 1.0f);
    gui.addButton("Reset particles", bResetData);
    gui.addSlider("Attraction (real-time)", fAttractionForce, 0.0f, 20.0f);    
    
    gui.show();
    
    isGUIActive = true;
    
// --------------------------------------------

// MARK: SHADER
    
    shader.load("shaders/led.vert", "shaders/led.frag");
    isFilterActive = false;
    ledRatio = 4;
    
// --------------------------------------------


// MARK: BOX2D
    
	box2d.init();
	box2d.setGravity(0, 0);
	box2d.createBounds(0, 0, FBO_W, FBO_H);
	box2d.setFPS(30.0);
	box2d.registerGrabbing();   
    
    personalCenter.set(OUTPUT_SCREEN_W/6, 2*OUTPUT_SCREEN_H/3);
    neighborhoodCenter.set(FBO_W/2, 2*OUTPUT_SCREEN_H/3);
    cityCenter.set(FBO_W - OUTPUT_SCREEN_W/6, 2*OUTPUT_SCREEN_H/3);
    
// --------------------------------------------
    
    fbo.allocate(FBO_W, FBO_H);
    
}

void testApp::setupData() {
    int delta = iMaxRandomParticles * ((100.0-iDeltaRandomParticles)/100.0);
    data.generateRandomValues(delta, iMaxRandomParticles);
    
    ppos = data.getPersonalPositives();
    pneu = data.getPersonalNeutrals();
    pneg = data.getPersonalNegatives();
    
    npos = data.getNeighborhoodPositives();
    nneu = data.getNeighborhoodNeutrals();
    nneg = data.getNeighborhoodNegatives();
    
    cpos = data.getCityPositives();
    cneu = data.getCityNeutrals();
    cneg = data.getCityNegatives();
    
    string datalog = "- DATA --------------------------------------------- \n";
    datalog += "personal     (" + ofToString(ppos+pneu+pneg) + "): ";
    datalog += ofToString(ppos) + " / " + ofToString(pneu) + " / " + ofToString(pneg) + "\n";
    datalog += "neighborhood (" + ofToString(npos+nneu+nneg) + "): ";
    datalog += ofToString(npos) + " / " + ofToString(nneu) + " / " + ofToString(nneg) + "\n";
    datalog += "city         (" + ofToString(cpos+cneu+cneg) + "): ";
    datalog += ofToString(cpos) + " / " + ofToString(cneu) + " / " + ofToString(cneg) + "\n";
    datalog += "---------------------------------------------------- \n";
    ofLog(OF_LOG_NOTICE, datalog);

    b2dParticles.clear();
    
    addParticles(PERSONAL, NEG, pneg);
    addParticles(PERSONAL, NEU, pneu);
    addParticles(PERSONAL, POS, ppos);
    
    addParticles(NEIGHBORHOOD, NEG, nneg);
    addParticles(NEIGHBORHOOD, NEU, nneu);
    addParticles(NEIGHBORHOOD, POS, npos);
    
    addParticles(CITY, NEG, cneg);
    addParticles(CITY, NEU, cneu);
    addParticles(CITY, POS, cpos);
}

void testApp::addParticles(int scope, int type, int num) {
    addParticles(scope, type, num, fDensity, fBounce, fFriction);
}

void testApp::addParticles(int scope, int type, int num, float density, float bounce, float friction) {    

    ofVec2f attract;
    switch(scope) {
        case 0: 
            attract.set(personalCenter.x, personalCenter.y);
            break;
        case 1: 
            attract.set(neighborhoodCenter.x, neighborhoodCenter.y);
            break;
        case 2: 
            attract.set(cityCenter.x, cityCenter.y);
            break;
    }
    int delta = 120;

    for(int i = 0; i < num ; i++) {
        CustomParticle p;        
        
        //density, restitution/bounce, friction
        p.setPhysics(density, bounce, friction);

        float x = ofRandom(attract.x - delta, attract.x + delta);
        float y = ofRandom(attract.y - delta, attract.y + delta);
        
        p.setup(box2d.getWorld(), x, y, 4);
        p.setupTheCustomData(scope, type, attract.x, attract.y);
        
        b2dParticles.push_back(p);    
    }
    
}

//--------------------------------------------------------------
void testApp::update(){
    
    ofBackground(0);
    
    kinect.updateThreshPar(iFarThreshold, iNearThreshold);
    kinect.updateBlobPar(iMinBlobSize, iMaxBlobSize, iMaxNumBlobs);
    kinect.update();
    
    if(bResetData) {
        bResetData = false;
        setupData();
    }
    
    box2d.update();	

    for(int i = 0; i < b2dParticles.size(); i++) {        
        Data * customData = (Data*)b2dParticles[i].getData();        
        b2dParticles[i].addAttractionPoint(customData->attractionPoint, fAttractionForce);
    }
    
}

//--------------------------------------------------------------
void testApp::draw(){

    
//    int inc = 20;
//    for (int i = 0; i < kinect.pointCloud.size(); i+=inc) {
//        
//        float z = (kinect.pointCloud[i].z < 0.001) ? 1 : kinect.pointCloud[i].z;
//
//        float prox = (1.0 - z);
//        float sz = pow(prox, 3) * inc;
//        ofCircle(kinect.pointCloud[i].x * (float)ofGetWidth(), kinect.pointCloud[i].y * (float)ofGetHeight(), sz);
//
//        int limit_w = kinect.getOutputWidth();
//        if(i % limit_w == 0) {
//            i += inc * limit_w;
//        }
//    }
    
    
// MARK: DRAW BOX2D PARTICLES TO FBO
    fbo.begin();
    ofClear(0, 0, 0, 255);
    
    if(isFilterActive) {
        shader.begin();
        shader.setUniform1i("u_ratio", ledRatio);
    }
    
    for(int i = 0; i < b2dParticles.size(); i++) {
        CustomParticle p = b2dParticles[i];
        
        if(kinect.pointCloud.size() > 0) {
            //            int relativeX = ofMap(p.getPosition().x, 0, FBO_W, 0, kinect.getOutputWidth());            
            int relativeX = ofMap(p.getPosition().x, 0, FBO_W, 0, OUTPUT_SCREEN_W);
            relativeX = ofMap(relativeX, 0, OUTPUT_SCREEN_W, 0, kinect.getOutputWidth());
            int relativeY = ofMap(p.getPosition().y, 0, FBO_H, 0, kinect.getOutputHeight());
            
            int relativeKinectIndex = relativeX + (kinect.getOutputWidth() * relativeY);
            
            if(relativeKinectIndex < kinect.pointCloud.size()) {
                ofPoint kinectPoint = kinect.pointCloud[relativeKinectIndex];
                float z = (kinectPoint.z < 0.001) ? 1 : kinectPoint.z;
                float prox = 1.0 - z;
                float sz = 4 + (pow(prox, 2) * 50.0);
                
                p.setRadius(sz);
            }
        }    

        p.draw();
    }
    
    ofSetColor(70, 255);
    ofCircle(personalCenter, 6);
    ofCircle(neighborhoodCenter, 6);
    ofCircle(cityCenter, 6);
    

    if(isFilterActive) {        
        shader.end();
    }
    
    fbo.end();
    
    ofSetColor(255);
    fbo.draw(0,0);
// --------------------------------------------
    
// CHEAT: PILLARS MASK
    ofSetColor(25, 255);
    ofFill();
    ofRect(OUTPUT_SCREEN_W/3.0, 0, (FBO_W-OUTPUT_SCREEN_W)/2.0, FBO_H);
    ofRect(FBO_W - (OUTPUT_SCREEN_W/3.0) - (FBO_W-OUTPUT_SCREEN_W)/2.0, 0, (FBO_W-OUTPUT_SCREEN_W)/2.0, FBO_H);
// --------------------------------------------
    
    if(isGUIActive) {
        drawGUI();
    }
    
}

//--------------------------------------------------------------
void testApp::drawGUI() {
    
    ofEnableAlphaBlending();
    ofSetColor(0, 0, 0, 127);
    ofFill();
    ofRect(0, 0, ofGetWidth(), ofGetHeight());
    ofDisableAlphaBlending();
    
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
// KEYS MAP
//
// G: controls (turns on/off) the GUI drawing
// F: controls (turns on/off) the filter/shader
//--------------------------------------------------------------
void testApp::keyPressed(int key){

    if(key == 'g' || key == 'G') {
        isGUIActive = !isGUIActive;
        if(isGUIActive) gui.show();
        else gui.hide();
	} else if(key == 'f' || key =='F') {
        isFilterActive = !isFilterActive;
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