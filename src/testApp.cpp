//--------------------------------------------------------------
// [fturcheti] TODO list
//--------------------------------------------------------------
//
// TODO: test particles size proportional to particles count
// ?:  what about the maximum size
//
// TODO: implement gestures interactions
// !:  transform blobs into box2d rects
// !:  map the blobs/rects by its ids
// ?:  mind the rigid joints
//
// TODO: implement ghosts particles
//
// TODO: refactor everything related to the graphics
//
// TODO: implement "add new particle"
// !:  animation, particle falling down
//
//--------------------------------------------------------------


#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
    
    ofSetLogLevel(OF_LOG_VERBOSE);

// MARK: CONTROL VARIABLES SETUP
    
    iNearThreshold  = 0;
    iFarThreshold   = 255;
    iMinBlobSize    = 1000;
    iMaxBlobSize    = 300000;
    iMaxNumBlobs    = 10;
    
    bResetData            = true;
    iMaxRandomParticles   = 20;
    iDeltaRandomParticles = 60;
    fAttractionForce      = 2.0;
    
    fDensity = 8.0;
    fBounce = 0.8; // with the joints it's better with a high value
    fFriction  = 0.1;

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
    
    bLockKinTilt = true;
    fKin1TiltAngle = 0;
    fKin2TiltAngle = 0;
    
    
    gui.addSlider("Display Modes", iMode, 0, 2);
    gui.addToggle("Lock Tilt Angle", bLockKinTilt);
    gui.addSlider("Kin 1 Tilt Angle", fKin1TiltAngle, -30, 30);
    gui.addSlider("Kin 2 Tilt Angle", fKin2TiltAngle, -30, 30);
    gui.addSlider("Near Threshold", iNearThreshold, 0, 255);
    gui.addSlider("Far Threshold", iFarThreshold, 255, 0);
    gui.addSlider("Min Blob Size", iMinBlobSize, 0, 40000);
    gui.addSlider("Max Blob Size", iMaxBlobSize, 1, 307200);
    gui.addSlider("Max Num Blobs", iMaxNumBlobs, 1, 30);
    
    gui.addPage("Particles");
    gui.addSlider("Random Max", iMaxRandomParticles, 1, 60);
    gui.addSlider("Random Delta", iDeltaRandomParticles, 0, 100);
    gui.addSlider("Density", fDensity, 0.0f, 50.0f);
    gui.addSlider("Bounce", fBounce, 0.0f, 1.0f);
    gui.addSlider("Friction", fFriction, 0.0f, 1.0f);
    gui.addButton("Reset particles", bResetData);
    gui.addSlider("Attraction (real-time)", fAttractionForce, 0.0f, 20.0f);    
    
    gui.loadFromXML();
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
    
    isDebugingBox2d = false;
    
    personalCenter.set(OUTPUT_SCREEN_W/6, 2*OUTPUT_SCREEN_H/3);
    neighborhoodCenter.set(FBO_W/2, 2*OUTPUT_SCREEN_H/3);
    cityCenter.set(FBO_W - OUTPUT_SCREEN_W/6, 2*OUTPUT_SCREEN_H/3);
    
    personalAnchorBottom.setup(box2d.getWorld(), personalCenter.x, personalCenter.y+100, 2);
    personalAnchorTop.setup(box2d.getWorld(), personalCenter.x, personalCenter.y-100, 2);
    personalAnchorLeft.setup(box2d.getWorld(), personalCenter.x-150, personalCenter.y, 2);
    personalAnchorRight.setup(box2d.getWorld(), personalCenter.x+150, personalCenter.y, 2);
    
    neighborhoodAnchorBottom.setup(box2d.getWorld(), neighborhoodCenter.x, neighborhoodCenter.y+100, 2);
    neighborhoodAnchorTop.setup(box2d.getWorld(), neighborhoodCenter.x, neighborhoodCenter.y-100, 2);
    neighborhoodAnchorLeft.setup(box2d.getWorld(), neighborhoodCenter.x-150, neighborhoodCenter.y, 2);
    neighborhoodAnchorRight.setup(box2d.getWorld(), neighborhoodCenter.x+150, neighborhoodCenter.y, 2);

    cityAnchorBottom.setup(box2d.getWorld(), cityCenter.x, cityCenter.y+100, 2);
    cityAnchorTop.setup(box2d.getWorld(), cityCenter.x, cityCenter.y-100, 2);
    cityAnchorLeft.setup(box2d.getWorld(), cityCenter.x-150, cityCenter.y, 2);
    cityAnchorRight.setup(box2d.getWorld(), cityCenter.x+150, cityCenter.y, 2);
    
// --------------------------------------------
    
    fbo.allocate(FBO_W, FBO_H);
    
}

float testApp::getMinParticleSize(int particlesCount) {    
    float pc = sqrt( ofClamp(particlesCount, 1, 64) );
    return ofMap(pc, 1, 8, 30, 6);
}

void testApp::setupData() {

    // GENERATING RANDOM VALUES
    int delta = iMaxRandomParticles * ((100.0-iDeltaRandomParticles)/100.0);
    data.generateRandomValues(delta, iMaxRandomParticles);
    
    // GETTING THE VALUES
    ppos = data.getPersonalPositives();
    pneu = data.getPersonalNeutrals();
    pneg = data.getPersonalNegatives();
    personalMinParticleSize = getMinParticleSize(ppos+pneu+pneg);
    
    npos = data.getNeighborhoodPositives();
    nneu = data.getNeighborhoodNeutrals();
    nneg = data.getNeighborhoodNegatives();
    neighborhoodMinParticleSize = getMinParticleSize(npos+nneu+nneg);
    
    cpos = data.getCityPositives();
    cneu = data.getCityNeutrals();
    cneg = data.getCityNegatives();
    cityMinParticleSize = getMinParticleSize(cpos+cneu+cneg);
    
    // LOGGING THE DATA
    string datalog = "- DATA --------------------------------------------- \n";
    datalog += "personal     (" + ofToString(ppos+pneu+pneg) + "): ";
    datalog += ofToString(ppos) + " / " + ofToString(pneu) + " / " + ofToString(pneg) + "\n";
    datalog += "neighborhood (" + ofToString(npos+nneu+nneg) + "): ";
    datalog += ofToString(npos) + " / " + ofToString(nneu) + " / " + ofToString(nneg) + "\n";
    datalog += "city         (" + ofToString(cpos+cneu+cneg) + "): ";
    datalog += ofToString(cpos) + " / " + ofToString(cneu) + " / " + ofToString(cneg) + "\n";
    datalog += "---------------------------------------------------- \n";
    ofLog(OF_LOG_NOTICE, datalog);

    // CLEARING VECTORS
    // the joints must be destroyed before the bodies they attach
    for(int j = 0; j < b2dJoints.size(); j++) {
        b2dJoints[j].destroy();
    }
    for(int i = 0; i < b2dParticles.size(); i++) {
        b2dParticles[i].destroy();
    }
    b2dJoints.clear();
    b2dParticles.clear();
    
    // ADDING PARTICLES
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
        case PERSONAL: 
            attract.set(personalCenter.x, personalCenter.y);
            break;
        case NEIGHBORHOOD: 
            attract.set(neighborhoodCenter.x, neighborhoodCenter.y);
            break;
        case CITY: 
            attract.set(cityCenter.x, cityCenter.y);
            break;
    }
    
    int deltaX = 120;
    int typeY = 0;
    switch(type) {
        case NEG:
            typeY = attract.y + 100;
            break;
        case NEU:
            typeY = attract.y;
            break;
        case POS:
            typeY = attract.y - 100;
            break;
    }

    for(int i = 0; i < num ; i++) {
        
    // MARK: create particle
        CustomParticle p;        
        
        //density, restitution/bounce, friction
        p.setPhysics(density, bounce, friction);

        float x = ofRandom(attract.x - deltaX, attract.x + deltaX);
        float y = typeY;
        
        switch(scope) {
            case PERSONAL:
                p.setup(box2d.getWorld(), x, y, personalMinParticleSize);
                break;
            case NEIGHBORHOOD:
                p.setup(box2d.getWorld(), x, y, neighborhoodMinParticleSize);
                break;
            case CITY:
                p.setup(box2d.getWorld(), x, y, cityMinParticleSize);
                break;                
        }
        p.setupTheCustomData(scope, type, attract.x, attract.y);
        
        
    // MARK: create joints        
		ofxBox2dJoint jointLeft;
        ofxBox2dJoint jointRight;
		
        switch(scope) {
            case PERSONAL: 
                jointLeft.setup(box2d.getWorld(), personalAnchorLeft.body, p.body);		
                jointRight.setup(box2d.getWorld(), personalAnchorRight.body, p.body);
                if( type == NEG ) {
                    ofxBox2dJoint jointBottom;
                    jointBottom.setup(box2d.getWorld(), personalAnchorBottom.body, p.body);
                    jointBottom.setDamping(0.2);
                    jointBottom.setFrequency(1.0);
                    jointBottom.setLength(50);
                    b2dJoints.push_back(jointBottom);
                } else if( type == POS ) {
                    ofxBox2dJoint jointTop;
                    jointTop.setup(box2d.getWorld(), personalAnchorTop.body, p.body);
                    jointTop.setDamping(0.2);
                    jointTop.setFrequency(1.0);
                    jointTop.setLength(50);
                    b2dJoints.push_back(jointTop);
                }
                break;
            case NEIGHBORHOOD: 
                jointLeft.setup(box2d.getWorld(), neighborhoodAnchorLeft.body, p.body);		
                jointRight.setup(box2d.getWorld(), neighborhoodAnchorRight.body, p.body);		
                if( type == NEG ) {
                    ofxBox2dJoint jointBottom;
                    jointBottom.setup(box2d.getWorld(), neighborhoodAnchorBottom.body, p.body);
                    jointBottom.setDamping(0.2);
                    jointBottom.setFrequency(1.0);
                    jointBottom.setLength(50);
                    b2dJoints.push_back(jointBottom);
                } else if( type == POS ) {
                    ofxBox2dJoint jointTop;
                    jointTop.setup(box2d.getWorld(), neighborhoodAnchorTop.body, p.body);
                    jointTop.setDamping(0.2);
                    jointTop.setFrequency(1.0);
                    jointTop.setLength(50);
                    b2dJoints.push_back(jointTop);
                }
                break;
            case CITY: 
                jointLeft.setup(box2d.getWorld(), cityAnchorLeft.body, p.body);		
                jointRight.setup(box2d.getWorld(), cityAnchorRight.body, p.body);		
                if( type == NEG ) {
                    ofxBox2dJoint jointBottom;
                    jointBottom.setup(box2d.getWorld(), cityAnchorBottom.body, p.body);
                    jointBottom.setDamping(0.2);
                    jointBottom.setFrequency(1.0);
                    jointBottom.setLength(50);
                    b2dJoints.push_back(jointBottom);
                } else if( type == POS ) {
                    ofxBox2dJoint jointTop;
                    jointTop.setup(box2d.getWorld(), cityAnchorTop.body, p.body);
                    jointTop.setDamping(0.2);
                    jointTop.setFrequency(1.0);
                    jointTop.setLength(50);
                    b2dJoints.push_back(jointTop);
                }
                break;
        }
		
        jointLeft.setDamping(0.2);
        jointLeft.setFrequency(1.0);
		jointLeft.setLength(50);

        jointRight.setDamping(0.2);
        jointRight.setFrequency(1.0);
		jointRight.setLength(50);
        
        
    // MARK: add joints and particle to relative vectors
		b2dJoints.push_back(jointLeft);
		b2dJoints.push_back(jointRight);
        b2dParticles.push_back(p);
    }
    
}

//--------------------------------------------------------------
void testApp::update(){
    
    ofBackground(0);
    
    kinect.updateThreshPar(iFarThreshold, iNearThreshold);
    kinect.updateBlobPar(iMinBlobSize, iMaxBlobSize, iMaxNumBlobs);
    
    if(!bLockKinTilt) {
        kinect.setKinTiltAngle(false, fKin1TiltAngle);
        kinect.setKinTiltAngle(true, fKin2TiltAngle);
    }
    
    kinect.update();
    
    if(bResetData) {
        bResetData = false;
        setupData();
    }
    
    box2d.wakeupShapes();    
    box2d.update();	

    // CONVERT TOP AND BOTTOM ANCHORS INTO POSITIVES AND NEGATIVES ATTRACTION POINTS
    for(int i = 0; i < b2dParticles.size(); i++) {
        Data * customData = (Data*)b2dParticles[i].getData();        
        switch(customData->scope) {
            case PERSONAL:
                if(customData->type == NEG) {
                    b2dParticles[i].addAttractionPoint(personalAnchorBottom.getPosition(), fAttractionForce);
                    b2dParticles[i].addRepulsionForce(personalCenter, fAttractionForce);
                } else if(customData->type == NEU) {
                    b2dParticles[i].addAttractionPoint(personalCenter, fAttractionForce);                    
                } else if(customData->type == POS) {
                    b2dParticles[i].addAttractionPoint(personalAnchorTop.getPosition(), fAttractionForce);
                    b2dParticles[i].addRepulsionForce(personalCenter, fAttractionForce);
                }
                break;
            case NEIGHBORHOOD:
                if(customData->type == NEG) {
                    b2dParticles[i].addAttractionPoint(neighborhoodAnchorBottom.getPosition(), fAttractionForce);
                    b2dParticles[i].addRepulsionForce(neighborhoodCenter, fAttractionForce);
                } else if(customData->type == NEU) {
                    b2dParticles[i].addAttractionPoint(neighborhoodCenter, fAttractionForce);                    
                } else if(customData->type == POS) {
                    b2dParticles[i].addAttractionPoint(neighborhoodAnchorTop.getPosition(), fAttractionForce);
                    b2dParticles[i].addRepulsionForce(neighborhoodCenter, fAttractionForce);
                }
                break;
            case CITY:
                if(customData->type == NEG) {
                    b2dParticles[i].addAttractionPoint(cityAnchorBottom.getPosition(), fAttractionForce);
                    b2dParticles[i].addRepulsionForce(cityCenter, fAttractionForce);
                } else if(customData->type == NEU) {
                    b2dParticles[i].addAttractionPoint(cityCenter, fAttractionForce);                    
                } else if(customData->type == POS) {
                    b2dParticles[i].addAttractionPoint(cityAnchorTop.getPosition(), fAttractionForce);
                    b2dParticles[i].addRepulsionForce(cityCenter, fAttractionForce);
                }
                break;
        }
    }
    
    
// MARK: UPDATE ATTRACTION FORCE
//    for(int i = 0; i < b2dParticles.size(); i++) {        
//        Data * customData = (Data*)b2dParticles[i].getData();        
//        b2dParticles[i].addAttractionPoint(customData->attractionPoint, fAttractionForce);
//    }
    
}

//--------------------------------------------------------------
void testApp::draw(){

// MARK: DRAW KINECT POINT CLOUD    
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
        Data * customData = (Data*)p.getData();        
        
        
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
                float sz = pow(prox, 2) * 50.0;
                switch(customData->scope) {
                    case PERSONAL:
                        sz += personalMinParticleSize;
                        break;
                    case NEIGHBORHOOD:
                        sz += neighborhoodMinParticleSize;
                        break;
                    case CITY:
                        sz += cityMinParticleSize;
                        break;
                }
                
                p.setRadius(sz);
            }
        }    

        p.draw();
    }
    
    if(isDebugingBox2d) { 
        ofEnableAlphaBlending();
        
        // DRAW JOINTS
        for(int j = 0; j < b2dJoints.size(); j++) {
            ofSetColor(255, 70);
            b2dJoints[j].draw();        
        }
        
        // DRAW ANCHORS
        ofSetColor(70);
        ofCircle(personalAnchorBottom.getPosition(), 4);
        ofCircle(personalAnchorTop.getPosition(), 4);
        ofCircle(personalAnchorLeft.getPosition(), 4);
        ofCircle(personalAnchorRight.getPosition(), 4);
        ofCircle(neighborhoodAnchorBottom.getPosition(), 4);
        ofCircle(neighborhoodAnchorTop.getPosition(), 4);
        ofCircle(neighborhoodAnchorLeft.getPosition(), 4);
        ofCircle(neighborhoodAnchorRight.getPosition(), 4);
        ofCircle(cityAnchorBottom.getPosition(), 4);
        ofCircle(cityAnchorTop.getPosition(), 4);
        ofCircle(cityAnchorLeft.getPosition(), 4);
        ofCircle(cityAnchorRight.getPosition(), 4);
    }

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
    } else if(key == 'b' || key == 'B') {
        isDebugingBox2d = !isDebugingBox2d;
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