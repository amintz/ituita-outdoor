#pragma once

#include "ofMain.h"

#include "ofxSimpleGuiToo.h"

#include "kinectCapture.h"
#include "ituitaBlobTracker.h"
#include "ituitaData.h"

#include "ofxBox2d.h"

// MARK: "USE TWO KINECTS" SWITCH (COMMENT TO USE JUST ONE)

//#define USE_TWO_KINECTS

#define NEG 0
#define NEU 1
#define POS 2

#define PERSONAL     0
#define NEIGHBORHOOD 1
#define CITY         2

#define GREEN  0x00B653
#define YELLOW 0xFAEB34
#define RED    0xED2849

#define FBO_W 1440
#define FBO_H 768
#define OUTPUT_SCREEN_W 1024
#define OUTPUT_SCREEN_H 768


// --------------------------------------------
// MARK : BOX2D CUSTOM DATA

// This is were you can store anything you want.
class Data {
public:
	ofColor color;
	int		type;
    int     scope;
    ofVec2f attractionPoint;
};


// A Custom Particle extedning the box2d circle
class CustomParticle : public ofxBox2dCircle {
	
public:
	
	void setupTheCustomData(int scope, int type, int attractionX, int attractionY) {
		
		static int colors[] = {RED, YELLOW, GREEN};
		
		// we are using a Data pointer because 
		// box2d needs to have a pointer not 
		// a referance
		setData(new Data());
		Data * cutomData = (Data*)getData();
		
        cutomData->scope = scope;
		cutomData->type = type;
		cutomData->color.setHex(colors[type]);
        cutomData->attractionPoint.set(attractionX, attractionY);        
	}
	
	void draw() {
		Data* cutomData = (Data*)getData();
		if(cutomData) {
			
			// Evan though we know the data object lets just 
			// see how we can get the data out from box2d
			// you would use this when using a contact listener
			// or tapping into box2d's solver.
			float radius = getRadius();
			ofPushMatrix();
			ofTranslate(getPosition());
			ofRotateZ(getRotation());
			ofSetColor(cutomData->color);
			ofFill();
			ofCircle(0, 0, radius);			
			ofPopMatrix();
		}
	}
    
};



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
   
        void            drawGUI();
        ofxSimpleGuiToo gui;
        bool            isGUIActive;

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
        
        int     iMaxRandomParticles, iDeltaRandomParticles;
        float   fAttractionForce;
        bool    bResetData;
        

// --------------------------------------------
// MARK: DATA

        ituitaData data;
        int ppos, pneu, pneg;
        int npos, nneu, nneg;
        int cpos, cneu, cneg;
    
// --------------------------------------------
// MARK: SHADER
        
        ofShader shader;
        bool    isFilterActive;
        int     ledRatio;
    
// --------------------------------------------
// MARK: BOX2D

        void setupData();
        void addParticles(int scope, int type, int num);
    
        ofxBox2d				box2d;			  //	the box2d world
        vector<CustomParticle>	particles;		  //	default box2d circles
    
        ofVec2f personalCenter;
        ofVec2f neighborhoodCenter;
        ofVec2f cityCenter;
    

// --------------------------------------------
// MARK: GRAPHICS
        ofFbo fbo;
    
};


