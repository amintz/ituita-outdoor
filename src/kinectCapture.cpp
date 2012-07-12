//
//  kinectCapture.cpp
//  ituita-outdoor
//
//  Created by André Goes Mintz on 7/11/12.
//  Copyright (c) 2012 __MyCompanyName__. All rights reserved.
//

#include "kinectCapture.h"
#include <iostream>

kinectCapture::kinectCapture() {
    
    //SETUP NORMALIZATION TABLES
    
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

}

kinectCapture::~kinectCapture() {
    close();
}

void kinectCapture::setup(bool _bTwoKinects) {
    
    bTwoKinects = _bTwoKinects;
    
    // SETUP KINECT ONE
    
    fKin1Angle = 0;
    
    bool success = false;
    int counter = 0;
    
    kinect1.init(false, false);
    success = kinect1.open(0);
    
    while (!success && counter < 10) {
        cout << "Problems found in connecting with Kinect 1. Trying again!" << endl;
        kinect1.close();
        kinect1.init(false, false);
        success = kinect1.open(0);
        counter++;
    }
    
    kinect1.setCameraTiltAngle(fKin1Angle);
    cvGrayKin1ThreshNear.allocate(640, 480);
    cvGrayKin1ThreshFar.allocate(640, 480);
    cvGrayKin1.allocate(640, 480);
    
    bKin1Refreshed = false;
    
    //IF USING TWO KINECTS, SETUP KINECT TWO
	
    if(bTwoKinects) {
    
        fKin2Angle = 0;
        
        success = false;
        counter = 0;
        
        kinect2.init(false, false);
        success = kinect2.open(1);
        
        while (!success && counter < 10) {
            cout << "Problems found in connecting with Kinect 2. Trying again!" << endl;
            kinect2.close();
            kinect2.init(false, false);
            success = kinect2.open(1);
            counter++;
        }
        
        kinect2.setCameraTiltAngle(fKin2Angle);
        cvGrayKin2ThreshNear.allocate(640, 480);
        cvGrayKin2ThreshFar.allocate(640, 480);
        cvGrayKin2.allocate(640, 480);
        
        bKin2Refreshed = false;
    
    }
    
    iNearThreshold  = 0;
    iFarThreshold   = 255;
    iMinBlobSize    = 20;
    iMaxBlobSize    = 20000;
    iMaxNumBlobs    = 10;
    
}

void kinectCapture::update() {
    
    kinect1.update();
    bKin1Refreshed = false;
    
    // IF KINECT ONE FRAME IS NEW ---------------------------------
    
    if (kinect1.isFrameNew()) {
        
        bKin1Refreshed = true;
        
        // DO: UPDATE ALL CV STUFF
        
        cvGrayKin1.setFromPixels(kinect1.getDepthPixels(), kinect1.width, kinect1.height);
        
        cvGrayKin1ThreshNear    = cvGrayKin1;
        cvGrayKin1ThreshFar     = cvGrayKin1;
        
        cvGrayKin1ThreshNear.threshold(255 - iNearThreshold, true);
        cvGrayKin1ThreshFar.threshold(255 - iFarThreshold);
        cvAnd(cvGrayKin1ThreshNear.getCvImage(), cvGrayKin1ThreshFar.getCvImage(), cvGrayKin1.getCvImage(), NULL);
        
        cvGrayKin1.flagImageChanged();
        cvContKin1.findContours(cvGrayKin1, iMinBlobSize, iMaxBlobSize, iMaxNumBlobs, false);
        
        kin1FoundBlobs.clear();
        
        // IF THERE ARE BLOBS -------------------------------------
        
        if (cvContKin1.blobs.size() > 0) {
            
            // DO: UPDATE ALL BLOB STUFF
            
            for (int i = 0; i < cvContKin1.blobs.size(); i++) {
                if(cvContKin1.blobs[i].pts.size() > 0) {
                    
                    kin1FoundBlobs.push_back(ofxCvBlob());
                    
                    ofxCvBlob newBlob = cvContKin1.blobs[i];
                    
                    kin1FoundBlobs[i].centroid.x = normWidth((int)cvContKin1.blobs[i].centroid.x, bTwoKinects);
                    kin1FoundBlobs[i].centroid.y = normHeight((int)cvContKin1.blobs[i].centroid.y);
                    kin1FoundBlobs[i].boundingRect.x = normWidth((int)cvContKin1.blobs[i].boundingRect.x, bTwoKinects);
                    kin1FoundBlobs[i].boundingRect.y = normHeight((int)cvContKin1.blobs[i].boundingRect.y);
                    kin1FoundBlobs[i].boundingRect.width = normWidth((int)cvContKin1.blobs[i].boundingRect.width, bTwoKinects);
                    kin1FoundBlobs[i].boundingRect.height = normHeight((int)cvContKin1.blobs[i].boundingRect.height);
                    
                    for (int j = 0; j < cvContKin1.blobs[i].pts.size(); j++) {
                        kin1FoundBlobs[i].pts.push_back(ofPoint(normWidth((int)cvContKin1.blobs[i].pts[j].x, bTwoKinects),normHeight((int)cvContKin1.blobs[i].pts[j].y)));
                    }
                }
            }
        }
        
        // END IF THERE ARE BLOBS ---------------------------------
        
    }
    
    // ENDIF KINECT ONE FRAME IS NEW

    // IF USING TWO KINECTS ---------------------------------------
    
    if (bTwoKinects) {
        
        kinect2.update();
        bKin2Refreshed = false;
        
        // IF KINECT TWO FRAME IS NEW -----------------------------
        
        if (kinect2.isFrameNew()) {
            
            bKin2Refreshed = true;
            
            // DO: UPDATE ALL CV STUFF
            
            cvGrayKin2.setFromPixels(kinect2.getDepthPixels(), kinect2.width, kinect2.height);
            
            cvGrayKin2ThreshNear    = cvGrayKin2;
            cvGrayKin2ThreshFar     = cvGrayKin2;
            
            cvGrayKin2ThreshNear.threshold(255 - iNearThreshold, true);
            cvGrayKin2ThreshFar.threshold(255 - iFarThreshold);
            cvAnd(cvGrayKin2ThreshNear.getCvImage(), cvGrayKin2ThreshFar.getCvImage(), cvGrayKin2.getCvImage(), NULL);
            
            cvGrayKin2.flagImageChanged();
            cvContKin2.findContours(cvGrayKin2, iMinBlobSize, iMaxBlobSize, iMaxNumBlobs, false);
            
            kin2FoundBlobs.clear();
            
            // IF THERE ARE BLOBS ---------------------------------
            
            if (cvContKin2.blobs.size() > 0) {
            
                // DO: UPDATE ALL BLOB STUFF
                
                for (int i = 0; i < cvContKin2.blobs.size(); i++) {
                    
                    if(cvContKin2.blobs[i].pts.size() > 0) {
                        
                        kin2FoundBlobs.push_back(ofxCvBlob());
                        
                        ofxCvBlob newBlob = cvContKin2.blobs[i];
                        
                        kin2FoundBlobs[i].centroid.x = normWidth((int)cvContKin2.blobs[i].centroid.x + KIN_W-KIN2_INTERS_W, true);
                        kin2FoundBlobs[i].centroid.y = normHeight((int)cvContKin2.blobs[i].centroid.y);
                        kin2FoundBlobs[i].boundingRect.x = normWidth((int)cvContKin2.blobs[i].boundingRect.x + KIN_W-KIN2_INTERS_W, true);
                        kin2FoundBlobs[i].boundingRect.y = normHeight((int)cvContKin2.blobs[i].boundingRect.y);
                        kin2FoundBlobs[i].boundingRect.width = normWidth((int)cvContKin2.blobs[i].boundingRect.width, true);
                        kin2FoundBlobs[i].boundingRect.height = normHeight((int)cvContKin2.blobs[i].boundingRect.height);
                        
                        for (int j = 0; j < cvContKin2.blobs[i].pts.size(); j++) {
                            kin2FoundBlobs[i].pts.push_back(ofPoint(normWidth((int)cvContKin2.blobs[i].pts[j].x + KIN_W-KIN2_INTERS_W, true),normHeight((int)cvContKin2.blobs[i].pts[j].y)));
                        }
                    }
                }
            }
            
            // ENDIF THERE ARE BLOBS ------------------------------
                   
        }
        
        // ENDIF KINECT TWO FRAME IS NEW --------------------------
        
        // IF EITHER KINECT FRAME IS NEW --------------------------
        
        if (bKin1Refreshed || bKin2Refreshed) {
            
            // DO: ASSIGN NEW BLOBS TO <FOUND BLOBS>
            
            foundBlobs.clear();
            foundBlobs = kin1FoundBlobs;
            foundBlobs.insert(foundBlobs.end(), kin2FoundBlobs.begin(), kin2FoundBlobs.end());
            
            // DO: ASSIGN NEW CLOUD TO <POINT CLOUD>
            
            pointCloud.clear();
            
            for (int x = 0; x < OUTPUT_W; x++) {
                for (int y = 0; y < KIN_H; y++) {
                    if (x <= KIN2_INTERS_W) {
                        pointCloud.push_back(ofPoint(normWidth(x, true), normHeight(y), normDepth((int)kinect1.getDistanceAt(x, y))));
                    }
                    else if (x > KIN2_INTERS_W && x <= KIN_W) {
                        int minDist = kinect1.getDistanceAt(x, y) < kinect2.getDistanceAt(x - KIN2_INTERS_W, y) ? kinect1.getDistanceAt(x, y) : kinect2.getDistanceAt(x - KIN2_INTERS_W, y);
                        pointCloud.push_back(ofPoint(normWidth(x, true), normHeight(y), normDepth(minDist)));
                    }
                    else if (x > KIN2_INTERS_W) {
                        pointCloud.push_back(ofPoint(normWidth(x, true), normHeight(y), normDepth((int)kinect2.getDistanceAt(x - KIN2_INTERS_W, y))));
                    }
                }
            }
            
        }
        
        // ENDIF EITHER KINECT FRAME IS NEW -----------------------
    
    }
    
    // ELSE (NOT USING TWO KINECTS) -------------------------------
    
    else {
        
        // IF KINECT ONE FRAME IS NEW
        
        if (bKin1Refreshed) {
            
            // DO: ASSIGN NEW BLOBS TO <FOUND BLOBS>
            
            foundBlobs.clear();
            foundBlobs = kin1FoundBlobs;
            
            // DO: ASSIGN NEW CLOUD TO <POINT CLOUD>
            
            pointCloud.clear();
            
            for (int x = 0; x < KIN_W; x++) {
                for (int y = 0; y < KIN_H; y++) {
                    pointCloud.push_back(ofPoint(normWidth(x, false), normHeight(y), normDepth((int)kinect1.getDistanceAt(x,y))));
                }
            }
            
        }
        
    }
    
    // ENDIF USING TWO KINECTS ------------------------------------
}

void kinectCapture::updateThreshPar(int _iFarThreshold, int _iNearThreshold) {
    
    iFarThreshold = _iFarThreshold;
    iNearThreshold= _iNearThreshold;
}

void kinectCapture::updateBlobPar(int _iMinBlobSize, int _iMaxBlobSize, int _iMaxNumBlobs) {
    
    iMinBlobSize = _iMinBlobSize;
    iMaxBlobSize = _iMaxBlobSize;
    iMaxNumBlobs = _iMaxNumBlobs;
}

void kinectCapture::drawDepth(int x, int y, int w, int h, bool kin2) {
    
    if (!kin2) {
        kinect1.drawDepth(x, y, w, h);
    }
    else if (bTwoKinects) {
        kinect2.drawDepth(x, y, w, h);
    }
    
}

void kinectCapture::drawThreshImg(int x, int y, int w, int h, bool kin2) {
    
    if (!kin2) {
        cvGrayKin1.draw(x, y, w, h);
    }
    else if(bTwoKinects) {
        cvGrayKin2.draw(x, y, w, h);
    }
    
}

void kinectCapture::drawContour(int x, int y, int w, int h, bool kin2) {
    
    if (!kin2) {
        cvContKin1.draw(x, y, w, h);
    }
    else if(bTwoKinects) {
        cvContKin2.draw(x, y, w, h);
    }
    
}

void kinectCapture::drawNormBlobs(int x, int y, int w, int h){
    
    ofPushMatrix();
    ofTranslate(x, y);
    ofSetColor(255, 0, 0);
    
    for (int i = 0; i < foundBlobs.size(); i++) {
        ofBeginShape();
        for (int j = 0; j < foundBlobs[i].pts.size(); j++) {
            ofVertex(foundBlobs[i].pts[j].x * w, foundBlobs[i].pts[j].y * h);
        }
        ofEndShape();
    }
    
    ofPopMatrix();
    
}

void kinectCapture::drawDepthFromCloud(int x, int y, int w, int h) {
    
    ofPushMatrix();
    ofTranslate(x,y);
    
    cout<< "Num Pontos: "<< pointCloud.size() << endl;
    
    for (int i = 0; i < pointCloud.size(); i++) {
        ofSetColor(pointCloud[i].z * (float)255);
        ofCircle(pointCloud[i].x * (float)w, pointCloud[i].y * (float)h, 1);
    }
    
    ofPopMatrix();
    
}

void kinectCapture::close() {
    
    kinect1.setCameraTiltAngle(0); // zero the tilt on exit
	kinect1.close();
    
    kinect2.setCameraTiltAngle(0);
	kinect2.close();
    
}

float kinectCapture::normWidth(int val, bool _bTwoKinects) {
    
    if(_bTwoKinects && val < 960) {
        return norm960[val];
    }
    else if(val < 480) {
        return norm480[val];
    }

}

float kinectCapture::normHeight(int val) {
    
    if (val < 480) {
        return norm480[val];
    }
}

float kinectCapture::normDepth(int val) {
    
    if(val < 4000) {
        return norm4000[val];
    }    
    
}

