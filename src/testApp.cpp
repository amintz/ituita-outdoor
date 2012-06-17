#include "testApp.h"

//--------------------------------------------------------------
void testApp::setup(){
    ofSetLogLevel(OF_LOG_VERBOSE);
	
	// enable depth->video image calibration
	kinect.setRegistration(true);
    
	kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	kinect.open();
	
#ifdef USE_TWO_KINECTS
	kinect2.init();
	kinect2.open();
#endif
    
    // zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
    
    bKeyDPressed = false;
    iRightCrop = 0;
    iLeftCrop = 0;
    
    
}

//--------------------------------------------------------------
void testApp::update(){
    
    ofBackground(100, 100, 100);
	
	kinect.update();
    
#ifdef USE_TWO_KINECTS
	kinect2.update();
#endif

}

//--------------------------------------------------------------
void testApp::draw(){
    
    ofSetColor(255, 255, 255);
	
    easyCam.begin();
    drawPointCloud();
    easyCam.end();
    
    if(bOverlay) {
        ofSetColor(0, 0, 0);
        ofRect(10, 10, 340, 500);
        ofSetColor(255, 255, 255);
        kinect.draw(20, 20, 320, 240);
        kinect.drawDepth(20, 260, 320, 240);
    }
}

void testApp::drawPointCloud() {
	int w = 640;
	int h = 480;
	ofMesh mesh;
	mesh.setMode(OF_PRIMITIVE_POINTS);
	int step = 1;
	for(int y = 0; y < h; y += step) {
		for(int x = iLeftCrop; x < 640-iRightCrop; x += step) {
			if(kinect.getDistanceAt(x, y) > 0) {
				mesh.addColor(kinect.getColorAt(x, y));
				mesh.addVertex(kinect.getWorldCoordinateAt(x, y));
			}
		}
	}
	glPointSize(3);
	ofPushMatrix();
	// the projected points are 'upside down' and 'backwards' 
	ofScale(1, -1, -1);
	ofTranslate(0, 0, -1000); // center the points a bit
	glEnable(GL_DEPTH_TEST);
	mesh.drawVertices();
	glDisable(GL_DEPTH_TEST);
	ofPopMatrix();
}

//--------------------------------------------------------------
void testApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	
#ifdef USE_TWO_KINECTS
	kinect2.close();
#endif
}

//--------------------------------------------------------------
void testApp::keyPressed(int key){
    
    switch (key) {
//		case ' ':
//			bThreshWithOpenCV = !bThreshWithOpenCV;
//			break;
//			
		case'p':
			bOverlay = !bOverlay;
			break;
//			
//		case '>':
//		case '.':
//			farThreshold ++;
//			if (farThreshold > 255) farThreshold = 255;
//			break;
//			
//		case '<':
//		case ',':
//			farThreshold --;
//			if (farThreshold < 0) farThreshold = 0;
//			break;
//			
//		case '+':
//		case '=':
//			nearThreshold ++;
//			if (nearThreshold > 255) nearThreshold = 255;
//			break;
//			
//		case '-':
//			nearThreshold --;
//			if (nearThreshold < 0) nearThreshold = 0;
//			break;
			
        case 'd':
            bKeyDPressed = true;
            break;
            
		case 'w':
			kinect.enableDepthNearValueWhite(!kinect.isDepthNearValueWhite());
			break;
			
		case 'o':
			kinect.setCameraTiltAngle(angle); // go back to prev tilt
			kinect.open();
			break;
			
		case 'c':
			kinect.setCameraTiltAngle(0); // zero the tilt
			kinect.close();
			break;
			
		case OF_KEY_UP:
			angle++;
			if(angle>30) angle=30;
			kinect.setCameraTiltAngle(angle);
			break;
			
		case OF_KEY_DOWN:
			angle--;
			if(angle<-30) angle=-30;
			kinect.setCameraTiltAngle(angle);
			break;
            
        case OF_KEY_LEFT:
            if(bKeyDPressed) {
                if (iRightCrop < 650 - iLeftCrop) {
                    iRightCrop ++;
                }
            }
            else {
                if (iLeftCrop > 0) {
                    iLeftCrop --;
                }
            }
            break;
            
        case OF_KEY_RIGHT:
            if(bKeyDPressed) {
                if (iRightCrop > 0 ) {
                    iRightCrop --;
                }
            }
            else {
                if (iLeftCrop < 640 - iRightCrop) {
                    iLeftCrop ++;
                }
                
            }
            break;
            
	}


}

//--------------------------------------------------------------
void testApp::keyReleased(int key){
    switch (key) {
        case 'd':
            bKeyDPressed = false;
            break;
            
        default:
            break;
    }

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