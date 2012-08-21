//
//  ituitaBlobTracker.h
//  ituita-outdoor
//
//  Adapted from ofxBlobTracker, by Ramsin Khoshabeh and Patricio Gonzalez Vivo
//

#ifndef ituita_outdoor_ituitaBlobTracker_h
#define ituita_outdoor_ituitaBlobTracker_h


#include "ofxBlob.h"
#include "ofxContourFinder.h"

class ituitaBlobTracker {

public:
    
	ituitaBlobTracker();
    
    void    update( ofxCvGrayscaleImage& input, int _threshold = -1, 
                   int minArea = 20 ,int maxArea = (340*240)/3, int nConsidered = 10,
                   double hullPress = 20, bool bFindHoles = false, bool bUseApproximation = true);
    
    void    update (ofxCvGrayscaleImage& input, ofxCvGrayscaleImage& prevInput, int _minArea = 20, int _maxArea = (320*240)/3, int _nConsidered = 10, double _hullPress = 20, bool _bFindHoles = false, bool _bUseApproximation = true);

    void    update (ofxCvGrayscaleImage& input, ofxCvGrayscaleImage& prevInput, int _thresholdNear = 0, int _thresholdFar = 255, int _minArea = 20, int _maxArea = (320*240)/3, int _nConsidered = 10, double _hullPress = 20, bool _bFindHoles = false, bool _bUseApproximation = true);
    
    void update(ofxCvGrayscaleImage& input, int _thresholdNear = 0, int _thresholdFar = 255, int _minArea = 20, int _maxArea = (320*240)/3, int _nConsidered = 10, double _hullPress = 20, bool _bFindHoles = false, bool _bUseApproximation = true);
    
    void    draw( float _x = 0, float _y = 0, float _width = 0, float _height = 0);
    
    int     size(){return trackedBlobs.size(); };
    
    ofxBlob operator[](unsigned int _n){ if ( (_n >= 0U) && (_n < trackedBlobs.size()) ) return trackedBlobs[_n]; };
    
    ofEvent<ofxBlob>    blobAdded;
    ofEvent<ofxBlob>    blobMoved;
    ofEvent<ofxBlob>    blobDeleted;
    
    int     movementFiltering;
    bool    bUpdateBackground;
    
	void    track(ofxContourFinder* newBlobs);
	int     trackKnn(ofxContourFinder *newBlobs, ofxBlob *track, int k, double thresh);
    
    ofxContourFinder    contourFinder;
    ofxCvGrayscaleImage backgroundImage;
    ofxCvGrayscaleImage diffImage;
    
    ofxCvGrayscaleImage inputNear, inputFar;
    
	vector<ofxBlob> trackedBlobs;		//tracked blobs
    
	int             IDCounter, numEnter, numLeave, nSize;
    int             width, height;
};



#endif
