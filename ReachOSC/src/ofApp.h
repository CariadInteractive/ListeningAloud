#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxKinect.h"
#include "ofxGui.h"
#include "ofxCv.h"
#include "ofxOsc.h"

#define HOST "localhost"
#define PORT 12345

class ofApp : public ofBaseApp{

	void setup();
	void update();
	void draw();
	void exit();
	
	void keyPressed(int key);
//    void keyReleased(int key);
//    void mouseMoved(int x, int y );
//	void mouseDragged(int x, int y, int button);
//	void mousePressed(int x, int y, int button);
//	void mouseReleased(int x, int y, int button);
//	void windowResized(int w, int h);
    
    //new events
    void trackerMinAreaRadiusChanged(int & aMinAreaRadius);
    void trackerMaxAreaRadiusChanged(int & aMaxAreaRadius);
    void trackerThresholdChanged(int & aThreshold);
    void trackerPeristenceBeforeForgettingChanged(int & aPersistence);
    void trackerMaximumDistancePerFrameChanged(int & aDistance);
	
	ofxKinect kinect;
	
    ofxCvGrayscaleImage			depthImg;
    ofxCvGrayscaleImage         warpedDepthImg;
    ofxCvGrayscaleImage greyThreshNear; // the near thresholded image
	ofxCvGrayscaleImage greyThreshFar; // the far thresholded image
    
    ofxCvGrayscaleImage greyThreshedWarped;
    
    //gui
    bool bShow;
    
	ofxPanel gui;
    
    // crop
    ofxFloatSlider leftCrop;
    ofxFloatSlider rightCrop;
    ofxFloatSlider topCrop;
    ofxFloatSlider bottomCrop;
    
    // depth
    ofxIntSlider	nearThreshold;
    ofxIntSlider	farThreshold;
    
    //tracking
	ofxCv::ContourFinder contourFinder;
    
    //tracking gui
    ofxIntSlider trackerMinAreaRadius;
    ofxIntSlider trackerMaxAreaRadius;
    ofxIntSlider trackerThreshold;
    ofxIntSlider trackerPeristenceBeforeForgetting;
    ofxIntSlider trackerMaximumDistancePerFrame;
    
    //osc
    ofxOscSender sender;
		
};
