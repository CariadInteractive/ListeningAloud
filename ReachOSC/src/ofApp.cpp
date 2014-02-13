#include "ofApp.h"

using namespace ofxCv;
using namespace cv;

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
    ofBackground(0);
	
	// enable depth->video image calibration
	kinect.setRegistration(false);
    
	//kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	kinect.init(false, false); // disable video image (faster fps)
	
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	
	// print the intrinsic IR sensor values
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
	
    // depth image stuff
    depthImg.allocate(kinect.width, kinect.height);
    warpedDepthImg.allocate(kinect.width, kinect.height);
    greyThreshNear.allocate(kinect.width, kinect.height);
    greyThreshFar.allocate(kinect.width, kinect.height);
    greyThreshedWarped.allocate(kinect.width, kinect.height);
	
	nearThreshold = 230;
	farThreshold = 70;
	
	ofSetFrameRate(60);
    
    trackerMinAreaRadius.addListener(this, &ofApp::trackerMinAreaRadiusChanged);
    trackerMaxAreaRadius.addListener(this, &ofApp::trackerMaxAreaRadiusChanged);
    trackerThreshold.addListener(this, &ofApp::trackerThresholdChanged);
    trackerPeristenceBeforeForgetting.addListener(this, &ofApp::trackerPeristenceBeforeForgettingChanged);
    trackerMaximumDistancePerFrame.addListener(this, &ofApp::trackerMaximumDistancePerFrameChanged);
    
    ofSetWindowTitle("ReachOSC");
    gui.setup();
    //crop
    gui.add(leftCrop.setup("Crop Left", 0.f, 0.f, 1.f));
    gui.add(rightCrop.setup("Crop Right", 1.f, 0.f, 1.f));
    gui.add(topCrop.setup("Crop Top", 0.f, 0.f, 1.f));
    gui.add(bottomCrop.setup("Crop Bottom", 1.f, 0.f, 1.f));
    gui.add(nearThreshold.setup("Crop Near", 255,0,255));
    gui.add(farThreshold.setup("Crop Far", 240,0,255));
    gui.add(trackerMinAreaRadius.setup("Track Min Area Radius", 1, 1,200));
	gui.add(trackerMaxAreaRadius.setup("Track Max Area Radius", 100, 1,200));
    gui.add(trackerThreshold.setup("Track Threshold", 15, 0,255));
    gui.add(trackerPeristenceBeforeForgetting.setup("Frames of persistence", 15, 1,30));
    gui.add(trackerMaximumDistancePerFrame.setup("Pixels per frame", 32, 1,64));
    
    bShow = true;
    
    //osc
    sender.setup(HOST, PORT);
    
    //gui.loadFromFile("settings.xml");
}

void ofApp::trackerMinAreaRadiusChanged(int & aMinAreaRadius){
    contourFinder.setMinAreaRadius(aMinAreaRadius);
}

void ofApp::trackerMaxAreaRadiusChanged(int & aMaxAreaRadius){
    contourFinder.setMaxAreaRadius(aMaxAreaRadius);
    
}

void ofApp::trackerThresholdChanged(int & aThreshold){
    contourFinder.setThreshold(aThreshold);
    
}
void ofApp::trackerPeristenceBeforeForgettingChanged(int & aPersistence){
    // wait for half a frame before forgetting something
    contourFinder.getTracker().setPersistence(aPersistence);
}

void ofApp::trackerMaximumDistancePerFrameChanged(int & aDistance){
    // an object can move up to 32 pixels per frame
    contourFinder.getTracker().setMaximumDistance(aDistance);
}

//--------------------------------------------------------------
void ofApp::update() {
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		// load grayscale depth image from the kinect source
		depthImg.setFromPixels(kinect.getDepthPixels(), kinect.width, kinect.height);
        
        float cropLeftInPixels = leftCrop * depthImg.getWidth();
        float cropRightInPixels = rightCrop * depthImg.getWidth();
        float cropTopInPixels = topCrop * depthImg.getHeight();
        float cropBottomInPixels = bottomCrop * depthImg.getHeight();
        
        ofPoint sourcePoints[4];
        sourcePoints[0] = ofPoint(cropLeftInPixels,cropTopInPixels);
        sourcePoints[1] = ofPoint(cropRightInPixels,cropTopInPixels);
        sourcePoints[2] = ofPoint(cropRightInPixels, cropBottomInPixels);
        sourcePoints[3] = ofPoint(cropLeftInPixels, cropBottomInPixels);
        
        ofPoint destPoints[4];
        destPoints[0] = ofPoint(0,0);
        destPoints[1] = ofPoint(depthImg.getWidth(),0);
        destPoints[2] = ofPoint(depthImg.getWidth(), depthImg.getHeight());
        destPoints[3] = ofPoint(0, depthImg.getHeight());
        
        //          http://forum.openframeworks.cc/t/simple-zoom-function-before-opencvs-blob-detection/6151 thanks obviousjim and irregular
        //        https://gist.github.com/yoggy/3246274
        
        
        //set the source to the bounds of the image, set the dest to the points where you want to crop out
        warpedDepthImg.warpIntoMe(depthImg, sourcePoints, destPoints);
        warpedDepthImg.flagImageChanged(); //TODO: is this necessary? either?
        
        greyThreshedWarped.set(0);
        
        greyThreshNear = warpedDepthImg;
        greyThreshFar = warpedDepthImg;
        greyThreshNear.threshold(nearThreshold, true);
        greyThreshFar.threshold(farThreshold);
        cvAnd(greyThreshNear.getCvImage(), greyThreshFar.getCvImage(), greyThreshedWarped.getCvImage(), NULL);
        
        greyThreshedWarped.flagImageChanged(); //TODO: is this necessary?
        
        contourFinder.findContours(greyThreshedWarped);
        
        ofxOscMessage m;
        m.setAddress("/kinect/blobs");
        
        for(int i = 0; i < contourFinder.size(); i++) {
            
            ofPoint centre = toOf(contourFinder.getCenter(i));
            ofColor zDepthAtCentre = warpedDepthImg.getPixelsRef().getColor(centre.x, centre.y);
            
            int blobID = contourFinder.getLabel(i);
            float blobXPosNormalised = centre.x / warpedDepthImg.getWidth();
            float blobYPosNormalised = centre.y / warpedDepthImg.getHeight();
            
            float zBrightness = zDepthAtCentre.getBrightness();
            
            float blobZDepthNormalised = ofNormalize(zBrightness, (float)farThreshold, (float)nearThreshold);
            
            //string msg = ofToString(label) + ":" + ofToString(tracker.getAge(label));
            //ofVec2f velocity = toOf(contourFinder.getVelocity(i));
            
            m.addIntArg(blobID); //ID of blob
            m.addFloatArg(blobXPosNormalised); //x pos (0..1) of blob
            m.addFloatArg(blobYPosNormalised); //y pos (0..1) of blob
            m.addFloatArg(blobZDepthNormalised); //z pos (0..1) of blob
            
//            cout << "Sending message, blob ID: " << blobID << "x: " << blobXPosNormalised << "y: " << blobYPosNormalised << "z: " << blobZDepthNormalised <<  endl;
        }
        
        sender.sendMessage(m);
	}
}

//--------------------------------------------------------------
void ofApp::draw() {
	ofSetColor(255, 255, 255);
    greyThreshedWarped.draw(0,0);
    //tracker
	RectTracker& tracker = contourFinder.getTracker();
    ofSetColor(255);
    
    float border = 14;
    ofDrawBitmapStringHighlight("Cropped and Thresholded Depth Image with Blobs and Labels",0, border);
    
    contourFinder.draw();
    
    for(int i = 0; i < contourFinder.size(); i++) {
        ofPoint center = toOf(contourFinder.getCenter(i));
        ofPushMatrix();
        ofTranslate(center.x, center.y);
        int label = contourFinder.getLabel(i);
        string msg = ofToString(label) + ":" + ofToString(tracker.getAge(label));
        ofDrawBitmapString(msg, 0, 0);
        ofVec2f velocity = toOf(contourFinder.getVelocity(i));
        ofScale(5, 5);
        ofLine(0, 0, velocity.x, velocity.y);
        ofPopMatrix();
    }
    
    float imageWidth = 320;
    float imageHeight = 240;
    
    float drawX = 0;
    float drawY = 0;
    
    drawY += kinect.getHeight();
    
    warpedDepthImg.draw(drawX, drawY, imageWidth, imageHeight);
    ofDrawBitmapStringHighlight("Cropped Depth Image", drawX, drawY+border);
    
    drawX += imageWidth+2;
    
    // draw from the live kinect
    depthImg.draw(drawX, drawY, imageWidth, imageHeight);
    ofDrawBitmapStringHighlight("Original Depth Image", drawX, drawY+border);
    
	
    //	// this chunk of code visualizes the creation and destruction of labels
    //	const vector<unsigned int>& currentLabels = tracker.getCurrentLabels();
    //	const vector<unsigned int>& previousLabels = tracker.getPreviousLabels();
    //	const vector<unsigned int>& newLabels = tracker.getNewLabels();
    //	const vector<unsigned int>& deadLabels = tracker.getDeadLabels();
    //	ofSetColor(cyanPrint);
    //	for(int i = 0; i < currentLabels.size(); i++) {
    //		int j = currentLabels[i];
    //		ofLine(j, 0, j, 4);
    //	}
    //	ofSetColor(magentaPrint);
    //	for(int i = 0; i < previousLabels.size(); i++) {
    //		int j = previousLabels[i];
    //		ofLine(j, 4, j, 8);
    //	}
    //	ofSetColor(yellowPrint);
    //	for(int i = 0; i < newLabels.size(); i++) {
    //		int j = newLabels[i];
    //		ofLine(j, 8, j, 12);
    //	}
    //	ofSetColor(ofColor::white);
    //	for(int i = 0; i < deadLabels.size(); i++) {
    //		int j = deadLabels[i];
    //		ofLine(j, 12, j, 16);
    //	}
    
    // auto draw?
	// should the gui control hiding?
	if( bShow ){
		gui.draw();
	}
}

//--------------------------------------------------------------
void ofApp::exit() {
	kinect.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
	switch (key) {
        case 'f':
            ofToggleFullscreen();
            break;
        case 'g':
            bShow = !bShow;
            break;
        default:
            break;
	}
}