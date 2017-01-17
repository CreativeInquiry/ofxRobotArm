#include "PathRecorder.h"
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
PathRecorder::PathRecorder(){
    recording = false;
}
PathRecorder::~PathRecorder(){
    
}
void PathRecorder::draw(){
    ofDrawBitmapString("Path Recording "+ofToString(count), ofGetWindowWidth()-20, ofGetWindowHeight()-20);
}
void PathRecorder::startRecording(){
    if(!recording){
        startTime = ofGetElapsedTimef();
        count = 0;
        path =ofToDataPath(ofGetTimestampString()+".rp");
        file.open(path, ofFile::WriteOnly);
    }
    recording = true;
}
void PathRecorder::endRecording(){
    if(recording){
        file.close();
        recording = false;
    }
    
}
void PathRecorder::addPose(vector<double> pose, float timestamp){
    for(int i = 0; i < pose.size(); i++){
        file<<pose[i];
        file<<",";
    }
    file<<(timestamp-startTime);
    file<<"\n";
    
    count++;
}