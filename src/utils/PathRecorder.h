// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
#pragma once
#include "ofMain.h"

class PathRecorder{
public:
    PathRecorder();
    ~PathRecorder();

    void draw();
    void startRecording();
    void endRecording();
    void addPose(vector<double> pose, float timestamp);
    
    bool recording;
    string path;
    ofFile file;
    float startTime;
    int count;
    
};