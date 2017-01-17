//
//  PathController.h
//  urModernDriverTest
//
//  Created by dantheman on 4/4/16.
//
//
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
#pragma once
#include "ofMain.h"
#include "Path.h"
#include "GMLPath.h"
#include "Path3D.h"
class PathController{
public:
    PathController();
    ~PathController();
    
    enum PathState{
        NOT_READY = 0,
        LOADED,
        DRAWING,
        PAUSED,
        FINISHED,
        READY
    };
    
    enum PathType{
        BASE_PATH = 0,
        GML_PATH,
        TWO_D_PATH,
        THREE_D_PATH,
        RECORDED_PATH
    };
    
    void setup();
    void setup(vector<Path *> paths);

    void update();
    ofMatrix4x4 getNextPose();
    void draw();
    void addPath(Path *path);
    void pauseDrawing();
    void startDrawing();
    void endDrawing();
    void loadPath(string file);
    int size();
    
    vector<Path *> paths;
    int pathIndex;
    
    PathState currentState;
    
    void keyPressed(int key);
    
    bool isDone;
    bool pause;
    
private:
    
};