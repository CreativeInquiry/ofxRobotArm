

#pragma once
#include "ofMain.h"

class Path{
public:
    Path(){};
    ~Path(){};
    virtual void setup(){};
    
    virtual ofMatrix4x4 getNextPose(){ return ofMatrix4x4();};
    virtual ofMatrix4x4 getPoseAt(int index){return ofMatrix4x4();};
    virtual int getPtIndex(){return -1;};
    virtual void setPtIndex(int index){};
    
    virtual void addPoint(ofVec3f pt){};
    virtual void addPath(vector<ofVec3f> pts){};
    virtual void addPath(ofPolyline line){};
    virtual void addPaths(vector<ofPolyline> lines){};
    
    virtual void draw(){};
    virtual void keyPressed(int key){};
    
    int ptIndex;
    virtual int size(){return 0;};
    ofPolyline path;
    
    // add retract & approach ???
    
protected:
    
};