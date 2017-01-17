#pragma once
#include "ofMain.h"

class PathPlayer{
    PathPlayer();
    ~PathPlayer();
    
    void setup();
    void update();
    void draw();
    void getNextPose(float t);
};