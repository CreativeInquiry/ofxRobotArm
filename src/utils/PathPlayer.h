#pragma once
#include "ofMain.h"
//
// Copyright (c) 2016, 2021 Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
////
class PathPlayer{
    PathPlayer();
    ~PathPlayer();
    
    void setup();
    void update();
    void draw();
    void getNextPose(float t);
};