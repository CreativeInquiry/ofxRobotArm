//
//  URToolHead.cpp
//  ofxURDriver
//
//  Created by Dan Moore on 2/20/16.
// Copyright (c) 2016,2020 Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
#include "ToolHead.h"
using namespace ofxRobotArm;
void ToolHead::setup(){
    
}
void ToolHead::setOrientation(ofQuaternion orientation){
    orientation.get(rot);
}
void ToolHead::update(){
    
}
ofMatrix4x4 ToolHead::getMatrix(){
    return rot;
}
void ToolHead::draw(){
    currentTool.mesh.draw();
    ofPushMatrix();
    float angle;
    ofVec3f axis;
    rot.getRotate().getRotate(angle, axis);
//to-do: change to ofRotateDeg or ofRotateRad
//    ofRotate(angle, axis.x, axis.y, axis.z);
    ofRotateDeg(angle, axis.x, axis.y, axis.z);
    ofDrawAxis(100);
    ofPopMatrix();
}
void ToolHead::setTool(Tool t){
    
}
Tool ToolHead::getCurrentTool(){
    
}
