//
//  KinectModel.h
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//

#pragma once
#include "ofMain.h"
#include "ofxAssimpModelLoader.h"
#include "Synchronized.h"
#include "Pose.h"
#include "Utils.h"
#include "RobotConstants.hpp"

//#include "ofxBullet.h"
namespace ofxRobotArm{
class RobotModel{
public:
    RobotModel();
    ~RobotModel();
    void setup(RobotType type);
    void drawSkeleton();
    void drawMesh(ofFloatColor color = ofFloatColor(1, 1, 1, 1), bool bDrawDebug=true);
    void draw(ofFloatColor color = ofFloatColor(1, 1, 1, 1), bool bDrawDebug=true);
    void setToolMesh(ofMesh mesh);
    void setPose(vector<double> pose);
    void setTCPPose(Pose pose);
    void setForwardPose(ofNode pose);
    void setEndEffector(string filename);
    void clearEndEffector();
    
    ofNode getTool();
    void setToolOffset(ofVec3f localOffset);
    ofQuaternion getToolPointQuaternion();
    Pose getModifiedTCPPose();
    
    RobotType type;
    ofxAssimpModelLoader loader;
    vector<ofMesh> meshes;
    ofMesh toolMesh;
    
    float elapsed_time, last_time;
    ofVec3f pt;
    vector<ofxRobotArm::Pose> pose;
    vector<double> poseRadians;
    Pose tool;
    
    Pose dtoolPoint;
    ofVec3f localOffset;
    ofNode forwardPose;
    ofNode tcpNode;
    ofNode toolNode;
    vector<ofNode> nodes;
    
    ofParameter<float> stage;
    ofParameter<bool> bDrawModel;

};
}
