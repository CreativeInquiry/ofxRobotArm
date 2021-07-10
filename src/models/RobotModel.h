//
//  KinectModel.h
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
//
// Copyright (c) 2016, 2021 Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
////

#pragma once
#include "ofMain.h"
#include "ofxAssimpModelLoader.h"
#include "ofxSTL.h"
#include "Synchronized.h"
#include "ofxXmlSettings.h"
#include "Pose.h"
#include "RobotConstants.hpp"

namespace ofxRobotArm
{

    class RobotModel
    {
    public:
        RobotModel();
        ~RobotModel();
        // void setup(RobotType type);
        void setup(string path, RobotType type);

        void setOrigin(ofNode node);
        void setOrigin(ofVec3f pos, ofQuaternion orientation);
  
        void loadURDF(string path, RobotType type);
        void loadModel(string path);

        void drawSkeleton();
        void drawMesh(ofColor color = ofColor::white, bool bDrawDebug = true);
        void draw(ofColor color = ofColor::white, bool bDrawDebug = true);
        void drawArc(float aStartAngleDegrees, float aEndAngleDegrees, ofVec3f aForwardAxis, ofVec3f aSideAxis,  bool fill = false);
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
        ofNode getForwardPose();

        RobotType type;
        ofxAssimpModelLoader loader;
        vector<ofMesh> meshes;
        ofMesh toolMesh;
        float elapsed_time, last_time;
        ofVec3f pt;
        vector<ofxRobotArm::Pose> pose;
        vector<double> poseRadians;
        vector<double> jointMin;
        vector<double> jointMax;
        Pose tool;


        ofNode originNode;
        Pose dtoolPoint;
        ofVec3f localOffset;
        ofNode forwardPose;
        ofNode tcpNode;
        ofNode toolNode;
        vector<ofNode> nodes;
    };
}
