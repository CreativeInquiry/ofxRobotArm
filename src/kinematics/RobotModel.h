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
            void update();
            void drawSkeleton();
            void draw(ofFloatColor color = ofFloatColor(1, 1, 1, 1), bool bDrawDebug=true);
            void setToolMesh(ofMesh mesh);
            void setPose(vector<double> pose);
            
            void setEndEffector(string filename);
            void clearEndEffector();
            
            void getArmIK( ofVec3f aTargetWorldPos, ofVec3f aElbowWorldPos, bool aBInvertElbow, float aDeltaTimef );
            
            ofNode getTool();
            void setToolOffset(ofVec3f localOffset);
            ofQuaternion getToolPointQuaternion();
            
            void setAngles( vector<double> aTargetRadians );
            
            RobotType type;
            ofxAssimpModelLoader loader;
            vector<ofMesh> meshs;
            ofMesh toolMesh;
            
            ofShader shader;
            float elapsed_time, last_time;
            ofVec3f pt;
            vector<ofxRobotArm::Pose> pose;
            
            Pose tool;
            
            Pose dtoolPoint;
            
            ofNode tcpNode;
            vector<ofNode> nodes;
            
            ofParameter<float> stage;
            ofParameter<bool> bDrawModel;
            ofParameter<bool> bDrawTargetModel;
            ofParameter<bool> bUseShader;
            
    };
}
