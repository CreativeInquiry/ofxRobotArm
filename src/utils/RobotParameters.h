//
// Copyright (c) 2016, 2021 Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
////

#pragma once
#include "ofMain.h"
#include "Pose.h"
#include "URDriver.h"

namespace ofxRobotArm{
    class RobotParameters{
        public :
        void setup(string path){
            
        }
        void setup(RobotType type=RobotType::UR5) {
           
            
        };
        
        RobotType getRobotType(){ return type; }
        
        ofParameterGroup robotArmParams;
        ofParameter<ofVec3f> targetTCPPosition;
        ofParameter<ofVec4f> targetTCPOrientation;
        ofParameter<ofVec4f> tcpOrientation;
        ofParameter<ofVec4f> calcTCPOrientation;
        ofParameter<ofVec4f> forwardTCPOrientation;
        ofParameter<ofVec3f> forwardTCPPosition;
        ofParameter<ofVec3f> tcpPosition;
        ofParameter<ofVec3f> tcpOffset;
        
        ofParameter<bool> bRecord;
        ofParameterGroup pathRecorderParams;
        
        ofParameterGroup joints;
        ofParameterGroup safety;
        ofParameterGroup targetJoints;
        ofParameterGroup jointSpeeds;
        ofParameterGroup jointsIK;
        
        ofParameter<bool> bUseTimeline;
        
        ofParameter<float> followLerp;
        ofParameter<float> poseLerp;
        
        vector<ofParameter<float> > ikPose;
        vector<ofParameter<float> > pCurrentPose;
        vector<ofParameter<float> > targetPose;
        vector<ofParameter<float> > jointVelocities;
        
        ofParameter<bool> bMove;
        ofParameter<bool> bTeachMode;
        ofParameter<bool> bTrace;
        ofParameter<bool> bFollow;
        ofParameter<bool> bCopy;
        ofParameter<bool> bUseOSC;
        ofParameter<bool> bStop;
        ofParameter<bool> bLookAtTCP;
        ofParameter<bool> bDoReconnect;
        ofParameter<bool> bUseIKFast;
        ofParameter<bool> bUseIKArm;
        ofParameter<bool> bSettingPoseExternally;
        string ipAddress;
        vector<double> currentPose;
        Pose actualTCP;
        Pose targetTCP;
        
        RobotType type;
        
    };
}

