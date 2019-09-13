// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//

#pragma once
#include "ofMain.h"
#include "URJoint.h"
namespace ofxRobotArm{
    class RobotParameters{
        public :
        void setup() {
            robotArmParams.setName("Controls");
            
            
            robotArmParams.add(bCopy.set("get TCP", false));
            
            robotArmParams.add(bFollow.set("set TCP", true));
            robotArmParams.add(followLerp.set("Follow Lerp", 0.04, 0.001, 0.99));
            
            robotArmParams.add(bTrace.set("Trace Path", false));
            robotArmParams.add(bTeachMode.set("Teach Mode", false));
            robotArmParams.add(bUseOSC.set("Use External OSC", false));
            robotArmParams.add(bUseIKFast.set("Use IKFast", true));
            robotArmParams.add(bUseTimeline.set("Use Timeline", false));
            robotArmParams.add(bUseIKArm.set("Use IKArm", false));
            
            robotArmParams.add(bLookAtTCP.set("Look at TCP", false));
            robotArmParams.add(bMove.set("Move", false));
            robotArmParams.add(bDoReconnect.set("TryReconnect", false));
            
            joints.setName("Joint Pos");
            targetJoints.setName("Target Joints");
            jointsIK.setName("IK Solver");
            for(int i = 0; i < 6; i++){
                pCurrentPose.push_back(ofParameter<float>());
                joints.add(pCurrentPose.back().set("actual joint "+ofToString(i), 0, -360, 360));
            }
            
            for(int i = 0; i < 6; i++){
                targetPose.push_back(ofParameter<float>());
                targetJoints.add(targetPose.back().set("target joint "+ofToString(i), 0, -360, 360));
                ikPose.push_back(ofParameter<float>());
                jointsIK.add(ikPose.back().set("ik joint "+ofToString(i), 0, -360, 360));
            }
            

            
            pathRecorderParams.setName("Path Recording");
            pathRecorderParams.add(bRecord.set("Record", false));
            
            
            
            
            joints.add(tcpPosition.set("Actual Robot TCP POS", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
            joints.add(tcpOrientation.set("Actual Robot TCP ORIENT", ofVec4f(0,0,0,1), ofVec4f(-1,-1,-1,-1), ofVec4f(1,1,1,1)));
            //            robotArmParams.add(calcTCPOrientation.set("Calculated Robot TCP ORIENT", ofVec4f(0,0,0,1), ofVec4f(-1,-1,-1,-1), ofVec4f(1,1,1,1)));
            forwardTCPOrientation.set("Forward TCP ORIENT", ofVec4f(0,0,0,1), ofVec4f(-1,-1,-1,-1), ofVec4f(1,1,1,1));
            //            robotArmParams.add(forwardTCPOrientation.set("Forward TCP ORIENT", ofVec4f(0,0,0,1), ofVec4f(-1,-1,-1,-1), ofVec4f(1,1,1,1)));
            forwardTCPPosition.set("Forward TCP Pos", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1));
            //            robotArmParams.add(forwardTCPPosition.set("Forward TCP Pos", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
            
            
            //joints.add(targetTCPPosition.set("Set TCP POS", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
           // joints.add(targetTCPOrientation.set("Set TCP ORIENT",ofVec4f(0,0,0,1), ofVec4f(-1,-1,-1,-1), ofVec4f(1,1,1,1)));
            
            //joints.add(tcpOffset.set("tcpOffset", ofVec3f(0, 0, 0), ofVec3f(-0.2, -0.2, -0.2), ofVec3f(0.2, 0.2, 0.2)));
            
            
        };
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
        
        string ipAddress;
        vector<double> currentPose;
        Joint actualTCP;
        Joint targetTCP;
        
    };
}

