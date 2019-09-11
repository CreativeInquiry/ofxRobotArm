// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//

#pragma once
#include "ofMain.h"
#include "URJoint.h"
namespace ofxRobotArm{
    class RobotParameters{
        public :
        void setup(bool getTCP = true, bool setTCP = true, bool toolOffset = true, bool drawpath = true, bool record = false ) {
            robotArmParams.setName("UR 5");
   
            
            avgAccel.set("avgAccel", 0, 0, 200);
            followLerp.set("followLerp", 1, 0, 1.0);
            
            if (getTCP)
                robotArmParams.add(bCopy.set("get TCP", false));
            if (setTCP)
                robotArmParams.add(bFollow.set("set TCP", false));
            
            robotArmParams.add(bUseOSC.set("Use External OSC", false));
            robotArmParams.add(bUseIK.set("Use IKFast", true));
            robotArmParams.add(bTimeline.set("Use Timeline", false));
            robotArmParams.add(bIKArm.set("Use IKArm", false));
            
  
            
            if (drawpath){
                robotArmParams.add(bTrace.set("bTrace GML", false));
                robotArmParams.add(b3DPath.set("ThreeDPath", false));
                robotArmParams.add(bFigure8.set("bFigure8", false));
            }
            
            robotArmParams.add(bMove.set("Move", false));
            robotArmParams.add(bDoReconnect.set("TryReconnect", false));
            
            joints.setName("Joint Pos");
            targetJoints.setName("Target Joints");
            jointSpeeds.setName("Joint Speeds");
            jointsIK.setName("IK Solver");
            for(int i = 0; i < 6; i++){
                pCurrentPose.push_back(ofParameter<float>());
                joints.add(pCurrentPose.back().set("joint "+ofToString(i), 0, -360, 360));
            }
            
            for(int i = 0; i < 6; i++){
                targetPose.push_back(ofParameter<float>());
                targetJoints.add(targetPose.back().set("target joint "+ofToString(i), 0, -360, 360));
                ikPose.push_back(ofParameter<float>());
                jointsIK.add(ikPose.back().set("ik joint "+ofToString(i), 0, -360, 360));
            }
            
            for(int i = 0; i < 6; i++){
                jointVelocities.push_back(ofParameter<float>());
                jointSpeeds.add(jointVelocities.back().set("Joint Speed"+ofToString(i), 0, -100, 100));
            }
            
            if (record){
                pathRecorderParams.setName("Path Recording");
                pathRecorderParams.add(bRecord.set("Record", false));
            }
            
            
            if (getTCP){
                robotArmParams.add(tcpPosition.set("Actual Robot TCP POS", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
                robotArmParams.add(tcpOrientation.set("Actual Robot TCP ORIENT", ofVec4f(0,0,0,1), ofVec4f(-1,-1,-1,-1), ofVec4f(1,1,1,1)));
                //            robotArmParams.add(calcTCPOrientation.set("Calculated Robot TCP ORIENT", ofVec4f(0,0,0,1), ofVec4f(-1,-1,-1,-1), ofVec4f(1,1,1,1)));
                forwardTCPOrientation.set("Forward TCP ORIENT", ofVec4f(0,0,0,1), ofVec4f(-1,-1,-1,-1), ofVec4f(1,1,1,1));
                //            robotArmParams.add(forwardTCPOrientation.set("Forward TCP ORIENT", ofVec4f(0,0,0,1), ofVec4f(-1,-1,-1,-1), ofVec4f(1,1,1,1)));
                forwardTCPPosition.set("Forward TCP Pos", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1));
                //            robotArmParams.add(forwardTCPPosition.set("Forward TCP Pos", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
            }
            if (setTCP){
                robotArmParams.add(targetTCPPosition.set("Set TCP POS", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
                robotArmParams.add(targetTCPOrientation.set("Set TCP ORIENT",ofVec4f(0,0,0,1), ofVec4f(-1,-1,-1,-1), ofVec4f(1,1,1,1)));
            }
            if (toolOffset){
                robotArmParams.add(tcpOffset.set("tcpOffset", ofVec3f(0, 0, 0), ofVec3f(-0.2, -0.2, -0.2), ofVec3f(0.2, 0.2, 0.2)));
            }
            
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
        ofParameterGroup targetJoints;
        ofParameterGroup jointSpeeds;
        ofParameterGroup jointsIK;
        
        ofParameter<bool> bTimeline;
        
        ofParameter<float> followLerp;
        ofParameter<float> avgAccel;
        vector<ofParameter<float> > ikPose;
        vector<ofParameter<float> > pCurrentPose;
        vector<ofParameter<float> > targetPose;
        vector<ofParameter<float> > jointVelocities;
        
        ofParameter<bool> bMove;
        ofParameter<bool> bFigure8;
        ofParameter<bool> bTrace;
        ofParameter<bool> bFollow;
        ofParameter<bool> bCopy;
        ofParameter<bool> bUseOSC;
        ofParameter<bool> bStop;
        ofParameter<bool> b3DPath;
        ofParameter<bool> bDoReconnect;
        ofParameter<bool> bUseIK;
        ofParameter<bool> bIKArm;
        
        string ipAddress;
        vector<double> currentPose;
        Joint actualTCP;
        Joint targetTCP;
        
    };
}

