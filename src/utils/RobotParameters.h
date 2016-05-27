//
//  RobotParameters.h
//  urModernDriverTest
//
//  Created by dantheman on 4/4/16.
//
//

#pragma once
#include "ofMain.h"
class RobotParameters{
    public :
    void setup(bool getTCP = true, bool setTCP = true, bool toolOffset = true, bool drawpath = true, bool record = true){
        robotArmParams.setName("UR 5");
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
        
        // should move to URMove
        robotArmParams.add(avgAccel.set("avgAccel", 0, 0, 200));
        robotArmParams.add(followLerp.set("followLerp", 1, 0, 1.0));
        
        if (getTCP)
            robotArmParams.add(bCopy.set("get TCP", false));
        if (setTCP)
            robotArmParams.add(bFollow.set("set TCP", false));
        
        if (drawpath){
            robotArmParams.add(bTrace.set("bTrace GML", false));
            robotArmParams.add(b3DPath.set("3DPath", false));
            robotArmParams.add(bFigure8.set("bFigure8", false));
        }
        
        robotArmParams.add(bMove.set("Move", false));
        
        
        joints.setName("Joint Pos");
        targetJoints.setName("Target Joints");
        jointSpeeds.setName("Joint Speeds");
        jointsIK.setName("IK Solver");
        for(int i = 0; i < 6; i++){
            jointPos.push_back(ofParameter<float>());
            joints.add(jointPos.back().set("joint "+ofToString(i), 0, -360, 360));
        }
        
        for(int i = 0; i < 6; i++){
            targetJointPos.push_back(ofParameter<float>());
            targetJoints.add(targetJointPos.back().set("target joint "+ofToString(i), 0, -360, 360));
            jointPosIKRaw.push_back(ofParameter<float>());
            jointsIK.add(jointPosIKRaw.back().set("ik joint "+ofToString(i), 0, -360, 360));
        }
        
        for(int i = 0; i < 6; i++){
            jointVelocities.push_back(ofParameter<float>());
            jointSpeeds.add(jointVelocities.back().set("Joint Speed"+ofToString(i), 0, -100, 100));
        }
        
        if (record){
            pathRecorderParams.setName("Path Recording");
            pathRecorderParams.add(bRecord.set("Record", false));
        }
        
    };
    ofParameterGroup robotArmParams;
    ofParameter<ofVec3f> targetTCPPosition;
    ofParameter<ofVec4f> targetTCPOrientation;
    ofParameter<ofVec4f> tcpOrientation;
    //    ofParameter<ofVec4f> calcTCPOrientation;
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
    
    
    
    ofParameter<float> followLerp;
    ofParameter<float> avgAccel;
    vector<ofParameter<float> > jointPosIKRaw;
    vector<ofParameter<float> > jointPos;
    vector<ofParameter<float> > targetJointPos;
    vector<ofParameter<float> > jointVelocities;
    
    ofParameter<bool> bMove;
    ofParameter<bool> bFigure8;
    ofParameter<bool> bTrace;
    ofParameter<bool> bFollow;
    ofParameter<bool> bCopy;
    ofParameter<bool> bStop;
    ofParameter<bool> b3DPath;
    
    vector<double> currentJointPos;
    Joint actualTCP;
    Joint targetTCP;
        
};

