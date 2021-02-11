//
//  RelaxedIKThread.hpp
//  example-simple
//
//  Created by Dan Moore on 1/2/21.
//
#pragma once
#include "ofMain.h"
#include "ofxTiming.h"
#include "Pose.h"
#include "Synchronized.h"


namespace ofxRobotArm{
class RelaxedSolver : public ofThread {
public:
    RelaxedSolver();
    ~RelaxedSolver();
    void start();
    void stop();
    void startThread();
    void stopThread();
    void setConfigurationPose(vector<double> pose);
    void setInitialPose(Pose pose);
    void setPose(Pose desiredPose);
   
    vector<double> getCurrentPose();
    void threadedFunction();
    bool isThreadRunning();
    bool bThreadStarted;
    Pose desiredPose;
    Pose initialPose;
    Synchronized<vector<double>> currentPose;
    
    void setAngle(double angleX, double angleY, double angleZ);
    double angleX, angleY, angleZ;
    

    ofVec3f u, v, w;
    void setMatrix(ofVec3f u, ofVec3f v, ofVec3f w);
    ofMatrix4x4 mat;
    uint64_t getFrame();
    uint64_t frameNum;
    uint64_t frameAvg;
};
}

