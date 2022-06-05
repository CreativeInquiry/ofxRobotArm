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
class RelaxedIKSolver : public ofThread {
public:
    RelaxedIKSolver();
    ~RelaxedIKSolver();
    void start();
    void stop();
    void startThread();
    void stopThread();
    void setInitialPose(vector<double> pose);
    void setPose(Pose desiredPose, Pose actualPose);
   
    vector<double> getCurrentPose();
    void threadedFunction();
    bool isThreadRunning();
    bool bThreadStarted;
    Pose desiredPose;
    Pose actualPose;
    Synchronized<vector<double>> currentPose;
    

    uint64_t getFrame();
    uint64_t frameNum;
    uint64_t frameAvg;
};
}

