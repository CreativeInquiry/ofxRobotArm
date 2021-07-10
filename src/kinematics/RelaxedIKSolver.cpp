//
//  Solver.cpp
//  example-simple
//
//  Created by Dan Moore on 1/2/21.
//

#include "RelaxedIKSolver.h"
#include "RelaxedIK.hpp"
using namespace ofxRobotArm;

RelaxedIKSolver::RelaxedIKSolver(){
    ofLog()<<"RelaxedIKSolver THREAD"<<endl;
    vector<double> foo(6., 0.0);
    currentPose.setup(foo);
    currentPose.getBack().assign(6, 0.0);
    currentPose.swapBack();
    frameNum = 0;
}

RelaxedIKSolver::~RelaxedIKSolver(){
    stop();
    waitForThread(false);
}

void RelaxedIKSolver::start(){
    startThread();
}
void RelaxedIKSolver::stop(){
    stopThread();
}
void RelaxedIKSolver::startThread(){
    bThreadStarted = true;
    ofThread::startThread();
}
void RelaxedIKSolver::stopThread(){
    if(isThreadRunning()){
        bThreadStarted = false;
        ofThread::stopThread();
    }
}
void RelaxedIKSolver::setPose(Pose desiredPose, Pose actualPose){
    lock();
    this->desiredPose = desiredPose;
    this->actualPose = actualPose;
    unlock();
}
vector<double> RelaxedIKSolver::getCurrentPose(){
    vector<double> ret;
    lock();
    currentPose.swapFront();
    ret = currentPose.getFront();;
    unlock();
    return ret;
}

void RelaxedIKSolver::setInitialPose(vector<double> pose){
    lock();
    // set_starting_config(pose.data(), pose.size());
    unlock();
}

void RelaxedIKSolver::setAngle(double angleX, double angleY, double angleZ){
    lock();
    this->angleX = angleX;
    this->angleY = angleY;
    this->angleZ = angleZ;
    unlock();
}

void RelaxedIKSolver::threadedFunction(){
    while(isThreadRunning()){
        lock();
                
        ofVec3f difPos = (desiredPose.position - actualPose.position);
        ofQuaternion rot  = (actualPose.orientation * desiredPose.orientation);
        ofVec4f r = ofVec4f(rot.x(), rot.y(), rot.z(), rot.w());
   
        std::vector<double> pos(3, 0.0);
        pos[0] = difPos.x;
        pos[1] = difPos.y;
        pos[2] = difPos.z;
        
        std::vector<double> quat(4, 0.0);
        quat[0] = r.x;
        quat[1] = r.y;
        quat[2] = r.z;
        quat[3] = r.w;
        
        Opt x = solve(pos.data(), (int) pos.size(), quat.data(), (int) quat.size());
        for (int i = 0; i < x.length; i++) {
            currentPose.getBack()[i] = x.data[i];
        }
        frameNum++;
        if(frameNum > 4000)
            frameNum = 0;
        currentPose.swapBack();
        unlock();
        
        ofSleepMillis(1);
    }
}

bool RelaxedIKSolver::isThreadRunning(){
    bool ret;
    lock();
    ret = bThreadStarted;
    unlock();
    return ret;
}

uint64_t RelaxedIKSolver::getFrame(){
    uint64_t t = 0;
    lock();
    frameAvg +=frameNum;
    frameAvg /=2;
    t = frameAvg;
    frameNum = 0;
    unlock();
    return t;
}
