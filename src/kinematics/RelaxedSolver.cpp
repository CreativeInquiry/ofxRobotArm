//
//  RelaxedSolver.cpp
//  example-simple
//
//  Created by Dan Moore on 1/2/21.
//

#include "RelaxedSolver.h"
#include "RelaxedIK.hpp"
using namespace ofxRobotArm;

RelaxedSolver::RelaxedSolver(){
    vector<double> foo(6., 0.0);
    currentPose.setup(foo);
    currentPose.getBack().assign(6, 0.0);
    currentPose.swapBack();
    frameNum = 0;
}

RelaxedSolver::~RelaxedSolver(){
    stop();
    waitForThread(false);
}

void RelaxedSolver::start(){
    startThread();
}
void RelaxedSolver::stop(){
    stopThread();
}
void RelaxedSolver::startThread(){
    bThreadStarted = true;
    ofThread::startThread();
}
void RelaxedSolver::stopThread(){
    if(isThreadRunning()){
        bThreadStarted = false;
        ofThread::stopThread();
    }
}

void RelaxedSolver::setInitialPose(Pose pose){
    lock();
    this->initialPose = pose;
    unlock();
}

void RelaxedSolver::setPose(Pose desiredPose){
    lock();
    this->desiredPose = desiredPose;
    unlock();
}
vector<double> RelaxedSolver::getCurrentPose(){
    vector<double> ret;
    lock();
    currentPose.swapFront();
    ret = currentPose.getFront();;
    unlock();
    return ret;
}

void RelaxedSolver::setConfigurationPose(vector<double> pose){
    lock();
//    set_starting_config(pose.data(), pose.size());
    unlock();
}

void RelaxedSolver::setAngle(double angleX, double angleY, double angleZ){
    lock();
    this->angleX = angleX;
    this->angleY = angleY;
    this->angleZ = angleZ;
    unlock();
}

void RelaxedSolver::threadedFunction(){
    while(isThreadRunning()){
        lock();

        ofVec3f difPos = (desiredPose.position - initialPose.position) * mat;
        
        ofQuaternion rot  = (desiredPose.orientation * initialPose.orientation);
   
        std::vector<double> pos(3, 0.0);
        pos[0] = difPos.x;
        pos[1] = difPos.y;
        pos[2] = difPos.z;
        std::vector<double> quat(4, 0.0);
        quat[0] = rot.x();
        quat[1] = rot.y();
        quat[2] = rot.z();
        quat[3] = rot.w();
        
        Opt x = solve(pos.data(), (int) pos.size(), quat.data(), (int) quat.size());
        for (int i = 0; i < x.length; i++) {
            currentPose.getBack()[i] = x.data[i];
        }
        frameNum++;
        if(frameNum > 4000)
            frameNum = 0;
        currentPose.swapBack();
        
        unlock();
    }
}

void RelaxedSolver::setMatrix(ofVec3f u, ofVec3f v, ofVec3f w){
    lock();

    mat = ofMatrix4x4(u.x, v.x, w.x, 0,
                      u.y, v.y, w.y, 0,
                      u.z, v.z, w.z, 0,
                      0, 0, 0, 1);

    unlock();
}

bool RelaxedSolver::isThreadRunning(){
    bool ret;
    lock();
    ret = bThreadStarted;
    unlock();
    return ret;
}

uint64_t RelaxedSolver::getFrame(){
    uint64_t t = 0;
    lock();
    frameAvg +=frameNum;
    frameAvg /=2;
    t = frameAvg;
    frameNum = 0;
    unlock();
    return t;
}
