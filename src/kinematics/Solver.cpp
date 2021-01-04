//
//  Solver.cpp
//  example-simple
//
//  Created by Dan Moore on 1/2/21.
//

#include "Solver.h"
#include "RelaxedIK.hpp"
using namespace ofxRobotArm;

Solver::Solver(){
    vector<double> foo(6., 0.0);
    currentPose.setup(foo);
    currentPose.getBack().assign(6, 0.0);
    currentPose.swapBack();
    frameNum = 0;
}

Solver::~Solver(){
    stop();
    waitForThread(false);
}

void Solver::start(){
    startThread();
}
void Solver::stop(){
    stopThread();
}
void Solver::startThread(){
    bThreadStarted = true;
    ofThread::startThread();
}
void Solver::stopThread(){
    if(isThreadRunning()){
        bThreadStarted = false;
        ofThread::stopThread();
    }
}
void Solver::setPose(Pose desiredPose, Pose actualPose){
    lock();
    this->desiredPose = desiredPose;
    this->actualPose = actualPose;
    unlock();
}
vector<double> Solver::getCurrentPose(){
    vector<double> ret;
    lock();
    currentPose.swapFront();
    ret = currentPose.getFront();;
    unlock();
    return ret;
}

void Solver::setInitialPose(vector<double> pose){
    lock();
//    set_starting_config(pose.data(), pose.size());
    unlock();
}

void Solver::setAngle(double angleX, double angleY, double angleZ){
    lock();
    this->angleX = angleX;
    this->angleY = angleY;
    this->angleZ = angleZ;
    unlock();
}

void Solver::threadedFunction(){
    while(isThreadRunning()){
        lock();
  
        //handed-ness?
        ofMatrix4x4 R = ofMatrix4x4(u.x, v.x, w.x, 0,
                                     u.y, v.y, w.y, 0,
                                     u.z, v.z, w.z, 0,
                                     0, 0, 0, 1);
        ofMatrix4x4 RX, RY, RZ;
        RX.makeRotationMatrix(angleX, ofVec3f(1, 0, 0));
        RY.makeRotationMatrix(angleY, ofVec3f(0, 1, 0));
        RZ.makeRotationMatrix(angleZ, ofVec3f(0, 0, 1));
        R *= RX*RY*RZ;
        
        ofVec3f difPos = (desiredPose.position - actualPose.position)*R;

        ofQuaternion rot  = (desiredPose.orientation * actualPose.orientation);
   
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

void Solver::setMatrix(ofVec3f u, ofVec3f v, ofVec3f w){
    lock();
    this->u = u;
    this->v = v;
    this->w = w;
    unlock();
}

bool Solver::isThreadRunning(){
    bool ret;
    lock();
    ret = bThreadStarted;
    unlock();
    return ret;
}

uint64_t Solver::getFrame(){
    uint64_t t = 0;
    lock();
    frameAvg +=frameNum;
    frameAvg /=2;
    t = frameAvg;
    frameNum = 0;
    unlock();
    return t;
}
