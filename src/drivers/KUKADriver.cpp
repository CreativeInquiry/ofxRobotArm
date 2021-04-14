//
//  KUKADriver.cpp
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
//
// Copyright (c) 2016, 2021 Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
////


#include "KUKADriver.h"
using namespace ofxRobotArm;
KUKADriver::KUKADriver(){
    currentSpeed.assign(6, 0.0);
    vector<double> foo;
    foo.assign(6, 0.0001);

    poseRaw.setup(foo);
    toolPoseRaw.setup(foo);
    poseProcessed.setup(foo);
    poseRaw.getBack().assign(6, 0.0001);
    poseProcessed.getBack().assign(6, 0.0001);
    toolPoseRaw.getBack().assign(6, 0.0001);
    numDeccelSteps = 120;
}

KUKADriver::~KUKADriver(){
    if(isConnected()){
        disconnect();
    }
}

void KUKADriver::stopThread(){
    if(isConnected()){
        disconnect();
    }
    if(isThreadRunning()){
        ofThread::stopThread();
    }
}
void KUKADriver::toggleTeachMode(){
    lock();

    unlock();
}

vector<double> KUKADriver::getInitPose(){
    vector<double> foo;
    foo.assign(6, 0.0001);
    return foo;
}

void KUKADriver::setTeachMode(bool enabled){
    lock();
    bTeachModeEnabled = enabled;
    unlock();
}

void KUKADriver::setAllowReconnect(bool bDoReconnect){
    bTryReconnect = bDoReconnect;
}

void KUKADriver::setup(){

}

void KUKADriver::setup(string ipAddress, double minPayload, double maxPayload){
    
}

void KUKADriver::setup(int port, double minPayload, double maxPayload){

}

void KUKADriver::setup(string ipaddress, int port, double minPayload, double maxPayload){
    cout << "KUKADriver :: setup : ipAddress: " << port << endl;
    if(ipaddress != "") {
        
    } else {
        ofLogError( "ip address parameter is empty. Not initializing robot." );
    }
    const std::string::size_type size = ipaddress.size();
    char *buffer = new char[size + 1];   //we need extra char for NUL
    memcpy(buffer, ipaddress.c_str(), size + 1);
    robot.startCommunicator(buffer, port);

    //Bounds for SetPayload service
    //Using a very conservative value as it should be set through the parameter server
    double min_payload = minPayload;
    double max_payload = maxPayload;
    
    poseProcessed.swapBack();
    poseRaw.swapBack();
    toolPoseRaw.swapBack();
    bStarted = false;

    bTriedOnce = false;
}
void KUKADriver::start(){
    ofLog(OF_LOG_NOTICE)<<"Starting KUKADriver Controller"<<endl;
    startThread();
}

bool KUKADriver::isConnected() {
    if( ofThread::isThreadRunning() ) {
        bool tConn = false;
        if(lock()) {
            tConn = bStarted;
            unlock();
        }
        return tConn;
    }
    return false;
}

void KUKADriver::disconnect(){
    robot.stopCommunicator();
}

bool KUKADriver::isDataReady(){
    if(bDataReady){
        bDataReady = false;
        return true;
    }else{
        return false;
    }
}
vector<double> KUKADriver::getToolPointRaw(){
    vector<double> ret;
    lock();
    toolPoseRaw.swapFront();
    ret = toolPoseRaw.getFront();
    unlock();
    return ret;
}

vector<double> KUKADriver::getCurrentPose(){
    vector<double> ret;
    
    lock();
    poseRaw.swapFront();
    ret = poseRaw.getFront();
    unlock();
    
    return ret;
}

ofVec4f KUKADriver::getCalculatedTCPOrientation(){
    ofVec4f ret;
    lock();
    ret = ofVec4f(dtoolPoint.orientation.x(), dtoolPoint.orientation.y(), dtoolPoint.orientation.z(), dtoolPoint.orientation.w());
    unlock();
    return ret;
}

float KUKADriver::getThreadFPS(){
    float fps = 0;
    lock();
    fps = timer.getFrameRate();
    unlock();
    return fps;
}

ofxRobotArm::Pose KUKADriver::getToolPose(){
    ofxRobotArm::Pose ret;
    lock();
    ret = tool;
    unlock();
    return ret;
}

void KUKADriver::setSpeed(vector<double> speeds, double accel){
    lock();
    currentSpeed = speeds;
    acceleration = accel;
    bMove = true;
    bMoveWithPos = false;
    unlock();
}

void KUKADriver::setPose(vector<double> pose){
    lock();
    currentPose = pose;
    bMove = true;
    bMoveWithPos = true;
    deccelCount = numDeccelSteps+4;
    bStop = false;
    unlock();
}

void KUKADriver::setToolOffset(ofVec3f localPos){
    
}

void KUKADriver::threadedFunction(){
    while(isThreadRunning()){
        timer.tick();
        if(!bStarted && !bTriedOnce) {
            
        }else{                

            if(bMove && currentPose.size()> 0){
                kuka::KukaPose p = kuka::KukaPose(currentPose[0], currentPose[1],currentPose[2],currentPose[3],currentPose[4],currentPose[5]);;
                robot.move(p, 1.0);
            }
            toolPoseRaw.swapBack();
            poseRaw.swapBack();
            poseProcessed.swapBack();
        }
    }
}
