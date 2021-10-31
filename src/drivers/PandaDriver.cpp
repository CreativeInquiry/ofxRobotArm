//
//  PandaDriver.cpp
//  example-urdf
//
//  Created by Dan Moore on 10/31/21.
//

#include "PandaDriver.h"

using namespace ofxRobotArm;
PandaDriver::PandaDriver(){
    
}

PandaDriver::~PandaDriver(){
    vector<double> foo;
    foo.assign(7, 0.001);
    foo[0] = 0.0754606;
    foo[1] = -0.337453;
    foo[2] = 0.150729;
    foo[3] = -2.46194;
    foo[4] = 0.0587094;
    foo[5] = 2.12597;
    foo[6] = 0.972193;
    poseRaw.setup(foo);
}

vector<double> PandaDriver::getInitPose(){
    vector<double> foo;
    foo.assign(7, 0.001);
    foo[0] = 0.0754606;
    foo[1] = -0.337453;
    foo[2] = 0.150729;
    foo[3] = -2.46194;
    foo[4] = 0.0587094;
    foo[5] = 2.12597;
    foo[6] = 0.972193;
    return foo;
}
void PandaDriver::setAllowReconnect(bool bDoReconnect){
    
}
void PandaDriver::setup(){
    
}
void PandaDriver::setup(string ipAddress, int port, double minPayload, double maxPayload){
    
}
void PandaDriver::setup(string ipAddress, double minPayload, double maxPayload){
    
}
void PandaDriver::setup(int port, double minPayload, double maxPayload){
    
}
void PandaDriver::start(){
    
}
bool PandaDriver::isConnected(){
    return false;
}
void PandaDriver::disconnect(){
    
}
void PandaDriver::stopThread(){
    
}
void PandaDriver::toggleTeachMode(){
    
}
void PandaDriver::setTeachMode(bool enabled){
    
}
void PandaDriver::threadedFunction(){
    
}
