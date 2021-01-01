//
//  URDriver.cpp
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//

#include "URDriver.h"
using namespace ofxRobotArm;
URDriver::URDriver(){
    currentSpeed.assign(6, 0.0);
    acceleration = 0.0;
    robot       = NULL;
    bStarted    =false;

    vector<double> foo;
    foo.assign(6, -PI/2.0);
    jointsRaw.setup(foo);
    toolPointRaw.setup(foo);
    jointsProcessed.setup(foo);
    jointsRaw.getBack().assign(6, -PI/2.0);
    jointsProcessed.getBack().assign(6, -PI/2.0);
    toolPointRaw.getBack().assign(6, -PI/2.0);
    numDeccelSteps = 120;
}

URDriver::~URDriver(){
    if(robot){
        disconnect();
        delete robot;
        robot = NULL;
    }
}

void URDriver::stopThread(){
    if(isConnected()){
        disconnect();
    }
    if(isThreadRunning()){
        ofThread::stopThread();
    }
}
void URDriver::toggleTeachMode(){
    lock();
    if(bTeachModeEnabled){
        bTeachModeEnabled = false;
        robot->setTeachModeDisabled();
    }else{
        bTeachModeEnabled = true;
        robot->setTeachModeEnabled();
    }
    unlock();
}

void URDriver::setTeachMode(bool enabled){
    lock();
    if(enabled){
        bTeachModeEnabled = true;
        robot->setTeachModeEnabled();
    }else{
        bTeachModeEnabled = false;
        robot->setTeachModeDisabled();
    }
    unlock();
}

void URDriver::setAllowReconnect(bool bDoReconnect){
    bTryReconnect = bDoReconnect;
}

void URDriver::setup(string ipAddress, double minPayload, double maxPayload){
    cout << "URDriver :: setup : ipAddress: " << ipAddress << endl;
    if( ipAddress != "" && ipAddress.length() > 3 ) {
        robot = new UrDriver(rt_msg_cond_,
                         msg_cond_, ipAddress);
    } else {
        ofLogError( "ipAddress parameter is empty. Not initializing robot." );
    }
    
    char buf[256];
//    vector<string> foo = robot->getJointNames();
    std::string joint_prefix = "ur_";
    std::vector<std::string> joint_names;
    joint_prefix = "URDriver-";
    joint_names.push_back(joint_prefix + "shoulder_pan_joint");
    joint_names.push_back(joint_prefix + "shoulder_lift_joint");
    joint_names.push_back(joint_prefix + "elbow_joint");
    joint_names.push_back(joint_prefix + "wrist_1_joint");
    joint_names.push_back(joint_prefix + "wrist_2_joint");
    joint_names.push_back(joint_prefix + "wrist_3_joint");
    if( robot ) {
        robot->setJointNames(joint_names);
    }
    
    //Bounds for SetPayload service
    //Using a very conservative value as it should be set through the parameter server
    double min_payload = minPayload;
    double max_payload = maxPayload;
    if( robot != NULL ) {
        robot->setMinPayload(min_payload);
        robot->setMaxPayload(max_payload);
    }
    sprintf(buf, "Bounds for set_payload service calls: [%f, %f]",
            min_payload, max_payload);
    ofLog(OF_LOG_NOTICE)<<buf;
    jointsProcessed.swapBack();
    jointsRaw.swapBack();
    toolPointRaw.swapBack();
    bStarted = false;
    
    joints.resize(6);
    
    bTriedOnce = false; 

}
void URDriver::start(){
    ofLog(OF_LOG_NOTICE)<<"Starting URDriver Controller"<<endl;
    startThread();
}

bool URDriver::isConnected() {
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



void URDriver::disconnect(){
    if( robot != NULL ) robot->halt();
    
}

bool URDriver::isDataReady(){
    if(bDataReady){
        bDataReady = false;
        return true;
    }else{
        return false;
    }
}
vector<double> URDriver::getToolPointRaw(){
    vector<double> ret;
    lock();
    toolPointRaw.swapFront();
    ret = toolPointRaw.getFront();
    unlock();
    return ret;
}

vector<double> URDriver::getCurrentPose(){
    vector<double> ret;
    
    lock();
    jointsRaw.swapFront();
    ret = jointsRaw.getFront();
    unlock();
    
    
    return ret;
}
vector<double> URDriver::getJointAngles(){
    vector<double> ret;
    lock();
    jointsProcessed.swapFront();
    ret = jointsProcessed.getFront();
    unlock();
    return ret;
}

ofVec4f URDriver::getCalculatedTCPOrientation(){
    ofVec4f ret;
    lock();
    ret = ofVec4f(dtoolPoint.orientation.x(), dtoolPoint.orientation.y(), dtoolPoint.orientation.z(), dtoolPoint.orientation.w());
    unlock();
    return ret;
}

float URDriver::getThreadFPS(){
    float fps = 0;
    lock();
    fps = timer.getFrameRate();
    unlock();
    return fps;
}

ofxRobotArm::Pose URDriver::getToolPose(){
    ofxRobotArm::Pose ret;
    lock();
    ret = tool;
    unlock();
    return ret;
}

void URDriver::moveJoints(vector<double> pos){
    lock();
    posBuffer.push_back(pos);
    unlock();
}

void URDriver::setSpeed(vector<double> speeds, double accel){
    lock();
    currentSpeed = speeds;
    acceleration = accel;
    bMove = true;
    bMoveWithPos = false;
    unlock();
}


void URDriver::setPose(vector<double> positions){
    lock();
    currentPosition = positions; 
    bMove = true;
    bMoveWithPos = true;
    deccelCount = numDeccelSteps+4;
    bStop = false;
    unlock();
}

ofQuaternion URDriver::convertAxisAngle(double rx, double ry, double rz) {
    float angle = ofVec3f(rx, ry, rz).normalize().length();
    double s = sin(angle/2);
    float x = (rx) * s;
    float y = (ry) * s;
    float z = (rz) * s;
    float w = cos(angle/2);
    return ofQuaternion(x, y, z, w);
}



vector <double> URDriver::getAchievablePosition(vector <double> position){
    
    float maxAccelDeg = 500.0;
    float maxSpeedPct = 1.0;
    
    if( !bMove && deccelCount > 0 ){
        maxSpeedPct = ofMap(deccelCount, 1, numDeccelSteps-1, 0.0, 1.0, true);
        if( maxSpeedPct < 0.9 ){
            acceleratePct = 0;
        }
        //cout << " deccelCount " << deccelCount << " maxSpeedPct " << maxSpeedPct << endl;
    }
    if( bMove ){
        acceleratePct += 0.02;
        if( acceleratePct > 1.0 ){
            acceleratePct = 1.0;
        }
    }
    
    maxSpeedPct *= acceleratePct;
    
    //this seeems to do much better with a hardcoded timedelta
    float timeDiff = 1.0/120.0;//timeNow-lastTimeSentMove;

    if( currentRobotPositionRadians.size() && position.size() ){
        
        bool bHasSpeed = true;
        vector <double> lastSpeed = calculatedSpeed;
        
        if( calculatedSpeed.size() != position.size() ){
            calculatedSpeed.assign(position.size(), 0);
            lastSpeed = calculatedSpeed;
            bHasSpeed = false;
        }
        
        for(int d= 0; d < position.size(); d++){
            calculatedSpeed[d] = (position[d]-currentRobotPositionRadians[d])/timeDiff;
        }
        
        vector <double> acceleration;
        acceleration.assign(calculatedSpeed.size(), 0);
        
        for(int d = 0; d < acceleration.size(); d++){
            acceleration[d] = (calculatedSpeed[d]-lastSpeed[d])/timeDiff;
            
            float accelDegPerSec = ofRadToDeg(acceleration[d]);
            
            //this is the max accel reccomended.
            //if we are over it we limit the new position to being the old position plus the current speed, plus the max acceleration
            //this seems to actually work - fuck yes!
            if( fabs( accelDegPerSec ) > maxAccelDeg ){
                
                //cout << d << " currentRobotPositionRadians is " << ofRadToDeg( currentRobotPositionRadians[d] ) << " request is " << ofRadToDeg(position[d]) <<  " speed is " << ofRadToDeg( calculatedSpeed[d] ) << " prev Speed is " << ofRadToDeg(lastSpeed[d])  << " accel is "  << accelDegPerSec << endl;

                float newAccel = ofDegToRad( maxAccelDeg ) * (float)ofSign(accelDegPerSec);
                float speedDiff = newAccel * timeDiff;
                float targetSpeed = lastSpeed[d] + speedDiff;
                
                position[d] = currentRobotPositionRadians[d] + ( targetSpeed * timeDiff * maxSpeedPct );
                
                //cout << "---- hit limit: accel is " << ofRadToDeg(newAccel) << " targetSpeed is now " << ofRadToDeg(targetSpeed) << " pos is now " << position[d] << endl;
            }else if( maxSpeedPct < 1.0 ){
                position[d] = currentRobotPositionRadians[d] + ( currentSpeed[d] * timeDiff * maxSpeedPct );
            }
            
        }
        
    }
    
    return position;
}

void URDriver::threadedFunction(){
    while(isThreadRunning()){
        timer.tick();
        if(!bStarted && !bTriedOnce) {
            if( robot ) {
                bStarted = robot->start();
                if( bStarted ){
                    cout << " about to upload " << endl;
                    robot->uploadProg();
                    cout << " uploaded = true " << endl;
                }
                if(bStarted){
                    ofLog(OF_LOG_NOTICE)<<"Robot Started"<<endl;
                } else {
                    ofLog(OF_LOG_ERROR)<<"Robot Not Started"<<endl;
                    if( !bTryReconnect ){
                        ofLog(OF_LOG_ERROR)<<"bTryReconnect is disabled - won't try again"<<endl;
                        bTriedOnce = true;
                    }
                }
            }
            
            
            
        }else{
            
            bDataReady = false;
            std::mutex msg_lock;
            std::unique_lock<std::mutex> locker(msg_lock);
            while (!robot->rt_interface_->robot_state_->getControllerUpdated()) {
                rt_msg_cond_.wait(locker);
            }
            bDataReady = true;
            
            jointsRaw.getBack() = robot->rt_interface_->robot_state_->getQActual();
            jointsProcessed.getBack() = jointsRaw.getBack();
            dtoolPoint.orientation = ofQuaternion();
            for(int i = 0; i < joints.size(); i++){
                jointsProcessed.getBack()[i] = ofRadToDeg(jointsRaw.getBack()[i]);
                if(i == 1 || i == 3){
                    jointsProcessed.getBack()[i]+=90;
                }
                
                joints[i].orientation.makeRotate(jointsProcessed.getBack()[i], joints[i].axis);
                dtoolPoint.orientation*=joints[i].orientation;
            }
            
            currentRobotPositionRadians = jointsRaw.getBack();
            
            //this is returning weird shit that doesn't return the same values.
            
            toolPointRaw.getBack() = robot->rt_interface_->robot_state_->getToolVectorActual();
            toolPointRaw.getBack()[3] = toolPointRaw.getBack()[3]/PI*180;
            toolPointRaw.getBack()[4] = toolPointRaw.getBack()[4]/PI*180;
            toolPointRaw.getBack()[5] = toolPointRaw.getBack()[5]/PI*180;
            ofVec3f fooRot = ofVec3f((toolPointRaw.getBack()[3]),(toolPointRaw.getBack()[4]),(toolPointRaw.getBack()[5]));
            
            float angle = fooRot.length();
            if( angle < epslion){
                tool.orientation = ofQuaternion(0, 0, 0, 0);
            }else{
                tool.orientation = ofQuaternion(angle, fooRot/angle);
            }
            
            tool.position = ofVec3f(toolPointRaw.getBack()[0], toolPointRaw.getBack()[1], toolPointRaw.getBack()[2]);
            
            robot->rt_interface_->robot_state_->setControllerUpdated();
            
            if(bMoveWithPos){
                //if we aren't moving but deccelCount isn't 0 lets deccelerate 
                if( (bMove && currentPosition.size()>0)|| (currentPosition.size()>0 && deccelCount>0) ){
                    timeNow = ofGetElapsedTimef();
                    if( bMove || timeNow-lastTimeSentMove >= 1.0/60.0){
                        currentPosition = getAchievablePosition(currentPosition);
                        robot->setPosition(currentPosition[0], currentPosition[1], currentPosition[2], currentPosition[3], currentPosition[4], currentPosition[5]);
                        if(!bMove){
                            deccelCount--;
                            if( deccelCount < 0){
                                deccelCount = 0;
                            }
                        }
                        lastTimeSentMove = timeNow;
                    }
                    bMove = false;
                }
            }else{
                if( bMove ){
                    robot->setSpeed(currentSpeed[0], currentSpeed[1], currentSpeed[2], currentSpeed[3], currentSpeed[4], currentSpeed[5], acceleration);
                }
                bMove = false;
            }
            

            toolPointRaw.swapBack();
            jointsRaw.swapBack();
            jointsProcessed.swapBack();
        }
    }
}
