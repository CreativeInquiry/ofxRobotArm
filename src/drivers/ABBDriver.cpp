//
//  ABBDriver.cpp
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//


#include "ABBDriver.h"
using namespace ofxRobotArm;
ABBDriver::ABBDriver(){
    currentSpeed.assign(6, 0.0);
    acceleration = 0.0;
    robot       = NULL;
    io_service = NULL;
    thread_group = NULL;
    bStarted    =false;

    vector<double> foo;
    foo.assign(6, 0.0);
    poseRaw.setup(foo);
    toolPoseRaw.setup(foo);
    poseProcessed.setup(foo);
    poseRaw.getBack().assign(6, 0.0);
    poseProcessed.getBack().assign(6, 0.0);
    toolPoseRaw.getBack().assign(6, 0.0);
    numDeccelSteps = 120;

}

ABBDriver::~ABBDriver(){
    if(robot){
        disconnect();
        delete robot;
        robot = NULL;
    }
}

void ABBDriver::stopThread(){
    if(isConnected()){
        disconnect();
    }
    if(isThreadRunning()){
        ofThread::stopThread();
    }
}
void ABBDriver::toggleTeachMode(){
    lock();
    if(bTeachModeEnabled){
        bTeachModeEnabled = false;

    }else{
        bTeachModeEnabled = true;

    }
    unlock();
}

void ABBDriver::setTeachMode(bool enabled){
    lock();
    if(enabled){
        bTeachModeEnabled = true;

    }else{
        bTeachModeEnabled = false;
 
    }
    unlock();
}

void ABBDriver::setAllowReconnect(bool bDoReconnect){
    bTryReconnect = bDoReconnect;
}

void ABBDriver::setup(string port, double minPayload, double maxPayload){
    cout << "ABBDriver :: setup : ipAddress: " << port << endl;
    if( port != "" && port.length() > 3 ) {

    } else {
        ofLogError( "ipAddress parameter is empty. Not initializing robot." );
    }
    
    char buf[256];
//    vector<string> foo = robot->getJointNames();
    std::string joint_prefix = "ur_";
    std::vector<std::string> joint_names;
    joint_prefix = "ABBDriver-";
    joint_names.push_back(joint_prefix + "joint_1");
    joint_names.push_back(joint_prefix + "joint_2");
    joint_names.push_back(joint_prefix + "joint_3");
    joint_names.push_back(joint_prefix + "joint_4");
    joint_names.push_back(joint_prefix + "joint_5");
    joint_names.push_back(joint_prefix + "joint_6");

    //Bounds for SetPayload service
    //Using a very conservative value as it should be set through the parameter server
    double min_payload = minPayload;
    double max_payload = maxPayload;
    
    sprintf(buf, "Bounds for set_payload service calls: [%f, %f]",
            min_payload, max_payload);
    ofLog(OF_LOG_NOTICE)<<buf;
    poseProcessed.swapBack();
    poseRaw.swapBack();
    toolPoseRaw.swapBack();
    bStarted = false;

    bTriedOnce = false;

}
void ABBDriver::start(){
    ofLog(OF_LOG_NOTICE)<<"Starting ABBDriver Controller"<<endl;
    startThread();
}

bool ABBDriver::isConnected() {
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



void ABBDriver::disconnect(){
    if( robot != NULL ){
        robot = NULL;
        delete robot;
    }
}

bool ABBDriver::isDataReady(){
    if(bDataReady){
        bDataReady = false;
        return true;
    }else{
        return false;
    }
}
vector<double> ABBDriver::getToolPointRaw(){
    vector<double> ret;
    lock();
    toolPoseRaw.swapFront();
    ret = toolPoseRaw.getFront();
    unlock();
    return ret;
}

vector<double> ABBDriver::getCurrentPose(){
    vector<double> ret;
    
    lock();
    poseRaw.swapFront();
    ret = poseRaw.getFront();
    unlock();
    
    
    return ret;
}
vector<double> ABBDriver::getJointAngles(){
    vector<double> ret;
    lock();
    poseProcessed.swapFront();
    ret = poseProcessed.getFront();
    unlock();
    return ret;
}

ofVec4f ABBDriver::getCalculatedTCPOrientation(){
    ofVec4f ret;
    lock();
    ret = ofVec4f(dtoolPoint.orientation.x(), dtoolPoint.orientation.y(), dtoolPoint.orientation.z(), dtoolPoint.orientation.w());
    unlock();
    return ret;
}

float ABBDriver::getThreadFPS(){
    float fps = 0;
    lock();
    fps = timer.getFrameRate();
    unlock();
    return fps;
}

ofxRobotArm::Pose ABBDriver::getToolPose(){
    ofxRobotArm::Pose ret;
    lock();
    ret = tool;
    unlock();
    return ret;
}

void ABBDriver::moveJoints(vector<double> pos){
    lock();
    poseBuffers.push_back(pos);
    unlock();
}

void ABBDriver::setSpeed(vector<double> speeds, double accel){
    lock();
    currentSpeed = speeds;
    acceleration = accel;
    bMove = true;
    bMoveWithPos = false;
    unlock();
}

void ABBDriver::setPose(vector<double> pose){
    lock();
    currentPose = pose;
    bMove = true;
    bMoveWithPos = true;
    deccelCount = numDeccelSteps+4;
    bStop = false;
    unlock();
}

ofQuaternion ABBDriver::convertAxisAngle(double rx, double ry, double rz) {
    float angle = ofVec3f(rx, ry, rz).normalize().length();
    double s = sin(angle/2);
    float x = (rx) * s;
    float y = (ry) * s;
    float z = (rz) * s;
    float w = cos(angle/2);
    return ofQuaternion(x, y, z, w);
}

void ABBDriver::setToolOffset(ofVec3f localPos){
    
}

vector <double> ABBDriver::getAchievablePosition(vector<double> position){
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

    if( currentPoseRadian.size() && position.size() ){
        
        bool bHasSpeed = true;
        vector <double> lastSpeed = calculatedSpeed;
        
        if( calculatedSpeed.size() != position.size() ){
            calculatedSpeed.assign(position.size(), 0);
            lastSpeed = calculatedSpeed;
            bHasSpeed = false;
        }
        
        for(int d= 0; d < position.size(); d++){
            calculatedSpeed[d] = (position[d]-currentPoseRadian[d])/timeDiff;
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
                
                position[d] = currentPoseRadian[d] + ( targetSpeed * timeDiff * maxSpeedPct );
                
                //cout << "---- hit limit: accel is " << ofRadToDeg(newAccel) << " targetSpeed is now " << ofRadToDeg(targetSpeed) << " pos is now " << position[d] << endl;
            }else if( maxSpeedPct < 1.0 ){
                position[d] = currentPoseRadian[d] + ( currentSpeed[d] * timeDiff * maxSpeedPct );
            }
            
        }
        
    }
    
    return position;
}


void ABBDriver::threadedFunction(){
    while(isThreadRunning()){
        timer.tick();
        if(!bStarted && !bTriedOnce) {
            if( robot ) {
                if(wait){
                    if(robot->isConnected())
                    {
                        if(robot->getStatus().rapid_execution_state() == abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED)
                        {
                            ofLogWarning()<<"RAPID execution state is UNDEFINED (might happen first time after controller start/restart). Try to restart the RAPID program.";
                        }
                        else
                        {
                            wait = robot->getStatus().rapid_execution_state() != abb::egm::wrapper::Status_RAPIDExecutionState_RAPID_RUNNING;
                        }
                    }
                }else{
                    bStarted = true;
                    bTriedOnce = true;
                }
            }
        }else{
            if(robot->waitForMessage(500))
            {
                // Read the message received from the EGM client.
                robot->read(&input);
                sequence_number = input.header().sequence_number();
                actualPose.CopyFrom(input.feedback().robot().joints().position());
                for(int i = 0 ; i < actualPose.values_size(); i++){
                    poseRaw.getBack()[i] = actualPose.values(i);
                }
                currentPoseRadian = poseRaw.getBack();
                poseProcessed.getBack() = poseRaw.getBack();
                if(sequence_number == 0)
                {
                    output.Clear();
                }
                else
                {
                    time = sequence_number/((double) egm_rate);
                    if(bMoveWithPos){
                        //if we aren't moving but deccelCount isn't 0 lets deccelerate
                        if( (bMove && currentPose.size()>0)|| (currentPose.size()>0 && deccelCount>0) ){
                            timeNow = ofGetElapsedTimef();
                            if( bMove || timeNow-lastTimeSentMove >= 1.0/60.0){
                                currentPose = getAchievablePosition(currentPose);
                                output.mutable_robot()->mutable_joints()->mutable_position()->set_values(0, currentPose[0]);
                                output.mutable_robot()->mutable_joints()->mutable_position()->set_values(1, currentPose[1]);
                                output.mutable_robot()->mutable_joints()->mutable_position()->set_values(2, currentPose[2]);
                                output.mutable_robot()->mutable_joints()->mutable_position()->set_values(3, currentPose[3]);
                                output.mutable_robot()->mutable_joints()->mutable_position()->set_values(4, currentPose[4]);
                                output.mutable_robot()->mutable_joints()->mutable_position()->set_values(5, currentPose[5]);
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
                    }
                }
                // Write references back to the EGM client.
                robot->write(output);
                
                toolPoseRaw.swapBack();
                poseRaw.swapBack();
                poseProcessed.swapBack();
            }
        }
    }
}
