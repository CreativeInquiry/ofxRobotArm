//
//  ABBDriver.cpp
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
//
// Copyright (c) 2016, 2021 Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
////


#include "ABBDriver.h"
using namespace ofxRobotArm;
ABBDriver::ABBDriver(){
    currentSpeed.assign(numJoints, 0.0);
    acceleration = 0.0;
    robot       = NULL;
    bStarted    =false;

    
//    -0.25, 0.25, 0.13, 2.5, 0.45, -2.59
    vector<double> foo;
    foo.assign(numJoints, 0.0001);

    poseRaw.setup(foo);
    toolPoseRaw.setup(foo);
    poseProcessed.setup(foo);
    poseRaw.getBack().assign(numJoints, 0.0001);
    poseProcessed.getBack().assign(numJoints, 0.0001);
    toolPoseRaw.getBack().assign(numJoints, 0.0001);
    numDeccelSteps = 120;
}

ABBDriver::~ABBDriver(){
    if(robot->isInitialized()){
        disconnect();
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

vector<double> ABBDriver::getInitPose(){
    vector<double> foo;
    foo.assign(6, 0.0001);
    return foo;
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

void ABBDriver::setup(){

}

void ABBDriver::setup(string ipAddress, double minPayload, double maxPayload){
    
}

void ABBDriver::setup(string ipAddress, int port, double minPayload, double maxPayload){
    
}

void ABBDriver::setup(int port, double minPayload, double maxPayload){
    ofLog(OF_LOG_NOTICE) << "ABBDriver :: setup : " << port << endl;

    
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
    
    abb::egm::BaseConfiguration configuration;
    robot = std::make_unique<abb::egm::EGMControllerInterface>(io_service, port, configuration);

    if(!robot->isInitialized())
    {
        ofLog(OF_LOG_ERROR)<<"EGM interface failed to initialize (e.g. due to port already bound)"<<endl;
    }else{
        ofLog(OF_LOG_NOTICE)<<"EGM interface initialized"<<endl;
    }
    
    thread_group.create_thread(boost::bind(&boost::asio::io_service::run, &io_service));
    wait = true;
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
    io_service.stop();
    thread_group.join_all();
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

ofVec4f ABBDriver::getCalculatedTCPOrientation(){
    ofVec4f ret;
    lock();
    ret = ofVec4f(dtoolPoint.orientation.x(), dtoolPoint.orientation.y(), dtoolPoint.orientation.z(), dtoolPoint.orientation.w());
    unlock();
    return ret;
}

ofxRobotArm::Pose ABBDriver::getToolPose(){
    ofxRobotArm::Pose ret;
    lock();
    ret = tool;
    unlock();
    return ret;
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



void ABBDriver::setToolOffset(ofVec3f localPos){
    
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
                    }else{
                        ofLogError()<<"NOT CONNECTRED"<<endl;
                        abb::egm::wrapper::Status s = robot->getStatus();
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
                    poseRaw.getBack()[i] = ofDegToRad(actualPose.values(i));
                }
                currentPoseRadian = poseRaw.getBack();
                poseProcessed.getBack() = poseRaw.getBack();
                if(sequence_number == 0)
                {
                    output.Clear();
                    initial_positions.CopyFrom(input.feedback().robot().joints().position());
                    output.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(initial_positions);
                    
                }
                else
                {
                    initial_positions.CopyFrom(input.feedback().robot().joints().position());
                    output.mutable_robot()->mutable_joints()->mutable_position()->CopyFrom(initial_positions);
                    time = sequence_number/((double) egm_rate);
                    if(bMoveWithPos){
                        //if we aren't moving but deccelCount isn't 0 lets deccelerate
                        if( (bMove && currentPose.size()>0)|| (currentPose.size()>0 && deccelCount>0) ){
                            timeNow = ofGetElapsedTimef();
                            if( bMove || timeNow-lastTimeSentMove >= 1.0/60.0){
                                currentPose = getAchievablePosition(currentPose);
                                if(output.mutable_robot()->mutable_joints()->mutable_position()->values_size() == currentPose.size())
                                {
                                    for(int i = 0 ; i < currentPose.size(); i++){
                                        output.mutable_robot()->mutable_joints()->mutable_position()->set_values(i, ofRadToDeg(currentPose[i]));
                                    }
                                }else{
                                    ofLog()<<"OUTPUT NOT BIG ENOUGH size "<<output.mutable_robot()->mutable_joints()->mutable_position()->values_size()<<endl;
                                }
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
