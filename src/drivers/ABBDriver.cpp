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
                }
            }
        }else{
            if(robot->waitForMessage(500))
            {
                // Read the message received from the EGM client.
                robot->read(&input);
                sequence_number = input.header().sequence_number();
                
                if(sequence_number == 0)
                {
                    // Reset all references, if it is the first message.
                    output.Clear();
                    initial_pose.CopyFrom(input.feedback().robot().cartesian().pose());
                    output.mutable_robot()->mutable_cartesian()->mutable_pose()->CopyFrom(initial_pose);
                }
                else
                {
                    time = sequence_number/((double) egm_rate);
                    
                    // Compute references for position (along X-axis), and orientation (around Y-axis).
                    position_reference = initial_pose.position().x() + position_amplitude*(1.0 + std::sin(2.0*M_PI*frequency*time - 0.5*M_PI));
                    orientation_reference = initial_pose.euler().y() + orientation_amplitude*(1.0 + std::sin(2.0*M_PI*frequency*time - 0.5*M_PI));
                    
                    // Set references.
                    // Note: The references are relative to the frames specified by the EGMActPose RAPID instruction.
                    output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position()->set_x(position_reference);
                    output.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_euler()->set_y(orientation_reference);
                    
                    if(sequence_number%egm_rate == 0)
                    {
                        ofLog()<<"References: " <<
                        "X position = " << position_reference << " [mm] | " <<
                        "Y orientation (Euler) = " << orientation_reference << " [degrees]"<<endl;
                    }
                }
                // Write references back to the EGM client.
                robot->write(output);
            }
        }
    }
}
