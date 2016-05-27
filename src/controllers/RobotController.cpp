#include "RobotController.h"


RobotController::RobotController(){
    
}

RobotController::~RobotController(){
    
    
}


void RobotController::setup(RobotParameters & params){
    robot.setup("192.168.1.9",0, 1);
    robot.start();
    robotParams = &params;
    movement.setup();
}

void RobotController::setup(string ipAddress, RobotParameters & params){
    robot.setup(ipAddress,0, 1);
    robot.start();
    robotParams = &params;
    movement.setup();
}

vector<double> RobotController::getJointPosition(){
    return robotParams->currentJointPos;
}

void RobotController::update(){
    toggleRecord();
    updateData();
    updateMovement();
}

void RobotController::updateMovement(){
    movement.setCurrentJointPosition(robotParams->currentJointPos);

//    robotParams->tcpOffset = robot.getToolNode().getGlobalPosition();
    
//    robotParams->targetTCP.position+=robotParams->tcpOffset;
    
    // send the target TCP to the kinematic solver
    movement.addTargetPoint(robotParams->targetTCP);
    movement.update();
    vector<double> rawIK = movement.getRawJointPos();
    for(int i = 0; i < rawIK.size(); i++){
        robotParams->jointPosIKRaw[i] = ofRadToDeg((float)rawIK[i]);
    }
    
    // get back the target joint trajectories
    vector<double> target = movement.getTargetJointPos();
    for(int i = 0; i < target.size(); i++){
        robotParams->targetJointPos[i] = ofRadToDeg((float)target[i]);
    }
    
    // set the joint speeds
    vector<double> tempSpeeds;
    tempSpeeds.assign(6, 0);
    tempSpeeds = movement.getCurrentSpeed();
    for(int i = 0; i < tempSpeeds.size(); i++){
        robotParams->jointVelocities[i] = (float)tempSpeeds[i];
    }
    // move the robot to the target TCP
    if(robotParams->bMove){
        robot.setSpeed(tempSpeeds, movement.getAcceleration());
    }
    
}
ofNode RobotController::getTCPNode(){
    return robot.model.getTool();
}

void RobotController::updateData(){
    // pass the current joints from the robot to the kinematic solver
    robotParams->currentJointPos = robot.getJointPositions();

//    robotParams->calcTCPOrientation = robot.getCalculatedTCPOrientation();
    
    for(int i = 0; i < robotParams->currentJointPos.size(); i++){
        robotParams->jointPos[i] = (float)robotParams->currentJointPos[i];
    }
    robotParams->actualTCP = robot.getToolPose();
    robotParams->tcpPosition = robotParams->actualTCP.position;
    ofQuaternion tcpO = robotParams->actualTCP.rotation;
    robotParams->tcpOrientation = ofVec4f(tcpO.x(), tcpO.y(), tcpO.z(), tcpO.w());
    if(robotParams->bRecord){
        recorder.addPose(robotParams->currentJointPos, ofGetElapsedTimef());
    }
    // update GUI params
    for(int i = 0; i < robotParams->currentJointPos.size(); i++){
        robotParams->jointPos[i] = ofRadToDeg((float)robotParams->currentJointPos[i]);
    }
    
    ofMatrix4x4 forwardIK = movement.forwardKinematics(robotParams->currentJointPos);
    robotParams->forwardTCPPosition = forwardIK.getTranslation();
    robotParams->forwardTCPOrientation = ofVec4f(forwardIK.getRotate().x(), forwardIK.getRotate().y(), forwardIK.getRotate().z(), forwardIK.getRotate().x());
    
}

void RobotController::close(){
    if(robot.isThreadRunning()){
        robot.stopThread();
    }
}

void RobotController::toggleRecord(){
    if(robotParams->bRecord){
        recorder.startRecording();
    }else{
        recorder.endRecording();
    }
}

void RobotController::draw(){
    
}