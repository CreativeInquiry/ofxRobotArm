
//
// Copyright (c) 2016, 2021, 2022 Manipulateur LLC, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
////

#include "RobotController.h"


using namespace ofxRobotArm;


RobotController::RobotController(){
    bMove.set("Move Robot", false);
}


RobotController::~RobotController(){
    
}


void RobotController::setType(RobotType type){
    if (type == UR3 || type == UR5 || type == UR10)
    {
        robot = new URDriver();
    }
    else if (type == IRB120)
    {
        robot = new ABBDriver();
    }
    else if (type == XARM7)
    {
        robot = new XARMDriver(type);
    }
    else if(type == PANDA){
        robot = new PandaDriver();
    }
}

void RobotController::setAddress(string ipaddress){
    this->ipaddress = ipaddress;
}

void RobotController::setPort(int port){
    this->port = port;
}

void RobotController::connect(){
    if (robot != nullptr)
    {
        robot->setAllowReconnect(bDoReconnect);

        if (type == UR3 || type == UR5 || type == UR10)
        {
            robot->setup(ipaddress, 0, 1);
        }
        else if (type == IRB120)
        {
            robot->setup(port, 0, 1);
        }
        else if (type == XARM7)
        {
            robot->setup(ipaddress, 0, 1);
        }
    }
}

void RobotController::disconnect(){
    if(robot != nullptr){
        robot->disconnect();
    }
}

void RobotController::update(vector<double> pose, vector<double> smoothing){
    if(robot != nullptr){
        targetPose = pose;
        currentPose = robot->getCurrentPose();
        
        if(targetPose.size() == currentPose.size()){
            if(smoothing.size() == targetPose.size()){
                int i = 0;
                for(auto a : currentPose){
                    targetPose[i] = ofLerp(currentPose[i], targetPose[i], smoothing[i]);
                    i++;
                }
            }
            robot->setPose(targetPose);
        }
    }
}


vector<double> RobotController::getCurrentPose(){
    if(robot != nullptr){
        currentPose = robot->getCurrentPose();
    }
    return currentPose;
}

bool RobotController::isConnected(){
    return robot->isConnected();
}

void RobotController::setEnableMovement(bool move){
    bMove = move;
}
