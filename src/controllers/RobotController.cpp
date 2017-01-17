#include "RobotController.h"
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
using namespace ofxRobotArm;
RobotController::RobotController(){
    
}

RobotController::~RobotController(){
    
    
}


void RobotController::setup(RobotParameters & params) {
    robot.setAllowReconnect(params.bDoReconnect);
    robot.setup(params.ipAddress,0, 1);
    robot.start();
    robotParams = &params;
    movement.setup();
    previewArm.setup();
    actualArm.setup();
    mIKArm.setup(actualArm.nodes[1].getGlobalPosition(), actualArm.nodes[2].getGlobalPosition(), actualArm.nodes[3].getGlobalPosition() );
    mIKArm.setDrawSize( 10 );
    
    // disable the elbow target and set to constrain along axis //
    mIKArm.setShoulderUpVectorEnabled( true );
    mIKArm.setShoulderUpVector( ofVec3f(0,-1,0));
    mIKArm.setInverted( true );
            cout << "Setting up the ik arm " << mIKArm.getArmLength() << " shoulder: " << mIKArm.getUpperArmLength() << " forearm len: " << mIKArm.getLowerArmLength() << " | " << ofGetFrameNum() << endl;
}

void RobotController::setup(string ipAddress, RobotParameters & params){
    robot.setup(ipAddress,0, 1);
    robot.start();
    robotParams = &params;
    movement.setup();
    previewArm.setup();
    actualArm.setup();
    
    mIKArm.setup(actualArm.nodes[1].getGlobalPosition(), actualArm.nodes[2].getGlobalPosition(), actualArm.nodes[3].getGlobalPosition() );
    mIKArm.setDrawSize( 10 );
    
    // disable the elbow target and set to constrain along axis //
    mIKArm.setShoulderUpVectorEnabled( true );
    mIKArm.setShoulderUpVector( ofVec3f(0,-1,0));
    mIKArm.setInverted( true );
    
            cout << "Setting up the ik arm " << mIKArm.getArmLength() << " shoulder: " << mIKArm.getUpperArmLength() << " forearm len: " << mIKArm.getLowerArmLength() << " | " << ofGetFrameNum() << endl;
}

vector<double> RobotController::getCurrentPose(){
    return robot.getCurrentPose();
}

void RobotController::update(){
    
    updateRobotData();
    if(robotParams->bUseIK){
        targetPoses = urKinematics.inverseKinematics(robotParams->targetTCP);
        int selectedSolution = urKinematics.selectSolution(targetPoses);
        if(selectedSolution > -1){
            targetPose = targetPoses[selectedSolution];
            for(int i = 0; i < targetPose.size(); i++){
                robotParams->ikPose[i] = targetPose[i];
            }
        }
    }else{
        if( mIKArm.getArmLength() <= 0.0f ) {
            mIKArm.setup(actualArm.nodes[1].getGlobalPosition(), actualArm.nodes[2].getGlobalPosition(), actualArm.nodes[3].getGlobalPosition() );
            mIKArm.setDrawSize( 10 );
            
            // disable the elbow target and set to constrain along axis //
            mIKArm.setShoulderUpVectorEnabled( true );
            mIKArm.setShoulderUpVector( ofVec3f(0,-1,0));
            mIKArm.setInverted( true );
            
            cout << "Setting up the ik arm " << mIKArm.getArmLength() << " shoulder: " << mIKArm.getUpperArmLength() << " forearm len: " << mIKArm.getLowerArmLength() << " | " << ofGetFrameNum() << endl;
        }
        
        mIKArm.setTarget(robotParams->targetTCP.position*1000);
        mIKArm.update();
        
       targetPose = robot.getCurrentPose();
        
        // figure out how to pull out the angles //
        // the y is the forward, so we can measure the rotation around the xaxis //
        ofQuaternion globalRot, elbowRot;
        globalRot   = mIKArm.getShoulderJoint()->getGlobalTransform().getRotate();
        
        elbowRot    = mIKArm.getElbowJoint()->localTransform.getRotate();
        
        
        ofVec3f shouldYPR = mIKArm.getYawPitchRoll( globalRot );
        //    ofQuaternion elbowRot   = mIKArm->getElbowJoint()->localTransform.getRotate();
        ofVec3f elbowYPR = mIKArm.getYawPitchRoll( elbowRot );
        
        
        
        targetPose[1] = ofWrapRadians( shouldYPR.z+PI/2, ofDegToRad(-360.f), ofDegToRad(360) );
        targetPose[2] = ofWrapRadians( elbowYPR.z+PI, ofDegToRad(-360.f), ofDegToRad(360) );
        
        
        for(int i = 0; i < targetPose.size(); i++){
            robotParams->ikPose[i] = targetPose[i];
        }
        
        
    }
    updateMovement();
    targetPose = movement.getTargetJointPose();
    for(int i = 0; i < targetPose.size(); i++){
        robotParams->targetPose[i] = ofRadToDeg(targetPose[i]);
    }
    previewArm.setPose(targetPose);
}
void RobotController::update(vector<double> _pose){
    targetPose = _pose;
}


void RobotController::updateMovement(){
    
    movement.addTargetJointPose(targetPose);
    movement.update();
    
    
    // set the joint speeds
    vector<double> tempSpeeds;
    tempSpeeds.assign(6, 0);
    tempSpeeds = movement.getCurrentJointVelocities();
    for(int i = 0; i < tempSpeeds.size(); i++){
        float tspeed = (float)tempSpeeds[i];
        if( isnan(tspeed) ) {
            tspeed = 0.f;
        }
        robotParams->jointVelocities[i] = tspeed;
        tempSpeeds[i] = tspeed;
    }
    // move the robot to the target TCP
    if(robotParams->bMove){
        //        robot.setSpeed(tempSpeeds, movement.getAcceleration());
        robot.setPosition(movement.getTargetJointPose());
        stopPosition = movement.getTargetJointPose();
        stopCount = 30;
    }
    //    else{
    //        if( stopCount > 0 && stopPosition.size() == movement.targetJointPose.size() && stopPosition.size() > 0 ){
    //            for(int d = 0; d < stopPosition.size(); d++){
    //                stopPosition[d] *= 0.9998;
    //            }
    //            robot.setPosition(stopPosition);
    //            cout << " Doing stop count " << stopCount << endl;
    //            stopCount--;
    //        }
    //    }
    
}

//READS AND SETS IMPORTANT GUI INFO AND THE CURRENT POSE OF THE ARM
void RobotController::updateRobotData(){
    // pass the current joints from the robot to the kinematic solver
    
    robotParams->currentPose = getCurrentPose();
    actualArm.setPose(robotParams->currentPose);
    robotParams->actualTCP = robot.getToolPose();
    robotParams->tcpPosition = robotParams->actualTCP.position;
    ofQuaternion tcpO = robotParams->actualTCP.rotation;
    robotParams->tcpOrientation = ofVec4f(tcpO.x(), tcpO.y(), tcpO.z(), tcpO.w());
    // update GUI params
    for(int i = 0; i < robotParams->currentPose.size(); i++){
        robotParams->pCurrentPose[i] = ofRadToDeg((float)robotParams->currentPose[i]);
    }
    ofMatrix4x4 forwardIK = urKinematics.forwardKinematics(robotParams->currentPose);
    robotParams->forwardTCPPosition = forwardIK.getTranslation();
    robotParams->forwardTCPOrientation = ofVec4f(forwardIK.getRotate().x(), forwardIK.getRotate().y(), forwardIK.getRotate().z(), forwardIK.getRotate().w());
    movement.setCurrentJointPose(robotParams->currentPose);
}

void RobotController::close(){
    if(robot.isThreadRunning()){
        robot.stopThread();
    }
}

void RobotController::draw(){
    actualArm.draw();
}

void RobotController::drawPreview(){
    previewArm.draw();
    mIKArm.draw();
}

void RobotController::enableControlJointsExternally() {
    m_bSettingJointsExternally = true;
}

void RobotController::disableControlJointsExternally() {
    m_bSettingJointsExternally = false;
}

bool RobotController::areJointsControlledExternally() {
    return m_bSettingJointsExternally;
}






