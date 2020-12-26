#include "RobotController.h"
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
using namespace ofxRobotArm;
RobotController::RobotController(){
   
}

RobotController::~RobotController(){
     robot->stopThread();
}

void RobotController::setup(RobotParameters & params) {
    setup(params.ipAddress, params, false);
}

void RobotController::setup(string ipAddress, RobotParameters & params, bool offline){
    if(params.get_robot_type() == UR3 || params.get_robot_type() == UR5 || params.get_robot_type()  == UR10){
        robot = new URDriver();
    }else if(params.get_robot_type() == IRB120){
        robot = new ABBDriver();
    }

    robotParams = params;
    
    desiredPose.setup(params.get_robot_type());
    actualPose.setup(params.get_robot_type());
 
    jointWeights.assign(6, 1.0f);
    smoothedPose.assign(6, 0.0f);
    
    inverseKinematics = InverseKinematics(params.get_robot_type(), &robotParams);
    
    if(!offline){
         robot->setAllowReconnect(params.bDoReconnect);
         robot->setup(ipAddress,0, 1);
    }
    smoothness = 0.01;
    bSmoothPose = true;

}


bool RobotController::arePoseControlledExternally() {
    return robotParams.bSettingPoseExternally;
}


void RobotController::setEndEffector(string filename){
    actualPose.setEndEffector(filename);
    desiredPose.setEndEffector(filename);
}

void RobotController::setHomePose(vector<double> pose){
    homePose = pose;
}

void RobotController::setToolOffset(ofVec3f local){
    desiredPose.setToolOffset(local);
    actualPose.setToolOffset(local);
}

void RobotController::start(){
    // Start the connection to the actual robot over TCP/IP
     robot->start();
}


vector<double> RobotController::getCurrentPose(){
    return  robot->getCurrentPose();
}

//------------------------------------------------------------------


void RobotController::toggleTeachMode(){
     robot->toggleTeachMode();
}

void RobotController::setTeachMode(){
    if(isTeachModeEnabled != robotParams.bTeachMode){
        isTeachModeEnabled = robotParams.bTeachMode;
         robot->setTeachMode(isTeachModeEnabled);
    }
}

#pragma mark - IK
void RobotController::updateIK(Pose pose){
    targetPoses = inverseKinematics.inverseKinematics(pose);
//    int i = 0;
//    for(auto d : targetPoses){
//        desiredPoses[i]->setPose(d);
//        i++;
//    }
    int selectedSolution = inverseKinematics.selectSolution(targetPoses,  robot->getCurrentPose(), jointWeights);
    if(selectedSolution > -1){
        targetPose = targetPoses[selectedSolution];
    }

    int i = 0;
    for(auto p : targetPose){
        float tpose = (float)p;
        if( isnan(tpose) ) {
            tpose = 0.f;
        }
        p = tpose;
        robotParams.targetPose[i].set(p);
        i++;
    }
}

#pragma mark - Update
void RobotController::update(){
    updateRobotData();
    // update the plane that visualizes the robot flange
    tcp_plane.update(target);
    updateIK(desiredPose.getModifiedTCPPose());
    updateMovement();


    if(targetPose.size() > 0){
        ofMatrix4x4 forwardIK = inverseKinematics.forwardKinematics(bSmoothPose?smoothedPose:targetPose);
        ofVec3f translation = forwardIK.getTranslation();
        ofNode forwardNode;
        forwardNode.setGlobalPosition(translation);
        forwardNode.setGlobalOrientation(forwardIK.getRotate());
        desiredPose.setPose(bSmoothPose?smoothedPose:targetPose);
        desiredPose.setForwardPose(forwardNode);
    }
}
void RobotController::update(vector<double> _pose){
    targetPose = _pose;
    updateMovement();
    if(targetPose.size() > 0){
        ofMatrix4x4 forwardIK = inverseKinematics.forwardKinematics(targetPose);
        ofVec3f translation = forwardIK.getTranslation();
        ofNode forwardNode;
        forwardNode.setGlobalPosition(translation);
        forwardNode.setGlobalOrientation(forwardIK.getRotate());
        desiredPose.setPose(targetPose);
        desiredPose.setForwardPose(forwardNode);
    }
}


#pragma mark - Safety
void RobotController::safetyCheck(){

    robotSafety.setCurrentRobotArmAnlges( robot->getCurrentPose());
    robotSafety.setDesiredAngles(targetPose);
    robotSafety.update(desiredPose);
    robotSafety.update(1/60);
    targetPose = robotSafety.getDesiredAngles();
    
}

#pragma mark - Movements
void RobotController::updateMovement(){
    
    int i = 0;
    for(auto p : prePose){
        if(abs(targetPose[i])-p >= TWO_PI || abs(targetPose[i])-p == 0){
            targetPose[i] = p;
        }
    }
    prePose = targetPose;
    
    if(bSmoothPose){
        int i = 0;
        vector<double> currentPose = getCurrentPose();
        for(auto p : targetPose){
            smoothedPose[i] = ofLerp(smoothedPose[i], p, smoothness);
            i++;
        }
    }
    

    if(robotParams.bMove){

        robot->setPose(bSmoothPose?smoothedPose:targetPose);
        stopPosition = targetPose;
        stopCount = 30;
    }
    else{
        if( stopCount > 0 && stopPosition.size() == targetPose.size() && stopPosition.size() > 0 ){
            for(int d = 0; d < stopPosition.size(); d++){
                stopPosition[d] *= 0.9998;
            }
             robot->setPose(stopPosition);
            cout << " Doing stop count " << stopCount << endl;
            stopCount--;
        }
    }
}

void RobotController::setDesired(ofNode target){
    // convert from mm to m
    robotParams.targetTCP.position = target.getGlobalPosition()/1000.0;
    robotParams.targetTCP.orientation = target.getGlobalOrientation();
    desiredPose.setTCPPose(robotParams.targetTCP);
    this->target = target;
}


#pragma mark - Data
//READS AND SETS IMPORTANT GUI INFO AND THE CURRENT POSE OF THE ARM
void RobotController::updateRobotData(){
    // pass the current joints from the robot to the kinematic solver
    
    robotParams.currentPose = getCurrentPose();
    actualPose.setPose(robotParams.currentPose);
    robotParams.actualTCP =  robot->getToolPose();
    robotParams.tcpPosition = robotParams.actualTCP.position;
    actualPose.setTCPPose(robotParams.actualTCP);
    ofQuaternion tcpO = robotParams.actualTCP.orientation;
    robotParams.tcpOrientation = ofVec4f(tcpO.x(), tcpO.y(), tcpO.z(), tcpO.w());
    // update GUI params
    for(int i = 0; i < robotParams.currentPose.size(); i++){
        robotParams.pCurrentPose[i] = ofRadToDeg((float)robotParams.currentPose[i]);
    }
    ofMatrix4x4 forwardIK = inverseKinematics.forwardKinematics(robotParams.currentPose);
    ofVec3f translation = forwardIK.getTranslation();
    forwardNode.setGlobalPosition(translation);
    forwardNode.setGlobalOrientation(forwardIK.getRotate());
    actualPose.setForwardPose(forwardNode);
}


void RobotController::close(){
    if( robot->isThreadRunning()){
         robot->stopThread();
    }
}

#pragma mark - drawing
void RobotController::draw(ofFloatColor color, bool debug){
    actualPose.draw(color, debug);
}

void RobotController::drawSafety(ofCamera & cam){
    robotSafety.draw();
}

void RobotController::drawIK(){
    inverseKinematics.draw();
}

void RobotController::drawDesired(ofFloatColor color){
    desiredPose.drawMesh(color, false);
    desiredPose.draw(color, false);

//    for(auto d : desiredPoses){
//        d->draw(color, false);
//    }
    tcp_plane.draw();
}







//void RobotController::updateIKArm(){
//
//    targetPoses = inverseKinematics.inverseKinematics(robotParams.targetTCP);
//    int selectedSolution = inverseKinematics.selectSolution(targetPoses,  robot->getCurrentPose(), jointWeights);
//    if(selectedSolution > -1){
//        targetPose = targetPoses[selectedSolution];
//        for(int i = 0; i < targetPose.size(); i++){
//            float tpose = (float)targetPose[i];
//            if( isnan(tpose) ) {
//                tpose = 0.f;
//            }
//            robotParams.ikPose[i] = tpose;
//        }
//    }
//    for(int i = 0; i < targetPose.size(); i++){
//        float tpose = (float)targetPose[i];
//        if( isnan(tpose) ) {
//            tpose = 0.f;
//        }
//        robotParams.targetPose[i] = ofRadToDeg(tpose);
//    }
//    targetPose = inverseKinematics.getArmIK(&actualPose, robotParams.targetTCP, targetPose, 1.0/60.0f);
//    for(int i = 0; i < targetPose.size(); i++){
//        float tpose = (float)targetPose[i];
//        if( isnan(tpose) ) {
//            tpose = 0.f;
//        }
//        robotParams.targetPose[i] = ofRadToDeg(tpose);
//    }
//
//    // update the look at angles after the IK has been applied //
//    // overrides the angles and sets them directly //
//    // alters joint[3] && joint[4]
////    vector< double > lookAtAngles = inverseKinematics.lookAtJoints(&actualPose, targetPose, 1.0/60.0f)
//    // determine if these angles should be added or not //
////    for( int i = 0; i < targetPose.size(); i++ ) {
////        targetPose[i] = lookAtAngles[i];
////    }
//
//    desiredPose.setPose(targetPose);
//}
