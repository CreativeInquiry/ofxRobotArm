#include "RobotController.h"
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
using namespace ofxRobotArm;
RobotController::RobotController(){
    
}

RobotController::~RobotController(){
    robot.stopThread();
}

void RobotController::setup(RobotParameters & params) {
    setup(params.ipAddress, params, false);
}

void RobotController::setup(string ipAddress, RobotType type){
    
    this->type = type;
    
    // setup robot parameters
    robotParams.setup(type);
    
   
    // @todo: this can go away
    //
    movement.setup();
    
    jointWeights.assign(6, 1.0f);
    // setup Kinematic Model
    inverseKinematics = InverseKinematics(type);
    desiredPose.setup(type);
    actualPose.setup(type);

}

void RobotController::setEndEffector(string filename){
    actualPose.setEndEffector(filename);
    desiredPose.setEndEffector(filename);
}

void RobotController::start(){
    // Start the connection to the actual robot over TCP/IP
    robot.start();
}

void RobotController::setup(string ipAddress, RobotParameters & params, bool offline){
    if(offline){
        robot.setAllowReconnect(params.bDoReconnect);
        robot.setup(ipAddress,0, 1);
        robot.start();
    }
    robotParams = params;
    

    
    
    movement.setup();
    for(int i = 0; i < 1; i++){//8; i++){
        RobotModel * foo = new RobotModel();
        foo->setup(params.get_robot_type());
        desiredPoses.push_back(foo);
    }
//    previewArm = desiredPoses[0];
    desiredPose.setup(params.get_robot_type());
    actualPose.setup(params.get_robot_type());
    
//    robotParams.safety.add(robotSafety.setup(params.get_robot_type()));
 
    robotParams.jointsIK.add(this->params);
    
    jointWeights.assign(6, 1.0f);
    
    // Set up URIKFast with dynamic RobotType
    inverseKinematics = InverseKinematics(params.get_robot_type());
}

vector<double> RobotController::getCurrentPose(){
    return robot.getCurrentPose();
}

//------------------------------------------------------------------


void RobotController::toggleTeachMode(){
    robot.toggleTeachMode();
}

void RobotController::setTeachMode(){
    if(isTeachModeEnabled != robotParams.bTeachMode){
        isTeachModeEnabled = robotParams.bTeachMode;
        robot.setTeachMode(isTeachModeEnabled);
    }
}

void RobotController::updateIKFast(){

    // update the plane that visualizes the robot flange
    tcp_plane.update(robotParams.targetTCP.position*1000, robotParams.targetTCP.orientation);

    targetPoses = inverseKinematics.inverseKinematics(robotParams.targetTCP);
    int selectedSolution = inverseKinematics.selectSolution(targetPoses, robot.getCurrentPose(), jointWeights);
    if(selectedSolution > -1){
        targetPose = targetPoses[selectedSolution];
    }
}

void RobotController::updateIKArm(){
    targetPoses = inverseKinematics.inverseKinematics(robotParams.targetTCP);
    int selectedSolution = inverseKinematics.selectSolution(targetPoses, robot.getCurrentPose(), jointWeights);
    if(selectedSolution > -1){
        targetPose = targetPoses[selectedSolution];
        for(int i = 0; i < targetPose.size(); i++){
            float tpose = (float)targetPose[i];
            if( isnan(tpose) ) {
                tpose = 0.f;
            }
            robotParams.ikPose[i] = tpose;
        }
    }
    for(int i = 0; i < targetPose.size(); i++){
        float tpose = (float)targetPose[i];
        if( isnan(tpose) ) {
            tpose = 0.f;
        }
        robotParams.targetPose[i] = ofRadToDeg(tpose);
    }
    targetPose = inverseKinematics.getArmIK(1.0/60.0f);
    for(int i = 0; i < targetPose.size(); i++){
        float tpose = (float)targetPose[i];
        if( isnan(tpose) ) {
            tpose = 0.f;
        }
        robotParams.targetPose[i] = ofRadToDeg(tpose);
    }
    
    // update the look at angles after the IK has been applied //
    // overrides the angles and sets them directly //
    // alters joint[3] && joint[4]
    vector< double > lookAtAngles = inverseKinematics.lookAtJoints(1.0/60.0f);
    // determine if these angles should be added or not //
    for( int i = 0; i < targetPose.size(); i++ ) {
        if(i > 2){
            targetPose[i] = lookAtAngles[i];
        }
    }
    
    
    desiredPose.setPose(targetPose);
}

#pragma mark - Update
void RobotController::update(){
    updateRobotData();
    updateIKArm();
//    if(robotParams.bUseIKFast){
//        updateIKFast();
//    }else if(robotParams.bUseIKArm){
//        updateIKArm();
//    }
//    safetyCheck();
    updateMovement();
    targetPose = movement.getTargetJointPose();

    //this is causing an error on my machine. an
//    for(int i = 0; i < targetPose.size(); i++){
//        float tpose = (float)targetPose[i];
//        if( isnan(tpose) ) {
//            tpose = 0.f;
//        }
//        robotParams.targetPose[i].set(tpose);
//    }
    if(targetPose.size() > 0){
        desiredPose.setPose(targetPose);
        ofMatrix4x4 forwardIK = inverseKinematics.forwardKinematics(targetPose);
        ofVec3f translation = forwardIK.getTranslation();
        forwardNode.setGlobalPosition(translation);
        forwardNode.setGlobalOrientation(forwardIK.getRotate());
        desiredPose.setForwardPose(forwardNode);
    }
}
void RobotController::update(vector<double> _pose){
    targetPose = _pose;
    update();
    
}

void RobotController::set_desired(ofNode target){
    // convert from mm to m
    robotParams.targetTCP.position = target.getGlobalPosition()/1000.0;
    robotParams.targetTCP.orientation = target.getGlobalOrientation();
    desiredPose.setTCPPose(robotParams.targetTCP);
}

#pragma mark - Safety
void RobotController::safetyCheck(){

    
    robotSafety.setCurrentRobotArmAnlges(robot.getCurrentPose());
    robotSafety.setDesiredAngles(targetPose);
    robotSafety.update(desiredPose);
    robotSafety.update(1/60);
    targetPose = robotSafety.getDesiredAngles();
    
}

#pragma mark - Movements
void RobotController::updateMovement(){
    movement.addTargetJointPose(targetPose);
    movement.update();
    // move the robot to the target TCP
    if(robotParams.bMove){
        //        robot.setSpeed(tempSpeeds, movement.getAcceleration());
        robot.setPosition(movement.getTargetJointPose());
        stopPosition = movement.getTargetJointPose();
        stopCount = 30;
    }
    else{
        if( stopCount > 0 && stopPosition.size() == movement.targetPose.size() && stopPosition.size() > 0 ){
            for(int d = 0; d < stopPosition.size(); d++){
                stopPosition[d] *= 0.9998;
            }
            robot.setPosition(stopPosition);
            cout << " Doing stop count " << stopCount << endl;
            stopCount--;
        }
    }
}


#pragma mark - Data
//READS AND SETS IMPORTANT GUI INFO AND THE CURRENT POSE OF THE ARM
void RobotController::updateRobotData(){
    // pass the current joints from the robot to the kinematic solver
    
    robotParams.currentPose = getCurrentPose();
    actualPose.setPose(robotParams.currentPose);
    robotParams.actualTCP = robot.getToolPose();
    robotParams.tcpPosition = robotParams.actualTCP.position;
    ofQuaternion tcpO = robotParams.actualTCP.orientation;
    robotParams.tcpOrientation = ofVec4f(tcpO.x(), tcpO.y(), tcpO.z(), tcpO.w());
    // update GUI params
    for(int i = 0; i < robotParams.currentPose.size(); i++){
        robotParams.pCurrentPose[i] = ofRadToDeg((float)robotParams.currentPose[i]);
    }
    ofMatrix4x4 forwardIK = inverseKinematics.forwardKinematics(robotParams.currentPose);
    movement.setCurrentJointPose(robotParams.currentPose);
}

#pragma mark - drawing
void RobotController::close(){
    if(robot.isThreadRunning()){
        robot.stopThread();
    }
}

void RobotController::draw(ofFloatColor color, bool debug){
    actualPose.draw(color, debug);
}

//void RobotController::drawPreviews(){
//    for(int i = 0; i < desiredPoses.size(); i++){
//        ofSetColor(255/(i+1), 0, 255/(i+1));
//        desiredPoses[i]->draw(false);
//    }
//}

void RobotController::drawSafety(ofCamera & cam){
    robotSafety.draw();
}

void RobotController::drawIK(){
    if(mIKArm) mIKArm->draw();
    if(mIKArmInverted)mIKArmInverted->draw();
}

void RobotController::drawDesired(ofFloatColor color){
    desiredPose.draw(color, false);
    tcp_plane.draw();
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






