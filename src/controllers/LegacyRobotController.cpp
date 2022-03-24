#include "LegacyRobotController.h"
//
// Copyright (c) 2016, 2021 Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
////
using namespace ofxRobotArm;
LegacyRobotController::LegacyRobotController()
{
}

LegacyRobotController::~LegacyRobotController()
{
}

void LegacyRobotController::setup(string ipAddress, int port, string urdfPath, RobotType robotType, IKType ikType, bool offline)
{
    this->robotType = robotType;
    setAddress(ipAddress);
    setPort(port);
    
    createRobot(this->robotType);
    loadURDF(urdfPath);
    setupRobot(offline);
    initKinematics(ikType);
    setupParams();
}

void LegacyRobotController::setAddress(string ipAddress){
    this->ipAddress = ipAddress;
}
void LegacyRobotController::setPort(int port){
    this->port = port;
}

void LegacyRobotController::createRobot(RobotType type)
{
    robotType = type;
    if (robotType == UR3 || robotType == UR5 || robotType == UR10)
    {
        robot = new URDriver();
    }
    else if (robotType == IRB120 || robotType == IRB4600 || robotType == IRB6700) 
    {
        robot = new ABBDriver();
    }
    else if (robotType == XARM7)
    {
        robot = new XARMDriver(type);
    }
    else if(robotType == PANDA){
        robot = new PandaDriver();
    }
}

void LegacyRobotController::loadURDF(string urdfpath)
{
    desiredModel.setup(urdfpath, robotType);
    actualModel.setup(urdfpath, robotType);
}

void LegacyRobotController::setupRobot(bool offline)
{
    if (!offline && robot != nullptr)
    {
        robot->setAllowReconnect(bDoReconnect);

        if (robotType == UR3 || robotType == UR5 || robotType == UR10)
        {
            robot->setup(ipAddress, 0, 1);
        }
        else if (robotType == IRB120 || robotType == IRB4600 || robotType == IRB6700)
        {
            robot->setup(port, 0, 1);
        }
        else if (robotType == XARM7)
        {
            robot->setup(ipAddress, 0, 1);
        }
    }
}

void LegacyRobotController::disconnectRobot()
{
    if (robot != nullptr)
    {
        robot->disconnect();
    }
}

void LegacyRobotController::setupParams()
{

    robotArmParams.setName("Robot Controls");
    robotArmParams.add(bLive.set("Live", false));
    robotArmParams.add(bTeachMode.set("Enable Teach Mode", false));
    robotArmParams.add(bDoReconnect.set("TryReconnect", false));
    robotArmParams.add(bSmoothPose.set("Smooth Pose", true));
    robotArmParams.add(bOverrideNthJoint.set("Override Nth Joint", false));
    robotArmParams.add(nthJoint.set("Nth Joint", 0, -TWO_PI, TWO_PI));
    robotArmParams.add(origin.set("Origin", ofVec3f(0, 0, 0), ofVec3f(-500, -500, -500), ofVec3f(500, 500, 500)));

    joints.setName("Joint Pos");
    targetJoints.setName("Target Joints");
    jointsIK.setName("IK Solver");
    for (unsigned int i = 0; i < currentPose.size(); i++)
    {
        pCurrentPose.push_back(ofParameter<double>());
        joints.add(pCurrentPose.back().set("actual joint " + ofToString(i), 0, -360, 360));
    }
    for (unsigned int i = 0; i < currentPose.size(); i++)
    {
        pTargetPose.push_back(ofParameter<double>());
        targetJoints.add(pTargetPose.back().set("target joint " + ofToString(i), 0, -360, 360));
        pIkPose.push_back(ofParameter<double>());
        jointsIK.add(pIkPose.back().set("ik joint " + ofToString(i), 0, -360, 360));
    }

    joints.add(tcpPosition.set("Actual Robot TCP POS", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
    joints.add(tcpOrientation.set("Actual Robot TCP ORIENT", ofVec4f(0, 0, 0, 1), ofVec4f(-1, -1, -1, -1), ofVec4f(1, 1, 1, 1)));
    joints.add(calcTCPOrientation.set("Relaxed Robot TCP ORIENT", ofVec4f(0, 0, 0, 1), ofVec4f(-1, -1, -1, -1), ofVec4f(1, 1, 1, 1)));
    joints.add(forwardTCPOrientation.set("Forward TCP ORIENT", ofVec4f(0, 0, 0, 1), ofVec4f(-1, -1, -1, -1), ofVec4f(1, 1, 1, 1)));
    joints.add(forwardTCPPosition.set("Forward TCP Pos", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
}

void LegacyRobotController::setNthJoint(double pose)
{
    nthJoint.set(pose);
}

void LegacyRobotController::setRobotOrigin(ofVec3f origin, ofQuaternion orientation)
{
    this->origin = origin;
    desiredModel.setOrigin(this->origin, orientation);
    actualModel.setOrigin(this->origin, orientation);
}

void LegacyRobotController::initKinematics(ofxRobotArm::IKType ikType)
{

    vector<double> pose = robot->getInitPose();
    jointWeights.assign(pose.size(), 1.0f);
    smoothedPose.assign(pose.size(), 0.0f);
    for(int i = 0 ; i < pose.size(); i++){
        ofParameter<double> smooth;
        smooth.set("Smooth-"+ofToString(i), 0.1, 0.001, 1.0);
        robotArmParams.add(smooth);
        smoothedWeights.push_back(smooth);
    }
    inverseKinematics.setup(robotType, ikType, pose, &actualModel);
    ofMatrix4x4 forwardIK = inverseKinematics.forwardKinematics(pose);
    forwardNode.setGlobalPosition(forwardIK.getTranslation());
    forwardNode.setGlobalOrientation(forwardIK.getRotate());
    initPose.position = forwardIK.getTranslation();
    initPose.orientation = forwardIK.getRotate();

    int i = 0;
    for (auto joint : smoothedPose)
    {
        joint = pose[i];
        i++;
    }
}

void LegacyRobotController::setIKType(ofxRobotArm::IKType ikType){
    inverseKinematics.setIKType(ikType);
}

void LegacyRobotController::setPoseExternally(bool externally){
    bSetPoseExternally = externally;
}

void LegacyRobotController::toggleLive()
{
    bLive = !bLive;
}

bool LegacyRobotController::isLive()
{
    return bLive.get();
}

bool LegacyRobotController::isConnected()
{
    return robot->isConnected();
}

void LegacyRobotController::setEnableMovement(bool move){
    bLive = move;
}

void LegacyRobotController::setHomePose(vector<double> pose)
{
    homePose = pose;
}

bool LegacyRobotController::isPoseControlledExternally()
{
    return bSetPoseExternally;
}

void LegacyRobotController::setEndEffector(string filename)
{
    actualModel.setEndEffector(filename);
    desiredModel.setEndEffector(filename);
}

void LegacyRobotController::setToolOffset(ofVec3f local)
{
    desiredModel.setToolOffset(local/1000);
    actualModel.setToolOffset(local/1000);
}

void LegacyRobotController::startConnection()
{
    // Start the connection to the actual robot over TCP/IP
    if(robot != nullptr){
        robot->start();
    }
}

vector<double> LegacyRobotController::getCurrentPose()
{
    return robot->getCurrentPose();
}

//------------------------------------------------------------------

void LegacyRobotController::toggleTeachMode()
{
    robot->toggleTeachMode();
}

void LegacyRobotController::setTeachMode(bool teachMode)
{
    robot->setTeachMode(teachMode);
}

#pragma mark - IK
void LegacyRobotController::updateIK(Pose pose)
{

    targetPoses = inverseKinematics.inverseKinematics(pose, initPose);
    if(targetPoses.size() > 0){
        targetPose = targetPoses[0];
    }
    
    ofQuaternion rot = initPose.orientation.inverse() * pose.orientation;
    calcTCPOrientation = ofVec4f(rot.x(), rot.y(), rot.z(), rot.w());

    int i = 0;
    for (auto p : targetPose)
    {
        float tpose = (float)p;
        if (isnan(tpose))
        {
            tpose = 0.f;
        }
        p = tpose;
        i++;
    }

    if (bOverrideNthJoint)
    {
        targetPose[targetPose.size() - 1] = nthJoint.get();
    }
}

#pragma mark - Update
void LegacyRobotController::update()
{
    updateRobotData();
    if(!bSetPoseExternally)updateIK(desiredModel.getModifiedTCPPose());
    updateMovement();
    
}

void LegacyRobotController::update(vector<double> _pose)
{
    updateRobotData();
    targetPose = _pose;
    updateMovement();
    if (targetPose.size() > 0)
    {
        ofMatrix4x4 forwardIK = inverseKinematics.forwardKinematics(targetPose);
        ofNode fN;
        fN.setGlobalPosition(forwardIK.getTranslation());
        fN.setGlobalOrientation(forwardIK.getRotate());
        desiredModel.setPose(targetPose);
        desiredModel.setForwardPose(fN);
    }
}

#pragma mark - Data
//READS AND SETS IMPORTANT GUI INFO AND THE CURRENT POSE OF THE ARM
void LegacyRobotController::updateRobotData()
{
    // pass the current joints from the robot to the kinematic solver

    currentPose = getCurrentPose();

    if(currentPose.size() > 0){
        actualModel.setPose(currentPose);

        ofMatrix4x4 forwardIK = inverseKinematics.forwardKinematics(currentPose);
        ofVec3f translation = forwardIK.getTranslation();
        forwardNode.setGlobalPosition(translation);
        forwardNode.setGlobalOrientation(forwardIK.getRotate());
        forwardPose.position = translation;
        forwardPose.orientation = forwardIK.getRotate();
        
        forwardTCPPosition = forwardIK.getTranslation()*1000;
        forwardTCPOrientation = forwardIK.getRotate().asVec4();

        actualModel.setForwardPose(forwardNode);

        actualTCP = robot->getToolPose();
        tcpPosition = actualTCP.position;
        actualModel.setTCPPose(actualTCP);

        tcpOrientation = actualTCP.orientation.asVec4();

        int i = 0;
        for (auto p : pCurrentPose)
        {
            p = ofRadToDeg((float)currentPose[i++]);
        }
    }
}

#pragma mark - Movements
void LegacyRobotController::updateMovement()
{
    tcp_plane.update(target);

    if (bSmoothPose)
    {
        int i = 0;
        for (auto p : targetPose)
        {
            smoothedPose[i] = ofLerp(smoothedPose[i], p, smoothedWeights[i]);
            targetPose[i] = smoothedPose[i];
            i++;
        }
    }
    
    if(bHome && homePose.size() == targetPose.size())
    {
        int i = 0;
        for(auto p: targetPose)
        {
            p = ofLerp(currentPose[i], homePose[i], smoothedWeights[i]);
            i++;
        }
    }
    
    if (targetPose.size() > 0)
    {
        ofMatrix4x4 forwardIK = inverseKinematics.forwardKinematics(targetPose);
        ofNode fN;
        fN.setGlobalPosition(forwardIK.getTranslation());
        fN.setGlobalOrientation(forwardIK.getRotate());
        desiredModel.setPose(targetPose);
        desiredModel.setForwardPose(fN);
    }

    if (bLive)
    {
        robot->setPose(targetPose);
        stopPosition = targetPose;
        stopCount = 30;
    }
    else
    {
        if (stopCount > 0 && stopPosition.size() == targetPose.size() && stopPosition.size() > 0)
        {
            for (int d = 0; d < stopPosition.size(); d++)
            {
                stopPosition[d] *= 0.9998;
            }
            robot->setPose(stopPosition);
            cout << " Doing stop count " << stopCount << endl;
            stopCount--;
        }
    }
}

void LegacyRobotController::setDesiredPose(ofNode target)
{
    // convert from mm to m
    targetTCP.position = (target.getGlobalPosition()) / 1000.0;
    targetTCP.orientation = target.getGlobalOrientation();
    desiredModel.setTCPPose(targetTCP);
    this->target = target;
}

ofNode LegacyRobotController::getTCPNode()
{
    return actualModel.getTool();
}

ofNode LegacyRobotController::getForwardNode(){
    return forwardNode;
}


void LegacyRobotController::close()
{
    if (robot->isThreadRunning())
    {
        robot->stopThread();
    }
}

#pragma mark - drawing
void LegacyRobotController::drawActual(ofColor color, bool debug)
{
    ofPushMatrix();
    {
        actualModel.drawMesh(color, debug);
        actualModel.draw(color, debug);
//        actualModel.drawSkeleton();
    }
    ofPopMatrix();
}


void LegacyRobotController::drawDesired(ofColor color)
{

    ofPushMatrix();
    {
        desiredModel.drawMesh(color, false);
        desiredModel.draw(color, false);
        desiredModel.drawSkeleton();
    }
    ofPopMatrix();

    tcp_plane.draw();
}

