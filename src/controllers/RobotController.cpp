#include "RobotController.h"
//
// Copyright (c) 2016, 2021 Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
////
using namespace ofxRobotArm;
RobotController::RobotController()
{
}

RobotController::~RobotController()
{
}

void RobotController::setup(string ipAddress, string urdfPath, RobotType robotType, IKType ikType, bool offline)
{
    this->robotType = robotType;
    this->ipAddress = ipAddress;

    createRobot(this->robotType);
    createModels(urdfPath);
    connectRobot(offline);
    initKinematics(ikType);
    setupParams();
}

void RobotController::createRobot(RobotType type)
{
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
}

void RobotController::createModels(string urdfpath)
{
    desiredModel.setup(urdfpath, robotType);
    actualModel.setup(urdfpath, robotType);
}

void RobotController::connectRobot(bool offline)
{
    if (!offline && robot != nullptr)
    {
        robot->setAllowReconnect(bDoReconnect);

        if (robotType == UR3 || robotType == UR5 || robotType == UR10)
        {
            robot->setup(ipAddress, 0, 1);
        }
        else if (robotType == IRB120)
        {
            robot->setup(ipAddress, 0, 1);
        }
        else if (robotType == XARM7)
        {
            robot->setup(ipAddress, 0, 1);
        }
    }
}

void RobotController::setupParams()
{

    robotArmParams.setName("Robot Controls");
    robotArmParams.add(bLive.set("Live", false));
    robotArmParams.add(bTeachMode.set("Enable Teach Mode", false));
    robotArmParams.add(bUseIKFast.set("Use IKFast", false));
    robotArmParams.add(bUseRelaxedIK.set("Use RelaxedIK", true));
    robotArmParams.add(bUseIKArm.set("Use IKArm", false));
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

    pathRecorderParams.setName("Path Recording");
    pathRecorderParams.add(bRecord.set("Record", false));

    joints.add(tcpPosition.set("Actual Robot TCP POS", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1)));
    joints.add(tcpOrientation.set("Actual Robot TCP ORIENT", ofVec4f(0, 0, 0, 1), ofVec4f(-1, -1, -1, -1), ofVec4f(1, 1, 1, 1)));
    joints.add(calcTCPOrientation.set("Relaxed Robot TCP ORIENT", ofVec4f(0, 0, 0, 1), ofVec4f(-1, -1, -1, -1), ofVec4f(1, 1, 1, 1)));
    forwardTCPOrientation.set("Forward TCP ORIENT", ofVec4f(0, 0, 0, 1), ofVec4f(-1, -1, -1, -1), ofVec4f(1, 1, 1, 1));
    forwardTCPPosition.set("Forward TCP Pos", ofVec3f(0, 0, 0), ofVec3f(-1, -1, -1), ofVec3f(1, 1, 1));
}

void RobotController::setNthJoint(double pose)
{
    nthJoint.set(pose);
}

void RobotController::setRobotOrigin(ofVec3f origin, ofQuaternion orientation)
{
    this->origin = origin;
    desiredModel.setOrigin(this->origin, orientation);
    actualModel.setOrigin(this->origin, orientation);
}

void RobotController::initKinematics(ofxRobotArm::IKType ikType)
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
    inverseKinematics.setup(robotType, ikType, pose);
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

void RobotController::setIKType(ofxRobotArm::IKType ikType){
    inverseKinematics.setIKType(ikType);
}

void RobotController::toggleLive()
{
    bLive = !bLive;
}

bool RobotController::isLive()
{
    return bLive.get();
}

void RobotController::setHomePose(vector<double> pose)
{
    homePose = pose;
}

bool RobotController::arePoseControlledExternally()
{
    return bSetPoseExternally;
}

void RobotController::setEndEffector(string filename)
{
    actualModel.setEndEffector(filename);
    desiredModel.setEndEffector(filename);
}

void RobotController::setToolOffset(ofVec3f local)
{
    desiredModel.setToolOffset(local);
    actualModel.setToolOffset(local);
}

void RobotController::start()
{
    // Start the connection to the actual robot over TCP/IP
    if(robot != nullptr){
        robot->start();
    }
    
}

vector<double> RobotController::getCurrentPose()
{
    return robot->getCurrentPose();
}

//------------------------------------------------------------------

void RobotController::toggleTeachMode()
{
    robot->toggleTeachMode();
}

void RobotController::setTeachMode(bool teachMode)
{
    robot->setTeachMode(teachMode);
}

#pragma mark - IK
void RobotController::updateIK(Pose pose)
{

    ofQuaternion rot = initPose.orientation * pose.orientation;
    calcTCPOrientation = ofVec4f(rot.x(), rot.y(), rot.z(), rot.w());
    
    targetPoses = inverseKinematics.inverseKinematics(pose, initPose);
    if(targetPoses.size() > 0){
        targetPose = targetPoses[0];
    }

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
void RobotController::update()
{
    updateRobotData();
    updateIK(desiredModel.getModifiedTCPPose());
    updateMovement();
}

void RobotController::update(vector<double> _pose)
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
void RobotController::updateRobotData()
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

        actualModel.setForwardPose(forwardNode);

        actualTCP = robot->getToolPose();
        tcpPosition = actualTCP.position;
        actualModel.setTCPPose(actualTCP);

        ofQuaternion tcpO = actualTCP.orientation;
        tcpOrientation = ofVec4f(tcpO.x(), tcpO.y(), tcpO.z(), tcpO.w());
        // update GUI params
        int i = 0;
        for (auto p : pCurrentPose)
        {
            p = ofRadToDeg((float)currentPose[i++]);
        }
    }
}

#pragma mark - Movements
void RobotController::updateMovement()
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

void RobotController::setDesired(ofNode target)
{
    // convert from mm to m
    targetTCP.position = (target.getGlobalPosition()) / 1000.0;
    targetTCP.orientation = target.getGlobalOrientation();
    desiredModel.setTCPPose(targetTCP);
    this->target = target;
}

ofNode RobotController::getTCPNode()
{
    return actualModel.getForwardPose();
}

ofNode RobotController::getForwardNode(){
    return forwardNode;
}


void RobotController::close()
{
    if (robot->isThreadRunning())
    {
        robot->stopThread();
    }
}

#pragma mark - drawing
void RobotController::draw(ofColor color, bool debug)
{
    ofPushMatrix();
    {
        actualModel.drawMesh(color, debug);
        actualModel.draw(color, debug);
        actualModel.drawSkeleton();
    }
    ofPopMatrix();
}


void RobotController::drawDesired(ofColor color)
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

