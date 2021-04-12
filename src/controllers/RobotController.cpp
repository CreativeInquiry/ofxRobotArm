#include "RobotController.h"
//
// Copyright (c) 2016, 2021 Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
////
using namespace ofxRobotArm;
RobotController::RobotController()
{
    setupParams();
}

RobotController::~RobotController()
{
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
    robotArmParams.add(smoothness.set("Smooth", 0.01, 0.001, 1.0));
    robotArmParams.add(bOverrideNthJoint.set("Override Nth Joint", false));
    robotArmParams.add(nthJoint.set("Nth Joint", 0, -TWO_PI, TWO_PI));
    robotArmParams.add(origin.set("Origin", ofVec3f(0, 0, 0), ofVec3f(-500, -500, -500), ofVec3f(500, 500, 500)));

    joints.setName("Joint Pos");
    targetJoints.setName("Target Joints");
    jointsIK.setName("IK Solver");
    for (int i = 0; i < 6; i++)
    {
        pCurrentPose.push_back(ofParameter<double>());
        joints.add(pCurrentPose.back().set("actual joint " + ofToString(i), 0, -360, 360));
    }

    for (int i = 0; i < 6; i++)
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

void RobotController::setup(string ipAddress, string urdfPath, RobotType type, bool offline)
{
    this->type = type;

    if (this->type == UR3 || this->type == UR5 || this->type == UR10)
    {
        robot = new URDriver();
    }
    else if (type == IRB120)
    {
        robot = new ABBDriver();
    }

    desiredModel.setup(urdfPath, this->type);
    actualModel.setup(urdfPath, this->type);

    this->ipAddress = ipAddress;
    connectRobot(offline);
    initKinematics();
}

void RobotController::setup(string ipAddress, bool offline, RobotType type)
{
    this->type = type;

    if (this->type == UR3 || this->type == UR5 || this->type == UR10)
    {
        robot = new URDriver();
    }
    else if (type == IRB120)
    {
        robot = new ABBDriver();
    }

    desiredModel.setup(this->type);
    actualModel.setup(this->type);

    this->ipAddress = ipAddress;
    connectRobot(offline);
    initKinematics();
}

void RobotController::setNthJoint(double pose){
    nthJoint.set(pose);
}


void RobotController::setRobotOrigin(ofVec3f origin)
{
    this->origin = origin;
}

void RobotController::connectRobot(bool offline)
{
    if (!offline)
    {
        robot->setAllowReconnect(bDoReconnect);
        robot->setup(ipAddress, 0, 1);
    }
}

void RobotController::initKinematics()
{
    jointWeights.assign(6, 1.0f);
    smoothedPose.assign(6, 0.0f);
    vector<double> pose = robot->getInitPose();
    inverseKinematics.setup(type);
    inverseKinematics.setRelaxedPose(pose);
    ofMatrix4x4 forwardIK = inverseKinematics.forwardKinematics(pose);
    forwardNode.setGlobalPosition(forwardIK.getTranslation());
    forwardNode.setGlobalOrientation(forwardIK.getRotate());
    initPose.position = forwardIK.getTranslation();
    initPose.orientation = forwardIK.getRotate();
    for (int i = 0; i < smoothedPose.size(); i++)
    {
        smoothedPose[i] = pose[i];
    }
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
    robot->start();
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
    if (bUseRelaxedIK)
    {
        targetPoses = inverseKinematics.inverseRelaxedIK(pose, initPose);
    }
    if (bUseIKFast)
    {
        targetPoses = inverseKinematics.inverseIKFast(pose);
    }
    
    ofQuaternion rot = initPose.orientation * pose.orientation;
    calcTCPOrientation = ofVec4f(rot.x(), rot.y(), rot.z(), rot.w());
    int selectedSolution = inverseKinematics.selectSolution(targetPoses, robot->getCurrentPose(), jointWeights);
    if (selectedSolution > -1)
    {
        targetPose = targetPoses[selectedSolution];
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
    
    if(bOverrideNthJoint){
        targetPose[targetPose.size()-1] = nthJoint.get();
    }
}

#pragma mark - Update
void RobotController::update()
{
    updateRobotData();
    // update the plane that visualizes the robot flange
    tcp_plane.update(target);
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
        ofMatrix4x4 forwardIK = inverseKinematics.forwardKinematics(bSmoothPose ? smoothedPose : targetPose);
        ofNode fN;
        fN.setGlobalPosition(forwardIK.getTranslation());
        fN.setGlobalOrientation(forwardIK.getRotate());
        desiredModel.setPose(targetPose);
        desiredModel.setForwardPose(fN);
    }
}

#pragma mark - Safety
void RobotController::safetyCheck()
{

    // robotSafety.setCurrentRobotArmAnlges( robot->getCurrentPose());
    // robotSafety.setDesiredAngles(targetPose);
    // robotSafety.update(desiredModel);
    // robotSafety.update(1/60);
    // targetPose = robotSafety.getDesiredAngles();
}

#pragma mark - Movements
void RobotController::updateMovement()
{

    if (bSmoothPose)
    {
        int i = 0;
        vector<double> currentPose = getCurrentPose();
        for (auto p : targetPose)
        {
            smoothedPose[i] = ofLerp(smoothedPose[i], p, smoothness);
            targetPose[i] = (smoothedPose[i]);
            i++;
        }
    }

    if (targetPose.size() > 0 && smoothedPose.size() > 0)
    {
        ofMatrix4x4 forwardIK = inverseKinematics.forwardKinematics(bSmoothPose ? smoothedPose : targetPose);
        ofNode fN;
        fN.setGlobalPosition(forwardIK.getTranslation());
        fN.setGlobalOrientation(forwardIK.getRotate());
        desiredModel.setPose(bSmoothPose ? smoothedPose : targetPose);
        desiredModel.setForwardPose(fN);
    }

    if (bLive)
    {
        robot->setPose(bSmoothPose ? smoothedPose : targetPose);
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
    targetTCP.position = (target.getGlobalPosition() - origin) / 1000.0;
    targetTCP.orientation = target.getGlobalOrientation();
    desiredModel.setTCPPose(targetTCP);
    this->target = target;
}

ofNode RobotController::getActualTCPNode()
{
    return forwardNode;
}

#pragma mark - Data
//READS AND SETS IMPORTANT GUI INFO AND THE CURRENT POSE OF THE ARM
void RobotController::updateRobotData()
{
    // pass the current joints from the robot to the kinematic solver

    currentPose = getCurrentPose();
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
    for (unsigned int i = 0; i < currentPose.size(); i++)
    {
        pCurrentPose[i] = ofRadToDeg((float)currentPose[i]);
    }
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
        ofTranslate(origin.get());
        actualModel.drawMesh(color, debug);
        actualModel.draw(color, debug);
    }
    ofPopMatrix();
}

void RobotController::drawSafety(ofCamera &cam)
{
    // robotSafety.draw();
}

void RobotController::drawIK()
{
    inverseKinematics.draw();
}

void RobotController::drawDesired(ofColor color)
{

    ofPushMatrix();
    {
        ofTranslate(origin.get());
        desiredModel.drawMesh(color, false);
        desiredModel.draw(color, false);
        desiredModel.drawSkeleton();
    }
    ofPopMatrix();

    tcp_plane.draw();
}

//void RobotController::updateIKArm(){
//
//    targetPoses = inverseKinematics.inverseKinematics(targetTCP);
//    int selectedSolution = inverseKinematics.selectSolution(targetPoses,  robot->getCurrentPose(), jointWeights);
//    if(selectedSolution > -1){
//        targetPose = targetPoses[selectedSolution];
//        for(int i = 0; i < targetPose.size(); i++){
//            float tpose = (float)targetPose[i];
//            if( isnan(tpose) ) {
//                tpose = 0.f;
//            }
//            ikPose[i] = tpose;
//        }
//    }
//    for(int i = 0; i < targetPose.size(); i++){
//        float tpose = (float)targetPose[i];
//        if( isnan(tpose) ) {
//            tpose = 0.f;
//        }
//        targetPose[i] = ofRadToDeg(tpose);
//    }
//    targetPose = inverseKinematics.getArmIK(&actualModel, targetTCP, targetPose, 1.0/60.0f);
//    for(int i = 0; i < targetPose.size(); i++){
//        float tpose = (float)targetPose[i];
//        if( isnan(tpose) ) {
//            tpose = 0.f;
//        }
//        targetPose[i] = ofRadToDeg(tpose);
//    }
//
//    // update the look at angles after the IK has been applied //
//    // overrides the angles and sets them directly //
//    // alters joint[3] && joint[4]
////    vector< double > lookAtAngles = inverseKinematics.lookAtJoints(&actualModel, targetPose, 1.0/60.0f)
//    // determine if these angles should be added or not //
////    for( int i = 0; i < targetPose.size(); i++ ) {
////        targetPose[i] = lookAtAngles[i];
////    }
//
//    desiredModel.setPose(targetPose);
//}
