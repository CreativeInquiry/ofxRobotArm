#include "RobotController.h"
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
using namespace ofxRobotArm;
RobotController::RobotController(){
    
}

RobotController::~RobotController(){
//    if(robotParams){
//        delete robotParams;
//        robotParams = NULL;
//    }
//    if(previewArm){
//        delete previewArm;
//        previewArm = NULL;
//    }
}

void RobotController::setup(RobotParameters & params) {
    setup(params.ipAddress, params, false);
}

void RobotController::setup(string ipAddress, RobotType type, bool offline){
    
    this->type = type;
    
    // setup robot parameters
    robotParams.setup(type);
    robotParams.ipAddress = ipAddress;
    
    // setup Kinematic Model
    movement.setup();       // speed profile
    
    previewArm.setup(type); // should be called sim or desired or something
    
    jointWeights.assign(6, 1.0f);
    urKinematics = URIKFast(type);
    
    // setup actual robot
    actualArm.setup(type);
    
    // pass the ip address to the robot
    if(offline){
        robot.setAllowReconnect(false);
    }
    else{
        robot.setAllowReconnect(true);
    }
    robot.setup(ipAddress,0,1);
}

void RobotController::setEndEffector(string filename){
    actualArm.setEndEffector(filename);
    previewArm.setEndEffector(filename);
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
    
    this->params.setName("IKArm Commands");
    this->params.add( bControlIkWithMouse.set("ControlIkWithMouse", false ));
    this->params.add( bOnlyUseInverseIk.set("OnlyUseInverseIK", false ));
    
    this->params.add( ikRobotMinY.set( "IkRobotMinY", -725, -2000, 2000 ));
    this->params.add( ikRobotMaxY.set( "IkRobotMaxY", 0, -2000, 2000 ));
    
    this->params.add( ikRobotMinZ.set( "IkRobotMinZ", 300, -500, 3000 ));
    this->params.add( ikRobotMaxZ.set( "IkRobotMaxZ", 700, -500, 3000 ));
    
    
    
    this->params.add( mIKRampStartPct.set("IKRampStartPct", 0.3, 0.0, 1.0 ));
    this->params.add( mIKRampEndPct.set("IKRampEndPct", 1.5, 1.0, 2.0 ));
    this->params.add( mIKRampHeightPct.set("IKRampHeightPct", 0.3, 0.0, 1.0 ));
    
    
    movement.setup();
    for(int i = 0; i < 1; i++){//8; i++){
        RobotKinematicModel * foo = new RobotKinematicModel();
        foo->setup(params.get_robot_type());
        previewArms.push_back(foo);
    }
//    previewArm = previewArms[0];
    previewArm.setup(params.get_robot_type());
    actualArm.setup(params.get_robot_type());
    
//    robotParams.safety.add(robotSafety.setup(params.get_robot_type()));
 
    robotParams.jointsIK.add(this->params);
    
    jointWeights.assign(6, 1.0f);
    
    // Set up URIKFast with dynamic RobotType
    urKinematics = URIKFast(params.get_robot_type());
}

vector<double> RobotController::getCurrentPose(){
    return robot.getCurrentPose();
}

//------------------------------------------------------------------
vector< double > RobotController::getArmIK( float aDeltaTimef ) {
    vector <double> armFrom = getArmIK( robotParams.targetTCP.position*1000.0f, ofVec3f(), true, aDeltaTimef );
    return armFrom;
}


//------------------------------------------------------------------
vector< double > RobotController::getArmIK( ofVec3f aTargetWorldPos, ofVec3f aElbowWorldPos, bool aBInvertElbow, float aDeltaTimef ) {
    // ok, now let's address the ik //
    if( !mIKArm ) {
        mIKArm = shared_ptr< ofxIKArm >( new ofxIKArm() );
        
    }
    if( !mIKArmInverted ) {
        mIKArmInverted = shared_ptr< ofxIKArm >( new ofxIKArm() );
    }
    if( mIKArm->getArmLength() <= 0.0f ) {
        mIKArm->setDrawSize( 10 );
        mIKArm->setup(actualArm.nodes[1].getGlobalPosition(), actualArm.nodes[2].getGlobalPosition(), actualArm.nodes[3].getGlobalPosition() );
        mIKArm->setDrawSize( 10 );
        
        // disable the elbow target and set to constrain along axis //
        mIKArm->setShoulderUpVectorEnabled( true );
        mIKArm->setShoulderUpVector( ofVec3f(0,-1,0));
        mIKArm->setInverted( true );
        
        ofLog(OF_LOG_VERBOSE) << "Setting up the ik arm " << mIKArm->getArmLength() << " shoulder: " << mIKArm->getUpperArmLength() << " forearm len: " << mIKArm->getLowerArmLength() << " | " << ofGetFrameNum() << endl;
    }
    if( mIKArmInverted->getArmLength() <= 0.0f ) {
        mIKArmInverted->setup( actualArm.nodes[1].getGlobalPosition(), actualArm.nodes[2].getGlobalPosition(), actualArm.nodes[3].getGlobalPosition() );
        mIKArmInverted->setDrawSize( 10 );
        
        // disable the elbow target and set to constrain along axis //
        mIKArmInverted->setShoulderUpVectorEnabled( true );
        mIKArmInverted->setShoulderUpVector( ofVec3f(0,1,0) );
        mIKArmInverted->setInverted( false );
        
        ofLog(OF_LOG_VERBOSE) << "Setting up the inverted ik arm " << mIKArmInverted->getArmLength() << " shoulder: " << mIKArmInverted->getUpperArmLength() << " forearm len: " << mIKArmInverted->getLowerArmLength() << " | " << ofGetFrameNum() << endl;
    }
    
    // now we are ready to ik all day //
    //if( aBInvertElbow ) {
    mIKArmInverted->setTarget( aTargetWorldPos );
    //        mIKArmInverted->setElbowTarget( aElbowWorldPos );
    mIKArmInverted->update();
    //} else {
    mIKArm->setTarget( aTargetWorldPos );
    //        mIKArm->setElbowTarget( aElbowWorldPos );
    mIKArm->update();
    //}
    
    vector< double > ttargetAngles = targetPose;
    
    // figure out how to pull out the angles //
    // the y is the forward, so we can measure the rotation around the xaxis //
    ofQuaternion globalRot, elbowRot;//  = mIKArm->getShoulderJoint()->getGlobalTransform().getRotate();
    ofQuaternion globalRotI, elbowRotI;//
    
    globalRotI   = mIKArmInverted->getShoulderJoint()->getGlobalTransform().getRotate();
    elbowRotI    = mIKArmInverted->getElbowJoint()->localTransform.getRotate();
    
    globalRot   = mIKArm->getShoulderJoint()->getGlobalTransform().getRotate();
    elbowRot    = mIKArm->getElbowJoint()->localTransform.getRotate();
    
    
    ofVec3f shouldYPR       = getYawPitchRoll( globalRot );
    ofVec3f elbowYPR        = getYawPitchRoll( elbowRot );
    
    //    targetPose[0] = ofWrapRadians( shouldYPR.x - PI/2 );
    
    if( aBInvertElbow ) {
        // the up axis is reversed, so we don't need to invert calculations //
        ttargetAngles[1] = ofWrapRadians( shouldYPR.z+PI/2, ofDegToRad(-180.f), ofDegToRad(180) );
        ttargetAngles[2] = ofWrapRadians( elbowYPR.z+PI, ofDegToRad(-180.f), ofDegToRad(180) );
        //        ttargetAngles[1] = ofWrapRadians( PI-shouldYPR.z + PI/2+PI, ofDegToRad(-360.f), ofDegToRad(360) );
        //        ttargetAngles[2] = ofWrapRadians( PI-elbowYPR.z, ofDegToRad(-360.f), ofDegToRad(360) );
    } else {
        //        ttargetAngles[1] = ofWrapRadians( PI-shouldYPR.z + PI/2+PI, ofDegToRad(-360.f), ofDegToRad(360) );
        //        ttargetAngles[2] = ofWrapRadians( PI-elbowYPR.z, ofDegToRad(-360.f), ofDegToRad(360) );
        
        ttargetAngles[1] = ofWrapRadians( shouldYPR.z+PI/2, ofDegToRad(-180), ofDegToRad(180) );
        ttargetAngles[2] = ofWrapRadians( elbowYPR.z+PI, ofDegToRad(-180), ofDegToRad(180) );
    }
    
    //    ofLog(OF_LOG_VERBOSE) << "PersonReact :: angles [1]: " << ofRadToDeg(ttargetAngles[1]) << " [2]: " << ofRadToDeg(ttargetAngles[2]) << " | " << ofGetFrameNum() << endl;
    //    ofLog(OF_LOG_VERBOSE) << "PersonReact :: getArmIK :: angle [2]: " << ofRadToDeg(ttargetAngles[2]) << " | " << ofGetFrameNum() << endl;
    
    //    mModelTarget->setAngles( targetPose );
    
    return ttargetAngles;
}




#pragma mark - IK Utils
//------------------------------------------------------------------
ofVec3f RobotController::getYawPitchRoll( ofQuaternion aquat ) {
    float qx = aquat.x();
    float qy = aquat.y();
    float qz = aquat.z();
    float qw = aquat.w();
    
    float yaw   =  atan2(2*qx*qy + 2*qw*qz, qw*qw + qx*qx - qy*qy - qz*qz);
    float pitch = -asin(2*qw*qy - 2*qx*qz);
    float roll  = -atan2(2*qy*qz + 2*qw*qx, -qw*qw + qx*qx + qy*qy - qz*qz);
    
    return ofVec3f( yaw, pitch, roll );
}

//------------------------------------------------------------------
float RobotController::getNeckAngleAlignedWithVector( ofVec3f avec ) {
    // let's figure out the head position //
    ofQuaternion orotate;
    orotate.makeRotate( ofVec3f(1,0,0), ofVec3f(0,-1,0) );
    
    ofVec3f upAxis( 1,0,0 );
    ofVec3f diff = avec.getNormalized();//ttarget - actualArm.nodes[3].getGlobalPosition();
    ofQuaternion tquat;
    ofVec3f txaxis = upAxis * orotate;
    tquat.makeRotate( txaxis, diff );
    
    ofMatrix4x4 invParent(ofMatrix4x4::getInverseOf(actualArm.nodes[3].getParent()->getGlobalTransformMatrix()));
    ofMatrix4x4 m44(ofMatrix4x4(tquat) * invParent);
    ofQuaternion localRot = m44.getRotate();
    
    ofVec3f newYPR = getYawPitchRoll( localRot );
    //        targetPose[ 3 ] = PI-newYPR.z + PI/2;
    //    ttargetAngles[ 3 ] = newYPR.z + PI/2;
    return newYPR.z;// + PI/2.f;
}

//------------------------------------------------------------------
ofVec3f RobotController::lerp( ofVec3f aStartVec, ofVec3f aEndVec, float aLerpAmnt ) {
    ofVec3f tmp;
    tmp.x = ofLerp( aStartVec.x, aEndVec.x, aLerpAmnt );
    tmp.y = ofLerp( aStartVec.y, aEndVec.y, aLerpAmnt );
    tmp.z = ofLerp( aStartVec.z, aEndVec.z, aLerpAmnt );
    return tmp;
}

//float ofAngleDifferenceRadians(float currentAngle, float targetAngle) {
//    return  ofWrapRadians(targetAngle - currentAngle);
//}

//--------------------------------------------------
float RobotController::lerpRadians(float currentAngle, float targetAngle, float pct, float alerp ) {
    //    return ofWrapRadians( currentAngle + (targetAngle - currentAngle) * pct );
    // lets add a lerp //
    float tLerpAmnt = ofAngleDifferenceDegrees( currentAngle, targetAngle ) * pct * alerp;
    return ofWrapRadians( currentAngle + tLerpAmnt );
    //    return ofWrapRadians( currentAngle + ofAngleDifferenceRadians( currentAngle, targetAngle ) * pct );
    //    return ofWrapRadians(currentAngle + ofWrapRadians( ofWrapRadians(targetAngle)-ofWrapRadians(currentAngle), -PI, PI ) * pct, -PI, PI);
    //    return ofWrapRadians(currentAngle + ofWrapRadians( targetAngle-currentAngle, -PI, PI ) * pct, -PI, PI);
}

//------------------------------------------------------------------
ofVec3f RobotController::getIKRobotTargetForWorldPos( ofVec3f aWorldTarget, bool bRepel ) {
    ofVec3f retTarget = aWorldTarget;
    retTarget.z     = getZValueForIkRobotLocalY( retTarget.y, aWorldTarget.z );
    retTarget.x     = 0.1;
    return retTarget;
}

//------------------------------------------------------------------
vector<double> RobotController::lookAtJoints( float aDeltaTimef ) {
    // figure out the global orientation of the models //
    // look at person //
    
    vector< double > ttargetAngles = targetPose;
    
    ofVec3f ttarget = ofVec3f(1000, 500, 1000);
    
    if( actualArm.nodes.size() >= 4 && robot.getCurrentPose().size() >= 4 ) {
        
        //START THEO
        
        //        facePos = mModelActual->nodes[5].getGlobalPosition();
        //        faceVec = mModelActual->tcpNode.getGlobalPosition() - facePos;
        //        personVec = apos - facePos;
        //
        //        personVec.normalize();
        //        faceVec.normalize();
        //
        //
        //        ofPoint delta = personVec-faceVec;
        //
        //        float angleZ = atan2(delta.y, delta.x);
        ////        targetJointPos[ 4 ] = angleZ;
        //
        //        debugString = " angleZ " + ofToString(angleZ * RAD_TO_DEG) + "\n";
        //END THEO
        
        
        // start Nick //
        // having these values exactly zero can make weird things happen //
        if( ttarget.x == 0 ) {
            ttarget.x = 0.01;
        }
        if( ttarget.y == 0 ) {
            ttarget.y = 0.01;
        }
        if( ttarget.z == 0 ) {
            ttarget.z = 0.01;
        }
        
        // let's figure out the head position //
        ofQuaternion orotate;
        orotate.makeRotate( 0, ofVec3f(0,0,1) );
        
        ofVec3f upAxis( 1,0,0 );
        ofVec3f diff = ttarget - actualArm.nodes[4].getGlobalPosition();
        diff.normalize();
        ofQuaternion tquat;
        ofVec3f txaxis = upAxis * orotate;
        tquat.makeRotate( txaxis, diff );
        
        ofMatrix4x4 tinvParent(ofMatrix4x4::getInverseOf(actualArm.nodes[4].getParent()->getGlobalTransformMatrix()));
        ofMatrix4x4 tm44(ofMatrix4x4(tquat) * tinvParent);
        ofVec3f newYPR = getYawPitchRoll( tm44.getRotate() );
        ttargetAngles[ 4 ] = newYPR.x + PI/2;
        
        orotate.makeRotate( ofVec3f(1,0,0), ofVec3f(0,-1,0) );
        
        
        // if the base is pointing towards a user on one side of the robot and the robot tries to look
        // at a user behind it, the head peeks upside down //
        ofVec3f cBaseLook = -actualArm.nodes[0].getXAxis();
        // this is the direction that the base is pointing, it has no parent, so we should be able to use the axis as is //
        
        
        upAxis.set( 1,0,0 );
        diff = ttarget - actualArm.nodes[3].getGlobalPosition();
        diff.normalize();
        
        
        txaxis = upAxis * orotate;
        tquat.makeRotate( txaxis, diff );
        
        ofMatrix4x4 invParent(ofMatrix4x4::getInverseOf(actualArm.nodes[3].getParent()->getGlobalTransformMatrix()));
        ofMatrix4x4 m44(ofMatrix4x4(tquat) * invParent);
        ofQuaternion localRot = m44.getRotate();
        
        newYPR = getYawPitchRoll( localRot );
        //        targetJointPos[ 3 ] = PI-newYPR.z + PI/2;
        ttargetAngles[ 3 ] = newYPR.z + PI/2;
        
    }
    return ttargetAngles;
    
}

//------------------------------------------------------------------
float RobotController::getZValueForIkRobotLocalY( float aLocalY, float aWorldZ ) {
    float retZ = ikRobotMinZ;
    
    float tstartPctOfWidth  = MIN( mIKRampStartPct, mIKRampEndPct );
    float tendPctOfWidth    = MAX( mIKRampEndPct, mIKRampStartPct );
    float tPctOfHeight      = mIKRampHeightPct;
    
    float twidth    = fabs( ikRobotMaxY - ikRobotMinY);
    float theight   = fabs( ikRobotMaxZ - ikRobotMinZ );
    
    float wpct = ofMap( aLocalY, ikRobotMinY, ikRobotMaxY, 0.f, 1.f, true );
    float thpct = 0.f;
    if( wpct >= tstartPctOfWidth && wpct < tendPctOfWidth ) {
        float tpct = ofMap( wpct, tstartPctOfWidth, tendPctOfWidth, 0.f, 1.f, true );
        thpct = sin( tpct * PI );
        thpct = powf( thpct, 2 );
    }
    
    
    float bottomZ   = ikRobotMinZ + thpct * (tPctOfHeight * theight);
    retZ            = ofClamp(aWorldZ, bottomZ, ikRobotMaxZ );
    
    
    return retZ;
}

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
    tcp_plane.update(robotParams.targetTCP.position*1000, robotParams.targetTCP.rotation);

    targetPoses = urKinematics.inverseKinematics(robotParams.targetTCP);
    int selectedSolution = urKinematics.selectSolution(targetPoses, robot.getCurrentPose(), jointWeights);
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
}

void RobotController::updateIKArm(){
    targetPoses = urKinematics.inverseKinematics(robotParams.targetTCP);
    int selectedSolution = urKinematics.selectSolution(targetPoses, robot.getCurrentPose(), jointWeights);
    if(selectedSolution > -1){
        targetPose = targetPoses[selectedSolution];
        for(int i = 0; i < targetPose.size(); i++){
            float tpose = (float)targetPose[i];
            if( isnan(tpose) ) {
                tpose = 0.f;
            }
            robotParams.ikPose[i] = tpose;
        }
        
        for(int i = 0; i < targetPoses.size(); i++)
        {
            previewArms[i]->setPose(targetPoses[i]);
        }
    }
    for(int i = 0; i < targetPose.size(); i++){
        float tpose = (float)targetPose[i];
        if( isnan(tpose) ) {
            tpose = 0.f;
        }
        robotParams.targetPose[i] = ofRadToDeg(tpose);
    }
    targetPose = getArmIK(1.0/60.0f);
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
    vector< double > lookAtAngles = lookAtJoints(1.0/60.0f);
    // determine if these angles should be added or not //
    for( int i = 0; i < targetPose.size(); i++ ) {
        targetPose[i] = lookAtAngles[i];
    }
    
    previewArm.setPose(targetPose);
}

#pragma mark - Update
void RobotController::update(){
    updateRobotData();
    if(robotParams.bUseIKFast){
        updateIKFast();
    }else if(robotParams.bUseIKArm){
        updateIKArm();
    }
    
//    safetyCheck();
    updateMovement();
      targetPose = movement.getTargetJointPose();
    for(int i = 0; i < targetPose.size(); i++){
        float tpose = (float)targetPose[i];
        if( isnan(tpose) ) {
            tpose = 0.f;
        }
        robotParams.targetPose[i] = ofRadToDeg(tpose);
    }
    previewArm.setPose(targetPose);
    cout << "target pose:\t[";
    for (auto p : targetPose){
        cout << p << ", ";
    }
    cout << "]" << endl;
}
void RobotController::update(vector<double> _pose){
    targetPose = _pose;
    update();
    
}

void RobotController::set_desired(ofNode target){
    // convert from mm to m
    robotParams.targetTCP.position = target.getGlobalPosition()/1000.0;
    robotParams.targetTCP.rotation = target.getGlobalOrientation();
}

void RobotController::set_live(bool val){
    robotParams.bMove.set(val);
}

#pragma mark - Safety
void RobotController::safetyCheck(){

    
    robotSafety.setCurrentRobotArmAnlges(robot.getCurrentPose());
    robotSafety.setDesiredAngles(targetPose);
    robotSafety.update(previewArm);
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
    actualArm.setPose(robotParams.currentPose);
    robotParams.actualTCP = robot.getToolPose();
    robotParams.tcpPosition = robotParams.actualTCP.position;
    ofQuaternion tcpO = robotParams.actualTCP.rotation;
    robotParams.tcpOrientation = ofVec4f(tcpO.x(), tcpO.y(), tcpO.z(), tcpO.w());
    // update GUI params
    for(int i = 0; i < robotParams.currentPose.size(); i++){
        robotParams.pCurrentPose[i] = ofRadToDeg((float)robotParams.currentPose[i]);
    }
    ofMatrix4x4 forwardIK = urKinematics.forwardKinematics(robotParams.currentPose);
    
    movement.setCurrentJointPose(robotParams.currentPose);
}

#pragma mark - drawing
void RobotController::close(){
    if(robot.isThreadRunning()){
        robot.stopThread();
    }
}

void RobotController::draw(ofFloatColor color, bool debug){
    actualArm.draw(color, debug);
}

//void RobotController::drawPreviews(){
//    for(int i = 0; i < previewArms.size(); i++){
//        ofSetColor(255/(i+1), 0, 255/(i+1));
//        previewArms[i]->draw(false);
//    }
//}

void RobotController::drawSafety(ofCamera & cam){
    robotSafety.draw();
}

void RobotController::drawIK(){
    if(mIKArm) mIKArm->draw();
    if(mIKArmInverted)mIKArmInverted->draw();
}

void RobotController::drawPreview(ofFloatColor color){
    previewArm.draw(color, false);
//    tcp_plane.draw();
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






