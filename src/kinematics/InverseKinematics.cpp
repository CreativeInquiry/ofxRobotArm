#include "InverseKinematics.h"
//
// Copyright (c) 2016, 2021 Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
////
using namespace ofxRobotArm;



InverseKinematics::InverseKinematics(){

}

InverseKinematics::~InverseKinematics(){

}

void InverseKinematics::setup(RobotParameters * params, bool useRelaxedIK){
    robotParams = params;
    this->params.setName("IKArm Commands");
    this->params.add( bControlIkWithMouse.set("ControlIkWithMouse", false ));
    this->params.add( bOnlyUseInverseIk.set("OnlyUseInverseIK", true ));
    
    this->params.add( ikRobotMinY.set( "IkRobotMinY", -725, -2000, 2000 ));
    this->params.add( ikRobotMaxY.set( "IkRobotMaxY", 0, -2000, 2000 ));
    
    this->params.add( ikRobotMinZ.set( "IkRobotMinZ", 300, -500, 3000 ));
    this->params.add( ikRobotMaxZ.set( "IkRobotMaxZ", 700, -500, 3000 ));
    
    this->params.add( mIKRampStartPct.set("IKRampStartPct", 0.3, 0.0, 1.0 ));
    this->params.add( mIKRampEndPct.set("IKRampEndPct", 1.5, 1.0, 2.0 ));
    this->params.add( mIKRampHeightPct.set("IKRampHeightPct", 0.3, 0.0, 1.0 ));
    this->params.add( bUseRelaxedIK.set("Use RelaxedIK ", useRelaxedIK));
    robotParams->jointsIK.add(this->params);

    setRobotType(robotParams->getRobotType());
}


void InverseKinematics::setRobotType(ofxRobotArm::RobotType type){
    this->type = type;
    kinematics.setType(this->type);
    if(this->type == UR3 || this->type == UR5 || this->type  == UR10){
        relaxedIK.setMatrix(ofVec3f(-1, 0, 0), ofVec3f(0, -1, 0), ofVec3f(0, 0, 1));
    }else{
        relaxedIK.setMatrix(ofVec3f(1, 0, 0), ofVec3f(0, 1, 0), ofVec3f(0, 0, 1));
    }
}
/// \brief Converts a 4x4 matrix to a 1D array
/// \param input ofMatrix4x4 to convert
/// \return row-major array in UR World Cords
double* toIK(ofMatrix4x4 input){
    double* T = new double[16];
    for(int i = 0; i < 4; i++){
        T[i] = (double)input._mat[i][0];
        T[i+(4)] = (double)input._mat[i][1];
        T[i+(8)] = (double)input._mat[i][2];
        T[i+(12)] = (double)input._mat[i][3];
    }
    return T;
}

ofMatrix4x4 toOF(double * T){
    ofMatrix4x4 output;
    for(int i = 0; i < 4; i++){
        output._mat[i][0] = T[i];
        output._mat[i][1] = T[i+(4)];
        output._mat[i][2] = T[i+(8)];
        output._mat[i][3] = T[i+(12)];
    }
    return output;
}

int argMin(std::vector<double> vec)
{
    std::vector<double>::iterator mins = std::min_element(vec.begin(), vec.end()); //returns all mins
    double min = mins[0]; //select the zeroth min if multiple mins exist
    for(int i=0; i < vec.size(); i++)
    {
        //Note: could use fabs( (min - vec[i]) < 0.01) if worried about floating-point precision
        if(vec[i] == min)
            return i;
    }
    return -1;
}


int InverseKinematics::selectSolution(vector<vector<double> > & inversePosition, vector<double> currentQ, vector<double> weight)
{
//    int selectedSolution = 0;
//    if(type == ofxRobotArm::UR3 || type == ofxRobotArm::UR5 || type == ofxRobotArm::UR10){
//        for(int i = 0; i < inversePosition.size(); i++){
//            for(int j = 0; j < inversePosition[i].size(); j++){
////                if(j == 1 || j == 3){
////                    if(inversePosition[i][j] > PI){
////                        inversePosition[i][j]  = ofMap(inversePosition[i][j], PI, TWO_PI, -PI, 0, true);
////                    }
////                }
//            }
//        }
//    }
//
//    for(int i = 0; i < inversePosition.size(); i++){
//        for(int j = 0; j < inversePosition[i].size(); j++){
//            if(preInversePosition.size() > 0){
//                if(i == selectedSolution){
//                    if(preInversePosition[i][j]-inversePosition[i][j] >= TWO_PI){
//                        ofLog(OF_LOG_WARNING)<<"JOINT WRAPS SOL "<<ofToString(i)<<" Joint "<<ofToString(j)<<endl;
//                    }
//                }
//            }
//        }
//    }
//
//    vector<double> test_sol;
//    vector<vector<double> > valid_sols;
//    test_sol.assign(6, 9999.);
//    vector<double> addAngle = {-1*TWO_PI, 0, TWO_PI};
//    for(int i = 0; i < inversePosition.size(); i++){
//        for(int j = 0; j < inversePosition[i].size(); j++){
//            for(int k = 0; k < addAngle.size(); k++){
//                float test_ang = inversePosition[i][j]+addAngle[k];
//                if(fabs(test_ang - currentQ[j])  < fabs(test_sol[j] -  currentQ[j]) && abs(test_ang) <= TWO_PI){
//                    test_sol[j] = test_ang;
//                }
//            }
//        }
//        bool testValid = false;
//        for(int l = 0; l < test_sol.size(); l++){
//            if(test_sol[l] != 9999){
//                testValid = true;
//            }else{
//                testValid = false;
//            }
//        }
//        if(testValid){
//            valid_sols.push_back(test_sol);
//
//        }
//    }
//
//    vector<double> sumsValid;
//    sumsValid.assign(valid_sols.size(), 0);
//    for(int i = 0; i < valid_sols.size(); i++){
//        for(int j = 0; j < valid_sols[i].size(); j++){
//            sumsValid[i] = pow(weight[j]*(valid_sols[i][j] - currentQ[j]), 2);
//        }
//    }
    
    preInversePosition = inversePosition;
    
    
//    inversePosition = valid_sols;
    if(inversePosition.size() > 0){
        return 0;
    }else{
        return -1;
    }
}


vector<vector<double> > InverseKinematics::inverseKinematics(double o, double t, double th, double f, double fi, double s)
{
    double q[6] = {o, t, th, f, fi, s};
    double* T = new double[16];
    double q_sols[8*6];
    int num_sols = kinematics.inverseHK(T, q_sols);
    vector<vector<double> > sols;
    for(int i=0;i<num_sols;i++){
        vector<double> fooSol;
        fooSol.push_back(q_sols[i*6]);
        fooSol.push_back(q_sols[i*6+1]);
        fooSol.push_back(q_sols[i*6+2]);
        fooSol.push_back(q_sols[i*6+3]);
        fooSol.push_back(q_sols[i*6+4]);
        fooSol.push_back(q_sols[i*6+5]);
        sols.push_back(fooSol);
    }
    return sols;
}


vector<vector<double> > InverseKinematics::inverseKinematics(vector<double> input)
{
    if(input.size() == 6){
        return inverseKinematics(input[0], input[1], input[2], input[3], input[4], input[5]);
    }
    return vector<vector<double>>();
}

vector<vector<double> > InverseKinematics::inverseKinematics(ofxRobotArm::Pose targetPose, ofxRobotArm::Pose currentPose){
    
    if(bUseRelaxedIK){
        vector<double> sol = inverseRelaxed(targetPose, currentPose);
        vector<vector<double> > sols;
        sols.push_back(sol);
        return sols;
    }else{
    
        ofMatrix4x4 matPose;
        ofMatrix4x4 matT;
        ofMatrix4x4 matR;
        matPose.makeIdentityMatrix();
        matT.makeTranslationMatrix(targetPose.position);
        matR.makeRotationMatrix(targetPose.orientation);
        matPose = matR*matT;
        
        return inverseKinematics(matPose);
    }
}


vector<double> InverseKinematics::inverseRelaxed(Pose targetPose, Pose currentPose){
    if(!relaxedIK.isThreadRunning()){
        relaxedIK.start();
    }
    relaxedIK.setPose(targetPose, currentPose);
    return relaxedIK.getCurrentPose();
}

vector<vector<double> > InverseKinematics::inverseKinematics(ofMatrix4x4 pose)
{
    double q_sols[8*6];
    vector<vector<double> > sols;
    if(type == UR3 || type == UR5 || type == UR10){
        double* T = new double[16];
        T = toIK(pose);
        int num_sols = kinematics.inverseHK(T, q_sols);
        for(int i=0;i<num_sols;i++){
            vector<double> fooSol;
            fooSol.push_back(q_sols[i*6]);
            fooSol.push_back(q_sols[i*6+1]);
            fooSol.push_back(q_sols[i*6+2]);
            fooSol.push_back(q_sols[i*6+3]);
            fooSol.push_back(q_sols[i*6+4]);
            fooSol.push_back(q_sols[i*6+5]);
            if(kinematics.isValid(&fooSol[0])){
                kinematics.harmonizeTowardZero(&fooSol[0]);
                sols.push_back(fooSol);
            }
        }
    }
    if(type == IRB120){
        kinematics.inverseSW(pose, q_sols);
        for(int i=0;i<8;i++){
            vector<double> fooSol;
            fooSol.push_back(q_sols[i*6]);
            fooSol.push_back(q_sols[i*6+1]);
            fooSol.push_back(q_sols[i*6+2]);
            fooSol.push_back(q_sols[i*6+3]);
            fooSol.push_back(q_sols[i*6+4]);
            fooSol.push_back(q_sols[i*6+5]);
            if(kinematics.isValid(&fooSol[0])){
                kinematics.harmonizeTowardZero(&fooSol[0]);
                sols.push_back(fooSol);
            }
        }
    }
    return sols;
}


ofMatrix4x4 InverseKinematics::forwardKinematics(vector<double> pose)
{
    if(type == UR3 || type == UR5 || type == UR10){
        return toOF(forwardKinematics(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]));
    }
    if(type == IRB120){
        ofMatrix4x4 mat;
        kinematics.forwardSW(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5], mat);        
        return mat;
    }
    return ofMatrix4x4();
}


void InverseKinematics::setRelaxedPose(vector<double> pose){
    relaxedIK.setInitialPose(pose);
}



double* InverseKinematics::forwardKinematics(double o, double t, double th, double f, double fi, double s)
{
    
    double q[6] = {o, t, th, f, fi, s};
    double* T1 = new double[16];
    double* T2 = new double[16];
    double* T3 = new double[16];
    double* T4 = new double[16];
    double* T5 = new double[16];
    double* T6 = new double[16];
    
    kinematics.forward_allHK(q, T1, T2, T2, T4, T5, T6);
    return T6;
}


vector< double > InverseKinematics::getArmIK(RobotModel * actualPose, Pose targetTCP,  vector<double> targetPose, float aDeltaTimef ) {
    vector <double> armFrom = getArmIK(actualPose, targetPose, targetTCP.position*1000.0f, ofVec3f(), false, aDeltaTimef );
    return armFrom;
}


//------------------------------------------------------------------
vector< double > InverseKinematics::getArmIK(RobotModel * actualPose, vector<double> targetPose, ofVec3f aTargetWorldPos, ofVec3f aElbowWorldPos, bool aBInvertElbow, float aDeltaTimef ) {
    // ok, now let's address the ik //
    if( !mIKArm ) {
        mIKArm = shared_ptr< ofxIKArm >( new ofxIKArm() );
        
    }
    if( !mIKArmInverted ) {
        mIKArmInverted = shared_ptr< ofxIKArm >( new ofxIKArm() );
    }
    if( mIKArm->getArmLength() <= 0.0f ) {
        mIKArm->setDrawSize( 10 );
        mIKArm->setup(actualPose->nodes[1].getGlobalPosition(), actualPose->nodes[2].getGlobalPosition(), actualPose->nodes[3].getGlobalPosition() );
        mIKArm->setDrawSize( 10 );
        
        // disable the elbow target and set to constrain along axis //
        mIKArm->setShoulderUpVectorEnabled( true );
        mIKArm->setShoulderUpVector( ofVec3f(0,-1,0));
        mIKArm->setInverted( true );
        
        ofLog(OF_LOG_VERBOSE) << "Setting up the ik arm " << mIKArm->getArmLength() << " shoulder: " << mIKArm->getUpperArmLength() << " forearm len: " << mIKArm->getLowerArmLength() << " | " << ofGetFrameNum() << endl;
    }
    if( mIKArmInverted->getArmLength() <= 0.0f ) {
        mIKArmInverted->setup( actualPose->nodes[1].getGlobalPosition(), actualPose->nodes[2].getGlobalPosition(), actualPose->nodes[3].getGlobalPosition() );
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
ofVec3f InverseKinematics::getYawPitchRoll( ofQuaternion aquat ) {
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
float InverseKinematics::getNeckAngleAlignedWithVector(RobotModel * actualPose, ofVec3f avec ) {
    // let's figure out the head position //
    ofQuaternion orotate;
    orotate.makeRotate( ofVec3f(1,0,0), ofVec3f(0,-1,0) );
    
    ofVec3f upAxis( 1,0,0 );
    ofVec3f diff = avec.getNormalized();//ttarget - actualPose->nodes[3].getGlobalPosition();
    ofQuaternion tquat;
    ofVec3f txaxis = upAxis * orotate;
    tquat.makeRotate( txaxis, diff );
    
    ofMatrix4x4 invParent(ofMatrix4x4::getInverseOf(actualPose->nodes[3].getParent()->getGlobalTransformMatrix()));
    ofMatrix4x4 m44(ofMatrix4x4(tquat) * invParent);
    ofQuaternion localRot = m44.getRotate();
    
    ofVec3f newYPR = getYawPitchRoll( localRot );
    //        targetPose[ 3 ] = PI-newYPR.z + PI/2;
    //    ttargetAngles[ 3 ] = newYPR.z + PI/2;
    return newYPR.z;// + PI/2.f;
}

//------------------------------------------------------------------
ofVec3f InverseKinematics::lerp( ofVec3f aStartVec, ofVec3f aEndVec, float aLerpAmnt ) {
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
float InverseKinematics::lerpRadians(float currentAngle, float targetAngle, float pct, float alerp ) {
    //    return ofWrapRadians( currentAngle + (targetAngle - currentAngle) * pct );
    // lets add a lerp //
    float tLerpAmnt = ofAngleDifferenceDegrees( currentAngle, targetAngle ) * pct * alerp;
    return ofWrapRadians( currentAngle + tLerpAmnt );
    //    return ofWrapRadians( currentAngle + ofAngleDifferenceRadians( currentAngle, targetAngle ) * pct );
    //    return ofWrapRadians(currentAngle + ofWrapRadians( ofWrapRadians(targetAngle)-ofWrapRadians(currentAngle), -PI, PI ) * pct, -PI, PI);
    //    return ofWrapRadians(currentAngle + ofWrapRadians( targetAngle-currentAngle, -PI, PI ) * pct, -PI, PI);
}

//------------------------------------------------------------------
ofVec3f InverseKinematics::getIKRobotTargetForWorldPos( ofVec3f aWorldTarget, bool bRepel ) {
    ofVec3f retTarget = aWorldTarget;
    retTarget.z     = getZValueForIkRobotLocalY( retTarget.y, aWorldTarget.z );
    retTarget.x     = 0.1;
    return retTarget;
}

//------------------------------------------------------------------
vector<double> InverseKinematics::lookAtJoints(RobotModel * actualPose,  vector<double> targetPose, float aDeltaTimef, ofVec3f targetPos ) {
    // figure out the global orientation of the models //
    // look at person //
    
    vector< double > ttargetAngles = targetPose;
    
    ofVec3f ttarget = targetPos;
    
    if( actualPose->nodes.size() >= 4 && targetPose.size() >= 4 ) {
        
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
        ofVec3f diff = ttarget - actualPose->nodes[4].getGlobalPosition();
        diff.normalize();
        ofQuaternion tquat;
        ofVec3f txaxis = upAxis * orotate;
        tquat.makeRotate( txaxis, diff );
        
        ofMatrix4x4 tinvParent(ofMatrix4x4::getInverseOf(actualPose->nodes[4].getParent()->getGlobalTransformMatrix()));
        ofMatrix4x4 tm44(ofMatrix4x4(tquat) * tinvParent);
        ofVec3f newYPR = getYawPitchRoll( tm44.getRotate() );
        ttargetAngles[ 4 ] = newYPR.x + PI/2;
        
        orotate.makeRotate( ofVec3f(1,0,0), ofVec3f(0,-1,0) );
        
        
        // if the base is pointing towards a user on one side of the robot and the robot tries to look
        // at a user behind it, the head peeks upside down //
        ofVec3f cBaseLook = -actualPose->nodes[0].getXAxis();
        // this is the direction that the base is pointing, it has no parent, so we should be able to use the axis as is //
        
        
        upAxis.set( 1,0,0 );
        diff = ttarget - actualPose->nodes[3].getGlobalPosition();
        diff.normalize();
        
        
        txaxis = upAxis * orotate;
        tquat.makeRotate( txaxis, diff );
        
        ofMatrix4x4 invParent(ofMatrix4x4::getInverseOf(actualPose->nodes[3].getParent()->getGlobalTransformMatrix()));
        ofMatrix4x4 m44(ofMatrix4x4(tquat) * invParent);
        ofQuaternion localRot = m44.getRotate();
        
        newYPR = getYawPitchRoll( localRot );
        //        targetJointPos[ 3 ] = PI-newYPR.z + PI/2;
        ttargetAngles[ 3 ] = newYPR.z + PI/2;
        
    }
    return ttargetAngles;
    
}

//------------------------------------------------------------------
float InverseKinematics::getZValueForIkRobotLocalY( float aLocalY, float aWorldZ ) {
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


void InverseKinematics::draw(){
    if(mIKArm) mIKArm->draw();
    if(mIKArmInverted)mIKArmInverted->draw();
}
