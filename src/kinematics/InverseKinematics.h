#pragma once
#include "ofMain.h"
#include "kin.h"
//#include "UR5KinematicModel.h"
//#include "UR10KinematicModel.h"
//#include "RobotModel.h"
#include "Pose.h"
#include "ofxIKArm.h"
#include "Utils.h"
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
namespace ofxRobotArm {
    class InverseKinematics{
        public:
        InverseKinematics();
        InverseKinematics(ofxRobotArm::RobotType type);
            ~InverseKinematics();
            void setup(
            void setRobotType(ofxRobotArm::RobotType type);
            int selectSolution(vector<vector<double> > & inversePosition, vector<double> currentQ, vector<double> weight);
            ofMatrix4x4 forwardKinematics(vector<double> pose);
            double* forwardKinematics(double o, double t, double th, double f, double fi, double s);
            vector<vector<double> > inverseKinematics(ofxRobotArm::Pose pose);
            vector<vector<double> > inverseKinematics(ofMatrix4x4 pose);
            vector<vector<double> > inverseKinematics(vector<double> input);
            vector<vector<double> > inverseKinematics(double o, double t, double th, double f, double fi, double s);
            ofxRobotArm::RobotType type;
            Kinematics kinematics;
            vector<vector<double> > preInversePosition;
        
        
        vector< double > lookAtJoints(RobotModel * actualPose,  vector<double> targetPose, float aDeltaTimef );
        vector< double > getArmIK(  Pose targetTCP, float aDeltaTimef);
        vector< double > getArmIK( ofVec3f aTargetWorldPos, ofVec3f aElbowWorldPos, bool aBInvertElbow, float aDeltaTimef );
        
        ofVec3f getYawPitchRoll( ofQuaternion aquat );
        float getNeckAngleAlignedWithVector( ofVec3f avec );
        
        ofVec3f lerp( ofVec3f aStartVec, ofVec3f aEndVec, float aLerpAmnt );
        float lerpRadians(float currentAngle, float targetAngle, float pct, float alerp );
        
        ofVec3f getIKRobotTargetForWorldPos( ofVec3f aWorldTarget, bool bRepel );
        float getZValueForIkRobotLocalY( float aLocalY, float aWorldZ );
        
        ofParameterGroup params;
        ofParameter< float > mIKRampStartPct, mIKRampEndPct;
        ofParameter< float > mIKRampHeightPct;
        
        ofParameter< float > ikRobotMinY, ikRobotMaxY;
        ofParameter< float > ikRobotMinZ, ikRobotMaxZ;
        
        
        ofParameter< bool > bControlIkWithMouse;
        ofParameter< bool > bOnlyUseInverseIk;
        
        shared_ptr< ofxIKArm > mIKArm;
        shared_ptr< ofxIKArm > mIKArmInverted;
    };
}

