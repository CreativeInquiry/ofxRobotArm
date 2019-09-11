
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
#pragma once
#include "ofMain.h"
#include "ofxURDriver.h"
#include "RobotParameters.h"
#include "URIKFast.h"
#include "UR5KinematicModel.h"
//#include "ofxIKArm.h"
#include "RobotArmSafety.h"

namespace ofxRobotArm {
    

    class UR5Controller {
    public:
        UR5Controller();
        ~UR5Controller();
        
        /// \brief creates and connects to a new robot using a default IP Address
        /// \params params default parameters for the robot & GUI
        void setup(RobotParameters & params);
        
        /// \brief creates and connects to a new robot
        /// \params ipAddress ipAddress of the robot
        /// \params params default parameters for the robot & GUI
        void setup(string ipAddress, RobotParameters & params, bool offline);
        vector< double > updateJoints(float deltatime);
        vector< double > lookAtJoints( float aDeltaTimef );
        vector< double > getArmIK( float aDeltaTimef );
        vector< double > getArmIK( ofVec3f aTargetWorldPos, ofVec3f aElbowWorldPos, bool aBInvertElbow, float aDeltaTimef );

        
        bool shouldApplyIk();
        ofVec3f getYawPitchRoll( ofQuaternion aquat );
        float getNeckAngleAlignedWithVector( ofVec3f avec );
        
        ofVec3f lerp( ofVec3f aStartVec, ofVec3f aEndVec, float aLerpAmnt );
        float lerpRadians(float currentAngle, float targetAngle, float pct, float alerp );
        
        ofVec3f getIKRobotTargetForWorldPos( ofVec3f aWorldTarget, bool bRepel );
        float getZValueForIkRobotLocalY( float aLocalY, float aWorldZ );
        
        void safetyCheck();
        void updateMovement();
        void updateRobotData();
        void update();
        void update(vector<double> pose);
        void moveArm();
        void draw(bool debug = false);
        void drawPreview();
        void drawPreviews();
        void drawIK();
        void drawSafety(ofCamera & cam);
        
        void setPose(vector<double> pose);
        
        void enableControlJointsExternally();
        void disableControlJointsExternally();
        bool areJointsControlledExternally();
        
        void close();
        vector<double> getCurrentPose();
        ofxURDriver robot;
        URMove movement;
        RobotParameters * robotParams;
        UR5KinematicModel * previewArm;
        vector<UR5KinematicModel*> previewArms;
        
        UR5KinematicModel actualArm;
        URIKFast urKinematics;
        int stopCount = 0;
        shared_ptr< ofxIKArm > mIKArm;
        shared_ptr< ofxIKArm > mIKArmInverted;
        RobotArmSafety robotSafety;

    protected:
        vector <double> stopPosition;
        bool m_bSettingJointsExternally = false;
        vector<double> targetPose;
        vector<vector<double> > targetPoses;
        
        // smooth angles //
        vector< float > mSmoothAdditions;
        vector<double> jointWeights;
        ofParameterGroup params;
        ofParameter< float > mIKRampStartPct, mIKRampEndPct;
        ofParameter< float > mIKRampHeightPct;
        
        ofParameter< float > ikRobotMinY, ikRobotMaxY;
        ofParameter< float > ikRobotMinZ, ikRobotMaxZ;
        
        
        ofParameter< bool > bControlIkWithMouse;
        ofParameter< bool > bOnlyUseInverseIk;
    };
}




