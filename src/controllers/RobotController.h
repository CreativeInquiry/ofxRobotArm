
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
#pragma once
#include "ofMain.h"
#include "ofxURDriver.h"
#include "RobotParameters.h"
#include "URIKFast.h"
#include "UR5KinematicModel.h"
#include "ofxIKArm.h"

namespace ofxRobotArm {
    class RobotController{
    public:
        RobotController();
        ~RobotController();
        
        /// \brief creates and connects to a new robot using a default IP Address
        /// \params params default parameters for the robot & GUI
        void setup(RobotParameters & params);
        
        /// \brief creates and connects to a new robot
        /// \params ipAddress ipAddress of the robot
        /// \params params default parameters for the robot & GUI
        void setup(string ipAddress, RobotParameters & params);
        
        void updateMovement();
        void updateRobotData();
        void update();
        void update(vector<double> pose);
        void moveArm();
        void draw(bool bDrawDebug=true);
        void drawPreview(bool bDrawDebug=true);
        
        void enableControlJointsExternally();
        void disableControlJointsExternally();
        bool areJointsControlledExternally();
        
        void close();
        vector<double> getCurrentPose();
        ofxURDriver robot;
        URMove movement;
        RobotParameters * robotParams;
        UR5KinematicModel previewArm;
        UR5KinematicModel actualArm;
        URIKFast urKinematics;
        int stopCount = 0;
        ofxIKArm mIKArm;
    protected:
        vector <double> stopPosition;
        bool m_bSettingJointsExternally = false;
        vector<double> targetPose;
        vector<vector<double> > targetPoses;
    };
}




