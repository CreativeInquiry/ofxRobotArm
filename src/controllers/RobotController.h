
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
#pragma once
#include "ofMain.h"
#include "URDriver.h"
#include "RobotParameters.h"
#include "InverseKinematics.h"
#include "RobotModel.h"
#include "ofxIKArm.h"
#include "RobotArmSafety.h"
#include "Utils.h"
#include "RobotConstants.hpp"
#include "Plane.h"
#include "Move.h"
namespace ofxRobotArm {
    
    class RobotController {
    public:
        RobotController();
        ~RobotController();
        
        /// \brief creates and connects to a new robot using a default IP Address
        /// \params params default parameters for the robot & GUI
        void setup(RobotParameters & params);
        
        void setup(string ipAddress, RobotType type);
        void start();
        
        /// \brief creates and connects to a new robot
        /// \params ipAddress ipAddress of the robot
        /// \params params default parameters for the robot & GUI
        void setup(string ipAddress, RobotParameters & params, bool offline);
        vector< double > updateJoints(float deltatime);


        void toggleTeachMode();
        void setTeachMode();
        bool isTeachModeEnabled;
        
        
        void safetyCheck();
        void updateMovement();
        void updateRobotData();
        
        void update();
        void update(vector<double> pose);
        void updateIKFast();
        void updateIKArm();
        
        void set_desired(ofNode target);
        Plane tcp_plane;
        
        void moveArm();
        void draw(ofFloatColor color = ofFloatColor(1,1,1,1), bool debug = false);
        void drawDesired(ofFloatColor color = ofFloatColor(1,1,1,1));
//        void drawPreviews();
        void drawIK();
        void drawSafety(ofCamera & cam);
        
   
        
        void enableControlJointsExternally();
        void disableControlJointsExternally();
        bool areJointsControlledExternally();
        
        void close();
        vector<double> getCurrentPose();
        ofxURDriver robot;
        Move movement;
//        RobotParameters * robotParams;
        RobotParameters robotParams;
        RobotModel desiredPose;
        vector<RobotModel*> desiredPoses;
        ofNode forwardNode;
        RobotModel actualPose;
        InverseKinematics inverseKinematics;
        int stopCount = 0;

        RobotArmSafety robotSafety;
        
        void setEndEffector(string filename);

    protected:
        vector <double> stopPosition;
        bool m_bSettingJointsExternally = false;
        vector<double> targetPose;
        vector<vector<double> > targetPoses;
        
        // smooth angles //
        vector< float > mSmoothAdditions;
        vector<double> jointWeights;

    private:
        void setup_parameters();
        
        RobotType type;
    };
}




