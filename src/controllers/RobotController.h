
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
#pragma once
#include "ofMain.h"
#include "RobotDriver.h"
#include "URDriver.h"
#include "ABBDriver.h"
#include "RobotParameters.h"
#include "InverseKinematics.h"
#include "RobotModel.h"
#include "ofxIKArm.h"
#include "RobotArmSafety.h"
#include "Utils.h"
#include "RobotConstants.hpp"
#include "Plane.h"
#include "RobotConstants.hpp"
namespace ofxRobotArm {
    
    class RobotController {
    public:
        RobotController();
        ~RobotController();
        
        /// \brief creates and connects to a new robot using a default IP Address
        /// \params params default parameters for the robot & GUI
        void setup(RobotParameters & params);
        void setup(string ipAddress, RobotParameters & params, bool offline);
        
        void start();
        
        /// \brief creates and connects to a new robot
        /// \params ipAddress ipAddress of the robot
        /// \params params default parameters for the robot & GUI

        void updateJoints(float deltatime);
        void toggleTeachMode();
        void setTeachMode();
        void safetyCheck();
        void updateMovement();
        void updateRobotData();
        
        void update();
        void update(vector<double> pose);
        void updateIK(Pose pose);
//        void updateIKArm();
        
        void setDesired(ofNode target);
      
        
        void draw(ofFloatColor color = ofFloatColor(1,1,1,1), bool debug = false);
        void drawDesired(ofFloatColor color = ofFloatColor(1,1,1,1));
//        void drawPreviews();
        void drawIK();
        void drawSafety(ofCamera & cam);
        

        bool arePoseControlledExternally();
        
        void close();
        vector<double> getCurrentPose();
        RobotDriver * robot;
//        RobotParameters * robotParams;
        RobotParameters robotParams;
        RobotModel desiredPose;
        vector<RobotModel*> desiredPoses;
        ofNode forwardNode;
        RobotModel actualPose;
        InverseKinematics inverseKinematics;
        int stopCount = 0;

        RobotArmSafety robotSafety;
        
        void setToolOffset(ofVec3f local);
        
        void setEndEffector(string filename);

        
    protected:
        vector <double> stopPosition;
        vector<double> targetPose;
        vector<vector<double> > targetPoses;
        
        // smooth angles //
        vector< float > mSmoothAdditions;
        vector<double> jointWeights;
    private:
        
        bool isTeachModeEnabled;
        void setup_parameters();
        ofNode target;
        Plane tcp_plane;
    };
}




