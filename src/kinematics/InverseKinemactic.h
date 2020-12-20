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
    class InverseKinemactic{
        public:
            InverseKinemactic();
            InverseKinemactic(ofxRobotArm::RobotType type);
            ~InverseKinemactic();
            void setRobotType(ofxRobotArm::RobotType type);
            int selectSolution(vector<vector<double> > & inversePosition, vector<double> currentQ, vector<double> weight);
            ofMatrix4x4 forwardKinematics(vector<double> pose);
            double* forwardKinematics(double o, double t, double th, double f, double fi, double s);
            vector<vector<double> > inverseKinematics(ofxRobotArm::Pose pose);
            vector<vector<double> > inverseKinematics(ofMatrix4x4 pose);
            vector<vector<double> > inverseKinematics(vector<double> input);
            vector<vector<double> > inverseKinematics(double o, double t, double th, double f, double fi, double s);
            ofxRobotArm::RobotType type;
            vector<double> currentPosition;
            Kinematics kinematics;
            vector<vector<double> > preInversePosition;
    };
}
