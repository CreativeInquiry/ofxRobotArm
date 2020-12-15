//
//  Move.h
//  ofxURDriver
//
//  Created by Dan Moore on 2/20/16.
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//


#pragma once
#include "ofMain.h"
//#include "UR5KinematicModel.h"
//#include "UR10KinematicModel.h"
#include "ofxTiming.h"
#include "Synchronized.h"
#include "Joint.h"
namespace ofxRobotArm{
    class Move {
        public:
            Move();
            ~Move();
            void setup();
            void update();
            void computeVelocities();
            void updatePathDebug();
            
            void addTargetPoint(Joint target);
            
            float getAcceleration();
            
            void updateFromPose();
            
            ofParameterGroup movementParams;
            vector<double> getTargetJointPose();
            vector<double> getCurrentJointVelocities();

            
            void setCurrentJointPose(vector<double> & pose);
            void addTargetJointPose(vector<double> _target);

           
            
            
        //protected:
            float distance;
            float targetLength;
            ofParameter<float> speedDivider;
            ofParameter<float> jointSpeedLerpSpeed;
            ofParameter<float> jointAccelerationMultipler;
            ofParameter<float> maxAcceleration;
            ofParameter<float> maxSpeed;
            ofParameter<float> minSpeed;
            ofParameter<float> deltaTime;
            
         
            ofParameter<bool> reachPoint;
            ofParameter<float> targetTCPLerpSpeed;
            ofParameter<float> avgAccel;
            
            
            Joint targetPoint;

            float deltaT;
            RateTimer deltaTimer;
            
            vector<double> currentPose;
            vector<double> targetPose;
            vector<double> jointAccelerations;
            vector<double> currentJointVelocity;
            vector<double> previousJointVelocity;
            vector<double> targetJointPose;

            double maxAccel;
            float epslion;
            
            int selectedSolution;
            
            bool m_bApplyKinematics = true;
            

    };
}
