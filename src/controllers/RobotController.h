
//
// Copyright (c) 2016, 2021 Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
////
#pragma once
#include "ofMain.h"
#include "RobotDriver.h"
#include "URDriver.h"
#include "ABBDriver.h"
#include "XARMDriver.h"
#include "PandaDriver.h"
#include "InverseKinematics.h"
#include "URDFModel.h"
#include "RobotConstants.hpp"
#include "Plane.h"

namespace ofxRobotArm {
    
    class RobotController {
    public:
        RobotController();
        ~RobotController();
        
        /// \brief creates and connects to a new robot using a default IP Address
        /// \params params default parameters for the robot & GUI
        void setup(string ipAddress, int port, string urdfPath, RobotType robotType, IKType ikType, bool offline);
        void createRobot(RobotType type);
        void loadModels(string urdfpath);
        void connectRobot(bool offline);
        void setupParams();
        
        void initKinematics(ofxRobotArm::IKType type);
        
        void setNthJoint(double rotation);
        void setRobotOrigin(ofVec3f origin, ofQuaternion orientation);
        void setHomePose(vector<double> pose);
        void setIKType(ofxRobotArm::IKType ikType);
        
        void start();

        /// \brief creates and connects to a new robot
        /// \params ipAddress ipAddress of the robot
        /// \params params default parameters for the robot & GUI

        void updateJoints(double deltatime);
        void toggleTeachMode();
        void setTeachMode(bool teachMode);
        void updateMovement();
        void updateRobotData();
        void update();
        void update(vector<double> pose);
        void updateIK(Pose pose);
        
        void setDesiredPose(ofNode target);
        void drawActual(ofColor color = ofColor(255,255,255,255), bool debug = false);
        void drawDesired(ofColor color = ofColor(255,255,255,255));
        bool isPoseControlledExternally();
        void setToolOffset(ofVec3f local);
        ofNode getTCPNode();
        ofNode getForwardNode();
        void setEndEffector(string filename);
        void close();
        vector<double> getCurrentPose();
        bool isLive();
        bool isConnected();
        void toggleLive();

        RobotDriver * robot;
        RobotModel desiredModel;
        RobotModel actualModel;
        vector<RobotModel*> desiredModels;
        InverseKinematics inverseKinematics;
        // RobotArmSafety robotSafety;

        ofParameter<ofVec3f> origin;

        ofNode forwardNode;
    
        ofParameterGroup joints;
        ofParameterGroup safety;
        ofParameterGroup targetJoints;
        ofParameterGroup jointSpeeds;
        ofParameterGroup jointsIK;
        ofParameterGroup robotArmParams;
        
        protected:

        int stopCount = 0;

        vector<vector<double> > targetPoses;
        
        // smooth angles //
        vector <double> homePose;
        vector <double> stopPosition;
        vector <double> targetPose;
        vector <double> currentPose;
        vector <double> prePose;
        vector <double> smoothedPose;
        vector <double> jointWeights;

        ofParameter<bool> bSmoothPose;
        ofParameter<bool> bOverrideNthJoint;
        vector< ofParameter<double> > smoothedWeights;
        ofParameter<double> nthJoint;
        ofParameter<ofVec3f> targetTCPPosition;
        ofParameter<ofVec4f> targetTCPOrientation;
        ofParameter<ofVec4f> tcpOrientation;
        ofParameter<ofVec4f> calcTCPOrientation;
        ofParameter<ofVec4f> forwardTCPOrientation;
        ofParameter<ofVec3f> forwardTCPPosition;
        ofParameter<ofVec3f> tcpPosition;
        ofParameter<ofVec3f> tcpOffset;
        
        ofParameter<bool> bRecord;
        ofParameterGroup pathRecorderParams;
        
        ofParameter<double> followLerp;
        
        vector<ofParameter<double> > pIkPose;
        vector<ofParameter<double> > pCurrentPose;
        vector<ofParameter<double> > pTargetPose;
        vector<ofParameter<double> > pJointVelocities;
        
        ofParameter<bool> bLive;
        ofParameter<bool> bTeachMode;
        ofParameter<bool> bCopy;
    
        ofParameter<bool> bDoReconnect;
        ofParameter<bool> bSetPoseExternally;
        ofParameter<bool> bHome;

        std::string ipAddress;
        int port;
        Pose actualTCP;
        Pose targetTCP;

        ofxRobotArm::RobotType robotType;
        ofxRobotArm::IKType ikType;
        
        
    private:
        Pose initPose;
        Pose forwardPose;
        void setup_parameters();
        ofNode target;
        Plane tcp_plane;
    };
};




