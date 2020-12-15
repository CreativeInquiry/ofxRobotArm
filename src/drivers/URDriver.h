//
//  ofxURDriver.hpp
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
// Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//

#pragma once
#include "ofMain.h"
#include "ur_driver.h"
//#include "UR5KinematicModel.h"
//#include "UR10KinematicModel.h"
#include "RobotKinematicModel.h"
#include "ofxTiming.h"

class ofxURDriver : public ofThread{
public:
    ofxURDriver();
    ~ofxURDriver();
    void setAllowReconnect(bool bDoReconnect);
    void setup(string ipAddress, double minPayload = 0.0, double maxPayload = 1.0);
    void start();
    bool isConnected();
    void disconnect();
    void stopThread();
    void toggleTeachMode();
    void setTeachMode(bool enabled);
    bool bTeachModeEnabled;
    void setToolOffset(ofVec3f localPos);
    void threadedFunction();
    ofNode getToolNode();
    ofVec4f getCalculatedTCPOrientation();
    vector<double> getToolPointRaw();
    vector<double> getCurrentPose();
    vector<double> getJointAngles();
    
    vector <double> getAchievablePosition(vector<double> position);
    
    bool isDataReady();
    float getThreadFPS();
    bool bDataReady;
    bool bStarted;
    ofQuaternion convertAxisAngle(double rx, double ry, double rz);
    void moveJoints(vector<double> pos);
    void setSpeed(vector<double> speeds, double acceleration = 100.0);
    void setPosition(vector<double> positions);
    
    ofxRobotArm::Joint getToolPose();
    // Robot Arm
    UrDriver* robot;
    condition_variable rt_msg_cond_;
    condition_variable msg_cond_;
    bool has_goal_;
    std::thread* rt_publish_thread_;
    std::thread* mb_publish_thread_;
    double io_flag_delay_;
    double max_velocity_;
    vector<double> joint_offsets_;
    string base_frame_;
    string tool_frame_;
    bool use_ros_control_;
    bool bTryReconnect=true;
    std::thread* ros_control_thread_;
    vector<double> currentSpeed;
    vector<double> currentPosition;
    vector<double> targetPose;
    vector<double> calculatedSpeed;
    vector<double> currentRobotPositionRadians;

    double acceleration;

    RateTimer timer;
    float epslion = 0.00000000000000001;
    float lastTimeSentMove = -1;
    float timeNow = 0;
    deque<vector<double> > posBuffer;
    deque<vector<double> > speedBuffers;


    bool bMove;
    
    Synchronized<vector<double> > jointsProcessed;
    Synchronized<vector<double> > jointsRaw;
    Synchronized<vector<double> > toolPointRaw;
    ofxRobotArm::Joint tool;
    ofxRobotArm::Joint dtoolPoint;
    vector<ofxRobotArm::Joint> joints;
    

    bool bTriedOnce = false;
    bool bMoveWithPos = false;
    bool bMoveWithSpeed = false;
    bool bStop = true;
    float acceleratePct = 0.0;
    int deccelCount = 0;
    int numDeccelSteps = 60;

};
