//
//  URDriver.hpp
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
//
// Copyright (c) 2016, 2021 Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
////

#pragma once
#include "ofMain.h"
#include "RobotDriver.h"
#include "ur_driver.h"
#include "RobotModel.h"
#include "ofxTiming.h"
namespace ofxRobotArm{
class URDriver : public RobotDriver{
public:
    URDriver();
    ~URDriver();
    void setAllowReconnect(bool bDoReconnect);
    void setup();
    void setup(string ipAddress, int port, double minPayload = 0.0, double maxPayload = 1.0);
    void setup(string ipAddress, double minPayload = 0.0, double maxPayload = 1.0);
    void setup(int port, double minPayload = 0.0, double maxPayload = 1.0);
    void start();
    bool isConnected();
    void disconnect();
    void stopThread();
    void toggleTeachMode();
    void setTeachMode(bool enabled);
    void threadedFunction();
    ofVec4f getCalculatedTCPOrientation();
    vector<double> getToolPointRaw();
    vector<double> getCurrentPose();
    vector<double>  getInitPose();
    bool isDataReady();
    float getThreadFPS();
    bool bDataReady;
    bool bStarted;
    void moveJoints(vector<double> pos);
    void setSpeed(vector<double> speeds, double acceleration = 100.0);
    void setPose(vector<double> positions);
    
    ofxRobotArm::Pose getToolPose();
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
    bool bTeachModeEnabled;
    
    Synchronized<vector<double> > jointsProcessed;
    Synchronized<vector<double> > jointsRaw;
    Synchronized<vector<double> > toolPointRaw;
    ofxRobotArm::Pose tool;
    ofxRobotArm::Pose dtoolPoint;
    vector<ofxRobotArm::Pose> joints;
    

    bool bTriedOnce = false;
    bool bMoveWithPos = false;
    bool bMoveWithSpeed = false;
    bool bStop = true;
    float acceleratePct = 0.0;
    int deccelCount = 0;
    int numDeccelSteps = 60;

};
}
