//
//  RobotController.h
//  urModernDriverTest
//
//  Created by dantheman on 4/4/16.
//
//
#pragma once
#include "ofMain.h"
#include "RobotStateMachine.h"
#include "ofxURDriver.h"
#include "RobotParameters.h"
#include "PathRecorder.h"
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
    void updateData();
    void update();
    void moveArm();
    void draw();
    void toggleRecord();
    void close();
    ofNode getTCPNode();
    vector<double> getJointPosition();
    RobotStateMachine state;
    ofxURDriver robot;
    URMove movement;
    RobotParameters * robotParams;
    
    PathRecorder recorder;
    
    
    
    Joint workSurfaceTargetTCP;
};