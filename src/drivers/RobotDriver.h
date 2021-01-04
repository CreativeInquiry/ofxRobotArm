//
//  Driver.h
//  example-simple
//
//  Created by Dan Moore on 12/22/20.
//

#pragma once
#include "ofMain.h"
#include "ofxTiming.h"
#include "Pose.h"
#include "Synchronized.h"
namespace ofxRobotArm{
class RobotDriver : public ofThread{
public:
    RobotDriver(){
        
    };
    ~RobotDriver(){
        
    };
    virtual void setAllowReconnect(bool bDoReconnect)=0;
    virtual void setup(string ipAddress, double minPayload = 0.0, double maxPayload = 1.0)=0;
    virtual void start()=0;
    virtual bool isConnected()=0;
    virtual void disconnect()=0;
    virtual void stopThread()=0;
    virtual void toggleTeachMode()=0;
    virtual void setTeachMode(bool enabled)=0;
    virtual void threadedFunction()=0;
    virtual ofVec4f getCalculatedTCPOrientation()=0;
    virtual vector<double> getToolPointRaw()=0;
    virtual vector<double> getCurrentPose()=0;
    virtual vector<double> getJointAngles()=0;
    virtual vector <double> getAchievablePosition(vector<double> position)=0;
    virtual bool isDataReady()=0;
    virtual float getThreadFPS()=0;
    virtual ofQuaternion convertAxisAngle(double rx, double ry, double rz)=0;
    virtual void moveJoints(vector<double> pose)=0;
    virtual void setSpeed(vector<double> speeds, double acceleration = 100.0)=0;
    virtual void setPose(vector<double> positions)=0;
    virtual ofxRobotArm::Pose getToolPose()=0;
    // Robot Arm

    bool bTeachModeEnabled;
    bool bDataReady;
    bool bStarted;
    bool bTryReconnect=true;
    
    vector<double> currentSpeed;
    vector<double> calculatedSpeed;
    vector<double> currentPoseRadian;
    vector<double> currentPose;
    vector<double> targetPose;
    
    double acceleration;

    RateTimer timer;
    float epslion = 0.00000000000000001;
    float lastTimeSentMove = -1;
    float timeNow = 0;
    deque<vector<double> > poseBuffers;
    deque<vector<double> > speedBuffers;


    bool bMove;
    
    Synchronized<vector<double> > poseProcessed;
    Synchronized<vector<double> > poseRaw;
    Synchronized<vector<double> > toolPoseRaw;
    ofxRobotArm::Pose tool;
    ofxRobotArm::Pose dtoolPoint;
    vector<ofxRobotArm::Pose> pose;
    
    bool bTriedOnce = false;
    bool bMoveWithPos = false;
    bool bMoveWithSpeed = false;
    bool bStop = true;
    float acceleratePct = 0.0;
    int deccelCount = 0;
    int numDeccelSteps = 60;
};
}
