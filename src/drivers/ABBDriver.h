
#pragma once
#include "ofMain.h"
#include "ofxTiming.h"
#include "Joint.h"
#include "Synchronized.h"
#include <abb_libegm/egm_udp_server.h>
#include <abb_libegm/egm_controller_interface.h>

class ofxABBDriver : public ofThread{
    public:
    ofxABBDriver();
    ~ofxABBDriver();
    void setAllowReconnect(bool bDoReconnect);
    void setup(string ipAddress, int port, double minPayload = 0.0, double maxPayload = 1.0);
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
    abb::egm::EGMControllerInterface* robot;
    boost::asio::io_service * io_service;
    boost::thread_group * thread_group;
    // Robot Arm
  
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
    
    bool bTryReconnect = false;
    bool bTriedOnce = false;
    bool bMoveWithPos = false;
    bool bMoveWithSpeed = false;
    bool bStop = true;
    float acceleratePct = 0.0;
    int deccelCount = 0;
    int numDeccelSteps = 60;
    
    
    
    bool wait = true;
    abb::egm::wrapper::Input input;
    abb::egm::wrapper::CartesianPose initial_pose;
    const int egm_rate = 250.0; // [Hz] (EGM communication rate, specified by the EGMActPose RAPID instruction).
    int sequence_number = 0;    // [-] (sequence number of a received EGM message).
    double time = 0.0;          // [seconds] (elapsed time during an EGM communication session).

    abb::egm::wrapper::Output output;
    double position_reference = 0.0;      // [mm].
    double orientation_reference = 0.0;   // [degrees].
    double position_amplitude = 100.0;    // [mm].
    double orientation_amplitude = -10.0; // [degrees].
    double frequency = 0.25;
};
