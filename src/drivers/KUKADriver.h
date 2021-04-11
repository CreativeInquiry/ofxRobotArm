
#pragma once
#include "ofMain.h"
#include "RobotDriver.h"
#include "ofxTiming.h"
#include "Pose.h"
#include "robotKuka.hpp"
#include "Synchronized.h"

namespace ofxRobotArm{
class KUKADriver : public RobotDriver{
public:
    KUKADriver();
    ~KUKADriver();
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

    void setToolOffset(ofVec3f localPos);
    void threadedFunction();
    ofNode getToolNode();
    ofVec4f getCalculatedTCPOrientation();
    vector<double> getToolPointRaw();
    vector<double> getCurrentPose();
    vector<double> getInitPose();

    bool isDataReady();
    float getThreadFPS();
    bool bDataReady;
    bool bStarted;
    void moveJoints(vector<double> pos);
    void setSpeed(vector<double> speeds, double acceleration = 100.0);
    void setPose(vector<double> pose);
    
    ofxRobotArm::Pose getToolPose();

    kuka::RobotKuka robot;

    bool wait = true;

    bool bTryReconnect, bTeachModeEnabled;
    int sequence_number = 0;    // [-] (sequence number of a received EGM message).
    double time = 0.0;          // [seconds] (elapsed time during an EGM communication session).
    double position_reference = 0.0;      // [mm].
    double orientation_reference = 0.0;   // [degrees].
    double position_amplitude = 100.0;    // [mm].
    double orientation_amplitude = -10.0; // [degrees].
    double frequency = 0.25;
};
}
