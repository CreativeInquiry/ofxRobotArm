#pragma once
#include "RobotDriver.h"
#include "xarm/wrapper/xarm_api.h"
namespace ofxRobotArm
{
    class XARMDriver : public RobotDriver
    {
    public:
        XARMDriver();
        ~XARMDriver();
         void setAllowReconnect(bool bDoReconnect) ;
         void setup() ;
         void setup(string ipAddress, int port, double minPayload = 0.0, double maxPayload = 1.0) ;
         void setup(string ipAddress, double minPayload = 0.0, double maxPayload = 1.0) ;
         void setup(int port, double minPayload = 0.0, double maxPayload = 1.0) ;
         void start() ;
         bool isConnected() ;
         void disconnect() ;
         void stopThread() ;
         void toggleTeachMode() ;
         void setTeachMode(bool enabled) ;
         void threadedFunction() ;
         ofVec4f getCalculatedTCPOrientation() ;
         vector<double> getToolPointRaw() ;
         vector<double> getCurrentPose() ;
    
         bool isDataReady() ;
         float getThreadFPS() ;
         void setSpeed(vector<double> speeds, double acceleration = 100.0) ;
         void setPose(vector<double> positions) ;
         ofxRobotArm::Pose getToolPose() ;
         vector<double> getInitPose() ;
        
        void report_location_callback(const fp32* pose, const fp32* angles) ;
        void connect_changed_callback(bool connected, bool reported);
        void state_changed_callback(int state);
        void mode_changed_callback(int mode);
        void mtable_mtbrake_changed_callback(int mtable, int mtbrake);
        void error_warn_changed_callback(int err, int warn);
        void cmdnum_changed_callback(int cmdnum);
        
        // Robot Arm
        XArmAPI *robot;
    };
}
