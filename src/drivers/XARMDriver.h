#pragma once
#include "RobotDriver.h"
#include "RobotConstants.hpp"
#include "xarm/wrapper/xarm_api.h"
namespace ofxRobotArm
{
    class XARMDriver : public RobotDriver
    {
    public:
        XARMDriver();
        XARMDriver(RobotType type);
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
        vector<double> getInitPose();
        
        // Robot Arm
        XArmAPI *robot;
    };
}
