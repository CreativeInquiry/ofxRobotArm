//
//  Driver.h
//  example-simple
//
//  Created by Dan Moore on 12/22/20.
//

#pragma once
#include "RobotDriver.h"
namespace ofxRobotArm
{
    class PandaDriver : public RobotDriver
    {
    public:
        PandaDriver();
        ~PandaDriver();
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
        vector<double> getInitPose();
        int numJoints = 7;
    };
}
