//
//  RobotController.hpp
//  example-urdf
//
//  Created by Dan Moore on 3/21/22.
//

#pragma once
#include "ofMain.h"
#include "RobotDriver.h"
#include "URDriver.h"
#include "ABBDriver.h"
#include "XARMDriver.h"
#include "PandaDriver.h"
#include "RobotConstants.hpp"
#include "Plane.h"

namespace ofxRobotArm {
    
    class RobotController {
        public:
            RobotController();
            ~RobotController();
            
            
            void connect();
            void disconnect();
            void update(vector<double> pose, vector<double> smoothing);
            
            void setType(RobotType type);
            void setAddress(string ipAddress);
            void setPort(int port);
        
            bool isConnected();
            
            vector<double> getCurrentPose();
        
            RobotDriver * robot;
            RobotType type;
            string ipaddress;
            int port;
            
            vector<double> targetPose;
            vector<double> currentPose;
        
            ofParameter<bool> bDoReconnect;
          
            
    };
}
