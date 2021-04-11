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
namespace ofxRobotArm
{
    class RobotDriver : public ofThread
    {
    public:
        RobotDriver(){

        };
        ~RobotDriver(){

        };
        virtual void setAllowReconnect(bool bDoReconnect) = 0;
        virtual void setup() = 0;
        virtual void setup(string ipAddress, int port, double minPayload = 0.0, double maxPayload = 1.0) = 0;
        virtual void setup(string ipAddress, double minPayload = 0.0, double maxPayload = 1.0) = 0;
        virtual void setup(int port, double minPayload = 0.0, double maxPayload = 1.0) = 0;
        virtual void start() = 0;
        virtual bool isConnected() = 0;
        virtual void disconnect() = 0;
        virtual void stopThread() = 0;
        virtual void toggleTeachMode() = 0;
        virtual void setTeachMode(bool enabled) = 0;
        virtual void threadedFunction() = 0;
        virtual ofVec4f getCalculatedTCPOrientation() = 0;
        virtual vector<double> getToolPointRaw() = 0;
        virtual vector<double> getCurrentPose() = 0;
        vector<double> getAchievablePosition(vector<double> position)
        {
            float maxAccelDeg = 500.0;
            float maxSpeedPct = 1.0;

            if (!bMove && deccelCount > 0)
            {
                maxSpeedPct = ofMap(deccelCount, 1, numDeccelSteps - 1, 0.0, 1.0, true);
                if (maxSpeedPct < 0.9)
                {
                    acceleratePct = 0;
                }
                //cout << " deccelCount " << deccelCount << " maxSpeedPct " << maxSpeedPct << endl;
            }
            if (bMove)
            {
                acceleratePct += 0.02;
                if (acceleratePct > 1.0)
                {
                    acceleratePct = 1.0;
                }
            }

            maxSpeedPct *= acceleratePct;

            //this seeems to do much better with a hardcoded timedelta
            float timeDiff = 1.0 / 120.0; //timeNow-lastTimeSentMove;

            if (currentPoseRadian.size() && position.size())
            {

                vector<double> lastSpeed = calculatedSpeed;

                if (calculatedSpeed.size() != position.size())
                {
                    calculatedSpeed.assign(position.size(), 0);
                    lastSpeed = calculatedSpeed;
                }

                for (unsigned int d = 0; d < position.size(); d++)
                {
                    calculatedSpeed[d] = (position[d] - currentPoseRadian[d]) / timeDiff;
                }

                vector<double> acceleration;
                acceleration.assign(calculatedSpeed.size(), 0);

                for (unsigned int d = 0; d < acceleration.size(); d++)
                {
                    acceleration[d] = (calculatedSpeed[d] - lastSpeed[d]) / timeDiff;

                    float accelDegPerSec = ofRadToDeg(acceleration[d]);

                    //this is the max accel reccomended.
                    //if we are over it we limit the new position to being the old position plus the current speed, plus the max acceleration
                    //this seems to actually work - fuck yes!
                    if (fabs(accelDegPerSec) > maxAccelDeg)
                    {

                        //cout << d << " currentRobotPositionRadians is " << ofRadToDeg( currentRobotPositionRadians[d] ) << " request is " << ofRadToDeg(position[d]) <<  " speed is " << ofRadToDeg( calculatedSpeed[d] ) << " prev Speed is " << ofRadToDeg(lastSpeed[d])  << " accel is "  << accelDegPerSec << endl;

                        float newAccel = ofDegToRad(maxAccelDeg) * (float)ofSign(accelDegPerSec);
                        float speedDiff = newAccel * timeDiff;
                        float targetSpeed = lastSpeed[d] + speedDiff;

                        position[d] = currentPoseRadian[d] + (targetSpeed * timeDiff * maxSpeedPct);

                        //cout << "---- hit limit: accel is " << ofRadToDeg(newAccel) << " targetSpeed is now " << ofRadToDeg(targetSpeed) << " pos is now " << position[d] << endl;
                    }
                    else if (maxSpeedPct < 1.0)
                    {
                        position[d] = currentPoseRadian[d] + (currentSpeed[d] * timeDiff * maxSpeedPct);
                    }
                }
            }

            return position;
        }
        virtual bool isDataReady() = 0;
        virtual float getThreadFPS() = 0;
        virtual void moveJoints(vector<double> pose) = 0;
        virtual void setSpeed(vector<double> speeds, double acceleration = 100.0) = 0;
        virtual void setPose(vector<double> positions) = 0;
        virtual ofxRobotArm::Pose getToolPose() = 0;
        virtual vector<double> getInitPose() = 0;
        // Robot Arm

        bool bTeachModeEnabled;
        bool bDataReady;
        bool bStarted;
        bool bTryReconnect = true;

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
        deque<vector<double>> poseBuffers;
        deque<vector<double>> speedBuffers;

        bool bMove; 

        Synchronized<vector<double>> poseProcessed;
        Synchronized<vector<double>> poseRaw;
        Synchronized<vector<double>> toolPoseRaw;
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
