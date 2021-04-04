#pragma once
#include "ofMain.h"
#include "ofxXmlSettings.h"
#include "Pose.h"
namespace ofxRobotArm{
    class URDF{
        
    };

    class URDFModel{
        public:
            URDFModel();
            ~URDFModel();
            void load(string filepath);
            void draw();
            void drawSKeleton();
            void setPose(vector<double> pose);
            void setForwardPose(ofNode pose);
            void setEndEffector(string filename);
            void setToolMesh(ofMesh mesh);
        private:
            vector<ofxRobotArm::Pose> pose;
            vector<double> jointMin;
            vector<double> jointMax;
            vector<ofNode> nodes;
            ofNode forwardPose;
            ofNode tcpNode;
            ofNode toolNode;
            vector<double> poseRadians;
    };
};