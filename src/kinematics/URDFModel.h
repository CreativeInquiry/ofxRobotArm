#pragma once
#include "ofMain.h"
#include "RobotModel.h"
#include "ofxAssimpModelLoader.h"
#include "ofxXmlSettings.h"
#include "URDFParser.h"
#include "Pose.h"
namespace ofxRobotArm{
    class URDF{
        
    };

    class URDFModel : public RobotModel{
        public:
            URDFModel();
            ~URDFModel();
            bool setup(string path, bool forceFixedBase, int flag);
            void load(string filepath);
           
        private:
            vector<ofMesh> meshes;
            ofMesh toolMesh;
            vector<ofxRobotArm::Pose> pose;
            vector<double> jointMin;
            vector<double> jointMax;
            vector<ofNode> nodes;
            ofNode forwardPose;
            ofNode tcpNode;
            ofNode toolNode;
            vector<double> poseRadians;

            UrdfParser parser;
            int mFlag;
    };
};
