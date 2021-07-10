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
            
            void setup(string path, RobotType type);
            bool setup(string path, bool forceFixedBase, bool mergeFixedJoints, bool printDebug, bool parseSensors, ofxRobotArm::RobotType type);
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
    };
};
