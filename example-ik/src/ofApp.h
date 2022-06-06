#pragma once

#include "ofMain.h"
#include "ofxGizmo.h"
#include "RobotController.h"
#include "RelaxedIKSolver.h"
#include "RobotModel.h"
#include "Pose.h"
#include "hk.h"
#include "opw.h"
class ofApp : public ofBaseApp{

    public:
        void setup();
        void update();
        void draw();

        void keyPressed(int key);
        void keyReleased(int key);
        void mouseMoved(int x, int y );
        void mouseDragged(int x, int y, int button);
        void mousePressed(int x, int y, int button);
        void mouseReleased(int x, int y, int button);
        void mouseEntered(int x, int y);
        void mouseExited(int x, int y);
        void windowResized(int w, int h);
        void dragEvent(ofDragInfo dragInfo);
        void gotMessage(ofMessage msg);
    
    
    
        ofEasyCam cam;
        ofxRobotArm::RobotModel model;
        ofxRobotArm::OPWIK sphericalWristKinematics;
        ofxRobotArm::HKIK hawkinsKelseyKinematics;
        ofxRobotArm::RelaxedIKSolver relaxedIKSolver;
        ofxRobotArm::Pose desiredPose;
        ofxRobotArm::Pose initialPose;
    
        vector<double> opw_results;
        vector<double> hk_results;
        vector<double> rik_results;
    
    
        vector<double> offsets;
        vector<double> sign_correction;
        vector<double> joint_limit_min;
        vector<double> joint_limit_max;
        
        ofMatrix4x4 mat;
    
        ofxGizmo gizmo;
    
};
