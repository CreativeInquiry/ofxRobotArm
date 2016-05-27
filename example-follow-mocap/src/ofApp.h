#pragma once


//--------------------------------------------------------------
//
//
// Follow Motion Capture Example
//
//
//--------------------------------------------------------------


#include "ofMain.h"
#include "ofxGui.h"
#include "ofxNatNet.h"
#include "ofxGizmo.h"
#include "RobotController.h"
#include "PathController.h"
#include "RobotParameters.h"
#define N_CAMERAS 2

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
		
        RobotParameters parameters;
    
        ofxGizmo gizmo;
        ofNode tcpNode;
    
        void setupViewports();
        void setupGUI();
        void positionGUI();
        void drawGUI();
    
    
        ofxPanel panel;
        ofxPanel panelJoints;
        ofxPanel panelTargetJoints;
        ofxPanel panelJointsIK;
        ofxPanel panelJointsSpeed;
        
        
        RobotController robot;
        void moveArm();
        bool followRigidBody;
    

        // Motion Capture
        ofxNatNet mocap;
        void setupMocap(string myIP, string serverIP);
        void setupMocap(string myIP, string serverIP, int scale);
        void updateMocap();
        void drawMocap();
    
        ofxNatNet::RigidBody currentRB;
    
    
    
        // 3D Navigation
        void updateActiveCamera();
        vector<ofEasyCam*> cams;
        ofRectangle viewportReal;
        ofRectangle viewportSim;
        vector<ofMatrix4x4> savedCamMats;
        vector<string> viewportLabels;
        int activeCam;
        
        /**
         Use hotkeys to cyle through preset viewports.
         @param key
             '1' = Top View      <br/>
             '2' = Left View     <br/>
             '3' = Front View    <br/>
             '4' = Perspective   <br/>
         */
        void handleViewportPresets(int key);
        
        /// Highlights the active viewport.
        void hightlightViewports();

};
