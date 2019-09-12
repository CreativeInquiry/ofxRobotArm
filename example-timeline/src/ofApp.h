#pragma once

//--------------------------------------------------------------
//
//
// Basic Move Example
//
//
//--------------------------------------------------------------

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxGuiExtended2.h"
#include "ofxGizmo.h"
#include "UR5Controller.h"
#include "RobotParameters.h"
#include "URIKFast.h"
#include "ofxTimeline.h"
#include "RobotArmSafety.h"
#define N_CAMERAS 2

class ofApp : public ofBaseApp{
    
public:
    void setup();
    void update();
    void draw();
    void exit();
    
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
    
    ofxTimeline timeline;
    ofCamera camP0;
    ofCamera camP1;
    ofCamera camP2;
    
    ofRectangle viewportSimP0;
    ofRectangle viewportSimP1;
    ofRectangle viewportSimP2;
    
    ofRectangle viewportRealP0;
    ofRectangle viewportRealP1;
    ofRectangle viewportRealP2;
    
    URIKFast kinematics;
    ofxRobotArm::UR5Controller robot;
    ofxRobotArm::RobotParameters parameters;
    ofxGizmo gizmo;
    ofNode tcpNode;
    
    void setupViewports();
    void setupGUI();
    void positionGUI();
    void drawGUI();
    
    void addRealPoseKeyFrame();

    ofxGui gui;
    ofxGuiPanel* panel;
    ofxGuiPanel* panelJoints;
    ofxGuiPanel* panelJointsIK;
    
    int sim, real;
    ofVec3f camUp;
    
    vector<ofxTLCurves*> curves;
    
    ofParameter<bool> lookAtTCP;
    void moveTCP();
    
    bool isTeachModeEnabled;
    // 3D Navigation
    void updateActiveCamera();
    vector<ofEasyCam*> cams;
    vector<string> jointNames;
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
