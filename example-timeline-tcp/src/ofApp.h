//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
#pragma once
#define N_CAMERAS 2
#include "ofMain.h"
#include "ofxGui.h"
#include "UR10Controller.h"
#include "RobotParameters.h"
#include "ofxGizmo.h"
#include "ofxTimeline.h"
#include "ofxTLNodeTrack.h"
//#include "ofxSyphon.h"

//#define ENABLE_NATNET

using namespace ofxRobotArm;
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
    void moveArm();
    
    void setupViewports();
    void setupGUI();
    void positionGUI();
    void setupTimeline();
    /// \brief 3D mesh with paths for robot to follow

    ofxTLNodeTrack* nodeTrack;
    ofxTimeline timeline;
    
    ofRectangle viewportReal;
    ofRectangle viewportSim;
    
    RobotParameters parameters;

    ofxPanel panel;
    ofxPanel panelWorkSurface;
    ofxPanel panelJoints;
    ofxPanel panelTargetJoints;
    ofxPanel panelJointsIK;
    ofxPanel panelJointsSpeed;
    
    ofxGizmo gizmo;
    ofNode tcpNode;
    
    UR10Controller robot;

    float acceleration;
    vector<double> speeds;


    
    ofNode parent;
    
    int count;
    
   
    bool hideRobot;

    
    // 3D Navigation
    vector<ofEasyCam*> cams;
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
                 '5' = Custom View   <br/>
                 '6' = Save current for Custom View
     */
    void handleViewportPresets(int key);
    
    //ofxSyphonServer syphon;
};
