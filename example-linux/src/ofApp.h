#pragma once

#include "ofMain.h"
#include "ofxGui.h"
#include "ofxGizmo.h"
#include "RobotController.h"
#include "RobotParameters.h"
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
    
    ofParameter<int> ux, uy, uz, vx, vy, vz, wx, wy, wz;
    ofParameter<int> angleX, angleY, angleZ;
    ofNode tcp;
    ofNode lookAtNode;
    ofQuaternion initialRot;
    // Robot
    ofxRobotArm::RobotController robot;
    void keypressed_robot(int key);
    
    // Control & Interaction
    ofxGizmo tcp_target;
    ofxGizmo look_target;
    void keypressed_gizmo(int key);
    
    // Scene
    void setup_scene();
    void update_scene();
    void draw_scene();
    
    ofEasyCam cam;
    void setup_camera();
    void keypressed_camera(int key);
    bool disable_camera();
    
    // GUI
    void setup_gui();
    void draw_gui();
    
    ofVec3f home;
    

    ofxRobotArm::RobotParameters robotParams;
    ofxPanel panel;
    ofParameterGroup params;
    ofParameter<bool> show_gui, show_top, show_front, show_side, show_perspective;
    void listener_show_top(bool & val);
    void listener_show_front(bool & val);
    void listener_show_side(bool & val);
    void listener_show_perspective(bool & val);
    
    ofxPanel panel_robot;
    ofParameter<bool> robot_live;
    ofParameter<ofVec3f> offset;
    void draw_live_robot_warning();
    float minY, maxY;
};
