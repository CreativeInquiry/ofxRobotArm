#pragma once
#include <string>

#include "ofMain.h"
#include "ofAppGLFWWindow.h"
#include "ofxGui.h"
#include "RobotController.h"




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
    
#pragma mark - Robot
    
        ofxRobotArm::RobotController robot;
        int num_dofs = 6;
        vector<double> joint_position_targets;
    
        // EGM Values
        //    abb::egm::wrapper::Joints actualPose;
        //    abb::egm::wrapper::Joints initial_positions;
        // robot->getStatus().rapid_execution_state()
    
    
#pragma mark - GUI
        void setup_gui();
        int screen_coord_scale; // scaling for mac retina display
    
        ofxPanel panel;
        ofParameterGroup params;
        
        ofParameter<bool> animate;
        ofParameter<bool> reset_joint_values;
        void on_reset_joint_values(bool & val);
        
        ofParameter<bool> get_current_joint_positions;
        void on_get_current_joint_positions(bool & val);
        
        ofParameterGroup params_comms;
        ofParameter<int> robot_port;
        ofParameter<string> robot_status;
        ofParameter<bool> robot_connect;
        void on_robot_connect(bool & val);
        
        ofParameterGroup params_robot;
        ofParameter<string> robot_type;
        ofParameterGroup params_robot_joints;
        vector<ofParameter<double>> joint_sliders;
        void on_joint_changed(ofAbstractParameter &e);
    
        ofParameterGroup params_state;
		
};
