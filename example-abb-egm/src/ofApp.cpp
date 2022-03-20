#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    // OF mac retina screen bug: https://github.com/openframeworks/openFrameworks/issues/5943#issuecomment-377696206
    screen_coord_scale = ((ofAppGLFWWindow *)(ofGetWindowPtr()))->getPixelScreenCoordScale();
    ofSetWindowShape(screen_coord_scale*ofGetWidth(), screen_coord_scale*ofGetHeight());
    
    setup_gui();
    

    robot.createRobot(ofxRobotArm::IRB120);
//    robot.loadURDF("relaxed_ik_core/config/urdfs/irb120.urdf");
    robot.setPoseExternally(true);
    robot.setPort(robot_port);
    robot.setupRobot(false);
    robot.setupParams();
   
    for (int i=0; i<num_dofs; i++)
        joint_position_targets.push_back(0.0);
    
}

//--------------------------------------------------------------
void ofApp::update(){
    
    // Animate a joint value to send to the robot
    if (animate){
        double min = -300;
        double max = 300;
        double val = ofMap(sin(ofGetElapsedTimef()), -1, 1, max, min, max);
        if (joint_sliders.size() > 0){
            joint_sliders[0].set(val);
        }
    }

}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(60);

    panel.draw();
}

//--------------------------------------------------------------
void ofApp::setup_gui(){
    
    params.setName("Control_Parameters");
    params.add(animate.set("Animate_Joints", false));
    
    params_comms.setName("Communications");
    params_comms.add(robot_port.set("Port:", 6510));
    params_comms.add(robot_status.set("STATUS:", "NOT CONNECTED"));
    params_comms.add(robot_connect.set("Connect", false));
    robot_connect.addListener(this, &ofApp::on_robot_connect);
    
    
    params_robot.setName("Robot_Parameters");
    params_robot.add(robot_type.set("RobotType:","ABB IRB120"));
    params_robot.add(reset_joint_values.set("Reset_Joints", false));
    reset_joint_values.addListener(this, &ofApp::on_reset_joint_values);
    params_robot.add(get_current_joint_positions.set("Get_Current_Joints", false));
    get_current_joint_positions.addListener(this, &ofApp::on_get_current_joint_positions);
    params_robot_joints.setName("Joint_Controller");
    for (int i=0; i<num_dofs; i++){
        joint_sliders.push_back(ofParameter<double>());
        joint_sliders[i].set("Joint_"+ofToString(i), 0, -360, 360);
        params_robot_joints.add(joint_sliders[i]);
    }
    ofAddListener(params_robot_joints.parameterChangedE(), this, &ofApp::on_joint_changed);
    params_robot.add(params_robot_joints);
    
    panel.setup();
    panel.setName("ABB_EGM_Example");
    panel.add(params);
    panel.add(params_comms);
    panel.add(params_robot);
    panel.setPosition(10,10);

}

//--------------------------------------------------------------
void ofApp::on_reset_joint_values(bool &val){
    if (val){
        for (int i=0; i<num_dofs; i++)
            joint_sliders[i].set(0.0);
        reset_joint_values = false;
    }
}

//--------------------------------------------------------------
void ofApp::on_get_current_joint_positions(bool &val){
    if (val){
        if (robot.robot->isConnected()){
            auto curr_positions = robot.getCurrentPose();
            for (int i=0; i<num_dofs; i++)
                joint_sliders[i].set(curr_positions[i]);
            get_current_joint_positions = false;
        }
        else{
            ofLogWarning("ofApp::on_get_current_joint_positions", "Robot is not connected yet.");
        }
    }
}

//--------------------------------------------------------------
void ofApp::on_joint_changed(ofAbstractParameter &e){
    int index = stoi(string(1,e.getName().back()));
    joint_position_targets[index] = joint_sliders[index].get();
    
    if (robot.robot->isConnected()){
        robot.update(joint_position_targets);
    }
    
    cout << ofToString(joint_position_targets) << endl;
}

//--------------------------------------------------------------
void ofApp::on_robot_connect(bool & val){
    if (val){
        robot_status.set("CONNECTING");
        robot.setPort(robot_port);
        robot.startConnection();
        if (robot.robot->isConnected())
            robot_status.set("CONNECTED");
        else{
            robot_status.set("NOT CONNECTED");
        }
    }
    else{
        robot.disconnectRobot();
        robot_status.set("DISCONNECTED");
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    switch (key) {
        case ' ':
            animate.set(!animate);
            break;
        case 'f':
        case 'F':
            ofToggleFullscreen();
            break;
            
        default:
            break;
    }

}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){

}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){

}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
