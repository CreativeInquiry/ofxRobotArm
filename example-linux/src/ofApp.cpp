#include "ofApp.h"

using namespace ofxRobotArm;
//--------------------------------------------------------------
void ofApp::setup(){
    cout<<ofToDataPath("")<<endl;
    ofSetFrameRate(120);
    // setup scene
    setup_scene();
    robotParams.setup(RobotType::UR10);
    // setup robot
    robot.setup(robotParams);    // change IP string to your robot's IP address
    
    // setup gui
    setup_gui();
    
    // start robot
    robot.start();
    
    //    tcp.setPosition(0, 0, 0);
    //    tcp.setOrientation(ofQuaternion(0, 0, 0, 1));
    //    tcp.setPosition(250, 250, 250);
    //    lookAtNode.setPosition(10, 0, 0);
    //    ofQuaternion q;
    //    q.makeRotate(90, 0, 1, 0);
    //    tcp.setOrientation(q);
    //    lookAtNode.setOrientation(q);
    //    tcp_target.setNode(tcp);
    //    lookAtNode.setParent(tcp);
    //    look_target.setNode(lookAtNode);
    //

    robot.setToolOffset(offset);
    
    tcp = robot.getActualTCPNode();
    tcp.setPosition(tcp.getPosition()*1000);
    tcp.setOrientation(ofQuaternion(0, 1, 0, 1));
    tcp_target.setNode(tcp);
    
    //    lookAtNode.setPosition(-510, -510, 500);
    //    look_target.setNode(lookAtNode);
    
    initialRot = tcp.getOrientationQuat();
    int x = 500;
    int y = -1000;
    int z = 500;
    
    ofVec3f pos = ofVec3f(x, y, z);
    ofVec3f tgt = ofVec3f(0, 800 / 2, 0);
    cam.setGlobalPosition(pos);
    cam.setTarget(tgt);
    cam.lookAt(tgt, ofVec3f(0, 0, 1));
    cam.setGlobalPosition(pos);
    
    show_top = false;
    show_front = false;
    show_side = false;
    show_perspective = true;
    
    home.set(300, 0, 555);
}

//--------------------------------------------------------------
void ofApp::update(){
    
    
    robot.inverseKinematics.relaxedIK.setAngle(angleX, angleY, angleZ);

    ofVec3f u = ofVec3f(ux, uy, uz);
    ofVec3f v = ofVec3f(vx, vy, vz);
    ofVec3f w = ofVec3f(wx, wy, wz);
    robot.inverseKinematics.relaxedIK.setMatrix(u, v, w);

//
//    ofVec3f p = tcp_target.getTranslation();
//    ofVec3f p2 = look_target.getTranslation();
//
//    ofMatrix4x4 mat, matR, matRR, matRRR;
//    mat.makeLookAtMatrix(p2, p, ofVec3f(0, 0, 1));
////    mat = mat.getInverse();
////    matR.makeRotationMatrix(90, ofVec3f(1, 0, 0));
////    matRRR.makeRotationMatrix(90, ofVec3f(0, 0, 1));
////    mat = mat*matR*matRRR;

    tcp.setPosition(tcp_target.getTranslation());
    tcp.setOrientation(tcp_target.getRotation());

    robot.setToolOffset(offset);
    robot.setDesired(tcp);
    robot.update();
//    vector<double> zero(6, 0.0);
//    robot.update(zero);
    
}

//--------------------------------------------------------------
void ofApp::draw(){
    ofBackground(20);
    
    // Draw 3D Scene
    draw_scene();
    
    // Draw 2D Information
    if (show_gui){
        draw_gui();
    }
    // if robot is LIVE, draw indicator
    if (robot_live){
        draw_live_robot_warning();
    }

}

#pragma mark - Scene
//--------------------------------------------------------------
void ofApp::setup_scene(){
    
    
}

//--------------------------------------------------------------
void ofApp::draw_scene(){
    cam.begin();
    ofDrawAxis(1500);
    ofDrawGrid(100, 10, false, false, false, true);
    // Draw Real Robot
//    robot.draw(ofColor::red);
    // Draw Desired Robot
    robot.drawDesired(ofColor::whiteSmoke);

    
    
    tcp_target.draw(cam);
    look_target.draw(cam);
    cam.end();
}

//--------------------------------------------------------------
bool ofApp::disable_camera(){
    
    if (tcp_target.isInteracting() || look_target.isInteracting())
        return true;
    
    ofRectangle gui_rect;
    gui_rect.setX(panel.getPosition().x);
    gui_rect.setY(panel.getPosition().y);
    gui_rect.setWidth(panel.getWidth() + 20);
    gui_rect.setHeight(panel.getHeight() + panel_robot.getHeight() + 20);
    if (gui_rect.inside(mouseX, mouseY))
        return true;
    
    return false;
}

#pragma mark - GUI
//--------------------------------------------------------------
void ofApp::setup_gui(){
    
    params.setName("Navigation");
    params.add(show_gui.set("Show_GUI", true));
    params.add(show_top.set("TOP", true));
    params.add(show_front.set("FRONT", false));
    params.add(show_side.set("SIDE", false));
    params.add(show_perspective.set("PERSP", false));
    
    params.add(offset.set("offset", ofVec3f(0, 0, 0), ofVec3f(-100, -100, -100), ofVec3f(100, 100, 100)));
    params.add(ux.set("UX", 1, -1, 1));
    params.add(uy.set("UY", 0, -1, 1));
    params.add(uz.set("UZ", 0, -1, 1));
    params.add(vx.set("VX", 0, -1, 1));
    params.add(vy.set("VY", 1, -1, 1));
    params.add(vz.set("VZ", 0, -1, 1));
    params.add(wx.set("WX", 0, -1, 1));
    params.add(wy.set("WY", 0, -1, 1));
    params.add(wz.set("WZ", 1, -1, 1));
    
    show_top.addListener(this, &ofApp::listener_show_top);
    show_front.addListener(this, &ofApp::listener_show_front);
    show_side.addListener(this, &ofApp::listener_show_side);
    show_perspective.addListener(this, &ofApp::listener_show_perspective);
    
    panel.setup(params);
    panel.setPosition(10, 10);
    
    panel_robot.setup("Robot_Controller");
    panel_robot.add(robot_live.set("Robot_LIVE", false));
    panel_robot.setPosition(panel.getPosition().x, panel.getPosition().y + panel.getHeight() + 5);
    panel_robot.add(robotParams.joints);
    panel_robot.add(robotParams.targetJoints);
    
    ofSetCircleResolution(60);
}

//--------------------------------------------------------------
void ofApp::draw_gui(){
    panel.draw();
    panel_robot.draw();
    
    ofDrawBitmapStringHighlight("FPS: "+ofToString(ofGetFrameRate()), ofGetWidth()-100, 10);
//    ofDrawBitmapStringHighlight("SFF: "+ofToString(robot.inverseKinematics.relaxedIK.getFrame()), ofGetWidth()-100, 40);
}

//--------------------------------------------------------------
void ofApp::draw_live_robot_warning(){
    float alpha = ofMap(sin(ofGetElapsedTimef()*5), -1, 1, 50, 150);
    float line_width = 25;
    ofPushStyle();
    ofSetColor(255, 0, 0, alpha);
    ofSetLineWidth(line_width);
    ofNoFill();
    ofDrawLine(0, line_width/2, ofGetWidth(), line_width/2);
    ofDrawLine(0, ofGetHeight()-line_width/2, ofGetWidth(), ofGetHeight()-line_width/2);
    ofDrawLine(line_width/2, line_width, line_width/2, ofGetHeight()-line_width);
    ofDrawLine(ofGetWidth()-line_width/2, line_width, ofGetWidth()-line_width/2, ofGetHeight()-line_width);
    ofSetColor(255,alpha+50);
    ofDrawBitmapString("WARNING: ROBOT IS LIVE!", ofGetWidth()/2-100, 2*line_width/3);
    ofPopStyle();
}

//--------------------------------------------------------------
void ofApp::setup_camera(){
    cam.setFarClip(9999999);
    cam.setDistance(1500);
    ofNode tgt;
    tgt.setGlobalPosition(0, 0, 0);
    tgt.setGlobalOrientation(ofQuaternion(0, 0, 0, 1));
    cam.setTarget(tgt);
    cam.lookAt(ofVec3f(0, 0, -1), ofVec3f(0, 0, 1));
}

//--------------------------------------------------------------
void ofApp::listener_show_top(bool & val)
{
    if (val) {
        
        int x = 0;
        int y = 0;
        int z = 2000;
        
        
        ofVec3f pos = ofVec3f(x, y, z);
        ofVec3f tgt = ofVec3f(pos.x, pos.y, 0);
        cam.setGlobalPosition(pos);
        cam.setTarget(tgt);
        cam.lookAt(tgt, ofVec3f(1,0,0));
        
        show_top = true;
        show_front = false;
        show_side = false;
        show_perspective = false;
    }
}

//--------------------------------------------------------------
void ofApp::listener_show_front(bool & val)
{
    if (val) {
        
        int x = 2500;
        int y = 0;
        int z = 600;
        
        ofVec3f pos = ofVec3f(x, y, z);
        ofVec3f tgt = ofVec3f(0, pos.y, pos.z);
        cam.setGlobalPosition(pos);
        cam.setTarget(tgt);
        cam.lookAt(tgt, ofVec3f(0, 0, 1));
        
        show_top = false;
        show_front = true;
        show_side = false;
        show_perspective = false;
    }
}

//--------------------------------------------------------------
void ofApp::listener_show_side(bool & val)
{
    if (val) {
        
        int x = 0;
        int y = -2500;
        int z = 600;
        
        ofVec3f pos = ofVec3f(x, y, z);
        ofVec3f tgt = ofVec3f(pos.x, 0, pos.z);
        cam.setGlobalPosition(pos);
        cam.setTarget(tgt);
        cam.lookAt(tgt, ofVec3f(0, 0, 1));
        
        show_top = false;
        show_front = false;
        show_side = true;
        show_perspective = false;
    }
}

//--------------------------------------------------------------
void ofApp::listener_show_perspective(bool & val)
{
    if (val) {
        
        int x = 1500;
        int y = -1500;
        int z = 1500;
        
        ofVec3f pos = ofVec3f(x, y, z);
        ofVec3f tgt = ofVec3f(0, 0, 0);
        cam.setGlobalPosition(pos);
        cam.setTarget(tgt);
        cam.lookAt(tgt, ofVec3f(0, 0, 1));
        cam.setGlobalPosition(pos);
        
        show_top = false;
        show_front = false;
        show_side = false;
        show_perspective = true;
    }
}

#pragma mark - App Functions

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
    switch (key) {
        case 'f':
            ofToggleFullscreen();
            break;
            
        default:
            keypressed_robot(key);
            keypressed_camera(key);
            keypressed_gizmo(key);
            break;
    }
    
}

//--------------------------------------------------------------
void ofApp::keypressed_robot(int key){
    switch (key) {
            // 'm' for MOVE!
        case 't':
        case 'T':
            tcp_target.setType(ofxGizmo::OFX_GIZMO_MOVE);
            break;
        case 'g':
        case 'G':
            tcp_target.setType(ofxGizmo::OFX_GIZMO_ROTATE);
            break;
        case 'm':
        case 'M':
            robot_live = !robot_live;
            break;
        case OF_KEY_LEFT:
            offset+=ofVec3f(0, 1, 0);
            robot.setToolOffset(offset);
            break;
        case OF_KEY_RIGHT:
            offset-=ofVec3f(0, 1, 0);
            robot.setToolOffset(offset);
            break;
        case OF_KEY_UP:
            offset+=ofVec3f(1, 0, 0);
            robot.setToolOffset(offset);
            break;
        case OF_KEY_DOWN:
            offset-=ofVec3f(1, 0, 0);
            robot.setToolOffset(offset);
            break;
        case '-':
            
            break;
        case '=':

            break;
    }
}

//--------------------------------------------------------------
void ofApp::keypressed_camera(int key){
    bool val = true;
    switch (key) {
        case 'h':
        case 'H':
            show_gui.set(!show_gui);
            break;
        case 'q':
        case 'Q':
            listener_show_top(val);
            break;
        case 'w':
        case 'W':
            listener_show_front(val);
            break;
        case 'e':
        case 'E':
            listener_show_side(val);
            break;
        case 'r':
        case 'R':
            listener_show_perspective(val);
            break;
    }
}

//--------------------------------------------------------------
void ofApp::keypressed_gizmo(int key){
    bool val = true;
    switch (key) {
        case '1':
            tcp_target.getMatrix().setRotate(initialRot);
            break;
        case '2':
            tcp_target.getMatrix().setRotate(ofQuaternion(1, 0, 0, 0));
            break;
        case '3':
            tcp_target.getMatrix().setRotate(ofQuaternion(0, 0, 1, 1));
            break;
        case '4':
            tcp_target.getMatrix().setRotate(ofQuaternion(0, 1, 0, 1));
            break;
        case '5':
            tcp_target.getMatrix().setRotate(ofQuaternion(1, 1, 1, 1));
            break;
        case '6':
            tcp_target.getMatrix().setRotate(ofQuaternion(0, 1, 1, 1));
            break;
        case '7':
            tcp_target.getMatrix().setRotate(ofQuaternion(1, 0, 1, 1));
            break;
        case '8':
            tcp_target.getMatrix().setRotate(ofQuaternion(1, 0, 0, 0));
            break;
        case '9':
            tcp_target.getMatrix().setRotate(ofQuaternion(1, 1, 0, 0));
            break;
        case '0':
            tcp_target.getMatrix().setRotate(ofQuaternion(1, 1, 1, 0));
            break;
        case OF_KEY_RETURN:
            tcp.setGlobalPosition(robot.desiredPose.forwardPose.getPosition());
            tcp_target.getMatrix().setTranslation(tcp.getPosition());
            break;
        case ' ':
            tcp.setGlobalPosition(robot.getActualTCPNode().getPosition()*1000);
            tcp_target.getMatrix().setTranslation(tcp.getPosition());
            break;
        case '!':
            tcp.setGlobalPosition(robot.getActualTCPNode().getPosition()*1000+ofVec3f(0, 0, 200));
            tcp_target.getMatrix().setTranslation(tcp.getPosition());
            break;
        case '@':
            tcp.setGlobalPosition(robot.getActualTCPNode().getPosition()*1000+ofVec3f(100, 0, 100));
            tcp_target.getMatrix().setTranslation(tcp.getPosition());
            break;
        case '#':
            tcp.setGlobalPosition(ofVec3f(400, 100, 400));
            tcp_target.getMatrix().setTranslation(tcp.getPosition());
            break;
        case '$':
            tcp.setGlobalPosition(ofVec3f(200, 200, 200));
            tcp_target.getMatrix().setTranslation(tcp.getPosition());
            break;
        case '%':
            tcp.setGlobalPosition(robot.getActualTCPNode().getPosition()*1000+ofVec3f(100, 0, 0));
            tcp_target.getMatrix().setTranslation(tcp.getPosition());
            break;
        case '^':
            tcp.setGlobalPosition(ofVec3f(600, 300, 600));
            tcp_target.getMatrix().setTranslation(tcp.getPosition());
            break;
        case '&':
            look_target.getMatrix().setTranslation(0, 0, 1000);
            break;
        case '*':
            look_target.getMatrix().setTranslation(1000, 0, 0);
            break;
        case '(':
            look_target.getMatrix().setTranslation(1000, 1000, 0);
            break;
        case ')':
            look_target.getMatrix().setTranslation(tcp.getPosition()+ofVec3f(10, 0, 0));
            break;
            
    }
    
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}


//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    if (disable_camera()){
        cam.disableMouseInput();
    }
    else{
        cam.enableMouseInput();
    }
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    if (disable_camera()){
        cam.disableMouseInput();
    }
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    // Disable the camera if we're interacting with the GUI
    if (disable_camera()){
        cam.disableMouseInput();
    }
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    //    if (!cam.getMouseInputEnabled())
    cam.enableMouseInput();
    
}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){
    if (disable_camera()){
        cam.disableMouseInput();
    }
    else{
        cam.enableMouseInput();
    }
}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){
    if (disable_camera()){
        cam.disableMouseInput();
    }
    else{
        cam.enableMouseInput();
    }
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    tcp_target.setViewDimensions(w, h);
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 
    
}