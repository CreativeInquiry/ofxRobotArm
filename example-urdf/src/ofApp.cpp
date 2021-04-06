#include "ofApp.h"

using namespace ofxRobotArm;
//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(120);
    // setup scene
    setup_scene();
    ur5.setup("192.168.0.1", (string)"relaxed_ik_core/config/urdfs/ur5.urdf", ofxRobotArm::UR5);
    irb120.setup("192.168.0.1", (string)"relaxed_ik_core/config/urdfs/irb120.urdf", ofxRobotArm::IRB120);

    // setup robot
    // ur5.setup(robotParams);    // change IP string to your robot's IP address
    
    // setup gui
    setup_gui();
    
    // start robot
    ur5.start();
    irb120.start();
    
    ur5.setToolOffset(offset);
    
    tcp = irb120.getActualTCPNode();
    tcp.setPosition(tcp.getPosition()*1000);
    initialRot = ofQuaternion(ofVec4f(0, 0.7, 0, 0.7));
    tcp.setOrientation(initialRot);
    tcp_target.setNode(tcp);

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
    
    for(int i = 0 ; i < 360; i++){
        line.addVertex(ofVec3f(425, 0, 150)+ofVec3f(((i+1)/360)*150, 0.6*i*sin(ofDegToRad(i)), i*cos(ofDegToRad(i))));
    }
    line.close();

    FOLLOW_MODE = FOLLOW_GIZMO;
}



//--------------------------------------------------------------
void ofApp::update(){
    ofQuaternion q = tcp_target.getRotation();
    rot = ofVec4f(q.x(), q.y(), q.z(), q.w());
    

    if(FOLLOW_MODE == FOLLOW_GIZMO)
    {
        tcp.setPosition(tcp_target.getTranslation());
        ofQuaternion q = tcp_target.getRotation();
        tcp.setOrientation(q);
    }
    else if(FOLLOW_MODE == LOOK_AT_TARGET)
    {
        ofVec3f p = look_target.getTranslation();
        ofMatrix4x4 mat, mat2;
        mat.makeLookAtMatrix(p, tcp_target.getTranslation(), ofVec3f(0, 1, 0));
        tcp.setPosition(tcp_target.getTranslation());
        tcp.setOrientation(mat.getRotate());
    }
    else if(FOLLOW_MODE == FOLLOW_CIRCLE)
    {
        t+=feedSpeed*1/60;
        if(t>1)
        {
            t = 0;
        }
        ofVec3f p = line.getPointAtPercent(t);
        tcp.setPosition(p);
        tcp.setOrientation(tcp_target.getRotation());
    }
    // ur5.setToolOffset(offset);
    // ur5.setDesired(tcp);
    // ur5.update();

    irb120.update();
    irb120.setToolOffset(offset);
    irb120.setDesired(tcp);
    irb120.update();
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
    if (ur5.isLive()){
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
    // ur5.draw(ofColor::red);
    irb120.draw(ofColor::red);
    // Draw Desired Robot
    // ur5.drawDesired(ofColor::whiteSmoke);
    irb120.drawDesired(ofColor::whiteSmoke);

    ofPushStyle();
    ofSetColor(ofColor::aqua);
    line.draw();
    ofPopStyle();
    
    look_target.draw(cam);
    tcp_target.draw(cam);

    cam.end();
}

//--------------------------------------------------------------
bool ofApp::disable_camera(){
    
    if (tcp_target.isInteracting() || look_target.isInteracting())
        return true;
    
    ofRectangle gui_rect;
    gui_rect.setX(panel.getPosition().x-120);
    gui_rect.setY(panel.getPosition().y);
    gui_rect.setWidth(panel.getWidth() + 220);
    gui_rect.setHeight(panel.getHeight() + panel_robot.getHeight() + panel_joints.getHeight()+120);
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
    
    params.add(offset.set("offset", ofVec3f(0, 0, 0), ofVec3f(-300, -300, -300), ofVec3f(300, 300, 300)));
    
    show_top.addListener(this, &ofApp::listener_show_top);
    show_front.addListener(this, &ofApp::listener_show_front);
    show_side.addListener(this, &ofApp::listener_show_side);
    show_perspective.addListener(this, &ofApp::listener_show_perspective);
    
    panel.setup(params);
    panel.setPosition(10, 10);
    
    panel_robot.setup("Robot_Controller");
    panel_robot.setPosition(panel.getPosition().x, panel.getPosition().y + panel.getHeight() + 5);

    panel_robot.add(rot.set("TCP ROT", ofVec4f(0, 0, 0, 0)));
    panel_robot.add(feedSpeed.set("Feed Speed", 0.0001, 0.0001, 0.5));
    panel_robot.add(FOLLOW_MODE.set("Follow Mode", 1, 1, 3));
    panel_robot.add(ur5.robotArmParams);
    panel_robot.add(irb120.robotArmParams);


    panel_joints.setup("Joints");
    panel_joints.add(ur5.joints);
    panel_joints.setPosition(panel_robot.getPosition().x, panel_robot.getPosition().y + panel_robot.getHeight() + 5);
    
    ofSetCircleResolution(60);
}

//--------------------------------------------------------------
void ofApp::draw_gui(){
    panel.draw();
    panel_robot.draw();
    panel_joints.draw();
    
    ofDrawBitmapStringHighlight("FPS: "+ofToString(ofGetFrameRate()), ofGetWidth()-100, 10);
//    ofDrawBitmapStringHighlight("SFF: "+ofToString(ur5.inverseKinematics.relaxedIK.getFrame()), ofGetWidth()-100, 40);
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
        int z = 1700;
        
        
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
        
        int x = 1700;
        int y = 0;
        int z = 400;
        
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
        
        int x = 600;
        int y = -1700;
        int z = 400;
        
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
        
        int x = 800;
        int y = -1700;
        int z = 600;
        
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
            ur5.toggleLive();
            break;
        case OF_KEY_LEFT:
            offset+=ofVec3f(0, 1, 0);
            ur5.setToolOffset(offset);
            break;
        case OF_KEY_RIGHT:
            offset-=ofVec3f(0, 1, 0);
            ur5.setToolOffset(offset);
            break;
        case OF_KEY_UP:
            offset+=ofVec3f(1, 0, 0);
            ur5.setToolOffset(offset);
            break;
        case OF_KEY_DOWN:
            offset-=ofVec3f(1, 0, 0);
            ur5.setToolOffset(offset);
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
            FOLLOW_MODE = FOLLOW_GIZMO;
            break;
        case '2':
            FOLLOW_MODE = LOOK_AT_TARGET;
            break;
        case '3':
            FOLLOW_MODE = FOLLOW_CIRCLE;
            break;
        case '4':
            tcp.setOrientation(initialRot);
            tcp_target.getMatrix().setRotate(initialRot);
        case '5':
            break;
        case '6':
            break;
        case '7':
            break;
        case '8':
            break;
        case '9':
            break;
        case '0':
            break;
        case OF_KEY_RETURN:
            tcp.setGlobalPosition(ur5.desiredModel.forwardPose.getPosition());
            tcp_target.getMatrix().setTranslation(tcp.getPosition());
            break;
        case ' ':
            tcp.setGlobalPosition(ur5.getActualTCPNode().getPosition()*1000);
            tcp_target.getMatrix().setTranslation(tcp.getPosition());
            break;
        case '!':
            tcp.setGlobalPosition(ur5.getActualTCPNode().getPosition()*1000+ofVec3f(0, 0, 200));
            tcp_target.getMatrix().setTranslation(tcp.getPosition());
            break;
        case '@':
            tcp.setGlobalPosition(ur5.getActualTCPNode().getPosition()*1000+ofVec3f(100, 0, 100));
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
            tcp.setGlobalPosition(ur5.getActualTCPNode().getPosition()*1000+ofVec3f(100, 0, 0));
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

void ofApp::mouseScrolled(int x, int y, float scrollX, float scrollY ){
    if (disable_camera()){
        cam.disableMouseInput();
    }
    // ofLog(OF_LOG_NOTICE)<<"CAM "<<cam.getPosition()<<" "<<cam.getOrientationQuat()<<endl;
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    if (!cam.getMouseInputEnabled())
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
    look_target.setViewDimensions(w, h);
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 
    
}
