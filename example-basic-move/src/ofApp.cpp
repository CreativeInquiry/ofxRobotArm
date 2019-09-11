//Copyright (c) 2016, Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

//--------------------------------------------------------------
//
//
// Basic Move Example
//
//
//--------------------------------------------------------------

//
// This example shows you how to:
//  1.  Connect ofxRobotArm to your robot via ethernet
//  2.  Move & reiorient the simulated robot (dragging ofxGizmo)
//  3.  Move & reiorient the real-time robot (use 'm' to enable real-time movement)
//
// Remeber to swap in your robot's ip address in robot.setup() [line 40]
// If you don't know your robot's ip address, you may not be set up yet ...
//  -   Refer to http://www.universal-robots.com/how-tos-and-faqs/how-to/ur-how-tos/ethernet-ip-guide-18712/
//      for a walk-thru to setup you ethernet connection
// See the ReadMe for more tutorial details


#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    ofBackground(0);
    ofSetLogLevel(OF_LOG_VERBOSE);
    camUp = ofVec3f(0, 0, 1);
    
     setupViewports();
    parameters.setup();
    robot.setup("192.168.1.9", parameters, true); // <-- change to your robot's ip address
    
    robot.disableControlJointsExternally();
    safety.setup();
   
    setupGUI();
    positionGUI();
    
    
    sim = 0;
    real = 1;
}

void ofApp::exit(){
    robot.close();
}

//--------------------------------------------------------------
void ofApp::update(){
    moveTCP();
    robot.update();
    updateActiveCamera();
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    ofSetColor(255,160);
    ofDrawBitmapString("OF FPS "+ofToString(ofGetFrameRate()), 30, ofGetWindowHeight()-50);
    ofDrawBitmapString("Robot FPS "+ofToString(robot.robot.getThreadFPS()), 30, ofGetWindowHeight()-65);
    
    
    
    // show realtime robot
    cams[real]->begin(viewportReal);
    ofDrawAxis(100);
    tcpNode.draw();
    robot.drawPreview();
    robot.draw();
    cams[real]->end();
    
    // show simulation robot
    cams[sim]->begin(viewportSim);
    ofDrawAxis(100);
    tcpNode.draw();
    gizmo.draw(*cams[sim]);
    robot.drawPreview();
    cams[sim]->end();
    
    drawGUI();
    
}

void ofApp::moveTCP(){
    
    if(codeGesture) {
        if(!lastCodeGesture) {
            previousTCP = parameters.targetTCP;
        }
        float r = 200;
        float speed = .25 * TWO_PI;
        parameters.targetTCP.position.x = previousTCP.position.x + r * cos(speed * ofGetElapsedTimef())/1000.;
        parameters.targetTCP.position.z = previousTCP.position.z + r * sin(speed * ofGetElapsedTimef())/1000.;
    }
    lastCodeGesture = codeGesture;
    
    // assign the target pose to the current robot pose
    if(parameters.bCopy){
        parameters.bCopy = false;
        
        parameters.bCopy = false;
        parameters.targetTCP.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
        parameters.targetTCP.rotation*=ofQuaternion(90, ofVec3f(1, 0, 0));
        
        // get the robot's position
        parameters.targetTCP.position = parameters.actualTCP.position;
        parameters.targetTCP.rotation*=parameters.actualTCP.rotation;
        
        tcpNode.setPosition(parameters.targetTCP.position*1000);
        tcpNode.setOrientation(parameters.targetTCP.rotation);
        gizmo.setNode(tcpNode);
        // update GUI params
        parameters.targetTCPPosition = parameters.targetTCP.position;
        parameters.targetTCPOrientation = ofVec4f(parameters.targetTCP.rotation.x(), parameters.targetTCP.rotation.y(), parameters.targetTCP.rotation.z(), parameters.targetTCP.rotation.w());
        
    }
    else{
        gizmo.apply(tcpNode);
    }
    
    // follow the gizmo's position and orientation
    if(parameters.bFollow){
        parameters.targetTCP.position.interpolate(tcpNode.getPosition()/1000.0, parameters.followLerp);
        parameters.targetTCP.rotation = tcpNode.getOrientationQuat();
        parameters.targetTCPOrientation = ofVec4f(parameters.targetTCP.rotation.x(), parameters.targetTCP.rotation.y(), parameters.targetTCP.rotation.z(), parameters.targetTCP.rotation.w());
    }
}

void ofApp::setupViewports(){
    
    viewportReal = ofRectangle((21*ofGetWindowWidth()/24)/2, 0, (21*ofGetWindowWidth()/24)/2, 8*ofGetWindowHeight()/8);;
    viewportSim = ofRectangle(0, 0, (21*ofGetWindowWidth()/24)/2, 8*ofGetWindowHeight()/8);
    
    activeCam = real;
    
    gizmo.setViewDimensions(viewportSim.width, viewportSim.height);
    
    for(int i = 0; i < N_CAMERAS; i++){
        cams.push_back(new ofEasyCam());
        savedCamMats.push_back(ofMatrix4x4());
        viewportLabels.push_back("");
    }
    
    cams[real]->reset();
    cams[real]->setPosition(glm::vec3(-1, 1, 2500));
    cams[real]->lookAt(glm::vec3(0, 0, 0), camUp);
    
    cams[sim]->reset();
    cams[sim]->setPosition(glm::vec3(-1, 1, 2500));
    cams[sim]->lookAt(glm::vec3(0, 0, 0), camUp);
    
    cams[real]->begin(viewportReal);
    robot.draw();
    cams[real]->end();
    cams[real]->enableMouseInput();
    
    
    cams[sim]->begin(viewportSim);
    robot.draw();
    cams[sim]->end();
    cams[sim]->enableMouseInput();
    
}

//--------------------------------------------------------------
void ofApp::setupGUI(){
    
    panel.setup(parameters.robotArmParams);
    panel.add(lookAtTCP.set("lookAtTCP", true));
    panel.add(parameters.pathRecorderParams);
    
    panelJoints.setup(parameters.joints);
    panelTargetJoints.setup(parameters.targetJoints);
    panelJointsSpeed.setup(parameters.jointSpeeds);
    panelJointsIK.setup(parameters.jointsIK);
    
    panel.add(robot.movement.movementParams);
    parameters.bMove = false;
    // get the current pose on start up
    parameters.bCopy = true;
    panel.loadFromFile("settings/settings.xml");
    
    // setup Gizmo
    
    tcpNode.setPosition(ofVec3f(500, 500, 500));
    tcpNode.setOrientation(parameters.targetTCP.rotation);
    gizmo.setNode(tcpNode);
    gizmo.setDisplayScale(2.0);
    gizmo.show();
}

void ofApp::positionGUI(){
    //    viewportReal.height -= (panelJoints.getHeight());
    //    viewportReal.y +=(panelJoints.getHeight());
    panel.setPosition(viewportReal.x+viewportReal.width, 10);
    panelJointsSpeed.setPosition(viewportReal.x, 10);
    panelJointsIK.setPosition(panelJointsSpeed.getPosition().x+panelJoints.getWidth(), 10);
    panelTargetJoints.setPosition(panelJointsIK.getPosition().x+panelJoints.getWidth(), 10);
    panelJoints.setPosition(panelTargetJoints.getPosition().x+panelJoints.getWidth(), 10);
    
    panelJoints.add(robot.robotSafety.params);
    panelJoints.add(robot.robotSafety.mCollision->params);
    panelJoints.add(robot.robotSafety.mCylinderRestrictor->params);
    panelJoints.add(robot.robotSafety.m_jointRestrictor->params);
}

//--------------------------------------------------------------
void ofApp::drawGUI(){
    panel.draw();
    panelJoints.draw();
    panelJointsIK.draw();
    panelJointsSpeed.draw();
    panelTargetJoints.draw();
    
    hightlightViewports();
}


//--------------------------------------------------------------
void ofApp::updateActiveCamera(){
    
    if (viewportReal.inside(ofGetMouseX(), ofGetMouseY()))
    {
        activeCam = real;
        if(!cams[real]->getMouseInputEnabled()){
            cams[real]->enableMouseInput();
        }
        if(cams[sim]->getMouseInputEnabled()){
            cams[sim]->disableMouseInput();
        }
    }else{
        if(cams[real]->getMouseInputEnabled()){
            cams[real]->disableMouseInput();
        }
    }
    if(viewportSim.inside(ofGetMouseX(), ofGetMouseY()))
    {
        activeCam = sim;
        if(!cams[sim]->getMouseInputEnabled()){
            cams[sim]->enableMouseInput();
        }
        if(cams[real]->getMouseInputEnabled()){
            cams[real]->disableMouseInput();
        }
        if(gizmo.isInteracting() && cams[sim]->getMouseInputEnabled()){
            cams[sim]->disableMouseInput();
        }
    }else{
        if(cams[sim]->getMouseInputEnabled()){
            cams[sim]->disableMouseInput();
        }
    }
}

//--------------------------------------------------------------
//--------------------------------------------------------------
void ofApp::handleViewportPresets(int key){
    
    float dist = 2500;
    float zOffset = 450;
    
    if(activeCam != -1){
        glm::vec3 pos = lookAtTCP.get()==true?glm::vec3(0, 0, 0):tcpNode.getPosition();
        // TOP VIEW
        if (key == '1'){
            glm::vec3 offset = glm::vec3(-1, 1, dist);
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(pos+offset);
            cams[activeCam]->lookAt(lookAtTCP.get()==true?tcpNode.getPosition():glm::vec3(0, 0, 0), camUp);
            viewportLabels[activeCam] = "TOP VIEW";
            return;
        }
        // LEFT VIEW
        if (key == '2'){
            glm::vec3 offset = glm::vec3(dist, 1, 1);
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(pos+offset);
            cams[activeCam]->lookAt(lookAtTCP.get()==true?tcpNode.getPosition():glm::vec3(0, 0, 0), camUp);
            viewportLabels[activeCam] = "LEFT VIEW";
            return;
        }
        // FRONT VIEW
        if (key == '3'){
            glm::vec3 offset = glm::vec3(1, dist, 1);
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(pos+offset);
            cams[activeCam]->lookAt(lookAtTCP.get()==true?tcpNode.getPosition():glm::vec3(0, 0, 0), camUp);
            viewportLabels[activeCam] = "FRONT VIEW";
            return;
        }
        // PERSPECTIVE VIEW
        if (key == '4'){
            glm::vec3 offset = glm::vec3(dist/4, dist, dist/4);
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(pos+offset);
            cams[activeCam]->lookAt(lookAtTCP.get()==true?tcpNode.getPosition():glm::vec3(0, 0, 0), camUp);
            viewportLabels[activeCam] = "PERSPECTIVE VIEW";
            return;
        }
        if (key == '5'){
            glm::vec3 offset = glm::vec3(-dist/4, dist, -dist/4);
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(pos+offset);
            cams[activeCam]->lookAt(lookAtTCP.get()==true?tcpNode.getPosition():glm::vec3(0, 0, 0), camUp);
            viewportLabels[activeCam] = "PERSPECTIVE VIEW";
            return;
        }
    }
}


//--------------------------------------------------------------
void ofApp::hightlightViewports(){
    ofPushStyle();
    
    // highlight right viewport
    if (activeCam == 0){
        
        ofSetLineWidth(6);
        ofSetColor(ofColor::white,30);
        ofDrawRectangle(viewportReal);
        
    }
    // hightlight left viewport
    else{
        ofSetLineWidth(6);
        ofSetColor(ofColor::white,30);
        ofDrawRectangle(viewportSim);
    }
    
    // show Viewport info
    ofSetColor(ofColor::white,200);
    ofDrawBitmapString(viewportLabels[0], viewportReal.x+viewportReal.width-120, ofGetWindowHeight()-30);
    ofDrawBitmapString("REALTIME", ofGetWindowWidth()/2 - 90, ofGetWindowHeight()-30);
    ofDrawBitmapString(viewportLabels[1], viewportSim.x+viewportSim.width-120, ofGetWindowHeight()-30);
    ofDrawBitmapString("SIMULATED", 30, ofGetWindowHeight()-30);
    
    ofPopStyle();
}


//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
    if(key == 'm'){
        parameters.bMove = !parameters.bMove;
    }
    
    if( key == 'r' ) {
        gizmo.setType( ofxGizmo::OFX_GIZMO_ROTATE );
    }
    if( key == 'g' ) {
        gizmo.setType( ofxGizmo::OFX_GIZMO_MOVE );
    }
    if( key == 's' ) {
        gizmo.setType( ofxGizmo::OFX_GIZMO_SCALE );
    }
    if( key == 'e' ) {
        gizmo.toggleVisible();
    }
    
    handleViewportPresets(key);
}

//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    viewportLabels[activeCam] = "";
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
    setupViewports();
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 
    
}
