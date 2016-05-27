//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

//--------------------------------------------------------------
//
//
// Follow Motion Capture Path Example
//
//
//--------------------------------------------------------------

//
// This example shows you how to:
//  1.  Create 3D Paths using an Optitrack motion capture markers
//  2.  Move & reiorient 3D Paths using motion capture markers
//  3.  Move & reiorient the robot to follow paths using a PathController
//  4.  Move & reiorient the robot to follow a rigid body
//
// See the ReadMe for more tutorial details
//
// TO DO:
//      - Convert Mocap recorded path to Path3D


#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){

    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    ofBackground(0);
    ofSetLogLevel(OF_LOG_SILENT);
    
    setupViewports();
    
    parameters.setup();
    robot.setup("192.168.1.9",parameters); // <-- swap with your robot's ip address
    
    setupGUI();
    positionGUI();
    
    
    // setup mocap
    string localIP  = "192.168.1.140";
    string serverIP = "192.168.1.10";
    setupMocap(localIP,serverIP,1);
}

//--------------------------------------------------------------
void ofApp::update(){
    
    if (paths.size() > 0){
        paths.update();
    }
    
    updateMocap();
    
    moveArm();
    robot.update();
    
    updateActiveCamera();
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    ofSetColor(255,160);
    ofDrawBitmapString("OF FPS "+ofToString(ofGetFrameRate()), 30, ofGetWindowHeight()-50);
    ofDrawBitmapString("Robot FPS "+ofToString(robot.robot.getThreadFPS()), 30, ofGetWindowHeight()-65);
    
    gizmo.setViewDimensions(viewportSim.width, viewportSim.height);
    
    // show realtime robot
    cams[0]->begin(viewportReal);
    tcpNode.draw();
    paths.draw();
    robot.robot.model.draw();
    drawMocap();
    cams[0]->end();
    
    // show simulation robot
    cams[1]->begin(viewportSim);
    if (parameters.bFollow)
        gizmo.draw(*cams[1]);
    paths.draw();
    robot.movement.draw(0);
    drawMocap();
    cams[1]->end();
    
    drawGUI();
    
}

void ofApp::setupMocap(string myIP, string serverIP){
    mocap.setup(myIP, serverIP);  // interface name, server ip
    mocap.setScale(1);
    mocap.setDuplicatedPointRemovalDistance(.020);
}

void ofApp::setupMocap(string myIP, string serverIP, int scale){
    mocap.setup(myIP, serverIP);  // interface name, server ip
    mocap.setScale(scale);
    mocap.setDuplicatedPointRemovalDistance(.020);
    
}
void ofApp::updateMocap(){
    mocap.update();
    
    if (mocap.getNumRigidBody() > 0){
        const ofxNatNet::RigidBody &rb = mocap.getRigidBodyAt(0);  // more than one rigid body crashes ofxNatNet now
        
        // add to the rigid body history
        if (record){
            recordedPath.push_back(rb);
            
        }
        
        currentRB = rb;
        

        if (attach){
            keepAttached(currentRB, recordedPath);
        }
    }
    
}

void ofApp::drawMocap(){
    
    ofPushMatrix();
    ofScale(1000);
    
    // show path
    ofPolyline tp;
    for (auto &path: recordedPath){
        ofSetColor(ofColor::azure);
        tp.addVertex(path.getMatrix().getTranslation());
    }
    tp.draw();

    // show rigid body
    ofPolyline bodies;
    float alpha = 255;
    float step = 255 / (recordedPath.size()+1);
    for (auto &rb: recordedPath){
        ofSetColor(ofColor::navajoWhite, alpha);
        for (int i = 0; i < rb.markers.size(); i++)
            bodies.addVertex(rb.markers[i]);
        alpha -= step;
    }
    bodies.draw();
    
    
    
    // draw rigidbodies

    const ofxNatNet::RigidBody &RB = currentRB;
    
    if (RB.isActive())
        ofSetColor(0, 255, 0);
    else
        ofSetColor(255, 0, 0);
    
    //        cout << "rigidBody: " << RB.id << ", pos: " << RB.matrix.getTranslation().operator*=(10) << ", orientation: "<< RB.getMatrix().getRotate() << endl;
    
    ofPushMatrix();
    glMultMatrixf(RB.getMatrix().getPtr());
    ofDrawAxis(.030);
    ofPopMatrix();
    
    glBegin(GL_LINE_LOOP);
    for (int n = 0; n < RB.markers.size(); n++) {
        glVertex3fv(RB.markers[n].getPtr());
    }
    glEnd();
    
    for (int n = 0; n < RB.markers.size(); n++) {
        ofDrawBox(RB.markers[n], .005);
    }
    

    ofPopMatrix();
}


void ofApp::keepAttached(ofxNatNet::RigidBody &currRB, vector<ofxNatNet::RigidBody> &recordedBodies){

    // get the difference between the current rigid body and the previous rigid body in the recorded path
    ofMatrix4x4 diff = recordedBodies[recordedBodies.size()-1].matrix.getInverse() * currRB.matrix;
    
    for (auto &rb : recordedBodies){
        
        // update its transformation matrix
        rb.matrix *= diff;
        
        // update its markers
        for (auto &marker : rb.markers){
            marker = marker * diff;
        }
    }
    
}

void ofApp::moveArm(){
    
    // assign the target pose to the current robot pose
    if(parameters.bCopy){
        parameters.bCopy = false;
        parameters.targetTCP.rotation = ofQuaternion(90, ofVec3f(0, 0, 1));
        parameters.targetTCP.rotation*=ofQuaternion(90, ofVec3f(1, 0, 0));
        
        // get the robot's position
        parameters.targetTCP.position = parameters.actualTCP.position;
        parameters.targetTCP.rotation*=parameters.actualTCP.rotation;
        
        // update the gizmo controller
        tcpNode.setPosition(parameters.targetTCP.position*1000);
        tcpNode.setOrientation(parameters.targetTCP.rotation);
        gizmo.setNode(tcpNode);
        
        // update GUI params
        parameters.targetTCPPosition = parameters.targetTCP.position;
        parameters.targetTCPOrientation = ofVec4f(parameters.targetTCP.rotation.x(), parameters.targetTCP.rotation.y(), parameters.targetTCP.rotation.z(), parameters.targetTCP.rotation.w());
        
    }
    else if (!followRigidBody){
        // update the tool tcp
        tcpNode.setTransformMatrix(gizmo.getMatrix());
    }
    
    // update based on rigid body position and orientation
    if (followRigidBody){

        tcpNode.setPosition(currentRB.matrix.getTranslation().x*1000, currentRB.matrix.getTranslation().y*1000, currentRB.matrix.getTranslation().z*1000);
        
        
        // rotate quaternion so that robot aligns to rb's z-axis instead of x-axis
        ofQuaternion mocapOrient = currentRB.matrix.getRotate();
        mocapOrient *= mocapOrient.conj();
        mocapOrient.makeRotate(90, 0, 1, 0);
        mocapOrient *= currentRB.matrix.getRotate();
        
        tcpNode.setOrientation(mocapOrient);
        
        // update gizmo
        gizmo.setNode(tcpNode);
    }
    
    // follow a user-defined position and orientation
    if(parameters.bFollow){
        parameters.targetTCP.position.interpolate(tcpNode.getPosition()/1000.0, parameters.followLerp);
        parameters.targetTCP.rotation = tcpNode.getOrientationQuat();
        parameters.targetTCPOrientation = ofVec4f(parameters.targetTCP.rotation.x(), parameters.targetTCP.rotation.y(), parameters.targetTCP.rotation.z(), parameters.targetTCP.rotation.w());
        
    }
    
    if (followPath && !followRigidBody && !paths.pause){
        paths.getNextPose();
 
        ofMatrix4x4 m44 = recordedPath[paths.paths[0]->ptIndex].matrix;
        
         tcpNode.setPosition(m44.getTranslation().x*1000, m44.getTranslation().y*1000, m44.getTranslation().z*1000);
        
        // rotate quaternion so that robot aligns to rb's z-axis instead of x-axis
        ofQuaternion m44Orient = m44.getRotate();
        m44Orient *= m44Orient.conj();
        m44Orient.makeRotate(90, 0, 1, 0);
        m44Orient *= m44.getRotate();

        tcpNode.setOrientation(m44Orient);
        
        parameters.targetTCP.position.interpolate(tcpNode.getPosition()/1000.0, parameters.followLerp);
        parameters.targetTCP.rotation = tcpNode.getOrientationQuat();
        parameters.targetTCPOrientation = ofVec4f(parameters.targetTCP.rotation.x(), parameters.targetTCP.rotation.y(), parameters.targetTCP.rotation.z(), parameters.targetTCP.rotation.w());

    }
    
    
    
}

void ofApp::setupViewports(){
    
    viewportReal = ofRectangle((21*ofGetWindowWidth()/24)/2, 0, (21*ofGetWindowWidth()/24)/2, 8*ofGetWindowHeight()/8);
    viewportSim = ofRectangle(0, 0, (21*ofGetWindowWidth()/24)/2, 8*ofGetWindowHeight()/8);
    
    activeCam = 0;
    
    
    for(int i = 0; i < N_CAMERAS; i++){
        cams.push_back(new ofEasyCam());
        savedCamMats.push_back(ofMatrix4x4());
        viewportLabels.push_back("");
    }
    
    cams[0]->begin(viewportReal);
    cams[0]->end();
    cams[0]->enableMouseInput();
    
    
    cams[1]->begin(viewportSim);
    cams[1]->end();
    cams[1]->enableMouseInput();
    
}

//--------------------------------------------------------------
void ofApp::setupGUI(){
    
    panel.setup(parameters.robotArmParams);
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
    gizmo.setDisplayScale(1.0);
    tcpNode.setPosition(ofVec3f(0.5, 0.5, 0.5)*1000);
    tcpNode.setOrientation(parameters.targetTCP.rotation);
    gizmo.setNode(tcpNode);
}

void ofApp::positionGUI(){
    viewportReal.height -= (panelJoints.getHeight());
    viewportReal.y +=(panelJoints.getHeight());
    panel.setPosition(viewportReal.x+viewportReal.width, 10);
    panelJointsSpeed.setPosition(viewportReal.x, 10);
    panelJointsIK.setPosition(panelJointsSpeed.getPosition().x+panelJoints.getWidth(), 10);
    panelTargetJoints.setPosition(panelJointsIK.getPosition().x+panelJoints.getWidth(), 10);
    panelJoints.setPosition(panelTargetJoints.getPosition().x+panelJoints.getWidth(), 10);
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
        activeCam = 0;
        if(!cams[0]->getMouseInputEnabled()){
            cams[0]->enableMouseInput();
        }
        if(cams[1]->getMouseInputEnabled()){
            cams[1]->disableMouseInput();
        }
    }
    if(viewportSim.inside(ofGetMouseX(), ofGetMouseY()))
    {
        activeCam = 1;
        if(!cams[1]->getMouseInputEnabled()){
            cams[1]->enableMouseInput();
        }
        if(cams[0]->getMouseInputEnabled()){
            cams[0]->disableMouseInput();
        }
        if(gizmo.isInteracting() && cams[1]->getMouseInputEnabled()){
            cams[1]->disableMouseInput();
        }
    }
}

//--------------------------------------------------------------
//--------------------------------------------------------------
void ofApp::handleViewportPresets(int key){
    
    float dist = 2000;
    float zOffset = 450;
    
    if(activeCam != -1){
        // TOP VIEW
        if (key == '1'){
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(0, 0, dist);
            cams[activeCam]->lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
            viewportLabels[activeCam] = "TOP VIEW";
        }
        // LEFT VIEW
        else if (key == '2'){
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(dist, 0, 0);
            cams[activeCam]->lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
            viewportLabels[activeCam] = "LEFT VIEW";
        }
        // FRONT VIEW
        else if (key == '3'){
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(0, dist, 0);
            cams[activeCam]->lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
            viewportLabels[activeCam] = "FRONT VIEW";
        }
        // PERSPECTIVE VIEW
        else if (key == '4'){
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(dist, dist, dist/4);
            cams[activeCam]->lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
            viewportLabels[activeCam] = "PERSPECTIVE VIEW";
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
        
//        if (parameters.bMove)
//            paths.startDrawing();
//        else
//            paths.pauseDrawing();
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
    
    // mocap key pressed
    if (key == 'A'){
        attach = !attach;
        if (attach)
            record = false;
    }
    if (key == 'R'){
        record = !record;
//        if (record)
//            attach = false;
        if (!record){
//            attach = true;
            ofPolyline pl;
            vector<ofMatrix4x4> m44;
            for (auto &rb : recordedPath){
                pl.addVertex(ofPoint(rb.getMatrix().getTranslation()));
                m44.push_back(rb.getMatrix());
            }
            path.set(pl);
            
            paths.addPath(&path);
        }
    }
    
    if (key == 'F'){
        followRigidBody = !followRigidBody;
    }
    
    if (key == ' '){
        followPath = !followPath;
    }
    if (key == 'C')
        recordedPath.clear();
    
//    paths.keyPressed(key);

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

}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){

}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){ 

}
