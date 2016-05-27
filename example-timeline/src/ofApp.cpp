//Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.

#include "ofApp.h"
#include "URUtils.h"

//--------------------------------------------------------------
void ofApp::setup(){
    ofSetFrameRate(60);
    ofSetVerticalSync(true);
    ofBackground(0);
    ofSetLogLevel(OF_LOG_NOTICE);
    setupViewports();
    
    parameters.setup();
    robot.setup(parameters);
    setupGUI();
    setupTimeline();
    positionGUI();
}

void ofApp::setupViewports(){
    
    viewportReal = ofRectangle((21*ofGetWindowWidth()/24)/2, 0, (21*ofGetWindowWidth()/24)/2, 7*ofGetWindowHeight()/8);
    viewportSim = ofRectangle(0, 0, (21*ofGetWindowWidth()/24)/2, 7*ofGetWindowHeight()/8);
    
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

void ofApp::setupTimeline(){
    
    
    gizmo.setDisplayScale(1.0);
    tcpNode.setPosition(ofVec3f(0.5, 0.5, 0.5)*1000);
    tcpNode.setOrientation(parameters.targetTCP.rotation);
    gizmo.setNode( tcpNode);
    
    ofDirectory dir;
    string dirName = "timeline/saves/"+ofGetTimestampString();
    dir.createDirectory(ofToDataPath(dirName));
    
    timeline.setup();
    
    timeline.setFrameRate(60);
    timeline.setDurationInFrames(timeline.getFrameRate()*30);
    timeline.setLoopType(OF_LOOP_NORMAL);
    
    nodeTrack = new ofxTLNodeTrack();
    nodeTrack->setNode(tcpNode);
    nodeTrack->setTimeline(&timeline);
    nodeTrack->setXMLFileName(dirName+"/_keyframes.xml");
    timeline.addTrack("TargetTCP", nodeTrack);
    timeline.setWorkingFolder(dirName);
    timeline.setFrameBased(false);
    
    nodeTrack->lockNodeToTrack = true;
}

void ofApp::setupGUI(){
    
    panel.setup(parameters.robotArmParams);
    panel.add(parameters.pathRecorderParams);
    
    panelJoints.setup(parameters.joints);
    panelTargetJoints.setup(parameters.targetJoints);
    panelJointsSpeed.setup(parameters.jointSpeeds);
    panelJointsIK.setup(parameters.jointsIK);
    
    panel.add(robot.movement.movementParams);
    speeds.assign(6, 0);
    parameters.bMove = false;
    // get the current pose on start up
    parameters.bCopy = true;
    panel.loadFromFile("settings/settings.xml");
}

void ofApp::positionGUI(){
    viewportReal.height -= (panelJoints.getHeight());
    viewportReal.y +=(panelJoints.getHeight());
    panel.setPosition(viewportReal.x+viewportReal.width, 10);
    panelJointsSpeed.setPosition(viewportReal.x, 10);
    panelJointsIK.setPosition(panelJointsSpeed.getPosition().x+panelJoints.getWidth(), 10);
    panelTargetJoints.setPosition(panelJointsIK.getPosition().x+panelJoints.getWidth(), 10);
    panelJoints.setPosition(panelTargetJoints.getPosition().x+panelJoints.getWidth(), 10);
    timeline.setOffset(ofVec2f(viewportSim.x, viewportSim.y+viewportSim.height));
    timeline.setWidth(viewportReal.width+viewportSim.width);
    timeline.setHeight(2*(ofGetWindowHeight()-viewportSim.height)/3);
}

//--------------------------------------------------------------
void ofApp::update(){
    if(nodeTrack->lockNodeToTrack && !parameters.bCopy){
        gizmo.setNode(tcpNode);
    }else{
        tcpNode.setTransformMatrix( gizmo.getMatrix() );
    }
    moveArm();
    robot.update();
    
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

void ofApp::moveArm(){
    
    // assign the target pose to the current robot pose
    if(parameters.bCopy){
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
    // follow a user-defined position and orientation
    if(parameters.bFollow){
        
        parameters.targetTCP.position.interpolate(tcpNode.getPosition()/1000.0, parameters.followLerp);
        parameters.targetTCP.rotation = tcpNode.getOrientationQuat();
        parameters.targetTCPOrientation = ofVec4f(parameters.targetTCP.rotation.x(), parameters.targetTCP.rotation.y(), parameters.targetTCP.rotation.z(), parameters.targetTCP.rotation.w());
        
    }
}


//--------------------------------------------------------------
void ofApp::draw(){
    ofEnableAlphaBlending();
    
    ofSetColor(255,160);
    ofDrawBitmapString("OF FPS "+ofToString(ofGetFrameRate()), 30, ofGetWindowHeight()-50);
    ofDrawBitmapString("Robot FPS "+ofToString(robot.robot.getThreadFPS()), 30, ofGetWindowHeight()-65);
    gizmo.setViewDimensions(viewportSim.width, viewportSim.height);
    
    
    cams[1]->begin(viewportSim);
    gizmo.draw( *cams[1] );
    robot.movement.draw(0);
    cams[1]->end();
    
    cams[0]->begin(viewportReal);
    tcpNode.draw();
    
    if (!hideRobot){
        robot.robot.model.draw();
    }
    cams[0]->end();
    
    
    
    
    timeline.draw();
    
    panel.draw();
    panelJoints.draw();
    panelJointsIK.draw();
    panelWorkSurface.draw();
    panelJointsSpeed.draw();
    panelTargetJoints.draw();
    
    //    syphon.publishScreen();
}

void ofApp::exit(){
    parameters.bMove = false;
    panel.saveToFile("settings/settings.xml");
    robot.close();
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    if(key == 'L'){
        nodeTrack->lockNodeToTrack = !nodeTrack->lockNodeToTrack;
    }
    else if(key == 'T'){
        nodeTrack->addKeyframe();
    }
    if(key == 'm'){
        parameters.bMove = !parameters.bMove;
    }
    
    if(key == '8'){
        parameters.bFigure8 = !parameters.bFigure8;
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
    
    if (key == 'h'){
        hideRobot = !hideRobot;
    }
    
    handleViewportPresets(key);
}

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
            //        cams[activeCam]->movedManually();
            viewportLabels[activeCam] = "TOP VIEW";
        }
        // LEFT VIEW
        else if (key == '2'){
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(dist, 0, 0);
            cams[activeCam]->lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
            //        cams[activeCam]->movedManually();
            viewportLabels[activeCam] = "LEFT VIEW";
        }
        // FRONT VIEW
        else if (key == '3'){
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(0, dist, 0);
            cams[activeCam]->lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
            //        cams[activeCam]->movedManually();
            viewportLabels[activeCam] = "FRONT VIEW";
        }
        // PERSPECTIVE VIEW
        else if (key == '4'){
            cams[activeCam]->reset();
            cams[activeCam]->setPosition(dist, dist, dist/4);
            cams[activeCam]->lookAt(ofVec3f(0, 0, 0), ofVec3f(0, 0, 1));
            //        cams[activeCam]->movedManually();
            viewportLabels[activeCam] = "PERSPECTIVE VIEW";
        }
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
    setupViewports();
    positionGUI();
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    parameters.bMove = false;
    if(dragInfo.files.size() > 0){
        ofFile file;
        file.open(dragInfo.files[0]);
        if(file.isDirectory()){
            nodeTrack->clear();
            timeline.clear();
            ofDirectory dir;
            dir.listDir(dragInfo.files[0]);
            dir.allowExt(".xml");
            timeline.loadTracksFromFolder(dragInfo.files[0]);
            for(int i = 0; i < dir.size(); i++){
                if(dir.getName(i) == "_keyframes.xml"){
                    nodeTrack->loadFromXMLRepresentation(dir.getPath(i));
                    nodeTrack->setXMLFileName(dir.getPath(i));
                    
                }
            }
            timeline.setWorkingFolder(dragInfo.files[0]);
        }
    }
}
