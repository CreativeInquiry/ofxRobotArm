#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    model.setup(ofToDataPath("relaxed_ik_core/config/urdfs/irb4600_60_205.urdf"));
   
    sign_correction.assign(6, 1.0);
    offsets.assign(6, 0.0);
    joint_limit_min.assign(6, 0.0);
    joint_limit_max.assign(6, 0.0);
    
    joint_limit_min[0] = -180;
    joint_limit_min[1] = -90;
    joint_limit_min[2] = -180;
    joint_limit_min[3] = -400;
    joint_limit_min[4] = -125;
    joint_limit_min[5] = -400;

    joint_limit_max[0] = 180;
    joint_limit_max[1] = 150;
    joint_limit_max[2] = 75;
    joint_limit_max[3] = 400;
    joint_limit_max[4] = 120;
    joint_limit_max[5] = 400;
    
    hawkinsKelseyKinematics.setup(offsets, sign_correction, joint_limit_min,  joint_limit_max);
    vector<double> params;
    params.push_back(495);
    params.push_back(900);
    params.push_back(175);
    params.push_back(960);
    params.push_back(0.0);
    params.push_back(135);
    hawkinsKelseyKinematics.setParams(params);
    params.clear();
    
    
    offsets[2] = -PI / 2;
   
    sphericalWristKinematics.setup(offsets, sign_correction, joint_limit_min,  joint_limit_max);
    params.push_back(175);
    params.push_back(-175);
    params.push_back(0);
    params.push_back(495);
    params.push_back(900);
    params.push_back(960);
    params.push_back(135);
    sphericalWristKinematics.setParams(params);
    
    vector<double> pose;
    pose.assign(6, 0.0);
    ofMatrix4x4 forwardIK = sphericalWristKinematics.forward(pose);
    mat = forwardIK;
    initialPose.position = mat.getTranslation();
    initialPose.orientation = mat.getRotate();
   
    desiredPose.position = initialPose.position;
    desiredPose.orientation = initialPose.orientation;
    
    mat.setTranslation(mat.getTranslation());
    gizmo.setMatrix(mat);
    
}

//--------------------------------------------------------------
void ofApp::update(){
    mat.setRotate(gizmo.getRotation());
    mat.setTranslation(gizmo.getTranslation());
    desiredPose.position = mat.getTranslation()/1000;
    desiredPose.orientation = mat.getRotate();
    relaxedIKSolver.setPose(desiredPose, initialPose);
    rik_results = relaxedIKSolver.getCurrentPose();
}

//--------------------------------------------------------------
void ofApp::draw(){
    cam.begin();
    for(auto pose : sphericalWristKinematics.inverse(mat)){
        model.setPose(pose);
        model.drawMesh(ofColor::darkMagenta);
        model.drawSkeleton();
        
    }
    
    for(auto pose : hawkinsKelseyKinematics.inverse(mat)){
        model.setPose(pose);
        model.drawMesh(ofColor::blueSteel);
        model.drawSkeleton();
    }
    
    model.setPose(rik_results);
    model.drawMesh(ofColor::darkMagenta);
    model.drawSkeleton();

    
    gizmo.draw(cam);
    cam.end();
    
   
}

//--------------------------------------------------------------
void ofApp::keyPressed(int key){

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
