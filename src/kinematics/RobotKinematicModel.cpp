//
//  KinectModel.cpp
//  urModernDriverTest
//
//  Created by dantheman on 2/20/16.
// Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//

#include "RobotKinematicModel.h"
using namespace ofxRobotArm;
RobotKinematicModel::RobotKinematicModel(){
    
}
RobotKinematicModel::~RobotKinematicModel(){

}

// D-H Parameters for UR Robot Arms:
// https://www.universal-robots.com/articles/ur/parameters-for-calculations-of-kinematics-and-dynamics/
void RobotKinematicModel::setup(RobotType type){
    
    joints.resize(6);
    nodes.resize(6);
    vector<Joint> foojoints;
    foojoints.resize(6);
    
    if (type == RobotType::UR5){
        
        if(loader.loadModel(ofToDataPath("models/ur5.dae"))){
            for(int i = 0; i < loader.getNumMeshes(); i++){
                meshs.push_back(loader.getMesh(i));
            }
        }else{
            ofLogFatalError()<<"PLEASE PLACE THE 3D FILES OF THE UR ARM IN data/models/ur5.dae"<<endl;
        }
        
        
        joints[0].position.set(0, 0, 0);
        joints[1].position.set(0, -0.072238, 0.083204);
        joints[2].position.set(0, -0.077537,0.51141);
        joints[3].position.set(0, -0.070608, 0.903192);
        joints[4].position.set(0, -0.117242, 0.950973);
        joints[5].position.set(0, -0.164751, 0.996802);
    }
    else if (type == RobotType::UR10){
        
        // should load model from addon data path, not app
        if(loader.loadModel(ofToDataPath("models/ur10.dae"))){
            for(int i = 0; i < loader.getNumMeshes(); i++){
                meshs.push_back(loader.getMesh(i));
            }
        }else{
            ofLogFatalError()<<"PLEASE PLACE THE 3D FILES OF THE UR ARM IN data/models/ur10.dae"<<endl;
        }

        joints[0].position.set(0, 0, 0);
        
        // These are correct joint positions for the non-home D-H position.
        // Reference: https://asd.sutd.edu.sg/dfab/a-geometric-inverse-kinematics-solution-for-the-universal-robot/
//        joints[1].position.set(-.1273,  0, -.086);          //  -1.273,0.000,-0.860
//        joints[2].position.set(-.7393,  0, -.1163);         // -7.393,0.000,-1.163
//        joints[3].position.set(-1.3116, 0, .1094);          // -13.116,0.000,-1.094
//        joints[4].position.set(-1.3733, 0, -0.16395);       // -13.733,0.000,-1.639
//        joints[5].position.set(-1.4273, 0, -0.2561);        // -14.273,0.000,-2.561

        // These are correct joint positions for the HOME position (0 rotations on each joint).
        joints[1].position.set(0, -.086, .1273);          //(-.1273,0.000,-0.0860);   //
        joints[2].position.set(0, -.1163, .7393);         //(-.7393,0.000,-.1163);    //
        joints[3].position.set(0, -.1094, 1.3116);          //(-1.3116,0.000,-.1094);   //
        joints[4].position.set(0, -0.16395, 1.3733);       //(-1.3733,0.000,-.1639);   //
        joints[5].position.set(0, -0.22535, 1.4273);        //(-1.4273,0.000,-.2561);   //
    }


    tool.position.set(joints[5].position + ofVec3f(0,-0.0308,0)); // flange position
    
    for(int i = 1; i <joints.size(); i++){
        joints[i].offset =joints[i].position-joints[i-1].position;
        
    }
    tool.offset = joints[5].offset;
    
    // Setup the joint axes
    joints[0].axis.set(0, 0, 1);
    joints[1].axis.set(0, -1, 0);
    joints[2].axis.set(0, -1, 0);
    joints[3].axis.set(0, -1, 0);
    joints[4].axis.set(0, 0, 1);
    joints[5].axis.set(0, 1, 0);

    tool.axis.set(joints[5].axis);
    
    joints[0].rotation.makeRotate(0,joints[0].axis);
    joints[1].rotation.makeRotate(-90,joints[1].axis);
    joints[2].rotation.makeRotate(0,joints[2].axis);
    joints[3].rotation.makeRotate(-90,joints[3].axis);
    joints[4].rotation.makeRotate(0,joints[4].axis);
    joints[5].rotation.makeRotate(0,joints[5].axis);
    
    // Rig Joints
    nodes[0].setPosition(joints[0].position);
    nodes[0].setOrientation(joints[0].rotation);
    for(int i = 1; i <nodes.size(); i++){
        nodes[i].setParent(nodes[i-1]);
        nodes[i].setPosition(joints[i].offset*1000);
        nodes[i].setOrientation(joints[i].rotation);
    }
    
    
    // Set Tool Center Point Node
    tcpNode.setParent(nodes[5]);
    tcpNode.setPosition(ofVec3f(0.0, -0.0308, 0.0)*1000);
    
    // Set Tool Rotations
    tool.rotation =joints[5].rotation;
    
    shader.load("shaders/model");
    
    bDrawModel.set("Draw Model", true);
    bDrawTargetModel.set("Draw Target Model", true);
    bUseShader.set("Use Shader", true);
}

ofQuaternion RobotKinematicModel::getToolPointQuaternion(){
    return toOf(nodes[5].getGlobalTransformMatrix()).getRotate();
}


ofNode RobotKinematicModel::getTool(){
    return tcpNode;
}

void RobotKinematicModel::setToolOffset(ofVec3f localOffset){
    tcpNode.setPosition(localOffset);
}

void RobotKinematicModel::setAngles( vector<double> aTargetRadians ){
    for(int i = 0; i < joints.size(); i++){
        if(i == 1 || i == 3){
            joints[i].rotation.makeRotate(ofRadToDeg(aTargetRadians[i])+90,joints[i].axis);
        }else{
            joints[i].rotation.makeRotate(ofRadToDeg(aTargetRadians[i]),joints[i].axis);
        }
        nodes[i].setOrientation(joints[i].rotation);
    }
}

void RobotKinematicModel::setPose(vector<double> pose){
    for(int i = 0; i < pose.size(); i++){
        if(i == 1 || i == 3){
            joints[i].rotation.makeRotate(ofRadToDeg(pose[i])+90,joints[i].axis);
        }else{
            joints[i].rotation.makeRotate(ofRadToDeg(pose[i]),joints[i].axis);
        }
         nodes[i].setOrientation(joints[i].rotation);
    }
    
}

void RobotKinematicModel::setEndEffector(string filename){
    string path = ofToDataPath("models/"+filename);
   
    loader.clear();
    if (loader.loadModel("models/"+filename))
    {
        toolMesh = loader.getMesh(0);
    }
    else{
        ofLogFatalError()<<"PLEASE PLACE THE 3D FILES OF THE END EFFECTOR IN data/models/" << filename <<endl;
    }
}

void RobotKinematicModel::clearEndEffector(){
    toolMesh.clear();
}

void RobotKinematicModel::setToolMesh(ofMesh mesh){
    toolMesh = mesh;
}
void RobotKinematicModel::update(){

}
void RobotKinematicModel::draw(ofFloatColor color, bool bDrawDebug){
    ofPushMatrix();
    ofPushStyle();
    ofEnableDepthTest();
    ofSetColor(255, 255, 255);
    if(bDrawDebug) {
        ofPushStyle(); {
            ofDrawAxis(1000);
            ofSetColor(255, 255, 0);
            ofDrawSphere(tool.position*ofVec3f(1000, 1000, 1000), 40);
            ofSetColor(225, 225, 225);
        } ofPopStyle();
    }
    
    if(bDrawModel){
        ofQuaternion q;
        ofVec3f offset;
        
        ofMatrix4x4 gmat;
        gmat.makeIdentityMatrix();
        gmat.makeScaleMatrix( 1, 1, 1 );
        
        if(bUseShader){
        
            shader.begin();
            shader.setUniform4f("color", color);
        }
        ofPushMatrix();
        {
            ofPushMatrix();
            {
                for(int i = 0; i < 6; i++)//joints.size(); i++)
                {
                    float x;
                    ofVec3f axis;
                    q = joints[i].rotation;
                    q.getRotate(x, axis);
                    ofTranslate(joints[i].offset*1000);
                    gmat.translate( joints[i].offset*1000 );
                    
                    if(bDrawDebug) {
                        ofDrawAxis(30);
                    }
                    ofMatrix4x4 tmat;
                    if(i >= 3){
                        ofPushMatrix();
                        {
                            ofRotateDeg(-180, 0, 0, 1);
                            ofRotateDeg(-180, 1, 0, 0);
                            ofScale(100, 100, 100);
                            meshs[i].draw();
                        }
                        ofPopMatrix();
                    }
                    ofRotateDeg(x, axis.x, axis.y, axis.z);
                    if(i < 3){
                        ofPushMatrix();
                        {
                            ofRotateDeg(-180, 0, 0, 1);
                            ofRotateDeg(-180, 1, 0, 0);
                            ofScale(100, 100, 100);
                            
                            meshs[i].draw();
                        } ofPopMatrix();
                    }
                    if (i==5){
                        // include flange offset
                        ofTranslate(0, -0.0308 * 1000, 0);
                        // the x-axis was rotating backwards,
                        // so I'm doing some funny business here
                        ofRotateDeg(180, 0, 0, 1);
                        ofRotateDeg(-180, nodes[5].getXAxis().x,
                                    nodes[5].getXAxis().y,
                                    nodes[5].getXAxis().z);
                        toolMesh.drawWireframe();
                    }
                }
            }
            ofPopMatrix();
        }
        ofPopMatrix();
        
        if(bUseShader) shader.end();
        
        if (bDrawDebug) {
            ofPushMatrix();
            {
                //            ofRotate(180, 0, 0, 1);
                for(int i = 0; i < nodes.size(); i++){
                    nodes[i].draw();
                }
                tcpNode.draw();
            }
            ofPopMatrix();
        }
    }
    ofDisableDepthTest();
    ofPopStyle();
    ofPopMatrix();
}
