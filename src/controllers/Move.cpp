//
//  Move.cpp
//  ofxURDriver
//
//  Created by Dan Moore on 2/20/16.
//  Copyright (c) 2016, Daniel Moore, Madaline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//
#include "Move.h"
#include "Utils.h"
using namespace ofxRobotArm;
Move::Move(){
    
}
Move::~Move(){
    
}
void Move::setup(){
    float min = FLT_MIN;
    float max = FLT_MAX;
    movementParams.setName("UR Movements");
    movementParams.add(minSpeed.set("MIN Speed", 0, -5000, 5000));
    movementParams.add(maxSpeed.set("MAX Speed", 0, -5000, 5000));
    movementParams.add(targetTCPLerpSpeed.set("TCP LerpSpeed", 0.9, 0.001, 1.0));
    movementParams.add(maxAcceleration.set("Accel", 1, 0, 200));
    movementParams.add(jointAccelerationMultipler.set("Accel Multi", 1, 1, 1000));
    movementParams.add(speedDivider.set("Speed Divider", 1, 1, 10));
    movementParams.add(deltaTime.set("Delta T", 0.0, 0.0, 1.0));
    movementParams.add(jointSpeedLerpSpeed.set("Joint LerpSpeed", 0.9, 0.001, 1.0));
    
    

    selectedSolution = -1;
    deltaTimer.setSmoothing(0.99);
    distance = 0;
    jointAccelerations.assign(6, 0.0);
    previousJointVelocity.assign(6, 0.0);
    currentJointVelocity.assign(6, 0.0);
    currentPose.assign(6, 0.0);
    
    
    epslion = 0.00025;
    targetLength = 0.0;
}


void Move::update(){
    deltaTimer.tick();
    deltaTime = deltaTimer.getPeriod();
    computeVelocities();
}

void Move::addTargetJointPose(vector<double> _target){
    targetPose = _target;
}

vector<double> Move::getTargetJointPose(){
    return targetPose;
}
float Move::getAcceleration(){
    maxAcceleration = maxAccel*jointAccelerationMultipler;
    return maxAcceleration;
}
vector<double> Move::getCurrentJointVelocities(){
    return currentJointVelocity;
}


void Move::setCurrentJointPose(vector<double> & pose) {
    
    for(int i =0; i < MIN(pose.size(), currentPose.size()); i++){
        currentPose[i]= pose[i];
    }
}

void Move::computeVelocities(){
    
    if( targetJointPose.size() >0) {
        if( currentJointVelocity.size() != currentPose.size() ) {
            currentJointVelocity.assign( currentPose.size(), 0.0f );
        }
        
        avgAccel        = 0;
        previousJointVelocity = currentJointVelocity;
        minSpeed = FLT_MAX;
        maxSpeed = FLT_MIN;
        for( int i = 0; i < currentPose.size(); i++ ) {
            currentJointVelocity[i] =  ofAngleDifferenceRadians( currentPose[i], targetPose[i] )/deltaTime;
            
            currentJointVelocity[i] = ofClamp(currentJointVelocity[i], -PI/2, PI/2);
            minSpeed = MIN(minSpeed.get(), currentJointVelocity[i]);
            maxSpeed = MAX(maxSpeed.get(), currentJointVelocity[i]);
            
            
            jointAccelerations[i] = (currentJointVelocity[i] - previousJointVelocity[i])/deltaTime;
            if(fabs(jointAccelerations[i]) > fabs(avgAccel)){
                avgAccel =jointAccelerations[i];
            }
            
        }
        
        //            cout << "avgAccel : " << avgAccel << endl;
        
    } else {
        //            for(int i = 0; i < currentJointVelocity.size(); i++){
        //                currentJointVelocity[i] = 0;
        //            }
    }
}

