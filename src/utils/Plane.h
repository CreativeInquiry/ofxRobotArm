//
//  Plane.h
//
//  Created by @madelinegannon on 11/18/20.
//
//

#pragma once
#include "ofMain.h"

namespace ofxRobotArm{
class Plane{
public:
    ofVec3f axis;
    ofNode pose;
    float size = 100;
    
    void update(ofNode pose){
        this->pose.setGlobalPosition(pose.getGlobalPosition());
        this->pose.setGlobalOrientation(pose.getGlobalOrientation());
    }
    
    void update(ofVec3f position, ofQuaternion orientation){
        this->pose.setGlobalPosition(position);
        this->pose.setGlobalOrientation(orientation);
    }
    
    void draw(ofColor fill = ofColor::yellow, ofColor line = ofColor::cyan){
        
        ofPushStyle();
        ofPushMatrix();
        ofMultMatrix(pose.getGlobalTransformMatrix());
        
        ofSetLineWidth(3);
        ofDrawAxis(size);
        
        ofFill();
        ofSetColor(fill, 100);
        ofBeginShape();
        ofVertex(-size/2,-size/2,0);
        ofVertex(size/2,-size/2, 0);
        ofVertex(size/2,size/2, 0);
        ofVertex(-size/2,size/2,0);
        ofEndShape(OF_CLOSE);
        
        ofSetLineWidth(1);
        ofSetColor(line);
        ofNoFill();
        ofBeginShape();
        ofVertex(-size/2,-size/2,0);
        ofVertex(size/2,-size/2, 0);
        ofVertex(size/2,size/2, 0);
        ofVertex(-size/2,size/2,0);
        ofEndShape(OF_CLOSE);
        
        ofPopMatrix();
        ofPopStyle();
        
    }
};
}

