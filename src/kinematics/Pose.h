//
//  Pose.h
//  
//
//  Created by dantheman on 6/13/17.
//
//

#pragma once
namespace ofxRobotArm {
    struct Pose{
        ofVec3f offset;
        ofVec3f rotOffset;
        ofVec3f axis;
        ofVec3f position;
        ofQuaternion orientation;
        float rotation;
        string name;
        string type;
    };
}

