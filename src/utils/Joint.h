//
//  URJoint.h
//  MHTH-autonomous-movement
//
//  Created by dantheman on 6/13/17.
//
//

#pragma once
namespace ofxRobotArm {
    struct Joint{
        ofVec3f offset;
        ofVec3f axis;
        ofVec3f position;
        ofQuaternion rotation;
    };
}

