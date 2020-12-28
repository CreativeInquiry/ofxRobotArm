#pragma once
#include "ofMain.h"
namespace ofxRobotArm{
namespace RelaxedIK{
class RelaxedIKTools{
public:
    RelaxedIKTools(Robot robot){
        this-robot = robot;
    };
    ~RelaxedIKTools(){
        
    }
    Robot robot;
};
}
}
