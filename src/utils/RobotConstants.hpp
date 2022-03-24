//
//  RobotConstants.hpp
//  example-robotarm
//
//  Created by Dan Moore on 12/15/20.
//


#pragma once
namespace ofxRobotArm{
    enum RobotType{
        UR3,
        UR5,
        UR10,
        IRB120,
        IRB4600,
        IRB6700,
        XARM7,
        PANDA
    };
    enum IKType{
        SW,
        HK,
        RELAXED
    };
}
