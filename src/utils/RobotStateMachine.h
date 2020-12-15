//
//  RobotParameters.h
//  urModernDriverTest
//
//  Created by dantheman on 4/4/16.
//
//

#pragma once
namespace ofxRobotArm {
    class RobotStateMachine{
        public:
            enum RobotState{
                FOLLOW_TARGET, //Following the Target TCP Sliders
                FOLLOW_TARGET_PATH = 0, //Following a Polyline Path
                FOLLOW_TARGET_RIGID_BODY, //Follow a Rigid Body
                FREE_DRIVE, //Toggle Robit into Free Drive
                HALT
            };
            
            void setState(RobotState state){
                currentState = state;
            };
            
            RobotState getCurrentState(){
                return currentState;
            };
            
        protected:
            RobotState currentState;
        };
    }
}
