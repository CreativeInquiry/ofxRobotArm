//
//  RelaxedIK.hpp
//  example-simple
//
//  Created by Dan Moore on 12/26/20.
//

#pragma once
#include "ofMain.h"
#include "utils/SubscriberUtils.hpp"
#include "groove/Vars.hpp"
#include "ObjectiveController.hpp"
#include "Groove.hpp"
namespace ofxRobotArm{
namespace RelaxedIK{
class RelaxedIK{
public:
    RelaxedIK();
    ~RelaxedIK();
    void setup(string path);
    vector<double> solve(EEPoseGoalsSubscriber ee_sub);
    void solve_with_user_provided_goals(vector<ofVec3f> posGoals, vector<ofQuaternion> quatGoals);
    void solve_precise();
    void solve_randstart();
    

    vector<ofVec3f> posGoals;
    vector<ofQuaternion> quatGoals;
    vector<ofVec3f> goal_positions;
    vector<ofQuaternion> goal_quats;
    vector<ofVec3f> init_ee_positions;
    vector<ofQuaternion> init_ee_quats;
    bool position_mode_relative;
    bool rotation_mode_relative;
    RelaxedIKVars vars;
    ObjectiveController objectiveController;
    OptimizationEngineOpen groove;
    OptimizationEngineNLopt groove_nlopt;
//    pub collision_nn: CollisionNN
};
}
}
