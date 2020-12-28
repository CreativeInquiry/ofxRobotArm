//
//  RelaxedIK.cpp
//  example-simple
//
//  Created by Dan Moore on 12/26/20.
//

#include "RelaxedIK.hpp"
using namespace ofxRobotArm::RelaxedIK;

RelaxedIK::RelaxedIK(){
    
}
RelaxedIK::~RelaxedIK(){
    
}
void RelaxedIK::setup(string path){
    vars = RelaxedIKVars();
    vars.from_yaml_path(path, true, true)
    objectiveController = ObjectiveController();
    if mode == 0 {
        objectiveController.standard_ik(vars.robot.num_chains);
    }else{
        objectiveController.relaxed_ik(vars.robot.num_chains);
    }

    groove = OptimizationEngineOpen(vars.robot.num_dof);
    groove_nlopt = OptimizationEngineNLopt();

}

vector<double> RelaxedIK::solve(EEPoseGoalsSubscriber ee_sub){
    vector<double> out_x = vars.xopt;

    if (vars.rotation_mode_relative) {
        for(int i = 0; i < vars.robot.num_chains; i++)  {
            vars.goal_positions[i] = vars.init_ee_positions[i] + ee_sub.pos_goals[i];
            vars.goal_quats[i] = ee_sub.quat_goals[i] * vars.init_ee_quats[i];
        }
    } else {
        for(int i = 0; i < vars.robot.num_chains; i++)  {
            vars.goal_positions[i].set(ee_sub.pos_goals[i]);
            vars.goal_quats[i].set(ee_sub.quat_goals[i]);
        }
    }

    groove.optimize(out_x, vars, objectiveController, 100);

    vars.update(out_x);

    return out_x;
}
void RelaxedIK::solve_with_user_provided_goals(vector<ofVec3f> pos_goals, vector<ofQuaternion> quat_goals){
    EEPoseGoalsSubscriber ee_sub = EEPoseGoalsSubscriber();
    for( int i = 0 ; i < pos_goals.size(); i++) {
        ee_sub.pos_goals.push( pos_goals[i] );
        ee_sub.quat_goals.push( quat_goals[i] );
    }

    solve(&ee_sub)
}

void RelaxedIK::solve_precise(){
    
}
