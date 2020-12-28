#pragma once
#include "ofMain.h"
#include "CollisionNN.hpp"
#include "Robot.hpp"
namespace ofxRobotArm{
namespace RelaxedIK{
class RelaxedIKVars{
public:
    void setup(){
        
    }
    
    void from_yaml_path(string fp, bool position_mode_relative, bool rotation_mode_relative){
        
    };
    void from_yaml_path_with_init(string fp, vector<double> init_state, bool position_mode_relative, bool rotation_mode_relative){
        
    };
    void update( vector<double> xopt){
        
    };
    
    Robot robot;
    ThreadRobotSampler sampler;
    vector<double>  init_state;
    vector<double>  xopt;
    vector<double>  prev_state;
    vector<double>  prev_state2;
    vector<double> prev_state3;
    vector<ofVec3f> goal_positions;
    vector<ofQuaternion> goal_quats;
    vector<ofVec3f> init_ee_positions;
    ofQuaternion init_ee_quats;
    bool position_mode_relative;
    bool rotation_mode_relative;
    CollisionNN collision_nn;
};

class Vars{
public:
    Vars(vector<double> init_state){
        this->init_state = init_state;
        xopt = init_state;
        prev_state = init_state;
        prev_state2 = init_state;
        prev_state3 = init_state;
    };
    ~Vars(){
        
    };
    void update(vector<double> xopt){
        prev_state3 = prev_state2;
        prev_state2 = prev_state;
        prev_state = this->xopt;
        this->xopt = xopt;
    };
    vector<double> init_state;
    vector<double> xopt;
    vector<double> prev_state;
    vector<double> prev_state2;
    vector<double> prev_state3;
};
}
}
