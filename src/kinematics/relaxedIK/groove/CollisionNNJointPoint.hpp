//
//  CollisionNNJointPoint.hpp
//  example-simple
//
//  Created by Dan Moore on 12/26/20.
//

#pragma once
#include "ofMain.h"
namespace ofxRobotArm{
namespace RelaxedIK{
class CollisionNNJointPoint{
public:
    CollisionNNJointPoint();
    ~CollisionNNJointPoint();
    void from_yaml_path(string path);
    void predict(vector<double> pose);
    bool in_collision(vector<double> pose);
    void gradient_finite_diff(vector<double> pose);
    
    vector<ofMatrix4x4> coef_matrices;
    vector<ofMatrix4x4>  intercept_vectors;
    double split_point;
    int input_length;
    double result;
    ofMatrix4x4 _x_proxy;
    vector<ofMatrix4x4> _intermediate_vecs;
};
}
}
