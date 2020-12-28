//
//  CollisionNN.hpp
//  example-simple
//
//  Created by Dan Moore on 12/26/20.
//

#pragma once
#include "ofMain.h"
namespace ofxRobotArm{
namespace RelaxedIK{
class CollisionNN{
public:
    CollisionNN();
    ~CollisionNN();
    void setup();
    void predict_mutable();
    void predict();
    void in_collision();
    void gradient2();
    void gradient();
    void gradient_finite_diff();
    vector<ofMatrix4x4> coef_matrices;
    vector<ofMatrix4x4> intercept_vectors;
    double split_point;
    int input_length;
    double result;
    ofMatrix4x4 x_proxy;
    vector<ofMatrix4x4> intermediate_vecs;
    
};
}
}
