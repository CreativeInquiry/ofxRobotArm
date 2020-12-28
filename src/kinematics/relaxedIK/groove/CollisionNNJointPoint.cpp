//
//  CollisionNNJointPoint.cpp
//  example-simple
//
//  Created by Dan Moore on 12/26/20.
//
#include "CollisionNNJointPoint.hpp"
using namespace ofxRobotArm::RelaxedIK;

CollisionNNJointPoint::CollisionNNJointPoint(){
    
}
CollisionNNJointPoint::~CollisionNNJointPoint(){
    
}
void CollisionNNJointPoint::from_yaml_path(string path){
//    let parser = NeuralNetParser::from_yaml_path(fp.clone());
//    let input_length = parser.coefs[0].len();
//    let __x_proxy: DMatrix<f64> = DMatrix::from_element(1, input_length, 0.0);
//    let mut __intermediate_vecs: Vec<DMatrix<f64>> = Vec::new();
//    let mut result = 0.0;
//
//    for i in 0..parser.intercept_vectors.len() {
//        __intermediate_vecs.push(parser.intercept_vectors[i].clone());
//    }
//
//    Self{coef_matrices: parser.coef_matrices.clone(), intercept_vectors: parser.intercept_vectors.clone(), split_point: parser.split_point, input_length, result, __x_proxy, __intermediate_vecs}
}

void CollisionNNJointPoint::predict(vector<double> pose){
//    let jt_pt_vec = state_to_jt_pt_vec(x, robot);
//    let mut x_vec = DMatrix::from_element(1, jt_pt_vec.len(), 0.0);
//    for i in 0..jt_pt_vec.len() {
//        x_vec[i] = jt_pt_vec[i];
//    }
//    for i in 0..self.coef_matrices.len() {
//        x_vec =  x_vec * &self.coef_matrices[i] + &self.intercept_vectors[i];
//        x_vec.apply(relu);
//    }
//    x_vec[0]
}

bool CollisionNNJointPoint::in_collision(vector<double> pose){
//    let p = self.predict(x, robot);
//    if p > self.split_point {
//        return true;
//    } else {
//        return false;
//    }
}

void CollisionNNJointPoint::gradient_finite_diff(vector<double> pose){
//    let mut out: Vec<f64> = Vec::new();
//
//    let f_0 = self.predict(&x, robot);
//    for i in 0..x.len() {
//        let mut x_h = x.clone();
//        x_h[i] += 0.000001;
//        let f_h = self.predict(&x_h, robot);
//        out.push( (-f_0 + f_h) / 0.000001);
//    }
//
//    (f_0, out)
}
