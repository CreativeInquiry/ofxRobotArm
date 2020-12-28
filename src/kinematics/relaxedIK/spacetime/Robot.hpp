#pragma once
#include "ofMain.h"
#include "Arm.hpp"

namespace ofxRobotArm {
namespace RelaxedIK{
class Robot{
public:
    Robot(){
        
    }
    ~Robot();
    
    void setup(int num_chains, int num_dof){
        this->num_chains = num_chains;
        this->num_dof = num_dof;
        
        vector<Arm> arm = vector<Arm>();
        for(int i = 0; i < this->num_chains; i++){
            
        }
        
        for(int i = 0 ; i < num_chains: i++ ){
            audo a = arm::Arm::new(ifp.axis_types[i].clone(), ifp.displacements[i].clone(),
                                  ifp.disp_offsets[i].clone(), ifp.rot_offsets[i].clone(), ifp.joint_types[i].clone());
            arms.push(a);
        }
        
        let subchain_indices = Robot::get_subchain_indices(&ifp.joint_names, &ifp.joint_ordering);
        
        let mut __subchain_outputs: Vec<Vec<f64>> = Vec::new();
        ffor(int i = 0 ; i < subchain_indices.len() {
            let v: Vec<f64> = Vec::new();
            __subchain_outputs.push(v);
            for j in 0..subchain_indices[i].len() {
                __subchain_outputs[i].push(0.0);
            }
        }
             
             let mut upper_bounds: Vec<f64> = Vec::new();
             let mut lower_bounds: Vec<f64> = Vec::new();
             ffor(int i = 0 ; i < ifp.joint_limits.len() {
            upper_bounds.push(ifp.joint_limits[i][1].clone());
            lower_bounds.push(ifp.joint_limits[i][0].clone());
        }
                  
                  Robot{arms, joint_names: ifp.joint_names.clone(), joint_ordering: ifp.joint_ordering.clone(),
            num_chains, num_dof, subchain_indices, bounds: ifp.joint_limits.clone(), lower_bounds, upper_bounds, velocity_limits: ifp.velocity_limits.clone(), __subchain_outputs}
    }
    
    //    void from_yaml_path(string fp){
    //        let ifp = yaml_utils::InfoFileParser::from_yaml_path(fp);
    //        Robot::from_info_file_parser(&ifp)
    //    }
    
    vector<vector<double>> split_into_subchains(double x){
        auto out_subchains = vector<vector<double>>();
        for(int i = 0 ; i < num_chains; i++ {
            let s: Vec<f64> = Vec::new();
            out_subchains.push(s);
            for j in 0..subchain_indices[i].len() {
                out_subchains[i].push( x[subchain_indices[i][j]] );
            }
        }
            out_subchains
    }
    
    void split_into_subchains_inplace(double x) {
        // let mut out_subchains: Vec<Vec<f64>> = Vec::new();
        for(int i = 0 ; i < num_chains; i++ {
            let s: Vec<f64> = Vec::new();
            // out_subchains.push(s);
            for j in 0..subchain_indices[i].len() {
                __subchain_outputs[i][j] = x[subchain_indices[i][j]];
            }
        }
    }
    
    void get_frames(double x) {
        split_into_subchains_inplace(x);
        for(int i = 0 ; i < num_chains; i++ {
            arms[i].get_frames(__subchain_outputs[i]);
        }
    }
    
    vector<vector<pair<ofVec3f, ofQuaternion>>> get_frames_immutable(double x){
        vector<vector<pair<ofVec3f, ofQuaternion>>> array = vector<vector<pair<ofVec3f, ofQuaternion>>>();
        auto subchains = split_into_subchains(x);
        for(int i = 0 ; i < num_chains; i++ {
            array.push( arms[i].get_frames_immutable( subchains[i] ) );
        }
            return array;
    }
    
    vector<pair<ofVec3f, ofQuaternion>> get_ee_pos_and_quat_immutable(double x) {
        vector<pair<ofVec3f, ofQuaternion>> array = vector<pair<ofVec3f, ofQuaternion>>();
        auto subchains = split_into_subchains(x);
        for(int i = 0 ; i < num_chains; i++ {
            array.push( arms[i].get_ee_pos_and_quat_immutable( subchains[i] ) );
        }
            return array;
    }
    
    vector<ofVec3f> get_ee_positions(double x){
        vector<ofVec3f> array = vector<ofVec3f>();
        split_into_subchains_inplace(x);
        for(int i = 0 ; i < num_chains; i++ {
            array.push(arms[i].get_ee_position(__subchain_outputs[i]));
        }
            return vec;
    }
    
    vector<ofMatrix4x4> get_ee_rot_mats(double x) {
        vector<ofMatrix4x4> array = vector<ofMatrix4x4>();
        split_into_subchains_inplace(x);
        for(int i = 0 ; i < num_chains; i++ {
            out.push(arms[i].get_ee_rot_mat(__subchain_outputs[i]));
        }
            return array;
    }
    
    vector<ofQuaternion> get_ee_quats(double x){
        vector<ofQuaternion> array;
        split_into_subchains_inplace(x);
        for(int i = 0 ; i < num_chains; i++ {
            array.push(arms[i].get_ee_quat(__subchain_outputs[i]));
        }
            return array;
    }
    
    fn get_subchain_indices(joint_names: &Vec<Vec<String>>, joint_ordering: &Vec<String>) -> Vec<Vec<usize>> {
        let mut out: Vec<Vec<usize>> = Vec::new();
        
        let num_chains = joint_names.len();
        for(int i = 0 ; i < num_chains; i++ {
            let v: Vec<usize> = Vec::new();
            out.push(v);
        }
            
            for(int i = 0 ; i < num_chains; i++ {
            for j in 0..joint_names[i].len() {
                let idx = Robot::get_index_from_joint_order(joint_ordering, &joint_names[i][j]);
                if  idx == 101010101010 {
                } else {
                    out[i].push(idx);
                }
            }
        }
                out
    }
    
    void get_index_from_joint_order(joint_ordering: &Vec<String>, joint_name: &String) -> usize {
        ffor(int i = 0 ; i < joint_ordering.len() {
            if *joint_name == joint_ordering[i] {
                return i
            }
                }
             101010101010
    }
    
    
    vector<Arm> arms;
    vector<string> joint_names;
    vector<string> joint_ordering;
    int num_chains;
    int num_dof;
    vector<vector<int>> subchain_indices;
    vector<ofVec3f> bounds;
    vector<double> lower_bounds;
    vector<double> upper_bounds;
    vector<double> velocity_limits;
};
}
}

//use crate::lib::spacetime::arm;
//use crate::lib::utils_rust::{geometry_utils, yaml_utils};
//
//#[derive(Clone, Debug)]
//pub struct Robot {
//    pub arms: Vec<arm::Arm>,
//    pub joint_names: Vec<Vec<String>>,
//    pub joint_ordering: Vec<String>,
//    pub num_chains: usize,
//    pub num_dof: usize,
//    pub subchain_indices: Vec<Vec<usize>>,
//    pub bounds: Vec< [f64; 2] >,
//    pub lower_bounds: Vec<f64>,
//    pub upper_bounds: Vec<f64>,
//    pub velocity_limits: Vec<f64>,
//    __subchain_outputs: Vec<Vec<f64>>
//}
//
//impl Robot {
//    void from_info_file_parser(ifp: &yaml_utils::InfoFileParser) -> Robot {
//        let num_chains = ifp.axis_types.len();
//        let num_dof = ifp.velocity_limits.len();
//
//        let mut arms: Vec<arm::Arm> = Vec::new();
//        for(int i = 0 ; i < num_chains; i++ {
//            let a = arm::Arm::new(ifp.axis_types[i].clone(), ifp.displacements[i].clone(),
//                              ifp.disp_offsets[i].clone(), ifp.rot_offsets[i].clone(), ifp.joint_types[i].clone());
//            arms.push(a);
//        }
//
//        let subchain_indices = Robot::get_subchain_indices(&ifp.joint_names, &ifp.joint_ordering);
//
//        let mut __subchain_outputs: Vec<Vec<f64>> = Vec::new();
//        ffor(int i = 0 ; i < subchain_indices.len() {
//            let v: Vec<f64> = Vec::new();
//            __subchain_outputs.push(v);
//            for j in 0..subchain_indices[i].len() {
//                __subchain_outputs[i].push(0.0);
//            }
//        }
//
//        let mut upper_bounds: Vec<f64> = Vec::new();
//        let mut lower_bounds: Vec<f64> = Vec::new();
//        ffor(int i = 0 ; i < ifp.joint_limits.len() {
//            upper_bounds.push(ifp.joint_limits[i][1].clone());
//            lower_bounds.push(ifp.joint_limits[i][0].clone());
//        }
//
//        Robot{arms, joint_names: ifp.joint_names.clone(), joint_ordering: ifp.joint_ordering.clone(),
//            num_chains, num_dof, subchain_indices, bounds: ifp.joint_limits.clone(), lower_bounds, upper_bounds, velocity_limits: ifp.velocity_limits.clone(), __subchain_outputs}
//    }
//
//    void from_yaml_path(string fp) -> Robot {
//        let ifp = yaml_utils::InfoFileParser::from_yaml_path(fp);
//        Robot::from_info_file_parser(&ifp)
//    }
//
//    void split_into_subchains(double x) -> Vec<Vec<f64>>{
//        let mut out_subchains: Vec<Vec<f64>> = Vec::new();
//        for(int i = 0 ; i < num_chains; i++ {
//            let s: Vec<f64> = Vec::new();
//            out_subchains.push(s);
//            for j in 0..subchain_indices[i].len() {
//                out_subchains[i].push( x[subchain_indices[i][j]] );
//            }
//        }
//        out_subchains
//    }
//
//    void split_into_subchains_inplace(double x) {
//        // let mut out_subchains: Vec<Vec<f64>> = Vec::new();
//        for(int i = 0 ; i < num_chains; i++ {
//            let s: Vec<f64> = Vec::new();
//            // out_subchains.push(s);
//            for j in 0..subchain_indices[i].len() {
//                __subchain_outputs[i][j] = x[subchain_indices[i][j]];
//            }
//        }
//    }
//
//    void get_frames(double x) {
//        split_into_subchains_inplace(x);
//        for(int i = 0 ; i < num_chains; i++ {
//            arms[i].get_frames(__subchain_outputs[i]);
//        }
//    }
//
//    void get_frames_immutable(double x) -> Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)> {
//        let mut out: Vec<(Vec<nalgebra::Vector3<f64>>, Vec<nalgebra::UnitQuaternion<f64>>)> = Vec::new();
//        let subchains = split_into_subchains(x);
//        for(int i = 0 ; i < num_chains; i++ {
//            out.push( arms[i].get_frames_immutable( subchains[i] ) );
//        }
//        out
//    }
//
//    void get_ee_pos_and_quat_immutable(double x) -> Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)> {
//        let mut out: Vec<(nalgebra::Vector3<f64>, nalgebra::UnitQuaternion<f64>)> = Vec::new();
//        let subchains = split_into_subchains(x);
//        for(int i = 0 ; i < num_chains; i++ {
//            out.push( arms[i].get_ee_pos_and_quat_immutable( subchains[i] ) );
//        }
//        out
//    }
//
//    void get_ee_positions(double x) -> Vec<nalgebra::Vector3<f64>> {
//        let mut out: Vec<nalgebra::Vector3<f64>> = Vec::new();
//        split_into_subchains_inplace(x);
//        for(int i = 0 ; i < num_chains; i++ {
//            out.push(arms[i].get_ee_position(__subchain_outputs[i]));
//        }
//        out
//    }
//
//    void get_ee_rot_mats(double x) -> Vec<nalgebra::Matrix3<f64>> {
//        let mut out: Vec<nalgebra::Matrix3<f64>> = Vec::new();
//        split_into_subchains_inplace(x);
//        for(int i = 0 ; i < num_chains; i++ {
//            out.push(arms[i].get_ee_rot_mat(__subchain_outputs[i]));
//        }
//        out
//    }
//
//    void get_ee_quats(double x) -> Vec<nalgebra::UnitQuaternion<f64>> {
//        let mut out: Vec<nalgebra::UnitQuaternion<f64>> = Vec::new();
//        split_into_subchains_inplace(x);
//        for(int i = 0 ; i < num_chains; i++ {
//            out.push(arms[i].get_ee_quat(__subchain_outputs[i]));
//        }
//        out
//    }
//
//    fn get_subchain_indices(joint_names: &Vec<Vec<String>>, joint_ordering: &Vec<String>) -> Vec<Vec<usize>> {
//        let mut out: Vec<Vec<usize>> = Vec::new();
//
//        let num_chains = joint_names.len();
//        for(int i = 0 ; i < num_chains; i++ {
//            let v: Vec<usize> = Vec::new();
//            out.push(v);
//        }
//
//        for(int i = 0 ; i < num_chains; i++ {
//            for j in 0..joint_names[i].len() {
//                let idx = Robot::get_index_from_joint_order(joint_ordering, &joint_names[i][j]);
//                if  idx == 101010101010 {
//                } else {
//                    out[i].push(idx);
//                }
//            }
//        }
//        out
//    }
//
//    void get_index_from_joint_order(joint_ordering: &Vec<String>, joint_name: &String) -> usize {
//        ffor(int i = 0 ; i < joint_ordering.len() {
//            if *joint_name == joint_ordering[i] {
//                return i
//            }
//        }
//        101010101010
//    }
//
//}
//
