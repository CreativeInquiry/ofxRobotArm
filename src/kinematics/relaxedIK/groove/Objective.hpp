#pragma once
#include "ofMain.h"
#include "Gradient.hpp"
namespace ofxRobotArm{
namespace RelaxedIK{

class Objective{
public:
    struct Frames{
        vector<ofVec3f> pos;
        vector<ofQuaternion> rots;
    }
    struct Pose{
        ofVec3f pos;
        ofQuaternion rot;
    }
    Objective(){
        
    };
    ~Objective(){
        
    };
    virtual double call( double x, RelaxedIKVars v, vector<Frame> frames)>) = 0;
    virtual double call_lite( double x, RelaxedIKVars v, Pose ee_poses) = 0;
    virtual gradientReturn gradient( double x, RelaxedIKVars v, vector<Frame> frames)>) {
        let mut grad: Vec<f64> = Vec::new();
        let f_0 = self.call(x, v, frames);
        
        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.000000001;
            let frames_h = v.robot.get_frames_immutable(x_h.as_slice());
            let f_h = self.call(x_h.as_slice(), v, &frames_h);
            grad.push( (-f_0 + f_h) / 0.000000001);
        }
        
        (f_0, grad)
    }
    virtual gradientReturn gradient_lite( double x, RelaxedIKVars v, Pose ee_poses) {
        let mut grad: Vec<f64> = Vec::new();
        let f_0 = self.call_lite(x, v, ee_poses);
        
        for i in 0..x.len() {
            let mut x_h = x.to_vec();
            x_h[i] += 0.0000001;
            let ee_poses_h = v.robot.get_ee_pos_and_quat_immutable(x_h.as_slice());
            let f_h = self.call_lite(x_h.as_slice(), v, &ee_poses_h);
            grad.push( (-f_0 + f_h) / 0.0000001);
        }
        
        (f_0, grad)
    }
    int gradient_type(){return 1};  // manual diff = 0, finite diff = 1
};

class MatchEEPosGoals: public Objective{
public:
    MatchEEPosGoals(int arm_id){
        this->arm_id = arm_id;
    }
    ~MatchEEPosGoals();
    
    double call(double x, RelaxedIKVars v, vector<Frame> frames)>)  {
        let last_elem = frames[self.arm_idx].0.len() - 1;
        let x_val = ( frames[self.arm_idx].0[last_elem] - v.goal_positions[self.arm_idx] ).norm();
        
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
    
    double call_lite(double x, RelaxedIKVars v, Pose ee_poses)  {
        let x_val = ( ee_poses[self.arm_idx].0 - v.goal_positions[self.arm_idx] ).norm();
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
    int arm_id;
};


class MatchEEQuatGoals: public Objective{
public:
    MatchEEQuatGoals(int arm_id){
        this->arm_id = arm_id;
    }
    ~MatchEEQuatGoals();
    
    double call( double x, RelaxedIKVars v, vector<Frame> frames)>)  {
        let last_elem = frames[self.arm_idx].1.len() - 1;
        let tmp = Quaternion::new(-frames[self.arm_idx].1[last_elem].w, -frames[self.arm_idx].1[last_elem].i, -frames[self.arm_idx].1[last_elem].j, -frames[self.arm_idx].1[last_elem].k);
        let ee_quat2 = UnitQuaternion::from_quaternion(tmp);
        
        let disp = angle_between(v.goal_quats[self.arm_idx], frames[self.arm_idx].1[last_elem]);
        let disp2 = angle_between(v.goal_quats[self.arm_idx], ee_quat2);
        let x_val = disp.min(disp2);
        
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
    
    double call_lite(double x, RelaxedIKVars v, Pose ee_poses)  {
        let tmp = Quaternion::new(-ee_poses[self.arm_idx].1.w, -ee_poses[self.arm_idx].1.i, -ee_poses[self.arm_idx].1.j, -ee_poses[self.arm_idx].1.k);
        let ee_quat2 = UnitQuaternion::from_quaternion(tmp);
        
        let disp = angle_between(v.goal_quats[self.arm_idx], ee_poses[self.arm_idx].1);
        let disp2 = angle_between(v.goal_quats[self.arm_idx], ee_quat2);
        let x_val = disp.min(disp2);
        groove_loss(x_val, 0., 2, 0.1, 10.0, 2)
    }
    int arm_id;
};

class MinimizeJerk: public Objective{
public:
    double call( double x, RelaxedIKVars v, vector<Frame> frames)>)  {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            let v3 = v.prev_state[i] - v.prev_state2[i];
            let a1 = v1 - v2;
            let a2 = v2 - v3;
            x_val += (a1 - a2).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
    
    double call_lite( double x, RelaxedIKVars v, Pose ee_poses)  {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            let v3 = v.prev_state[i] - v.prev_state2[i];
            let a1 = v1 - v2;
            let a2 = v2 - v3;
            x_val += (a1 - a2).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
}

class MinimizeAcceleration: public Objective{
public:
    double call( double x, RelaxedIKVars v, vector<Frame> frames)>)  {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            x_val += (v1 - v2).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
    
    double call_lite( double x, RelaxedIKVars v, Pose ee_poses)  {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            let v1 = x[i] - v.xopt[i];
            let v2 = v.xopt[i] - v.prev_state[i];
            x_val += (v1 - v2).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
};

class MinimizeVelocity: public Objective{
public:
    double call( double x, RelaxedIKVars v, vector<Frame> frames)>)  {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            x_val += (x[i] - v.xopt[i]).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
    
    double call_lite( double x, RelaxedIKVars v, Pose ee_poses)  {
        let mut x_val = 0.0;
        for i in 0..x.len() {
            x_val += (x[i] - v.xopt[i]).powi(2);
        }
        x_val = x_val.sqrt();
        groove_loss(x_val, 0.0, 2, 0.1, 10.0, 2)
    }
};

class JointLimits: public Objective{
public:
    double call( double x, RelaxedIKVars v, vector<Frame> frames)>)  {
        let mut sum = 0.0;
        let penalty_cutoff: f64 = 0.9;
        let a = 0.05 / (penalty_cutoff.powi(50));
        for i in 0..v.robot.num_dof {
            let l = v.robot.bounds[i][0];
            let u = v.robot.bounds[i][1];
            let r = (x[i] - l) / (u - l);
            let n = 2.0 * (r - 0.5);
            sum += a*n.powf(50.);
        }
        groove_loss(sum, 0.0, 2, 0.32950, 0.1, 2)
    }
    
    double call_lite( double x, RelaxedIKVars v, Pose ee_poses)  {
        let mut sum = 0.0;
        let penalty_cutoff: f64 = 0.85;
        let a = 0.05 / (penalty_cutoff.powi(50));
        for i in 0..v.robot.num_dof {
            let l = v.robot.bounds[i][0];
            let u = v.robot.bounds[i][1];
            let r = (x[i] - l) / (u - l);
            let n = 2.0 * (r - 0.5);
            sum += a*n.powi(50);
        }
        groove_loss(sum, 0.0, 2, 0.32950, 0.1, 2)
    }
};

class NNSelfCollision: public Objective{
public:
    double call(double x, RelaxedIKVars v, vector<Frame> frames)>) {
                double x_val = v.collision_nn.predict(&x.to_vec());
                groove_loss(x_val, 0., 2, 2.1, 0.0002, 4)
    }
    
    double call_lite( double x, RelaxedIKVars v, Pose ee_poses){
                double x_val = v.collision_nn.predict(&x.to_vec());
                groove_loss(x_val, 0., 2, 2.1, 0.0002, 4)
    }
    
    gradientReturn gradient( double x, RelaxedIKVars v, vector<Frame> frames)>) {
                let (x_val, mut grad) = v.collision_nn.gradient(&x.to_vec());
                let g_prime = groove_loss_derivative(x_val, 0., 2, 2.1, 0.0002, 4);
                for i in 0..grad.len() {
                    grad[i] *= g_prime;
                }
                (x_val, grad)
    }
    
    gradientReturn gradient_lite( double x, RelaxedIKVars v, Pose ee_poses){
                let (x_val, mut grad) = v.collision_nn.gradient(&x.to_vec());
                let g_prime = groove_loss_derivative(x_val, 0., 2, 2.1, 0.0002, 4);
                for i in 0..grad.len() {
                    grad[i] *= g_prime;
                }
                (x_val, grad)
    }
    
    int gradient_type(){
        return 0
        
    }
};
}
}
