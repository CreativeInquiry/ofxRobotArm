/*********************************************************************
 *
 * Provides forward and inverse kinematics for Univeral robot designs
 * Author: Kelsey Hawkins (kphawkins@gatech.edu)
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2013, Georgia Institute of Technology
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the Georgia Institute of Technology nor the names of
 *     its contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
#pragma once
#include "ofMain.h"

#include "Pose.h"
#include "RobotConstants.hpp"
#include <complex>
// These kinematics find the tranfrom from the base link to the end effector.
// Though the raw D-H parameters specify a transform from the 0th link to the 6th link,
// offset transforms are specified in this formulation.
// To work with the raw D-H kinematics, use the inverses of the transforms below.

// Transform from base link to 0th link
// -1,  0,  0,  0
//  0, -1,  0,  0
//  0,  0,  1,  0
//  0,  0,  0,  1

// Transform from 6th link to end effector
//  0, -1,  0,  0
//  0,  0, -1,  0
//  1,  0,  0,  0
//  0,  0,  0,  1

typedef std::complex<float> CPX;

namespace ofxRobotArm{
class Kinematics {
public:
    Kinematics();
    ~Kinematics();
    //
    //adapted from https://github.com/Jmeyer1292/opw_kinematics/blob/master/include/opw_kinematics/opw_kinematics_impl.h
    //
    vector<double> solveSphere(ofMatrix4x4 pose);
    void setType(RobotType type);
    void inverseSW(ofMatrix4x4 pose, double * sol);
    void forwardSW(double t1, double t2, double t3, double t4, double t5, double t6, ofMatrix4x4& sol);
    // @param q       The 6 joint values
    // @param T       The 4x4 end effector pose in row-major ordering
    void forwardHK(const double* q, double* T);
    
    // @param q       The 6 joint values
    // @param Ti      The 4x4 link i pose in row-major ordering. If NULL, nothing is stored.
    void forward_allHK(const double* q, double* T1, double* T2, double* T3,
                       double* T4, double* T5, double* T6);
    
    // @param T       The 4x4 end effector pose in row-major ordering
    // @param q_sols  An 8x6 array of doubles returned, all angles should be in [0,2*PI)
    // @param q6_des  An optional parameter which designates what the q6 value should take
    //                in case of an infinite solution on that joint.
    // @return        Number of solutions found (maximum of 8)
    int inverseHK(const double* T, double* q_sols, double q6_des=0.0);
    

    
    void setParams(float _a1, float _a2, float _b, float _c1, float _c2, float _c3, float _c4);
    void setDH(float d1, float a2, float a3, float d4, float d5, float d6);
    void harmonizeTowardZero(double* qs)
    {
        for (int i = 0; i < 6; i++)
        {
            if (qs[i] >= PI)
                qs[i] -= TWO_PI;
            else if (qs[i] <= -PI)
                qs[i] += TWO_PI;
        }
    };
    
    bool isValid(const double* qs)
    {
        return std::isfinite(qs[0]) && std::isfinite(qs[1]) && std::isfinite(qs[2]) && std::isfinite(qs[3]) &&
        std::isfinite(qs[4]) && std::isfinite(qs[5]);
    };
    
    void inverse(ofMatrix4x4* target, vector< vector <double> >& sol);
    
    vector<double> boundSolution(vector<double> thetas){
        for(auto theta : thetas){
            if(abs(theta) > PI){
                double sign = abs(theta)/theta;
                theta = theta -(sign * TWO_PI);
            }
        }
        return thetas;
    };
    
  
private:
    RobotType type;
    // Retrive a specific value from a 4x4 matrix
    float get(ofMatrix4x4 mat, int row, int col);
    
    double d1;
    double a2;
    double a3;
    double d4;
    double d5;
    double d6;
    
    double a1, a2_2, b, c1, c2, c3, c4;
    vector<double> offsets;
    vector<double> sign_corrections;
    vector<double> joint_limit_min;
    vector<double> joint_limit_max;
};
}

