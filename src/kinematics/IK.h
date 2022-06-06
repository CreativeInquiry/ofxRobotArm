//
//  IK.h
//
//
//  Copyright (c) 2016, 2021 Daniel Moore, Madeline Gannon, and The Frank-Ratchye STUDIO for Creative Inquiry All rights reserved.
//


#pragma once
#include "ofMain.h"
#include "RobotModel.h"
namespace ofxRobotArm {
    class IK{
        public:
        IK(){ };
        ~IK(){ };
        
        virtual void setup(vector<double> offsets,
                           vector<double> sign_corrections,
                           vector<double> joint_limit_min,
                           vector<double> joint_limit_max) = 0;
        virtual void setParams(vector<double> params) = 0;
        virtual void computeParams(RobotModel robot) = 0;
        virtual ofMatrix4x4 forward(vector<double> pose) = 0;
        virtual vector<vector<double> > inverse(ofMatrix4x4 pose) = 0;
        
        const double ANGLE_THRESH = ofDegToRad(30);
        const double ZERO_THRESH = 0.00000001;
    protected:
        void harmonizeTowardZero(vector<double>& qs){
            for (auto& q : qs)
            {
                if (q >= PI)
                    q -= TWO_PI;
                else if (q <= -PI)
                    q += TWO_PI;
            }
        };
        
        bool isValid(vector<double>& qs)
        {
          return std::isfinite(qs[0]) && std::isfinite(qs[1]) && std::isfinite(qs[2]) && std::isfinite(qs[3]) &&
                 std::isfinite(qs[4]) && std::isfinite(qs[5]);
        };
        
        vector<double> boundSolution(vector<double> thetas)
        {
            for (auto theta : thetas)
            {
                if (abs(theta) > PI)
                {
                    double sign = abs(theta) / theta;
                    theta = theta - (sign * TWO_PI);
                }
            }
            return thetas;
        };
        
        float get(ofMatrix4x4 mat, int row, int col)
        {

            return (mat.getPtr())[row * 4 + col];
        };
        
        int SIGN(double x)
        {
            return (x > 0) - (x < 0);
        };

        vector<double> offsets;
        vector<double> sign_corrections;
        vector<double> joint_limit_min;
        vector<double> joint_limit_max;
        vector<vector<double>> preInversePosition;
    };
}
