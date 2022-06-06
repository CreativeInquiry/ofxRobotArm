//
//  hk.hpp
//  example-ik
//
//  Created by Dan Moore on 6/5/22.
//


#pragma once
#include "ofMain.h"
#include "IK.h"
namespace ofxRobotArm {
    class HK : public IK{
        public:
            HK(){ };
            ~HK(){ };
            
            void setup(vector<double> offsets,
                       vector<double> sign_corrections,
                       vector<double> joint_limit_min,
                       vector<double> joint_limit_max);
            void setParams(vector<double> params);
            void computeParams(RobotModel robot);
            ofMatrix4x4 forward(vector<double> pose);
            vector<vector<double> > inverse(ofMatrix4x4 pose);
        
            double *toIK(ofMatrix4x4 input)
            {
                double *T = new double[16];
                for (int i = 0; i < 4; i++)
                {
                    T[i] = (double)input._mat[i][0];
                    T[i + (4)] = (double)input._mat[i][1];
                    T[i + (8)] = (double)input._mat[i][2];
                    T[i + (12)] = (double)input._mat[i][3];
                }
                return T;
            };
            
            ofMatrix4x4 toOF(double *T)
            {
                ofMatrix4x4 output;
                for (int i = 0; i < 4; i++)
                {
                    output._mat[i][0] = T[i];
                    output._mat[i][1] = T[i + (4)];
                    output._mat[i][2] = T[i + (8)];
                    output._mat[i][3] = T[i + (12)];
                }
                return output;
            };

        
        private:
            double d1, a2, a3, d4, d5, d6;
    };
}
