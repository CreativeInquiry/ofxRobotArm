//
//  opw.hpp
//  example-ik
//
//  Created by Dan Moore on 6/5/22.
//

#pragma once
#include "ofMain.h"
#include "IK.h"
namespace ofxRobotArm {
    class OWP : public IK{
        public:
            OWP(){ };
            ~OWP(){ };
            
            void setup(vector<double> offsets,
                       vector<double> sign_corrections,
                       vector<double> joint_limit_min,
                       vector<double> joint_limit_max);
            void setParams(vector<double> params);
            void computeParams(RobotModel robot);
            ofMatrix4x4 forward(vector<double> pose);
            vector<vector<double> > inverse(ofMatrix4x4 pose);
        
        private:
            double a1, a2, b, c1, c2, c3, c4;
    };
}
