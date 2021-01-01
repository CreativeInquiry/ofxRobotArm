//
//  RelaxedIK.hpp
//  example-simple
//
//  Created by Dan Moore on 12/29/20.
//

#pragma once
#include "ofMain.h"

std::vector<double> solveIK(std::vector<double> pos, std::vector<double> quat);

#ifdef __cplusplus
extern "C" {
#endif
    class Opt {
        public:
            double* data;
            int length;
    };

    Opt solve(double* pos_arr, int pos_length, double* quat_arr, int quat_length);
//    void set_starting_config(double * pose, int pose_length);
#ifdef __cplusplus
};
#endif

