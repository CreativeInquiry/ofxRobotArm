//
//  RelaxedIK.hpp
//  example-simple
//
//  Created by Dan Moore on 12/29/20.
//

#pragma once
#include "ofMain.h"

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
//    void dynamic_obstacle_cb(char * name, double * pos, double * quat);
#ifdef __cplusplus
};
#endif

