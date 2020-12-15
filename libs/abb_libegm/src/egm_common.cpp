/***********************************************************************************************************************
 *
 * Copyright (c) 2015, ABB Schweiz AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with
 * or without modification, are permitted provided that
 * the following conditions are met:
 *
 *    * Redistributions of source code must retain the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer.
 *    * Redistributions in binary form must reproduce the
 *      above copyright notice, this list of conditions
 *      and the following disclaimer in the documentation
 *      and/or other materials provided with the
 *      distribution.
 *    * Neither the name of ABB nor the names of its
 *      contributors may be used to endorse or promote
 *      products derived from this software without
 *      specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ***********************************************************************************************************************
 */

#define _USE_MATH_DEFINES

#include <cmath>

#include "abb_libegm/egm_common.h"

namespace abb
{
namespace egm
{
/***********************************************************************************************************************
 * Struct definitions: Constants
 */

typedef Constants::RobotController RobotController;

const double RobotController::LOWEST_SAMPLE_TIME = 0.004;
const unsigned short RobotController::DEFAULT_PORT_NUMBER = 6511;
const int RobotController::DEFAULT_NUMBER_OF_ROBOT_JOINTS = 6;
const int RobotController::DEFAULT_NUMBER_OF_EXTERNAL_JOINTS = 6;
const int RobotController::MAX_NUMBER_OF_JOINTS = RobotController::DEFAULT_NUMBER_OF_ROBOT_JOINTS +
                                                  RobotController::DEFAULT_NUMBER_OF_EXTERNAL_JOINTS;

const double Constants::Conversion::RAD_2_DEG = 180.0 / M_PI;
const double Constants::Conversion::DEG_2_RAD = M_PI / 180.0;
const double Constants::Conversion::MM_TO_M = 0.001;
const double Constants::Conversion::MS_TO_S = 0.001;
const double Constants::Conversion::S_TO_US = 1e6;

} // end namespace egm
} // end namespace abb
