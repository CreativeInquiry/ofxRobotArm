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

#include <sstream>

#include "abb_libegm/egm_common.h"
#include "abb_libegm/egm_logger.h"

namespace abb
{
namespace egm
{
/***********************************************************************************************************************
 * Class definitions: EGMLogger
 */

/************************************************************
 * Primary methods
 */

EGMLogger::EGMLogger(const std::string& filename, const bool use_default_headers)
:
number_of_logged_messages_(0)
{
  log_stream_.open(filename.c_str(), std::ios::trunc);

  if (use_default_headers)
  {
    std::stringstream ss;
    ss << "TIMESTAMP,"
       // Robot feedback.
       << "R_FB_POS_RJ1,R_FB_POS_RJ2,R_FB_POS_RJ3,R_FB_POS_RJ4,R_FB_POS_RJ5,R_FB_POS_RJ6,"
       << "R_FB_POS_EJ1,R_FB_POS_EJ2,R_FB_POS_EJ3,R_FB_POS_EJ4,R_FB_POS_EJ5,R_FB_POS_EJ6,"
       << "R_FB_VEL_RJ1,R_FB_VEL_RJ2,R_FB_VEL_RJ3,R_FB_VEL_RJ4,R_FB_VEL_RJ5,R_FB_VEL_RJ6,"
       << "R_FB_VEL_EJ1,R_FB_VEL_EJ2,R_FB_VEL_EJ3,R_FB_VEL_EJ4,R_FB_VEL_EJ5,R_FB_VEL_EJ6,"
       << "R_FB_POS_X,R_FB_POS_Y,R_FB_POS_Z,"
       << "R_FB_EULER_X,R_FB_EULER_Y,R_FB_EULER_Z,"
       << "R_FB_QUAT_U0,R_FB_QUAT_U1,R_FB_QUAT_U2,R_FB_QUAT_U3,"
       << "R_FB_VEL_LX,R_FB_VEL_LY,R_FB_VEL_LZ,"
       << "R_FB_VEL_AX,R_FB_VEL_AY,R_FB_VEL_AZ,"
       // Robot planned.
       << "R_PL_POS_RJ1,R_PL_POS_RJ2,R_PL_POS_RJ3,R_PL_POS_RJ4,R_PL_POS_RJ5,R_PL_POS_RJ6,"
       << "R_PL_POS_EJ1,R_PL_POS_EJ2,R_PL_POS_EJ3,R_PL_POS_EJ4,R_PL_POS_EJ5,R_PL_POS_EJ6,"
       << "R_PL_VEL_RJ1,R_PL_VEL_RJ2,R_PL_VEL_RJ3,R_PL_VEL_RJ4,R_PL_VEL_RJ5,R_PL_VEL_RJ6,"
       << "R_PL_VEL_EJ1,R_PL_VEL_EJ2,R_PL_VEL_EJ3,R_PL_VEL_EJ4,R_PL_VEL_EJ5,R_PL_VEL_EJ6,"
       << "R_PL_POS_X,R_PL_POS_Y,R_PL_POS_Z,"
       << "R_PL_EULER_X,R_PL_EULER_Y,R_PL_EULER_Z,"
       << "R_PL_QUAT_U0,R_PL_QUAT_U1,R_PL_QUAT_U2,R_PL_QUAT_U3,"
       << "R_PL_VEL_LX,R_PL_VEL_LY,R_PL_VEL_LZ,"
       << "R_PL_VEL_AX,R_PL_VEL_AY,R_PL_VEL_AZ,"
       // Sensor references.
       << "S_REF_POS_RJ1,S_REF_POS_RJ2,S_REF_POS_RJ3,S_REF_POS_RJ4,S_REF_POS_RJ5,S_REF_POS_RJ6,"
       << "S_REF_POS_EJ1,S_REF_POS_EJ2,S_REF_POS_EJ3,S_REF_POS_EJ4,S_REF_POS_EJ5,S_REF_POS_EJ6,"
       << "S_REF_VEL_RJ1,S_REF_VEL_RJ2,S_REF_VEL_RJ3,S_REF_VEL_RJ4,S_REF_VEL_RJ5,S_REF_VEL_RJ6,"
       << "S_REF_VEL_EJ1,S_REF_VEL_EJ2,S_REF_VEL_EJ3,S_REF_VEL_EJ4,S_REF_VEL_EJ5,S_REF_VEL_EJ6,"
       << "S_REF_POS_X,S_REF_POS_Y,S_REF_POS_Z,"
       << "S_REF_EULER_X,S_REF_EULER_Y,S_REF_EULER_Z,"
       << "S_REF_QUAT_U0,S_REF_QUAT_U1,S_REF_QUAT_U2,S_REF_QUAT_U3,"
       << "S_REF_VEL_LX,S_REF_VEL_LY,S_REF_VEL_LZ,"
       << "S_REF_VEL_AX,S_REF_VEL_AY,S_REF_VEL_AZ\n";

    log_stream_ << ss.str();
    log_stream_.flush();
  }
}

EGMLogger::~EGMLogger()
{
  log_stream_.close();
}

void EGMLogger::flush()
{
  log_stream_ << "\n";
  log_stream_.flush();
  ++number_of_logged_messages_;
}

void EGMLogger::add(const wrapper::Header& header)
{
  log_stream_ << header.time_stamp() << ",";
}

void EGMLogger::add(const wrapper::Joints& robot, const wrapper::Joints& external)
{
  google::protobuf::RepeatedField<double>::const_iterator i;

  for (i = robot.values().begin(); i != robot.values().end(); ++i)
  {
    log_stream_ << *i << ",";
  }
  addMockJoints(true, robot.values_size(), external.values_size());

  for (i = external.values().begin(); i != external.values().end(); ++i)
  {
    log_stream_ << *i << ",";
  }
  addMockJoints(false, robot.values_size(), external.values_size());
}

void EGMLogger::add(const wrapper::CartesianPose& pose)
{
  log_stream_ << pose.position().x() << ","
              << pose.position().y() << ","
              << pose.position().z() << ",";

  log_stream_ << pose.euler().x() << ","
              << pose.euler().y() << ","
              << pose.euler().z() << ",";

  log_stream_ << pose.quaternion().u0() << ","
              << pose.quaternion().u1() << ","
              << pose.quaternion().u2() << ","
              << pose.quaternion().u3() << ",";
}

void EGMLogger::add(const wrapper::CartesianVelocity& velocity, const bool last)
{
  log_stream_ << velocity.linear().x() << ","
              << velocity.linear().y() << ","
              << velocity.linear().z() << ",";

  log_stream_ << velocity.angular().x() << ","
              << velocity.angular().y() << ","
              << velocity.angular().z() << (last ? "" : ",");
}

/************************************************************
 * Auxiliary methods
 */

void EGMLogger::addMockJoints(const bool robot, const size_t robot_size, const size_t external_size)
{
  if (robot)
  {
    // Add mock values for the missing robot joint data.
    size_t condition = Constants::RobotController::DEFAULT_NUMBER_OF_ROBOT_JOINTS;
    for (size_t i = robot_size; i < condition; ++i)
    {
      log_stream_ << 0.0 << ",";
    }
  }
  else
  {
    // Add mock values for the missing external joint data.
    size_t condition = Constants::RobotController::MAX_NUMBER_OF_JOINTS - robot_size;
    for (size_t i = external_size; i < condition; ++i)
    {
      log_stream_ << 0.0 << ",";
    }
  }
}

double EGMLogger::calculateTimeLogged(const double sample_time)
{
  return (double)number_of_logged_messages_*sample_time;
}

} // end namespace egm
} // end namespace abb
