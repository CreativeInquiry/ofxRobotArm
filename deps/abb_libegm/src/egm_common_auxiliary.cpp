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

#include <boost/math/quaternion.hpp>

#include "abb_libegm/egm_common_auxiliary.h"

namespace abb
{
namespace egm
{
/***********************************************************************************************************************
 * Math functions
 */

double saturate(const double value, const double lower, const double upper)
{
  return (value < lower ? lower : (value > upper ? upper : value));
}

void multiply(wrapper::Joints* p_j, const double factor)
{
  if (p_j)
  {
    for (int i = 0; i < p_j->values_size(); ++i)
    {
      p_j->set_values(i, p_j->values(i)*factor);
    }
  }
}

void multiply(wrapper::Cartesian* p_c, const double factor)
{
  if (p_c)
  {
    p_c->set_x(p_c->x()*factor);
    p_c->set_y(p_c->y()*factor);
    p_c->set_z(p_c->z()*factor);
  }
}

void multiply(wrapper::Euler* p_e, const double factor)
{
  if (p_e)
  {
    p_e->set_x(p_e->x()*factor);
    p_e->set_y(p_e->y()*factor);
    p_e->set_z(p_e->z()*factor);
  }
}

void multiply(wrapper::Quaternion* p_q, const double factor)
{
  if (p_q)
  {
    p_q->set_u0(p_q->u0()*factor);
    p_q->set_u1(p_q->u1()*factor);
    p_q->set_u2(p_q->u2()*factor);
    p_q->set_u3(p_q->u3()*factor);
  }
}

wrapper::Quaternion multiply(const wrapper::Quaternion& q1, const wrapper::Quaternion& q2)
{
  wrapper::Quaternion result;

  result.set_u0(q1.u0()*q2.u0() - q1.u1()*q2.u1() - q1.u2()*q2.u2() - q1.u3()*q2.u3());
  result.set_u1(q1.u0()*q2.u1() + q1.u1()*q2.u0() + q1.u2()*q2.u3() - q1.u3()*q2.u2());
  result.set_u2(q1.u0()*q2.u2() + q1.u2()*q2.u0() + q1.u3()*q2.u1() - q1.u1()*q2.u3());
  result.set_u3(q1.u0()*q2.u3() + q1.u3()*q2.u0() + q1.u1()*q2.u2() - q1.u2()*q2.u1());

  return result;
}

double dotProduct(const wrapper::Quaternion& q1, const wrapper::Quaternion& q2)
{
  return q1.u0()*q2.u0() + q1.u1()*q2.u1() + q1.u2()*q2.u2() + q1.u3()*q2.u3();
}

double euclideanNorm(const wrapper::Quaternion& q)
{
  return std::sqrt(dotProduct(q, q));
}

void normalize(wrapper::Quaternion* p_q)
{
  if (p_q)
  {
    double norm = euclideanNorm(*p_q);
    if (norm != 0.0)
    {
      p_q->set_u0(p_q->u0() / norm);
      p_q->set_u1(p_q->u1() / norm);
      p_q->set_u2(p_q->u2() / norm);
      p_q->set_u3(p_q->u3() / norm);
    }
  }
}

void convert(wrapper::Quaternion* p_q, const wrapper::Euler& e)
{
  if (p_q)
  {
    double z = e.z() * Constants::Conversion::DEG_2_RAD;
    double y = e.y() * Constants::Conversion::DEG_2_RAD;
    double x = e.x() * Constants::Conversion::DEG_2_RAD;

    // Convert ZYX Euler angles to a rotation matrix.
    // See for example https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles for the equations.

    double cx = std::cos(0.5*x);
    double sx = std::sin(0.5*x);
    double cy = std::cos(0.5*y);
    double sy = std::sin(0.5*y);
    double cz = std::cos(0.5*z);
    double sz = std::sin(0.5*z);

    p_q->set_u0(sx*sy*sz + cx*cy*cz);
    p_q->set_u1(-cx*sy*sz + sx*cy*cz);
    p_q->set_u2(sx*cy*sz + cx*sy*cz);
    p_q->set_u3(cx*cy*sz - sx*sy*cz);

    normalize(p_q);
  }
}

void convert(wrapper::Euler* p_e, const wrapper::Quaternion& q)
{
  if(p_e && euclideanNorm(q) != 0.0)
  {
    // Convert a quaternion to ZYX Euler angles.
    // See for example https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles for the equations.
    //
    // Handle singularities.
    // See for example http://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToEuler/index.htm
    // for indications of how to derive the equations.

    double y = 0.0;
    double z = 0.0;
    double x = 0.0;

    double u0 = q.u0();
    double u1 = q.u1();
    double u2 = q.u2();
    double u3 = q.u3();

    const double SINGULARITY_THRESHOLD = 0.000001;
    double singularity_check = u0*u2 - u1*u3;

    // Check for singularity (i.e. when y is close to +- 90 degrees).
    // This occur when the argument of y = sin(2*(u0*u2 - u1*u3)) is close to 1.0 -> u0*u2 - u1*u3 = 0.5.
    if (std::abs(singularity_check - 0.5) <= SINGULARITY_THRESHOLD)
    {
      // Y is close to 90 degrees.
      x = 2.0*std::atan2(u1, u0);
      y = M_PI_2;
    }
    else if (std::abs(singularity_check + 0.5) <= SINGULARITY_THRESHOLD)
    {
      // Y is close to -90 degrees.
      x = 2.0*std::atan2(u1, u0);
      y = -M_PI_2;
    }
    else
    {
      x = std::atan2(2.0*(u0*u1 + u2*u3), 1.0 - 2.0*(u1*u1 + u2*u2));
      y = std::asin(2.0*(u0*u2 - u1*u3));
      z = std::atan2(2.0*(u0*u3 + u1*u2), 1.0 - 2.0*(u2*u2 + u3*u3));
    }

    p_e->set_x(x*Constants::Conversion::RAD_2_DEG);
    p_e->set_y(y*Constants::Conversion::RAD_2_DEG);
    p_e->set_z(z*Constants::Conversion::RAD_2_DEG);
  }
}

void convert(wrapper::Quaternion* p_dq, const wrapper::Quaternion& previous_q, const wrapper::Euler& av)
{
  if (p_dq)
  {
    wrapper::Quaternion temp;
    temp.set_u0(0.0);
    temp.set_u1(av.x()*Constants::Conversion::DEG_2_RAD);
    temp.set_u2(av.y()*Constants::Conversion::DEG_2_RAD);
    temp.set_u3(av.z()*Constants::Conversion::DEG_2_RAD);

    p_dq->CopyFrom(multiply(temp, previous_q));
    multiply(p_dq, 0.5);
  }
}




/***********************************************************************************************************************
 * Estimation functions
 */

bool estimateVelocities(wrapper::Joints* p_estimate,
                        const wrapper::Joints& current,
                        const wrapper::Joints& previous,
                        const double sample_time)
{
  bool success = false;
  double delta_speed = 0.0;

  if (p_estimate && sample_time > 0.0)
  {
    p_estimate->Clear();

    for (int i = 0; i < current.values_size() && i < previous.values_size(); ++i)
    {
      delta_speed = current.values(i) - previous.values(i);
      p_estimate->add_values(delta_speed / sample_time);
    }
    success = true;
  }

  return success;
}

bool estimateVelocities(wrapper::Euler* p_estimate,
                        const wrapper::Quaternion& current,
                        const wrapper::Quaternion& previous,
                        const double sample_time)
{
  bool success = false;

  if (p_estimate && sample_time > 0.0)
  {
    // Estimate the angular velocity.
    // See for example https://en.wikipedia.org/wiki/Rotation_formalisms_in_three_dimensions for equations.
    // Note: Only valid for orientations, for the same object, at two points close in time.
    // Also assumes constant angular velocity between the points.
    boost::math::quaternion<double> q1(previous.u0(), previous.u1(),
                                       previous.u2(), previous.u3());

    boost::math::quaternion<double> q2(current.u0(), current.u1(),
                                       current.u2(), current.u3());

    boost::math::quaternion<double> estimation = ((2.0*(q2 - q1) / sample_time)*boost::math::conj(q1));

    p_estimate->set_x(estimation.R_component_2()*Constants::Conversion::RAD_2_DEG);
    p_estimate->set_y(estimation.R_component_3()*Constants::Conversion::RAD_2_DEG);
    p_estimate->set_z(estimation.R_component_4()*Constants::Conversion::RAD_2_DEG);

    success = true;
  }

  return success;
}

bool estimateVelocities(wrapper::CartesianVelocity* p_estimate,
                        const wrapper::CartesianPose& current,
                        const wrapper::CartesianPose& previous,
                        const double sample_time)
{
  bool success = false;

  if (p_estimate && sample_time > 0.0)
  {
    // Estimate the linear velocity.
    p_estimate->mutable_linear()->set_x((current.position().x() - previous.position().x()) / sample_time);
    p_estimate->mutable_linear()->set_y((current.position().y() - previous.position().y()) / sample_time);
    p_estimate->mutable_linear()->set_z((current.position().z() - previous.position().z()) / sample_time);

    // Estimate the angular velocity.
    success = estimateVelocities(p_estimate->mutable_angular(),
                                 current.quaternion(),
                                 previous.quaternion(),
                                 sample_time);
  }

  return success;
}




/***********************************************************************************************************************
 * Find functions
 */

double findMaxDifference(const wrapper::Joints& j1, const wrapper::Joints& j2)
{
  double max_difference = 0.0;

  for (int i = 0; i < j1.values_size() && i < j2.values_size(); ++i)
  {
    max_difference = std::max(max_difference, std::abs(j1.values(i) - j2.values(i)));
  }

  return max_difference;
}

double findMaxDifference(const wrapper::Cartesian& c1, const wrapper::Cartesian& c2)
{
  double max_difference = std::abs(c1.x() - c2.x());
  max_difference = std::max(max_difference, std::abs(c1.y() - c2.y()));
  max_difference = std::max(max_difference, std::abs(c1.z() - c2.z()));

  return max_difference;
}

double findMaxDifference(const wrapper::Euler& e1, const wrapper::Euler& e2)
{
  double max_difference = std::abs(e1.x() - e2.x());
  max_difference = std::max(max_difference, std::abs(e1.y() - e2.y()));
  max_difference = std::max(max_difference, std::abs(e1.z() - e2.z()));

  return max_difference;
}




/***********************************************************************************************************************
 * Copy functions
 */

void copyPresent(wrapper::Joints* p_target, const wrapper::Joints& source)
{
  if (p_target)
  {
    for (int i = 0; i < source.values_size() && i < p_target->values_size(); ++i)
    {
      p_target->set_values(i, source.values(i));
    }
  }
}

void copyPresent(wrapper::JointSpace* p_target, const wrapper::JointSpace& source)
{
  if (p_target)
  {
    if (source.has_position())
    {
      copyPresent(p_target->mutable_position(), source.position());
    }

    if (source.has_velocity())
    {
      copyPresent(p_target->mutable_velocity(), source.velocity());
    }
  }
}

void copyPresent(wrapper::Cartesian* p_target, const wrapper::Cartesian& source)
{
  if (p_target)
  {
    if (source.has_x())
    {
      p_target->set_x(source.x());
    }

    if (source.has_y())
    {
      p_target->set_y(source.y());
    }

    if (source.has_z())
    {
      p_target->set_z(source.z());
    }
  }
}

void copyPresent(wrapper::Euler* p_target, const wrapper::Euler& source)
{
  if (p_target)
  {
    if (source.has_x())
    {
      p_target->set_x(source.x());
    }

    if (source.has_y())
    {
      p_target->set_y(source.y());
    }

    if (source.has_z())
    {
      p_target->set_z(source.z());
    }
  }
}

void copyPresent(wrapper::Quaternion* p_target, const wrapper::Quaternion& source)
{
  if (p_target)
  {
    if (source.has_u0())
    {
      p_target->set_u0(source.u0());
    }

    if (source.has_u1())
    {
      p_target->set_u1(source.u1());
    }

    if (source.has_u2())
    {
      p_target->set_u2(source.u2());
    }

    if (source.has_u3())
    {
      p_target->set_u3(source.u3());
    }

    normalize(p_target);
  }
}

void copyPresent(wrapper::CartesianPose* p_target, const wrapper::CartesianPose& source)
{
  if (p_target)
  {
    if (source.has_position())
    {
      copyPresent(p_target->mutable_position(), source.position());
    }

    if (source.has_euler())
    {
      copyPresent(p_target->mutable_euler(), source.euler());
      convert(p_target->mutable_quaternion(), p_target->euler());
    }
    else if (source.has_quaternion())
    {
      copyPresent(p_target->mutable_quaternion(), source.quaternion());
      normalize(p_target->mutable_quaternion());
      convert(p_target->mutable_euler(), p_target->quaternion());
    }
  }
}

void copyPresent(wrapper::CartesianVelocity* p_target, const wrapper::CartesianVelocity& source)
{
  if (p_target)
  {
    if (source.has_linear())
    {
      copyPresent(p_target->mutable_linear(), source.linear());
    }

    if (source.has_angular())
    {
      copyPresent(p_target->mutable_angular(), source.angular());
    }
  }
}

void copyPresent(wrapper::CartesianSpace* p_target, const wrapper::CartesianSpace& source)
{
  if (p_target)
  {
    if (source.has_pose())
    {
      copyPresent(p_target->mutable_pose(), source.pose());
    }

    if (source.has_velocity())
    {
      copyPresent(p_target->mutable_velocity(), source.velocity());
    }
  }
}

void copyPresent(wrapper::Robot* p_target, const wrapper::Robot& source)
{
  if (p_target)
  {
    if (source.has_joints())
    {
      copyPresent(p_target->mutable_joints(), source.joints());
    }

    if (source.has_cartesian())
    {
      copyPresent(p_target->mutable_cartesian(), source.cartesian());
    }
  }
}

void copyPresent(wrapper::External* p_target, const wrapper::External& source)
{
  if (p_target)
  {
    if (source.has_joints())
    {
      copyPresent(p_target->mutable_joints(), source.joints());
    }
  }
}

void copyPresent(wrapper::Output* p_target, const wrapper::Output& source)
{
  if (p_target)
  {
    if (source.has_robot())
    {
      copyPresent(p_target->mutable_robot(), source.robot());
    }

    if (source.has_external())
    {
      copyPresent(p_target->mutable_external(), source.external());
    }
  }
}




/***********************************************************************************************************************
 * Parse functions
 */

bool parse(wrapper::Header* p_target, const EgmHeader& source)
{
  bool success = false;

  if (p_target && source.has_seqno() && source.has_tm() && source.has_mtype())
  {
    p_target->set_sequence_number(source.seqno());
    p_target->set_time_stamp(source.tm());

    switch (source.mtype())
    {
      case EgmHeader_MessageType_MSGTYPE_DATA:
      {
        p_target->set_message_type(wrapper::Header_MessageType_DATA);
        success = true;
      }
      break;

      case EgmHeader_MessageType_MSGTYPE_UNDEFINED:
      case EgmHeader_MessageType_MSGTYPE_COMMAND:
      case EgmHeader_MessageType_MSGTYPE_CORRECTION:
      case EgmHeader_MessageType_MSGTYPE_PATH_CORRECTION:
      default:
      {
        p_target->set_message_type(wrapper::Header_MessageType_UNDEFINED);
      }
    }
  }

  return success;
}

bool parse(wrapper::Status* p_target, const EgmRobot& source)
{
  bool success = false;

  if (p_target &&
      source.has_motorstate() && source.motorstate().has_state() &&
      source.has_mcistate() && source.mcistate().has_state() &&
      source.has_rapidexecstate() && source.rapidexecstate().has_state() &&
      source.has_mciconvergencemet())
  {
    switch (source.motorstate().state())
    {
      case EgmMotorState_MotorStateType_MOTORS_UNDEFINED:
      {
        p_target->set_motor_state(wrapper::Status_MotorState_MOTORS_UNDEFINED);
      }
      break;

      case EgmMotorState_MotorStateType_MOTORS_ON:
      {
        p_target->set_motor_state(wrapper::Status_MotorState_MOTORS_ON);
      }
      break;

      case EgmMotorState_MotorStateType_MOTORS_OFF:
      {
        p_target->set_motor_state(wrapper::Status_MotorState_MOTORS_OFF);
      }
      break;

      default:
      {
        p_target->set_motor_state(wrapper::Status_MotorState_MOTORS_UNDEFINED);
      }
    }

    switch (source.mcistate().state())
    {
      case EgmMCIState_MCIStateType_MCI_UNDEFINED:
      {
        p_target->set_egm_state(wrapper::Status_EGMState_EGM_UNDEFINED);
      }
      break;

      case EgmMCIState_MCIStateType_MCI_ERROR:
      {
        p_target->set_egm_state(wrapper::Status_EGMState_EGM_ERROR);
      }
      break;

      case EgmMCIState_MCIStateType_MCI_STOPPED:
      {
        p_target->set_egm_state(wrapper::Status_EGMState_EGM_STOPPED);
      }
      break;

      case EgmMCIState_MCIStateType_MCI_RUNNING:
      {
        p_target->set_egm_state(wrapper::Status_EGMState_EGM_RUNNING);
      }
      break;

      default:
      {
        p_target->set_egm_state(wrapper::Status_EGMState_EGM_UNDEFINED);
      }
    }

    switch (source.rapidexecstate().state())
    {
      case EgmRapidCtrlExecState_RapidCtrlExecStateType_RAPID_UNDEFINED:
      {
        p_target->set_rapid_execution_state(wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED);
      }
      break;

      case EgmRapidCtrlExecState_RapidCtrlExecStateType_RAPID_STOPPED:
      {
        p_target->set_rapid_execution_state(wrapper::Status_RAPIDExecutionState_RAPID_STOPPED);
      }
      break;

      case EgmRapidCtrlExecState_RapidCtrlExecStateType_RAPID_RUNNING:
      {
        p_target->set_rapid_execution_state(wrapper::Status_RAPIDExecutionState_RAPID_RUNNING);
      }
      break;

      default:
      {
        p_target->set_rapid_execution_state(wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED);
      }
    }

    p_target->set_egm_convergence_met(source.mciconvergencemet());

    if(source.has_utilizationrate())
    {
      p_target->set_utilization_rate(source.utilizationrate());
    }

    success = true;
  }

  return success;
}

bool parse(wrapper::Clock* p_target, const EgmClock& source)
{
  bool success = true;

  if (p_target && source.has_sec() && source.has_usec())
  {
    p_target->set_sec(source.sec());
    p_target->set_usec(source.usec());
  }
  else
  {
    success = false;
  }

  return success;
}

bool parse(wrapper::Joints* p_target_robot,
           wrapper::Joints* p_target_external,
           const EgmJoints& source_robot,
           const EgmJoints& source_external,
           const RobotAxes axes)
{
  bool success = false;

  if (p_target_robot && p_target_external)
  {
    p_target_robot->Clear();
    p_target_external->Clear();

    switch (axes)
    {
      case None:
      {
        if (source_robot.joints_size() == 0)
        {
          for (int i = 0; i < source_external.joints_size(); ++i)
          {
            p_target_external->add_values(source_external.joints(i));
          }

          success = true;
        }
      }
      break;

      case Six:
      {
        if (source_robot.joints_size() == Constants::RobotController::DEFAULT_NUMBER_OF_ROBOT_JOINTS)
        {
          for (int i = 0; i < source_robot.joints_size(); ++i)
          {
            p_target_robot->add_values(source_robot.joints(i));
          }

          for (int i = 0; i < source_external.joints_size(); ++i)
          {
            p_target_external->add_values(source_external.joints(i));
          }

          success = true;
        }
      }
      break;

      case Seven:
      {
        // If using a seven axes robot (e.g. IRB14000): Map to special case.
        if (source_robot.joints_size() == Constants::RobotController::DEFAULT_NUMBER_OF_ROBOT_JOINTS &&
            source_external.joints_size() >= 1)
        {
          p_target_robot->add_values(source_robot.joints(0));
          p_target_robot->add_values(source_robot.joints(1));
          p_target_robot->add_values(source_external.joints(0));
          p_target_robot->add_values(source_robot.joints(2));
          p_target_robot->add_values(source_robot.joints(3));
          p_target_robot->add_values(source_robot.joints(4));
          p_target_robot->add_values(source_robot.joints(5));

          for (int i = 1; i < source_external.joints_size(); ++i)
          {
            p_target_external->add_values(source_external.joints(i));
          }

          success = true;
        }
      }
      break;
    }
  }

  return success;
}

bool parse(wrapper::CartesianPose* p_target, const EgmPose& source)
{
  bool success = true;

  if (p_target)
  {
    p_target->Clear();

    if (source.has_pos() && source.pos().has_x() && source.pos().has_y() && source.pos().has_z())
    {
      p_target->mutable_position()->set_x(source.pos().x());
      p_target->mutable_position()->set_y(source.pos().y());
      p_target->mutable_position()->set_z(source.pos().z());
    }
    else
    {
      success = false;
    }

    if (success &&
        source.has_orient() &&
        source.orient().has_u0() &&
        source.orient().has_u1() &&
        source.orient().has_u2() &&
        source.orient().has_u3())
    {
      p_target->mutable_quaternion()->set_u0(source.orient().u0());
      p_target->mutable_quaternion()->set_u1(source.orient().u1());
      p_target->mutable_quaternion()->set_u2(source.orient().u2());
      p_target->mutable_quaternion()->set_u3(source.orient().u3());
    }
    else
    {
      success = false;
    }

    if (success)
    {
      if (source.has_euler() && source.euler().has_x() && source.euler().has_y() && source.euler().has_z())
      {
        p_target->mutable_euler()->set_x(source.euler().x());
        p_target->mutable_euler()->set_y(source.euler().y());
        p_target->mutable_euler()->set_z(source.euler().z());
      }
      else
      {
        convert(p_target->mutable_euler(), p_target->quaternion());
      }
    }
  }

  return success;
}

bool parse(wrapper::Feedback* p_target, const EgmFeedBack& source, const RobotAxes axes)
{
  bool success = false;

  if (p_target)
  {
    success = parse(p_target->mutable_robot()->mutable_joints()->mutable_position(),
                    p_target->mutable_external()->mutable_joints()->mutable_position(),
                    source.joints(), source.externaljoints(), axes);

    if (success)
    {
      if(axes == None)
      {
        success = !source.has_cartesian();
      }
      else
      {
        success = parse(p_target->mutable_robot()->mutable_cartesian()->mutable_pose(), source.cartesian());
      }

      if (success)
      {
        success = parse(p_target->mutable_time(), source.time());
      }
    }
  }

  return success;
}

bool parse(wrapper::Planned* p_target, const EgmPlanned& source, const RobotAxes axes)
{
  bool success = false;

  if (p_target)
  {
    success = parse(p_target->mutable_robot()->mutable_joints()->mutable_position(),
                    p_target->mutable_external()->mutable_joints()->mutable_position(),
                    source.joints(), source.externaljoints(), axes);

    if (success)
    {
      if(axes == None)
      {
        success = !source.has_cartesian();
      }
      else
      {
        success = parse(p_target->mutable_robot()->mutable_cartesian()->mutable_pose(), source.cartesian());
      }

      if (success)
      {
        success = parse(p_target->mutable_time(), source.time());
      }
    }
  }

  return success;
}




/***********************************************************************************************************************
 * Reset functions
 */

void reset(wrapper::Joints* p_joints, const unsigned int number_of_joints)
{
  if (p_joints)
  {
    p_joints->Clear();

    for (unsigned int i = 0; i < number_of_joints; ++i)
    {
      p_joints->add_values(0.0);
    }
  }
}

void reset(wrapper::Cartesian* p_cartesian)
{
  if (p_cartesian)
  {
    p_cartesian->set_x(0.0);
    p_cartesian->set_y(0.0);
    p_cartesian->set_z(0.0);
  }
}

void reset(wrapper::Euler* p_euler)
{
  if (p_euler)
  {
    p_euler->set_x(0.0);
    p_euler->set_y(0.0);
    p_euler->set_z(0.0);
  }
}




/***********************************************************************************************************************
 * Verify functions
 */

bool verify(const double value)
{
  return !std::isnan(value) && !std::isinf(value);
}

bool verify(const wrapper::Joints& joints)
{
  for (int i = 0; i < joints.values_size(); ++i)
  {
    if(!verify(joints.values(i)))
    {
      return false;
    }
  }

  return true;
}

bool verify(const wrapper::Cartesian& cartesian)
{
  if((cartesian.has_x() && !verify(cartesian.x())) ||
     (cartesian.has_y() && !verify(cartesian.y())) ||
     (cartesian.has_z() && !verify(cartesian.z())))
  {
    return false;
  }

  return true;
}

bool verify(const wrapper::Euler& euler)
{
  if((euler.has_x() && !verify(euler.x())) ||
     (euler.has_y() && !verify(euler.y())) ||
     (euler.has_z() && !verify(euler.z())))
  {
    return false;
  }

  return true;
}

bool verify(const wrapper::Quaternion& quaternion)
{
  if((quaternion.has_u0() && !verify(quaternion.u0())) ||
     (quaternion.has_u1() && !verify(quaternion.u1())) ||
     (quaternion.has_u2() && !verify(quaternion.u2())) ||
     (quaternion.has_u3() && !verify(quaternion.u3())))
  {
    return false;
  }

  return true;
}

bool verify(const wrapper::CartesianPose& pose)
{
  if((pose.has_position() && !verify(pose.position())) ||
     (pose.has_euler() && !verify(pose.euler())) ||
     (pose.has_quaternion() && !verify(pose.quaternion())))
  {
    return false;
  }

  return true;
}

bool verify(const wrapper::CartesianVelocity& velocity)
{
  if((velocity.has_angular() && !verify(velocity.angular())) ||
     (velocity.has_linear() && !verify(velocity.linear())))
  {
    return false;
  }

  return true;
}

} // end namespace egm
} // end namespace abb
