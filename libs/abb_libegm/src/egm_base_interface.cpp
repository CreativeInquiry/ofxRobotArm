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
#include <sstream>

#include "abb_libegm/egm_base_interface.h"
#include "abb_libegm/egm_common_auxiliary.h"

namespace abb
{
namespace egm
{
/***********************************************************************************************************************
 * Class definitions: EGMBaseInterface::InputContainer
 */

/************************************************************
 * Primary methods
 */

EGMBaseInterface::InputContainer::InputContainer()
:
has_new_data_(false),
first_call_(true),
first_message_(false),
estimated_sample_time_(Constants::RobotController::LOWEST_SAMPLE_TIME)
{};

bool EGMBaseInterface::InputContainer::parseFromArray(const char* data, const int bytes_transferred)
{
  has_new_data_ = false;

  if (data)
  {
    has_new_data_ = egm_robot_.ParseFromArray(data, bytes_transferred);
  }

  if (has_new_data_)
  {
    first_message_ = (first_call_ || egm_robot_.header().seqno() == 0);
    first_call_ = false;
  }

  return has_new_data_;
}

bool EGMBaseInterface::InputContainer::extractParsedInformation(const RobotAxes& axes)
{
  bool success = false;

  detectRWAndEGMVersions();

  if (has_new_data_ &&
      parse(current_.mutable_header(), egm_robot_.header()) &&
      parse(current_.mutable_feedback(), egm_robot_.feedback(), axes) &&
      parse(current_.mutable_planned(), egm_robot_.planned(), axes) &&
      parse(current_.mutable_status(), egm_robot_))
  {
    if (first_message_)
    {
      initial_.CopyFrom(current_);
      previous_.CopyFrom(current_);
    }

    estimated_sample_time_ = estimateSampleTime();
    success = estimateAllVelocities();

    has_new_data_ = false;
  }

  return success;
}

void EGMBaseInterface::InputContainer::updatePrevious()
{
  previous_.CopyFrom(current_);
}

bool EGMBaseInterface::InputContainer::statesOk() const
{
  // EGM knows about the following RAPID execution states:
  // - UNDEFINED
  // - STOPPED
  // - RUNNING
  //
  // There is a bug in EGM that after a restart of the robot controller, then RAPID is in the UNDEFINED state even if
  // it has clearly been started. Allowing both RUNNING and UNDEFINED states to be acceptable is a workaround for this.
  // The rationale is also that the robot controller should internally ignore EGM commands if it's in a bad state.
  return (current_.status().motor_state() == wrapper::Status_MotorState_MOTORS_ON &&
          (current_.status().rapid_execution_state() == wrapper::Status_RAPIDExecutionState_RAPID_UNDEFINED ||
           current_.status().rapid_execution_state() == wrapper::Status_RAPIDExecutionState_RAPID_RUNNING) &&
          current_.status().egm_state() == wrapper::Status_EGMState_EGM_RUNNING);
}

/************************************************************
 * Auxiliary methods
 */

void EGMBaseInterface::InputContainer::detectRWAndEGMVersions()
{
  if(has_new_data_)
  {
    // Time field was added in RobotWare '6.07', as well as fix of inconsistent units (e.g. radians and degrees).
    if(egm_robot_.feedback().has_time())
    {
      // If time field present:
      // - RW greater than or equal to '6.07'.
      // - EGM protocol '1.1' (with consistent units).
      current_.mutable_header()->set_egm_version(wrapper::Header_EGMVersion_EGM_1_1);

      // Utilization field was added in RobotWare '6.10'.
      if(egm_robot_.has_utilizationrate())
      {
        // If utilization field present:
        // - RW greater than or equal to '6.10'.
        current_.mutable_header()->set_rw_version(wrapper::Header_RWVersion_RW_6_10_AND_NEWER);
      }
      else
      {
        // If utilization field absent:
        // - RW between '6.07' and '6.09.02'.
        current_.mutable_header()->set_rw_version(wrapper::Header_RWVersion_RW_BETWEEN_6_07_AND_6_09_02);
      }
    }
    else
    {
      // If time field absent:
      // - RW between '6.0' and '6.06.03'.
      // - EGM protocol '1.0' (with inconsistent units).
      current_.mutable_header()->set_rw_version(wrapper::Header_RWVersion_RW_BETWEEN_6_AND_6_06_03);
      current_.mutable_header()->set_egm_version(wrapper::Header_EGMVersion_EGM_1_0);
    }
  }
  else
  {
    current_.mutable_header()->set_rw_version(wrapper::Header_RWVersion_RW_UNKNOWN);
    current_.mutable_header()->set_egm_version(wrapper::Header_EGMVersion_EGM_UNKNOWN);
  }
}

double EGMBaseInterface::InputContainer::estimateSampleTime()
{
  double estimate = 0.0;

  if (current_.has_feedback() && previous_.has_feedback() &&
      current_.feedback().has_time() && previous_.feedback().has_time() &&
      current_.feedback().time().has_sec() && previous_.feedback().time().has_sec() &&
      current_.feedback().time().has_usec() && previous_.feedback().time().has_usec())
  {
    google::protobuf::uint64 diff_s = (current_.feedback().time().sec() - previous_.feedback().time().sec());
    google::protobuf::uint64 diff_us = (current_.feedback().time().usec() - previous_.feedback().time().usec());
    if (diff_s > 0)
    {
      diff_us += diff_s*((google::protobuf::uint64) Constants::Conversion::S_TO_US);
    }

    estimate = std::floor(((double) diff_us) * Constants::Conversion::MS_TO_S) * Constants::Conversion::MS_TO_S;
  }

  if (estimate < Constants::RobotController::LOWEST_SAMPLE_TIME)
  {
    estimate = Constants::RobotController::LOWEST_SAMPLE_TIME;
  }

  return estimate;
}

bool EGMBaseInterface::InputContainer::estimateAllVelocities()
{
  //---------------------------------------------------------
  // Feedback
  //---------------------------------------------------------
  bool success = estimateVelocities(current_.mutable_feedback()->mutable_robot()->mutable_joints()->mutable_velocity(),
                                    current_.feedback().robot().joints().position(),
                                    previous_.feedback().robot().joints().position(),
                                    estimated_sample_time_);

  if (success)
  {
    success = estimateVelocities(current_.mutable_feedback()->mutable_robot()->mutable_cartesian()->mutable_velocity(),
                                 current_.feedback().robot().cartesian().pose(),
                                 previous_.feedback().robot().cartesian().pose(),
                                 estimated_sample_time_);
  }

  if (success)
  {
    success = estimateVelocities(current_.mutable_feedback()->mutable_external()->mutable_joints()->mutable_velocity(),
                                 current_.feedback().external().joints().position(),
                                 previous_.feedback().external().joints().position(),
                                 estimated_sample_time_);
  }

  //---------------------------------------------------------
  // Planned
  //---------------------------------------------------------
  if (success)
  {
    success = estimateVelocities(current_.mutable_planned()->mutable_robot()->mutable_joints()->mutable_velocity(),
                                 current_.planned().robot().joints().position(),
                                 previous_.planned().robot().joints().position(),
                                 estimated_sample_time_);
  }

  if (success)
  {
    success = estimateVelocities(current_.mutable_planned()->mutable_robot()->mutable_cartesian()->mutable_velocity(),
                                 current_.planned().robot().cartesian().pose(),
                                 previous_.planned().robot().cartesian().pose(),
                                 estimated_sample_time_);
  }

  if (success)
  {
    success = estimateVelocities(current_.mutable_planned()->mutable_external()->mutable_joints()->mutable_velocity(),
                                 current_.planned().external().joints().position(),
                                 previous_.planned().external().joints().position(),
                                 estimated_sample_time_);
  }

  return success;
}




/***********************************************************************************************************************
 * Class definitions: EGMBaseInterface::OutputContainer
 */

/************************************************************
 * Primary methods
 */

EGMBaseInterface::OutputContainer::OutputContainer() : sequence_number_(0) {}

void EGMBaseInterface::OutputContainer::prepareOutputs(const InputContainer& inputs)
{
  sequence_number_ = (inputs.isFirstMessage() ? 0 : sequence_number_ + 1);

  if (inputs.isFirstMessage())
  {
    const wrapper::Feedback& feedback = inputs.current().feedback();
    wrapper::Robot* p_robot = current.mutable_robot();
    wrapper::External* p_external = current.mutable_external();

    // Joint positions and Cartesian pose.
    p_robot->mutable_joints()->mutable_position()->CopyFrom(feedback.robot().joints().position());
    p_robot->mutable_cartesian()->mutable_pose()->CopyFrom(feedback.robot().cartesian().pose());
    p_external->mutable_joints()->mutable_position()->CopyFrom(feedback.external().joints().position());

    // Joint velocities.
    p_robot->mutable_joints()->clear_velocity();
    for (int i = 0; i < feedback.robot().joints().velocity().values_size(); ++i)
    {
      p_robot->mutable_joints()->mutable_velocity()->add_values(0.0);
    }

    p_external->mutable_joints()->clear_velocity();
    for (int i = 0; i < feedback.external().joints().velocity().values_size(); ++i)
    {
      p_external->mutable_joints()->mutable_velocity()->add_values(0.0);
    }

    // Cartesian velocities.
    p_robot->mutable_cartesian()->mutable_velocity()->mutable_linear()->set_x(0.0);
    p_robot->mutable_cartesian()->mutable_velocity()->mutable_linear()->set_y(0.0);
    p_robot->mutable_cartesian()->mutable_velocity()->mutable_linear()->set_z(0.0);
    p_robot->mutable_cartesian()->mutable_velocity()->mutable_angular()->set_x(0.0);
    p_robot->mutable_cartesian()->mutable_velocity()->mutable_angular()->set_y(0.0);
    p_robot->mutable_cartesian()->mutable_velocity()->mutable_angular()->set_z(0.0);

    previous_.CopyFrom(current);
  }
}

void EGMBaseInterface::OutputContainer::generateDemoOutputs(const InputContainer& inputs)
{
  unsigned int seqno = sequence_number_;                         // Current sequence number.
  const double TS = inputs.estimatedSampleTime();                // Estimated sample time [s].
  const double RAMP_IN = 3.0;                                    // Ramp in time [s].
  unsigned int seqno_ramp_in_end = (unsigned int)(RAMP_IN / TS); // Sequence number when the ramp in is finished [-].

  const wrapper::Feedback& initial_fdb = inputs.initial().feedback();
  const wrapper::Feedback& current_fdb = inputs.current().feedback();

  wrapper::Joints* rob_pos = current.mutable_robot()->mutable_joints()->mutable_position();
  wrapper::Joints* rob_vel = current.mutable_robot()->mutable_joints()->mutable_velocity();
  wrapper::Joints* ext_pos = current.mutable_external()->mutable_joints()->mutable_position();
  wrapper::Joints* ext_vel = current.mutable_external()->mutable_joints()->mutable_velocity();
  wrapper::CartesianPose* rob_pose = current.mutable_robot()->mutable_cartesian()->mutable_pose();

  //---------------------------------------------------------
  // Joint space
  //---------------------------------------------------------
  double amplitude = 2.0;
  double a = amplitude / 2.0;
  double b = M_PI*TS / RAMP_IN;

  double freq = 0.5;
  double dist = a*std::cos(b*seqno) - a;
  double speed_dist = -(a*b)*std::sin(b*seqno) / TS;
  if (seqno > seqno_ramp_in_end + 1)
  {
    dist = -2.0*a*cos(b*(seqno - seqno_ramp_in_end)*freq);
    speed_dist = 2.0*(a*b)*freq*std::sin(b*(seqno - seqno_ramp_in_end)*freq) / TS;
  }

  // Robot joints.
  for (int i = 0; i < current_fdb.robot().joints().position().values_size(); ++i)
  {
    rob_pos->set_values(i, initial_fdb.robot().joints().position().values(i) + dist);
  }

  for (int i = 0; i < current_fdb.robot().joints().velocity().values_size(); ++i)
  {
    rob_vel->set_values(i, speed_dist);
  }

  // External joints.
  for (int i = 0; i < current_fdb.external().joints().position().values_size(); ++i)
  {
    ext_pos->set_values(i, initial_fdb.external().joints().position().values(i) + dist);
  }

  for (int i = 0; i < current_fdb.external().joints().velocity().values_size(); ++i)
  {
    ext_vel->set_values(i, speed_dist);
  }

  //---------------------------------------------------------
  // Cartesian space
  //---------------------------------------------------------
  amplitude = 200.0;
  a = amplitude / 2.0;
  b = M_PI*TS;
  freq = 0.2;

  double x_dist = a*(std::cos(0.25*b*seqno*freq + M_PI) + 1);
  double y_dist = x_dist;
  double z_dist = a / 2.0*(std::cos(0.5*b*seqno*freq + 1.5*M_PI));

  rob_pose->mutable_position()->set_x(initial_fdb.robot().cartesian().pose().position().x() + x_dist);
  rob_pose->mutable_position()->set_y(initial_fdb.robot().cartesian().pose().position().y() + y_dist);
  rob_pose->mutable_position()->set_z(initial_fdb.robot().cartesian().pose().position().z() + z_dist);

  double t = 0.5*(std::cos(0.25*b*seqno*freq + M_PI) + 1);
  generateDemoQuaternions(inputs, t);

  // Demo references for including velocity control.
  //rob_pose->mutable_position()->set_x(current_fdb.robot().cartesian().pose().position().x());
  //current.mutable_robot()->mutable_cartesian()->mutable_velocity()->mutable_linear->set_x(-10.0);

  // Demo references for only velocity control.
  // Note: Need to set PosCorrGain to zero in the RAPID instruction.
  //current.mutable_robot()->mutable_cartesian()->mutable_velocity()->mutable_linear->set_x(-10.0);
  //current.mutable_robot()->mutable_cartesian()->mutable_velocity()->mutable_linear->set_y(-10.0);
  //current.mutable_robot()->mutable_cartesian()->mutable_velocity()->mutable_linear->set_z(-10.0);
  //current.mutable_robot()->mutable_cartesian()->mutable_velocity()->mutable_angular->set_x(4.0);
}

void EGMBaseInterface::OutputContainer::constructReply(const BaseConfiguration& configuration)
{
  constructHeader();
  bool success = constructJointBody(configuration);

  if (success && configuration.axes != None)
  {
    success = constructCartesianBody(configuration);
  }

  if (success)
  {
    success = egm_sensor_.SerializeToString(&reply_);
  }

  if (!success)
  {
    reply_.clear();
  }
}

void EGMBaseInterface::OutputContainer::updatePrevious()
{
  previous_.CopyFrom(current);
}

/************************************************************
 * Auxiliary methods
 */

void EGMBaseInterface::OutputContainer::generateDemoQuaternions(const InputContainer& inputs, const double t)
{
  // Quaternion demo using Slerp (spherical linear interpolation).
  // See for example https://en.wikipedia.org/wiki/Slerp for the equations.

  const wrapper::Feedback& initial = inputs.initial().feedback();

  //---------------------------------------------------------
  // Setup the initial data and the target.
  //---------------------------------------------------------
  double q1_u0 = initial.robot().cartesian().pose().quaternion().u0();
  double q1_u1 = initial.robot().cartesian().pose().quaternion().u1();
  double q1_u2 = initial.robot().cartesian().pose().quaternion().u2();
  double q1_u3 = initial.robot().cartesian().pose().quaternion().u3();

  wrapper::Quaternion q;
  wrapper::Euler e;
  e.set_x(-90.0);
  e.set_y(0.0);
  e.set_z(180);
  convert(&q, e);

  if (dotProduct(q, initial.robot().cartesian().pose().quaternion()) < 0.0)
  {
    q.set_u0(-q.u0());
    q.set_u1(-q.u1());
    q.set_u2(-q.u2());
    q.set_u3(-q.u3());
  }

  double q2_u0 = q.u0();
  double q2_u1 = q.u1();
  double q2_u2 = q.u2();
  double q2_u3 = q.u3();

  //---------------------------------------------------------
  // Calculate the Slerp.
  //---------------------------------------------------------
  double dot_prod = q1_u0*q2_u0 + q1_u1*q2_u1 + q1_u2*q2_u2 + q1_u3*q2_u3;

  double omega = acos(dot_prod);
  double k = 1.0 / sin(omega);

  double a = sin((1 - t)*omega)*k;
  double b = sin(t*omega)*k;

  double q3_u0 = a*q1_u0 + b*q2_u0;
  double q3_u1 = a*q1_u1 + b*q2_u1;
  double q3_u2 = a*q1_u2 + b*q2_u2;
  double q3_u3 = a*q1_u3 + b*q2_u3;

  //---------------------------------------------------------
  // Normalize and set the references.
  //---------------------------------------------------------
  double norm = sqrt(q3_u0*q3_u0 + q3_u1*q3_u1 + q3_u2*q3_u2 + q3_u3*q3_u3);

  current.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion()->set_u0(q3_u0 / norm);
  current.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion()->set_u1(q3_u1 / norm);
  current.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion()->set_u2(q3_u2 / norm);
  current.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion()->set_u3(q3_u3 / norm);

  convert(current.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_euler(),
          current.robot().cartesian().pose().quaternion());
}

void EGMBaseInterface::OutputContainer::constructHeader()
{
  egm_sensor_.mutable_header()->set_seqno((google::protobuf::uint32) sequence_number_);
  egm_sensor_.mutable_header()->set_tm((google::protobuf::uint32) 0);
  egm_sensor_.mutable_header()->set_mtype(EgmHeader_MessageType_MSGTYPE_CORRECTION);
}

bool EGMBaseInterface::OutputContainer::constructJointBody(const BaseConfiguration& configuration)
{
  bool position_ok = false;
  bool speed_ok = !configuration.use_velocity_outputs;

  int rob_condition = Constants::RobotController::DEFAULT_NUMBER_OF_ROBOT_JOINTS;
  int ext_condition = Constants::RobotController::DEFAULT_NUMBER_OF_EXTERNAL_JOINTS;

  if (current.robot().joints().has_position())
  {
    // Outputs.
    const wrapper::Joints& robot_position = current.robot().joints().position();
    const wrapper::Joints& external_position = current.external().joints().position();

    // Verify that there are no NaN or infinity values.
    if(!verify(robot_position) || !verify(external_position))
    {
      return false;
    }

    // EGM sensor message.
    EgmPlanned* planned = egm_sensor_.mutable_planned();
    planned->clear_joints();
    planned->clear_externaljoints();

    switch (configuration.axes)
    {
      case None:
      {
        if (robot_position.values_size() == 0)
        {
          for (int i = 0; i < external_position.values_size() && i < ext_condition; ++i)
          {
            planned->mutable_externaljoints()->add_joints(external_position.values(i));
          }

          position_ok = true;
        }
      }
      break;

      case Six:
      {
        if (robot_position.values_size() == rob_condition)
        {
          for (int i = 0; i < robot_position.values_size(); ++i)
          {
            planned->mutable_joints()->add_joints(robot_position.values(i));
          }

          for (int i = 0; i < external_position.values_size() && i < ext_condition; ++i)
          {
            planned->mutable_externaljoints()->add_joints(external_position.values(i));
          }

          position_ok = true;
        }
      }
      break;

      case Seven:
      {
        // If using a seven axes robot (e.g. IRB14000): Map to special case.
        if (robot_position.values_size() == rob_condition + 1)
        {
          planned->mutable_joints()->add_joints(robot_position.values(0));
          planned->mutable_joints()->add_joints(robot_position.values(1));
          planned->mutable_joints()->add_joints(robot_position.values(3));
          planned->mutable_joints()->add_joints(robot_position.values(4));
          planned->mutable_joints()->add_joints(robot_position.values(5));
          planned->mutable_joints()->add_joints(robot_position.values(6));
          planned->mutable_externaljoints()->add_joints(robot_position.values(2));

          for (int i = 0; i < external_position.values_size() && i < ext_condition - 1; ++i)
          {
            planned->mutable_externaljoints()->add_joints(external_position.values(i));
          }

          position_ok = true;
        }
      }
      break;
    }
  }

  if (configuration.use_velocity_outputs && current.robot().joints().has_velocity())
  {
    // Outputs.
    const wrapper::Joints& robot_velocity = current.robot().joints().velocity();
    const wrapper::Joints& external_velocity = current.external().joints().velocity();

    // Verify that there are no NaN or infinity values.
    if(!verify(robot_velocity) || !verify(external_velocity))
    {
      return false;
    }

    // EGM sensor message.
    EgmSpeedRef* speed_reference = egm_sensor_.mutable_speedref();
    speed_reference->clear_joints();
    speed_reference->clear_externaljoints();

    switch (configuration.axes)
    {
      case None:
      {
        if (robot_velocity.values_size() == 0)
        {
          for (int i = 0; i < external_velocity.values_size() && i < ext_condition; ++i)
          {
            speed_reference->mutable_externaljoints()->add_joints(external_velocity.values(i));
          }

          speed_ok = true;
        }
      }
      break;

      case Six:
      {
        if (robot_velocity.values_size() == rob_condition)
        {
          for (int i = 0; i < robot_velocity.values_size(); ++i)
          {
            speed_reference->mutable_joints()->add_joints(robot_velocity.values(i));
          }

          for (int i = 0; i < external_velocity.values_size() && i < ext_condition; ++i)
          {
            speed_reference->mutable_externaljoints()->add_joints(external_velocity.values(i));
          }

          speed_ok = true;
        }
      }
      break;

      case Seven:
      {
        // If using a seven axes robot (e.g. IRB14000): Map to special case.
        if (robot_velocity.values_size() == rob_condition + 1)
        {
          speed_reference->mutable_joints()->add_joints(robot_velocity.values(0));
          speed_reference->mutable_joints()->add_joints(robot_velocity.values(1));
          speed_reference->mutable_joints()->add_joints(robot_velocity.values(3));
          speed_reference->mutable_joints()->add_joints(robot_velocity.values(4));
          speed_reference->mutable_joints()->add_joints(robot_velocity.values(5));
          speed_reference->mutable_joints()->add_joints(robot_velocity.values(6));
          speed_reference->mutable_externaljoints()->add_joints(robot_velocity.values(2));

          for (int i = 0; i < external_velocity.values_size() && i < ext_condition - 1; ++i)
          {
            speed_reference->mutable_externaljoints()->add_joints(external_velocity.values(i));
          }

          speed_ok = true;
        }
      }
      break;
    }
  }

  return (position_ok && speed_ok);
}

bool EGMBaseInterface::OutputContainer::constructCartesianBody(const BaseConfiguration& configuration)
{
  bool position_ok = false;
  bool speed_ok = !configuration.use_velocity_outputs;

  if (current.robot().cartesian().has_pose())
  {
    // Outputs.
    const wrapper::CartesianPose& pose = current.robot().cartesian().pose();;

    // Verify that there are no NaN or infinity values.
    if(!verify(pose))
    {
      return false;
    }

    // EGM sensor message.
    EgmPlanned* planned = egm_sensor_.mutable_planned();
    planned->clear_cartesian();

    if (pose.has_position())
    {
      planned->mutable_cartesian()->mutable_pos()->set_x(pose.position().x());
      planned->mutable_cartesian()->mutable_pos()->set_y(pose.position().y());
      planned->mutable_cartesian()->mutable_pos()->set_z(pose.position().z());
    }

    if (pose.has_euler())
    {
      planned->mutable_cartesian()->mutable_euler()->set_x(pose.euler().x());
      planned->mutable_cartesian()->mutable_euler()->set_y(pose.euler().y());
      planned->mutable_cartesian()->mutable_euler()->set_z(pose.euler().z());
    }

    if (pose.has_quaternion())
    {
      planned->mutable_cartesian()->mutable_orient()->set_u0(pose.quaternion().u0());
      planned->mutable_cartesian()->mutable_orient()->set_u1(pose.quaternion().u1());
      planned->mutable_cartesian()->mutable_orient()->set_u2(pose.quaternion().u2());
      planned->mutable_cartesian()->mutable_orient()->set_u3(pose.quaternion().u3());
    }

    position_ok = true;
  }

  if (configuration.use_velocity_outputs && current.robot().cartesian().has_velocity())
  {
    // References.
    const wrapper::CartesianVelocity& velocity = current.robot().cartesian().velocity();

    // Verify that there are no NaN or infinity values.
    if(!verify(velocity))
    {
      return false;
    }

    // EGM sensor message.
    EgmSpeedRef* speed_reference = egm_sensor_.mutable_speedref();
    speed_reference->clear_cartesians();

    if (velocity.has_linear())
    {
      speed_reference->mutable_cartesians()->add_value(velocity.linear().x());
      speed_reference->mutable_cartesians()->add_value(velocity.linear().y());
      speed_reference->mutable_cartesians()->add_value(velocity.linear().z());
    }
    else
    {
      speed_reference->mutable_cartesians()->add_value(0.0);
      speed_reference->mutable_cartesians()->add_value(0.0);
      speed_reference->mutable_cartesians()->add_value(0.0);
    }

    if (velocity.has_angular())
    {
      speed_reference->mutable_cartesians()->add_value(velocity.angular().x());
      speed_reference->mutable_cartesians()->add_value(velocity.angular().y());
      speed_reference->mutable_cartesians()->add_value(velocity.angular().z());
    }
    else
    {
      speed_reference->mutable_cartesians()->add_value(0.0);
      speed_reference->mutable_cartesians()->add_value(0.0);
      speed_reference->mutable_cartesians()->add_value(0.0);
    }

    speed_ok = true;
  }

  return (position_ok && speed_ok);
}




/***********************************************************************************************************************
 * Class definitions: EGMBaseInterface
 */

// See https://stackoverflow.com/questions/16957458/static-const-in-c-class-undefined-reference/16957554
const unsigned int EGMBaseInterface::WAIT_TIME_MS;

/************************************************************
 * Primary methods
 */

EGMBaseInterface::EGMBaseInterface(boost::asio::io_service& io_service,
                                   const unsigned short port_number,
                                   const BaseConfiguration& configuration)
:
udp_server_(io_service, port_number, this),
configuration_(configuration)
{
  if (configuration_.active.use_logging)
  {
    std::stringstream ss;
    ss << "port_" << port_number << +"_log.csv";
    p_logger_.reset(new EGMLogger(ss.str()));
  }
}

const std::string& EGMBaseInterface::callback(const UDPServerData& server_data)
{
  // Initialize the callback by:
  // - Parsing and extracting data from the received message.
  // - Updating any pending configuration changes.
  // - Preparing the outputs.
  if (initializeCallback(server_data))
  {
    // Handle demo execution.
    if (configuration_.active.use_demo_outputs)
    {
      outputs_.generateDemoOutputs(inputs_);
    }

    // Log inputs and outputs.
    if (configuration_.active.use_logging && p_logger_)
    {
      logData(inputs_, outputs_, configuration_.active.max_logging_duration);
    }

    // Constuct the reply message.
    outputs_.constructReply(configuration_.active);

    // Prepare for the next callback.
    inputs_.updatePrevious();
    outputs_.updatePrevious();
  }

  // Return the reply.
  return outputs_.reply();
}

/************************************************************
 * Auxiliary methods
 */

void EGMBaseInterface::logData(const InputContainer& inputs, const OutputContainer& outputs, const double max_time)
{
  if (p_logger_ && p_logger_->calculateTimeLogged(inputs_.estimatedSampleTime()) <= max_time)
  {
    const wrapper::Feedback& feedback = inputs.current().feedback();
    const wrapper::Planned& planned = inputs.current().planned();
    const wrapper::Output& output = outputs.current;

    // Header (i.e. time stamp).
    p_logger_->add(inputs.current().header());

    // Robot feedback.
    p_logger_->add(feedback.robot().joints().position(), feedback.external().joints().position());
    p_logger_->add(feedback.robot().joints().velocity(), feedback.external().joints().velocity());
    p_logger_->add(feedback.robot().cartesian().pose());
    p_logger_->add(feedback.robot().cartesian().velocity());

    // Robot planned.
    p_logger_->add(planned.robot().joints().position(), planned.external().joints().position());
    p_logger_->add(planned.robot().joints().velocity(), planned.external().joints().velocity());
    p_logger_->add(planned.robot().cartesian().pose());
    p_logger_->add(planned.robot().cartesian().velocity());

    // Server outputs.
    p_logger_->add(output.robot().joints().position(), output.external().joints().position());
    p_logger_->add(output.robot().joints().velocity(), output.external().joints().velocity());
    p_logger_->add(output.robot().cartesian().pose());
    p_logger_->add(output.robot().cartesian().velocity(), true);

    // Write to the log file.
    p_logger_->flush();
  }
}

bool EGMBaseInterface::initializeCallback(const UDPServerData& server_data)
{
  bool success = false;

  // Parse the received message.
  if (server_data.p_data)
  {
    success = inputs_.parseFromArray(server_data.p_data, server_data.bytes_transferred);
  }

  // Update configuration, if requested to do so.
  if (success && inputs_.isFirstMessage())
  {
    boost::lock_guard<boost::mutex> lock(configuration_.mutex);

    if (configuration_.has_pending_update)
    {
      configuration_.active = configuration_.update;
      configuration_.has_pending_update = false;
    }
  }

  // Extract information from the parsed message.
  if (success)
  {
    success = inputs_.extractParsedInformation(configuration_.active.axes);

    {
      boost::lock_guard<boost::mutex> lock(session_data_.mutex);

      // Update the session data.
      if (success)
      {
        session_data_.header.CopyFrom(inputs_.current().header());
        session_data_.status.CopyFrom(inputs_.current().status());
      }
      else
      {
        session_data_.header.Clear();
        session_data_.status.Clear();
      }
    }
  }

  // Prepare the outputs.
  outputs_.clearReply();
  if (success)
  {
    outputs_.prepareOutputs(inputs_);
  }

  return success;
}

/************************************************************
 * User interaction methods
 */

bool EGMBaseInterface::isInitialized()
{
  return udp_server_.isInitialized();
}

bool EGMBaseInterface::isConnected()
{
  wrapper::Header header_1;
  wrapper::Header header_2;

  {
    boost::lock_guard<boost::mutex> lock(session_data_.mutex);
    header_1.CopyFrom(session_data_.header);
  }

  boost::this_thread::sleep(boost::posix_time::milliseconds(WAIT_TIME_MS));

  {
    boost::lock_guard<boost::mutex> lock(session_data_.mutex);
    header_2.CopyFrom(session_data_.header);
  }

  // Check for if an EGM communication session is connected or not. This is determined by comparing
  // two header messages, received at two different time instances, according to:
  // 1. Both headers must have a sequence number and a time stamp.
  // 2. The sequence number of the second header must be larger than that of the first header.
  // 3. The time stamp of the second header must be larger than that of the first header.
  // 4. The difference in time stamp must be near the wait time.
  return (header_1.has_sequence_number() && header_1.has_time_stamp()) &&
         (header_2.has_sequence_number() && header_2.has_time_stamp()) &&
         (header_2.sequence_number() > header_1.sequence_number()) &&
         (header_2.time_stamp() > header_1.time_stamp()) &&
         (header_2.time_stamp() - header_1.time_stamp() <= 2 * WAIT_TIME_MS);
};

wrapper::Status EGMBaseInterface::getStatus()
{
  wrapper::Status status;

  {
    boost::lock_guard<boost::mutex> lock(session_data_.mutex);
    status.CopyFrom(session_data_.status);
  }

  return status;
};

BaseConfiguration EGMBaseInterface::getConfiguration()
{
  boost::lock_guard<boost::mutex> lock(configuration_.mutex);

  return configuration_.update;
}

void EGMBaseInterface::setConfiguration(const BaseConfiguration& configuration)
{
  boost::lock_guard<boost::mutex> lock(configuration_.mutex);

  configuration_.update = configuration;
  configuration_.has_pending_update = true;
}

} // end namespace egm
} // end namespace abb
