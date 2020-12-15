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

#include <math.h>

#include <sstream>

#include "abb_libegm/egm_common_auxiliary.h"
#include "abb_libegm/egm_trajectory_interface.h"

namespace abb
{
namespace egm
{

using namespace wrapper;
using namespace wrapper::trajectory;

/***********************************************************************************************************************
 * Class definitions: EGMTrajectoryInterface::TrajectoryMotion::StateManager
 */

/************************************************************
 * Primary methods
 */

void EGMTrajectoryInterface::TrajectoryMotion::StateManager::setPendingState(const States& desired_state,
                                                                             const SubStates& desired_sub_state)
{
  bool accepted = false;

  switch (current_state_)
  {
    case Normal:
    {
      switch (current_sub_state_)
      {
        case None:
          // Should never occur.
        break;

        case Running:
          accepted = (desired_state == RampDown && desired_sub_state == None);
        break;

        case Finished:
          // Should never occur.
        break;
      }
    }
    break;

    case RampDown:
    {
      switch (current_sub_state_)
      {
        case None:
          accepted = (desired_state == RampDown && desired_sub_state == Running);
        break;

        case Running:
          accepted = (desired_state == RampDown && desired_sub_state == Finished);
        break;

        case Finished:
          accepted = (desired_state == Normal && desired_sub_state == Running) ||
                     (desired_state == StaticGoal && desired_sub_state == None);
        break;
      }
    }
    break;

    case StaticGoal:
    {
      switch (current_sub_state_)
      {
        case None:
          accepted = (desired_state == StaticGoal && desired_sub_state == Running);
        break;

        case Running:
          accepted = (desired_state == StaticGoal && desired_sub_state == Finished) ||
                     (desired_state == RampDown && desired_sub_state == None);
        break;

        case Finished:
          accepted = (desired_state == RampDown && desired_sub_state == None);
        break;
      }
    }
    break;
  }

  if (accepted)
  {
    pending_state_ = desired_state;
    pending_sub_state_ = desired_sub_state;
    has_pending_state_ = true;
  }
}

/************************************************************
 * Auxiliary methods
 */

wrapper::trajectory::ExecutionProgress_State EGMTrajectoryInterface::TrajectoryMotion::StateManager::mapState()
{
  switch (current_state_)
  {
    case Normal:
      return wrapper::trajectory::ExecutionProgress_State_NORMAL;
    break;

    case RampDown:
      return wrapper::trajectory::ExecutionProgress_State_RAMP_DOWN;
    break;

    case StaticGoal:
      return wrapper::trajectory::ExecutionProgress_State_STATIC_GOAL;
    break;

    default:
      return wrapper::trajectory::ExecutionProgress_State_UNDEFINED;
  }
}

wrapper::trajectory::ExecutionProgress_SubState EGMTrajectoryInterface::TrajectoryMotion::StateManager::mapSubState()
{
  switch (current_sub_state_)
  {
    case None:
      return wrapper::trajectory::ExecutionProgress_SubState_NONE;
    break;

    case Running:
      return wrapper::trajectory::ExecutionProgress_SubState_RUNNING;
    break;

    case Finished:
      return wrapper::trajectory::ExecutionProgress_SubState_FINISHED;
    break;

    default:
      return wrapper::trajectory::ExecutionProgress_SubState_NONE;
  }
}




/***********************************************************************************************************************
 * Class definitions: EGMTrajectoryInterface::TrajectoryMotion::MotionStep
 */

/************************************************************
 * Primary methods
 */

void EGMTrajectoryInterface::TrajectoryMotion::MotionStep::resetMotionStep()
{
  unsigned int robot_joints = data.feedback.robot().joints().position().values_size();
  unsigned int external_joints = data.feedback.external().joints().position().values_size();

  internal_goal.Clear();
  external_goal.Clear();

  //---------------------------------------------------------
  // Internal goal field
  //---------------------------------------------------------
  const wrapper::Feedback& feedback = data.feedback;
  wrapper::trajectory::JointGoal* p_robot_joints = internal_goal.mutable_robot()->mutable_joints();
  wrapper::trajectory::CartesianGoal* p_robot_cartesian = internal_goal.mutable_robot()->mutable_cartesian();
  wrapper::trajectory::ExternalGoal* p_external = internal_goal.mutable_external();

  //-----------------------------
  // Positions
  //-----------------------------
  // Copy joint values.
  p_robot_joints->mutable_position()->CopyFrom(feedback.robot().joints().position());
  p_external->mutable_joints()->mutable_position()->CopyFrom(feedback.external().joints().position());

  // Copy Cartesian values. Note: The internal goal's Euler field is used to contain angular velocities.
  p_robot_cartesian->mutable_pose()->mutable_position()->CopyFrom(feedback.robot().cartesian().pose().position());
  p_robot_cartesian->mutable_pose()->mutable_quaternion()->CopyFrom(feedback.robot().cartesian().pose().quaternion());

  //-----------------------------
  // Velocities and accelerations
  //-----------------------------
  // Reset joint values.
  reset(p_robot_joints->mutable_velocity(), robot_joints);
  reset(p_robot_joints->mutable_acceleration(), robot_joints);
  reset(p_external->mutable_joints()->mutable_velocity(), external_joints);
  reset(p_external->mutable_joints()->mutable_acceleration(), external_joints);

  // Reset Cartesian values. Note: The internal goal's Euler field is used to contain angular velocities.
  reset(p_robot_cartesian->mutable_velocity());
  reset(p_robot_cartesian->mutable_acceleration());
  reset(p_robot_cartesian->mutable_pose()->mutable_euler());

  //---------------------------------------------------------
  // The remaining fields
  //---------------------------------------------------------
  data.time_passed = 0.0;
  data.mode = EGMJoint;
  interpolation.CopyFrom(internal_goal);
}

void EGMTrajectoryInterface::TrajectoryMotion::MotionStep::prepareNormalGoal(const bool last_point)
{
  unsigned int robot_joints = data.feedback.robot().joints().position().values_size();
  unsigned int external_joints = data.feedback.external().joints().position().values_size();

  data.mode = (external_goal.robot().has_cartesian() ? EGMPose : EGMJoint);

  // Reset the internal goal's velocity and acceleration values.
  // Note: The Euler field is internally used to contain angular velocities.
  reset(internal_goal.mutable_robot()->mutable_joints()->mutable_velocity(), robot_joints);
  reset(internal_goal.mutable_robot()->mutable_joints()->mutable_acceleration(), robot_joints);
  reset(internal_goal.mutable_robot()->mutable_cartesian()->mutable_velocity());
  reset(internal_goal.mutable_robot()->mutable_cartesian()->mutable_acceleration());
  reset(internal_goal.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_euler());
  reset(internal_goal.mutable_external()->mutable_joints()->mutable_velocity(), external_joints);
  reset(internal_goal.mutable_external()->mutable_joints()->mutable_acceleration(), external_joints);

  // Set up the internal goal's reach condition.
  internal_goal.set_reach(external_goal.has_reach() ? external_goal.reach() : false);

  // Transfer external goal values to the internal goal.
  transfer(external_goal.robot());
  transfer(external_goal.external());

  // Reset velocity and acceleration values, if it is the last point in the current trajectory.
  if (last_point)
  {
    internal_goal.set_reach(true);

    // Reset robot joint values.
    reset(internal_goal.mutable_robot()->mutable_joints()->mutable_velocity(), robot_joints);
    reset(internal_goal.mutable_robot()->mutable_joints()->mutable_acceleration(), robot_joints);

    // Reset Cartesian values. Note: The Euler field is internally used to contain angular velocities.
    reset(internal_goal.mutable_robot()->mutable_cartesian()->mutable_velocity());
    reset(internal_goal.mutable_robot()->mutable_cartesian()->mutable_acceleration());
    reset(internal_goal.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_euler());

    // Reset external joint values.
    reset(internal_goal.mutable_external()->mutable_joints()->mutable_velocity(), external_joints);
    reset(internal_goal.mutable_external()->mutable_joints()->mutable_acceleration(), external_joints);
  }

  // Set up the internal goal's duration.
  double duration = (external_goal.has_duration() ? external_goal.duration() : estimateDuration());
  internal_goal.set_duration(data.duration_factor*duration);

  // Prepare the interpolation conditions.
  interpolator_conditions_.mode = data.mode;
  interpolator_conditions_.duration = internal_goal.duration();
  interpolator_conditions_.operation = EGMInterpolator::Normal;
  interpolator_conditions_.spline_method = configurations_.spline_method;
}

void EGMTrajectoryInterface::TrajectoryMotion::MotionStep::prepareRampDownGoal(const bool do_stop)
{
  // Prepare the interpolation conditions.
  interpolator_conditions_.mode = data.mode;
  interpolator_conditions_.operation = EGMInterpolator::RampDown;

  // Prepare for a complete stop, or a ramp down to half of the current velocity.
  if (do_stop)
  {
    interpolator_conditions_.duration = RAMP_DOWN_STOP_DURATION;
    interpolator_conditions_.ramp_down_factor = 0.0;
  }
  else
  {
    interpolator_conditions_.duration = RAMP_DOWN_STOP_DURATION / 2.0;
    interpolator_conditions_.ramp_down_factor = 0.5;
  }
}

void EGMTrajectoryInterface::TrajectoryMotion::MotionStep::prepareStaticGoal(const StaticPositionGoal& position_goal,
                                                                             const bool fast_transition)
{
  data.mode = (position_goal.robot().has_cartesian() ? EGMPose : EGMJoint);

  internal_goal.CopyFrom(interpolation);

  // Transfer the static position goal values to the internal goal.
  transfer(position_goal);

  // Prepare the interpolation conditions.
  interpolator_conditions_.mode = data.mode;
  interpolator_conditions_.duration = (fast_transition ? STATIC_GOAL_DURATION_SHORT : STATIC_GOAL_DURATION);
  interpolator_conditions_.operation = EGMInterpolator::RampInPosition;
}

void EGMTrajectoryInterface::TrajectoryMotion::MotionStep::prepareStaticGoal(const StaticVelocityGoal& velocity_goal,
                                                                             const bool fast_transition)
{
  data.mode = (velocity_goal.robot().has_cartesian() ? EGMPose : EGMJoint);

  internal_goal.CopyFrom(interpolation);

  // Transfer the static velocity goal values to the internal goal.
  transfer(velocity_goal);

  // Prepare the interpolation conditions.
  interpolator_conditions_.mode = data.mode;
  interpolator_conditions_.duration = (fast_transition ? STATIC_GOAL_DURATION_SHORT : STATIC_GOAL_DURATION);
  interpolator_conditions_.operation = EGMInterpolator::RampInVelocity;
}

bool EGMTrajectoryInterface::TrajectoryMotion::MotionStep::conditionMet()
{
  condition_met_ = true;

  switch (data.mode)
  {
    case EGMJoint:
    {
      // Robot joints.
      checkConditions(internal_goal.robot().joints().position(), data.feedback.robot().joints().position());

      // External joints.
      checkConditions(internal_goal.external().joints().position(), data.feedback.external().joints().position());
    }
    break;

    case EGMPose:
    {
      // Position.
      checkConditions(internal_goal.robot().cartesian().pose().position(),
                      data.feedback.robot().cartesian().pose().position());

      // Orientation.
      checkConditions(internal_goal.robot().cartesian().pose().quaternion(),
                      data.feedback.robot().cartesian().pose().quaternion());

      // External joints.
      checkConditions(internal_goal.external().joints().position(), data.feedback.external().joints().position());
    }
    break;
  }

  return condition_met_;
}

/************************************************************
 * Auxiliary methods
 */

double EGMTrajectoryInterface::TrajectoryMotion::MotionStep::estimateDuration()
{
  // Note: The duration estimation should only be used if no duration has been specified externally, and it includes:
  //       * Resetting any velocity and acceleration goals.
  //       * Estimation of the duration based on the goal and feedback position differences,
  //         and it assumes 1 degree/s and 1 mm/s as desired velocities.

  double estimate = 0.0;

  unsigned int robot_joints = data.feedback.robot().joints().position().values_size();
  unsigned int external_joints = data.feedback.external().joints().position().values_size();

  // Reset robot joint values.
  reset(internal_goal.mutable_robot()->mutable_joints()->mutable_velocity(), robot_joints);
  reset(internal_goal.mutable_robot()->mutable_joints()->mutable_acceleration(), robot_joints);

  // Reset Cartesian values. Note: The Euler field is internally used to contain angular velocities.
  reset(internal_goal.mutable_robot()->mutable_cartesian()->mutable_velocity());
  reset(internal_goal.mutable_robot()->mutable_cartesian()->mutable_acceleration());
  reset(internal_goal.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_euler());

  // Reset external joint values.
  reset(internal_goal.mutable_external()->mutable_joints()->mutable_velocity(), external_joints);
  reset(internal_goal.mutable_external()->mutable_joints()->mutable_acceleration(), external_joints);

  // Estimate the duration.
  switch (data.mode)
  {
    case EGMJoint:
    {
      estimate = std::max(estimate, findMaxDifference(internal_goal.robot().joints().position(),
                                                      data.feedback.robot().joints().position()));

      estimate = std::max(estimate, findMaxDifference(internal_goal.external().joints().position(),
                                                      data.feedback.external().joints().position()));
    }
    break;

    case EGMPose:
    {
      estimate = findMaxDifference(internal_goal.robot().cartesian().pose().position(),
                                   data.feedback.robot().cartesian().pose().position());

      Euler temp;
      convert(&temp, internal_goal.robot().cartesian().pose().quaternion());
      estimate = std::max(estimate, findMaxDifference(temp, data.feedback.robot().cartesian().pose().euler()));

      estimate = std::max(estimate, findMaxDifference(internal_goal.external().joints().position(),
                                                      data.feedback.external().joints().position()));
    }
    break;
  }

  return estimate;
}

void EGMTrajectoryInterface::TrajectoryMotion::MotionStep::checkConditions(const Joints& feedback, const Joints& goal)
{
  for (int i = 0; condition_met_ && i < feedback.values_size() && i < goal.values_size(); ++i)
  {
    condition_met_ = (std::abs(feedback.values(i) - goal.values(i)) < CONDITION);
  }
}

void EGMTrajectoryInterface::TrajectoryMotion::MotionStep::checkConditions(const Cartesian& feedback,
                                                                           const Cartesian& goal)
{
  double delta_x = std::abs(feedback.x() - goal.x());
  double delta_y = std::abs(feedback.y() - goal.y());
  double delta_z = std::abs(feedback.z() - goal.z());

  condition_met_ = (delta_x < CONDITION && delta_y < CONDITION && delta_z < CONDITION);
}

void EGMTrajectoryInterface::TrajectoryMotion::MotionStep::checkConditions(const Quaternion& feedback,
                                                                           const Quaternion& goal)
{
  condition_met_ = (condition_met_ && std::abs(dotProduct(feedback, goal)) >= 1.0);
}

void EGMTrajectoryInterface::TrajectoryMotion::MotionStep::transfer(const RobotGoal& source)
{
  JointGoal* p_joints = internal_goal.mutable_robot()->mutable_joints();
  CartesianGoal* p_cartesian = internal_goal.mutable_robot()->mutable_cartesian();

  // Set up the internal robot joint goal.
  copyPresent(p_joints->mutable_position(), source.joints().position());
  copyPresent(p_joints->mutable_velocity(), source.joints().velocity());
  copyPresent(p_joints->mutable_acceleration(), source.joints().acceleration());

  // Set up the internal robot Cartesian goal.
  copyPresent(p_cartesian->mutable_pose()->mutable_position(), source.cartesian().pose().position());
  copyPresent(p_cartesian->mutable_velocity(), source.cartesian().velocity());
  copyPresent(p_cartesian->mutable_acceleration(), source.cartesian().acceleration());

  // Note: The internal goal's Euler field is used to contain angular velocities.
  //       Therefore, convert any Euler goal to quaternions.
  if (source.cartesian().pose().has_euler())
  {
    Euler temp;
    convert(&temp, p_cartesian->mutable_pose()->quaternion());
    copyPresent(&temp, source.cartesian().pose().euler());
    convert(p_cartesian->mutable_pose()->mutable_quaternion(), temp);
    normalize(p_cartesian->mutable_pose()->mutable_quaternion());
  }
  else if (source.cartesian().pose().has_quaternion())
  {
    copyPresent(p_cartesian->mutable_pose()->mutable_quaternion(), source.cartesian().pose().quaternion());
  }
}

void EGMTrajectoryInterface::TrajectoryMotion::MotionStep::transfer(const ExternalGoal& source)
{
  JointGoal* p_joints = internal_goal.mutable_external()->mutable_joints();

  // Set up the internal external joint goal.
  copyPresent(p_joints->mutable_position(), source.joints().position());
  copyPresent(p_joints->mutable_velocity(), source.joints().velocity());
  copyPresent(p_joints->mutable_acceleration(), source.joints().acceleration());
}

void EGMTrajectoryInterface::TrajectoryMotion::MotionStep::transfer(const StaticPositionGoal& source)
{
  JointGoal* p_robot_joints = internal_goal.mutable_robot()->mutable_joints();
  CartesianGoal* p_cartesian = internal_goal.mutable_robot()->mutable_cartesian();
  JointGoal* p_external_joints = internal_goal.mutable_external()->mutable_joints();

  // Set up the internal goal.
  copyPresent(p_robot_joints->mutable_position(), source.robot().joints());
  copyPresent(p_cartesian->mutable_pose()->mutable_position(), source.robot().cartesian().position());
  copyPresent(p_external_joints->mutable_position(), source.external());

  // Note: The internal goal's Euler field is used to contain angular velocities.
  //       Therefore, convert any Euler goal to quaternions.
  if (source.robot().cartesian().has_euler())
  {
    Euler temp;
    convert(&temp, p_cartesian->pose().quaternion());
    copyPresent(&temp, source.robot().cartesian().euler());
    convert(p_cartesian->mutable_pose()->mutable_quaternion(), temp);
    normalize(p_cartesian->mutable_pose()->mutable_quaternion());
  }
  else if (source.robot().cartesian().has_quaternion())
  {
    copyPresent(p_cartesian->mutable_pose()->mutable_quaternion(), source.robot().cartesian().quaternion());
  }
}

void EGMTrajectoryInterface::TrajectoryMotion::MotionStep::transfer(const StaticVelocityGoal& source)
{
  JointGoal* p_robot_joints = internal_goal.mutable_robot()->mutable_joints();
  CartesianGoal* p_cartesian = internal_goal.mutable_robot()->mutable_cartesian();
  JointGoal* p_external_joints = internal_goal.mutable_external()->mutable_joints();

  // Set up the internal goal.
  copyPresent(p_robot_joints->mutable_velocity(), source.robot().joints());
  copyPresent(p_cartesian->mutable_velocity(), source.robot().cartesian().linear());
  copyPresent(p_external_joints->mutable_velocity(), source.external());

  // Note: The internal goal's Euler field is used to contain angular velocities.
  copyPresent(p_cartesian->mutable_pose()->mutable_euler(), source.robot().cartesian().angular());
}




/***********************************************************************************************************************
 * Class definitions: EGMTrajectoryInterface::TrajectoryMotion::Controller
 */

/************************************************************
 * Primary methods
 */

void EGMTrajectoryInterface::TrajectoryMotion::Controller::update(const States state,
                                                                  const MotionStep& motion_step,
                                                                  const TrajectoryConfiguration& configurations)
{
  is_normal_state_ = (state == Normal);
  is_linear_ = (configurations.spline_method == TrajectoryConfiguration::Linear);

  egm_mode_ = motion_step.data.mode;
  initial_references_.CopyFrom(motion_step.interpolation);
}

void EGMTrajectoryInterface::TrajectoryMotion::Controller::calculate(Output* p_outputs, MotionStep* p_motion_step)
{
  do_velocity_transition_ = false;

  if (p_outputs && p_motion_step)
  {
    p_outputs->mutable_robot()->mutable_cartesian()->mutable_pose()->clear_euler();

    double t = saturate(p_motion_step->data.time_passed / p_motion_step->interpolation.duration(), 0.0, 1.0);

    // Calculate the scale factors.
    a_ = (p_motion_step->interpolation.reach() ? 0.5*std::cos(M_PI*t) + 0.5 : 1.0);
    b_ = 0.5*std::cos(M_PI*t + M_PI) + 0.5;

    switch (egm_mode_)
    {
      case EGMJoint:
      {
        // Robot and external joint positions.
        calculate(p_outputs->mutable_robot()->mutable_joints()->mutable_position(),
                  p_motion_step->interpolation.mutable_robot()->mutable_joints()->mutable_position(),
                  p_motion_step->data.feedback.robot().joints().position(),
                  initial_references_.robot().joints().position());

        calculate(p_outputs->mutable_external()->mutable_joints()->mutable_position(),
                  p_motion_step->interpolation.mutable_external()->mutable_joints()->mutable_position(),
                  p_motion_step->data.feedback.external().joints().position(),
                  initial_references_.external().joints().position());

        // Check if velocity values should be transitioned.
        checkForVelocityTransition(&(p_motion_step->interpolation));

        // Robot and external joint velocities.
        calculate(p_outputs->mutable_robot()->mutable_joints()->mutable_velocity(),
                  p_motion_step->interpolation.mutable_robot()->mutable_joints()->mutable_velocity(),
                  p_motion_step->data.feedback.robot().joints().velocity(),
                  initial_references_.robot().joints().velocity());

        calculate(p_outputs->mutable_external()->mutable_joints()->mutable_velocity(),
                  p_motion_step->interpolation.mutable_external()->mutable_joints()->mutable_velocity(),
                  p_motion_step->data.feedback.external().joints().velocity(),
                  initial_references_.external().joints().velocity());
      }
      break;

      case EGMPose:
      {
        // Cartesian pose and orientation.
        calculate(p_outputs->mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position(),
                  p_motion_step->interpolation.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_position(),
                  p_motion_step->data.feedback.robot().cartesian().pose().position(),
                  initial_references_.robot().cartesian().pose().position());

        calculate(p_outputs->mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_quaternion(),
                  p_motion_step->interpolation.robot().cartesian().pose().quaternion(),
                  p_motion_step->data.feedback.robot().cartesian().pose().quaternion());

        // External joint positions.
        calculate(p_outputs->mutable_external()->mutable_joints()->mutable_position(),
                  p_motion_step->interpolation.mutable_external()->mutable_joints()->mutable_position(),
                  p_motion_step->data.feedback.external().joints().position(),
                  initial_references_.external().joints().position());

        // Check if velocity values should be transitioned.
        checkForVelocityTransition(&(p_motion_step->interpolation));

        // Cartesian linear and external joint velocities.
        calculate(p_outputs->mutable_robot()->mutable_cartesian()->mutable_velocity()->mutable_linear(),
                  p_motion_step->interpolation.mutable_robot()->mutable_cartesian()->mutable_velocity(),
                  p_motion_step->data.feedback.robot().cartesian().velocity().linear(),
                  initial_references_.robot().cartesian().velocity());

        calculate(p_outputs->mutable_external()->mutable_joints()->mutable_velocity(),
                  p_motion_step->interpolation.mutable_external()->mutable_joints()->mutable_velocity(),
                  p_motion_step->data.feedback.external().joints().velocity(),
                  initial_references_.external().joints().velocity());

        // Cartesian angular velocities. Note: The Euler field is internally used to contain angular velocities.
        calculate(p_outputs->mutable_robot()->mutable_cartesian()->mutable_velocity()->mutable_angular(),
                  p_motion_step->interpolation.mutable_robot()->mutable_cartesian()->mutable_pose()->mutable_euler(),
                  p_motion_step->data.feedback.robot().cartesian().velocity().angular(),
                  initial_references_.robot().cartesian().pose().euler());
      }
      break;
    }
  }
}

/************************************************************
 * Auxiliary methods
 */

void EGMTrajectoryInterface::TrajectoryMotion::Controller::checkForVelocityTransition(PointGoal* p_references)
{
  if (is_normal_state_)
  {
    do_velocity_transition_ = (p_references->reach() ? true : is_linear_);
  }

  if (do_velocity_transition_)
  {
    // Ramp out acceleration references.
    multiply(p_references->mutable_robot()->mutable_joints()->mutable_acceleration(), a_);
    multiply(p_references->mutable_robot()->mutable_cartesian()->mutable_acceleration(), a_);
    multiply(p_references->mutable_external()->mutable_joints()->mutable_acceleration(), a_);
  }
}

void EGMTrajectoryInterface::TrajectoryMotion::Controller::calculate(Joints* p_out,
                                                                     Joints* p_ref,
                                                                     const Joints& fdb,
                                                                     const Joints& start)
{
  for (int i = 0;
       i < p_out->values_size() && i < p_ref->values_size() && i < fdb.values_size() && i < start.values_size();
       ++i)
  {
    if (do_velocity_transition_)
    {
      if (is_linear_)
      {
        p_ref->set_values(i, a_*(start.values(i) + b_*(p_ref->values(i) - start.values(i))));
      }
      else
      {
        p_ref->set_values(i, a_*p_ref->values(i));
      }
    }

    p_out->set_values(i, fdb.values(i) + k_*(p_ref->values(i) - fdb.values(i)));
  }
}

void EGMTrajectoryInterface::TrajectoryMotion::Controller::calculate(wrapper::Cartesian* p_out,
                                                                     wrapper::Cartesian* p_ref,
                                                                     const wrapper::Cartesian& fdb,
                                                                     const wrapper::Cartesian& start)
{
  if (do_velocity_transition_)
  {
    if (is_linear_)
    {
      p_ref->set_x(a_*(start.x() + b_*(p_ref->x() - start.x())));
      p_ref->set_y(a_*(start.y() + b_*(p_ref->y() - start.y())));
      p_ref->set_z(a_*(start.z() + b_*(p_ref->z() - start.z())));
    }
    else
    {
      multiply(p_ref, a_);
    }
  }

  p_out->set_x(fdb.x() + k_*(p_ref->x() - fdb.x()));
  p_out->set_y(fdb.y() + k_*(p_ref->y() - fdb.y()));
  p_out->set_z(fdb.z() + k_*(p_ref->z() - fdb.z()));
}

void EGMTrajectoryInterface::TrajectoryMotion::Controller::calculate(wrapper::Euler* p_out,
                                                                     wrapper::Euler* p_ref,
                                                                     const wrapper::Euler& fdb,
                                                                     const wrapper::Euler& start)
{
  if (is_normal_state_)
  {
    p_ref->set_x(a_*(start.x() + b_*(p_ref->x() - start.x())));
    p_ref->set_y(a_*(start.y() + b_*(p_ref->y() - start.y())));
    p_ref->set_z(a_*(start.z() + b_*(p_ref->z() - start.z())));
  }

  p_out->set_x(fdb.x() + k_*(p_ref->x() - fdb.x()));
  p_out->set_y(fdb.y() + k_*(p_ref->y() - fdb.y()));
  p_out->set_z(fdb.z() + k_*(p_ref->z() - fdb.z()));
}

void EGMTrajectoryInterface::TrajectoryMotion::Controller::calculate(wrapper::Quaternion* p_out,
                                                                     const wrapper::Quaternion& ref,
                                                                     const wrapper::Quaternion& fdb)
{
  p_out->set_u0(fdb.u0() + k_*(ref.u0() - fdb.u0()));
  p_out->set_u1(fdb.u1() + k_*(ref.u1() - fdb.u1()));
  p_out->set_u2(fdb.u2() + k_*(ref.u2() - fdb.u2()));
  p_out->set_u3(fdb.u3() + k_*(ref.u3() - fdb.u3()));

  normalize(p_out);
}




/***********************************************************************************************************************
 * Class definitions: EGMTrajectoryInterface::TrajectoryMotion
 */

/************************************************************
 * Primary methods
 */

void EGMTrajectoryInterface::TrajectoryMotion::generateOutputs(Output* p_outputs, const InputContainer& inputs)
{
  boost::lock_guard<boost::mutex> data_lock(data_.mutex);
  boost::lock_guard<boost::mutex> trajectory_lock(trajectories_.mutex);

  // Prepare for trajectory motion.
  prepare(inputs);

  // Only generate outputs, if the EGM session states are ok.
  if(inputs.statesOk())
  {
    // Update the current state.
    state_manager_.updateState();

    // Process the current state.
    switch (state_manager_.getState())
    {
      case Normal:     processNormalState();     break;
      case RampDown:   processRampDownState();   break;
      case StaticGoal: processStaticGoalState(); break;
    }

    // Prepare for any new goal.
    if (data_.has_new_goal)
    {
      // Update the interpolator.
      motion_step_.updateInterpolator();

      // Update the controller.
      controller_.update(state_manager_.getState(), motion_step_, configurations_);
    }

    // Generate the outputs.
    if (p_outputs && data_.has_active_goal)
    {
      // Evaluate the interpolator.
      motion_step_.evaluateInterpolator();

      // Calculate the outputs to the robot controller.
      controller_.calculate(p_outputs, &motion_step_);
    }
  }

  // Update the execution progress.
  if(p_outputs)
  {
    data_.execution_progress.set_state(state_manager_.mapState());
    data_.execution_progress.set_sub_state(state_manager_.mapSubState());
    data_.execution_progress.mutable_inputs()->CopyFrom(inputs.current());
    data_.execution_progress.mutable_outputs()->CopyFrom(*p_outputs);
    data_.execution_progress.set_goal_active(data_.has_active_goal);
    data_.execution_progress.mutable_goal()->CopyFrom(motion_step_.internal_goal);
    data_.execution_progress.set_time_passed(motion_step_.data.time_passed);
    data_.execution_progress.clear_active_trajectory();
    data_.execution_progress.mutable_active_trajectory()->add_points()->CopyFrom(motion_step_.external_goal);
    if (trajectories_.p_current)
    {
      trajectories_.p_current->copyTo(data_.execution_progress.mutable_active_trajectory());
    }
    if (trajectories_.temporary_queue.size() > 0)
    {
      data_.execution_progress.set_pending_trajectories((unsigned int) trajectories_.temporary_queue.size());
    }
    else
    {
      data_.execution_progress.set_pending_trajectories((unsigned int) trajectories_.primary_queue.size());
    }
    data_.has_updated_execution_progress = true;
  }
}

/************************************************************
 * Auxiliary methods
 */

void EGMTrajectoryInterface::TrajectoryMotion::prepare(const InputContainer& inputs)
{
  // Pre-prepare the auxiliary data.
  motion_step_.data.estimated_sample_time = inputs.estimatedSampleTime();
  motion_step_.data.feedback.CopyFrom(inputs.current().feedback());

  // Reset internal components, if a new EGM session has started.
  if (inputs.isFirstMessage())
  {
    resetTrajectoryMotion();
    motion_step_.resetMotionStep();
    state_manager_.resetStateManager();
  }

  // Activate the state manager if a new EGM session's states are ok. Otherwise,
  // reset the internal components if an active EGM session's states has become not ok.
  if (inputs.statesOk())
  {
    if (state_manager_.verifyState(Normal, None))
    {
      state_manager_.activateStateManager();
    }
  }
  else
  {
    if (!state_manager_.verifyState(Normal, None))
    {
      resetTrajectoryMotion();
      motion_step_.resetMotionStep();
      state_manager_.resetStateManager();

      trajectories_.primary_queue.clear();
      trajectories_.temporary_queue.clear();
    }
  }

  // Assume no new goal.
  data_.has_new_goal = false;
}

void EGMTrajectoryInterface::TrajectoryMotion::resetTrajectoryMotion()
{
  if (trajectories_.p_current)
  {
    storeNormalGoal();
    trajectories_.primary_queue.push_front(trajectories_.p_current);
    trajectories_.p_current.reset();
  }

  data_.has_active_goal = false;
  data_.has_new_goal = false;
  data_.execution_progress.Clear();
  data_.has_updated_execution_progress = false;
}

void EGMTrajectoryInterface::TrajectoryMotion::processNormalState()
{
  switch (state_manager_.getSubState())
  {
    case None:
      // Should never occur.
    break;

    case Running:
    {
      if (data_.pending_events.do_ramp_down)
      {
        state_manager_.setPendingState(RampDown, None);
      }
      else
      {
        if (trajectories_.p_current)
        {
          if (motion_step_.interpolationDurationReached())
          {
            if (motion_step_.internal_goal.reach())
            {
              if (motion_step_.conditionMet())
              {
                updateNormalGoal();
              }
            }
            else
            {
              updateNormalGoal();
            }
          }
        }
        else
        {
          if (!trajectories_.primary_queue.empty())
          {
            trajectories_.p_current = trajectories_.primary_queue.front();
            trajectories_.primary_queue.pop_front();
            updateNormalGoal();
          }
        }
      }
    }
    break;

    case Finished:
      // Should never occur.
    break;
  }
}

void EGMTrajectoryInterface::TrajectoryMotion::processRampDownState()
{
  switch (state_manager_.getSubState())
  {
    case None:
    {
      motion_step_.prepareRampDownGoal(data_.pending_events.do_stop);

      // Store any active goal.
      if (data_.has_active_goal)
      {
        storeNormalGoal();
      }

      data_.has_new_goal = true;
      data_.has_active_goal = true;
      state_manager_.setPendingState(RampDown, Running);
    }
    break;

    case Running:
    {
      if (motion_step_.interpolationDurationReached())
      {
        state_manager_.setPendingState(RampDown, Finished);
      }
    }
    break;

    case Finished:
    {
      // Handle the duration scale factor update event.
      if (data_.pending_events.do_duration_factor_update)
      {
        motion_step_.data.duration_factor = data_.pending_events.duration_factor;
        data_.pending_events.do_duration_factor_update = false;
      }

      // Handle the discard event.
      if (data_.pending_events.do_discard)
      {
        trajectories_.p_current.reset();
        trajectories_.primary_queue.clear();
        trajectories_.primary_queue.swap(trajectories_.temporary_queue);
        data_.pending_events.do_discard = false;
      }

      if (data_.pending_events.do_stop)
      {
        // If a complete stop has been ordered, then wait for a resume order.
        if (data_.pending_events.do_resume)
        {
          data_.pending_events.do_ramp_down = false;
          data_.pending_events.do_stop = false;
          data_.pending_events.do_resume = false;

          if (data_.pending_events.do_static_goal_start)
          {
            state_manager_.setPendingState(StaticGoal, None);
          }
          else
          {
            updateNormalGoal();
            state_manager_.setPendingState(Normal, Running);
          }
        }
      }
      else
      {
        data_.pending_events.do_ramp_down = false;
        updateNormalGoal();
        state_manager_.setPendingState(Normal, Running);
      }
    }
    break;
  }
}

void EGMTrajectoryInterface::TrajectoryMotion::processStaticGoalState()
{
  switch (state_manager_.getSubState())
  {
    case None:
    {
      // Store any currently executing trajectory into the queue.
      if (trajectories_.p_current)
      {
        trajectories_.primary_queue.push_front(trajectories_.p_current);
        trajectories_.p_current.reset();
      }

      data_.has_active_goal = false;
      state_manager_.setPendingState(StaticGoal, Running);
    }
    break;

    case Running:
    {
      // Update the internal goal, with the new static goal.
      if (data_.pending_events.do_static_position_goal_update && !data_.pending_events.do_ramp_down)
      {
        motion_step_.prepareStaticGoal(data_.pending_events.static_position_goal,
                                       data_.pending_events.do_static_goal_fast_update);
        data_.pending_events.static_position_goal.Clear();
        data_.pending_events.do_static_position_goal_update = false;
        data_.has_new_goal = true;
        data_.has_active_goal = true;
      }
      else if (data_.pending_events.do_static_velocity_goal_update && !data_.pending_events.do_ramp_down)
      {
        motion_step_.prepareStaticGoal(data_.pending_events.static_velocity_goal,
                                       data_.pending_events.do_static_goal_fast_update);
        data_.pending_events.static_velocity_goal.Clear();
        data_.pending_events.do_static_velocity_goal_update = false;
        data_.has_new_goal = true;
        data_.has_active_goal = true;
      }

      if (data_.pending_events.do_static_goal_finish)
      {
        state_manager_.setPendingState(StaticGoal, Finished);
      }
      else if (data_.pending_events.do_ramp_down)
      {
        state_manager_.setPendingState(RampDown, None);
      }
    }
    break;

    case Finished:
    {
      // Clear the static goal event flags.
      data_.pending_events.do_static_goal_start = false;
      data_.pending_events.do_static_goal_finish = false;
      data_.pending_events.do_static_position_goal_update = false;
      data_.pending_events.do_static_velocity_goal_update = false;

      // Perform a ramp down stop.
      data_.pending_events.do_ramp_down = true;
      data_.pending_events.do_stop = true;
      state_manager_.setPendingState(RampDown, None);
    }
    break;
  }
}

void EGMTrajectoryInterface::TrajectoryMotion::updateNormalGoal()
{
  bool success = false;

  if (trajectories_.p_current)
  {
    if (trajectories_.p_current->retriveNextTrajectoryPoint(&motion_step_.external_goal))
    {
      bool last_point = trajectories_.p_current->size() == 0;

      if (motion_step_.external_goal.reach())
      {
        // Check if the conditions are already fulfilled. If so, retrive another goal.
        do
        {
          motion_step_.prepareNormalGoal(last_point);
          success = !motion_step_.conditionMet();
        }
        while (!success && trajectories_.p_current->retriveNextTrajectoryPoint(&motion_step_.external_goal));
      }
      else
      {
        motion_step_.prepareNormalGoal(last_point);
        success = true;
      }
    }

    if (!success && trajectories_.p_current->size() == 0)
    {
      trajectories_.p_current.reset();
    }
  }

  data_.has_new_goal = success;
  data_.has_active_goal = success;
}

void EGMTrajectoryInterface::TrajectoryMotion::storeNormalGoal()
{
  if (trajectories_.p_current)
  {
    motion_step_.external_goal.set_duration(std::max(Constants::RobotController::LOWEST_SAMPLE_TIME,
                                            motion_step_.internal_goal.duration() - motion_step_.data.time_passed));

    trajectories_.p_current->addTrajectoryPointFront(motion_step_.external_goal);
  }
}

/************************************************************
 * User interaction methods
 */

bool EGMTrajectoryInterface::TrajectoryMotion::addTrajectory(const trajectory::TrajectoryGoal& trajectory,
                                                             const bool override_trajectories)
{
  boost::shared_ptr<EGMTrajectoryInterface::Trajectory> p_traj(new EGMTrajectoryInterface::Trajectory(trajectory));

  boost::lock_guard<boost::mutex> data_lock(data_.mutex);
  boost::lock_guard<boost::mutex> trajectory_lock(trajectories_.mutex);

  bool accepted = state_manager_.verifyState(Normal, Running);

  if (accepted)
  {
    if (override_trajectories)
    {
      trajectories_.temporary_queue.clear();
      trajectories_.temporary_queue.push_back(p_traj);
      data_.pending_events.do_ramp_down = true;
      data_.pending_events.do_stop = true;
      data_.pending_events.do_discard = true;
      data_.pending_events.do_resume = true;
    }
    else
    {
      if (data_.pending_events.do_discard)
      {
        trajectories_.temporary_queue.push_back(p_traj);
      }
      else
      {
        trajectories_.primary_queue.push_back(p_traj);
      }
    }
  }

  return accepted;
}

bool EGMTrajectoryInterface::TrajectoryMotion::stopTrajectory(const bool discard_trajectories)
{
  boost::lock_guard<boost::mutex> lock(data_.mutex);

  bool accepted = state_manager_.verifyState(Normal, Running);

  if (accepted)
  {
    data_.pending_events.do_ramp_down = true;
    data_.pending_events.do_stop = true;
    data_.pending_events.do_discard = discard_trajectories;
  }

  return accepted;
}

bool EGMTrajectoryInterface::TrajectoryMotion::resumeTrajectory()
{
  boost::lock_guard<boost::mutex> lock(data_.mutex);

  bool accepted = state_manager_.verifyState(RampDown, Finished);

  if (accepted)
  {
    data_.pending_events.do_resume = true;
  }

  return accepted;
}

bool EGMTrajectoryInterface::TrajectoryMotion::updateDurationFactor(double factor)
{
  boost::lock_guard<boost::mutex> lock(data_.mutex);

  bool accepted = state_manager_.verifyState(Normal, Running);

  if (accepted)
  {
    data_.pending_events.do_ramp_down = true;
    data_.pending_events.do_duration_factor_update = true;
    data_.pending_events.duration_factor = saturate(factor, DURATION_FACTOR_MIN, DURATION_FACTOR_MAX);
  }

  return accepted;
}

bool EGMTrajectoryInterface::TrajectoryMotion::startStaticGoal(const bool discard_trajectories)
{
  boost::lock_guard<boost::mutex> lock(data_.mutex);

  bool accepted = state_manager_.verifyState(Normal, Running);

  if (accepted)
  {
    data_.pending_events.do_ramp_down = true;
    data_.pending_events.do_stop = true;
    data_.pending_events.do_discard = discard_trajectories;
    data_.pending_events.do_resume = true;
    data_.pending_events.do_static_goal_start = true;
  }

  return accepted;
}

bool EGMTrajectoryInterface::TrajectoryMotion::setStaticGoal(const StaticPositionGoal& position_goal, const bool fast_transition)
{
  boost::lock_guard<boost::mutex> lock(data_.mutex);

  bool accepted = state_manager_.verifyState(StaticGoal, Running);

  if (accepted)
  {
    data_.pending_events.do_ramp_down = !fast_transition;
    data_.pending_events.do_stop = !fast_transition;
    data_.pending_events.do_resume = true;
    data_.pending_events.do_static_goal_fast_update = fast_transition;
    data_.pending_events.do_static_position_goal_update = true;
    data_.pending_events.do_static_velocity_goal_update = false;
    data_.pending_events.static_position_goal.CopyFrom(position_goal);
  }

  return accepted;
}

bool EGMTrajectoryInterface::TrajectoryMotion::setStaticGoal(const StaticVelocityGoal& velocity_goal, const bool fast_transition)
{
  boost::lock_guard<boost::mutex> lock(data_.mutex);

  bool accepted = state_manager_.verifyState(StaticGoal, Running);

  if (accepted)
  {
    data_.pending_events.do_ramp_down = !fast_transition;
    data_.pending_events.do_stop = !fast_transition;
    data_.pending_events.do_resume = true;
    data_.pending_events.do_static_goal_fast_update = fast_transition;
    data_.pending_events.do_static_velocity_goal_update = true;
    data_.pending_events.do_static_position_goal_update = false;
    data_.pending_events.static_velocity_goal.CopyFrom(velocity_goal);
  }

  return accepted;
}

bool EGMTrajectoryInterface::TrajectoryMotion::finishStaticGoal(const bool resume)
{
  boost::lock_guard<boost::mutex> lock(data_.mutex);

  bool accepted = state_manager_.verifyState(StaticGoal, Running);

  if (accepted)
  {
    data_.pending_events.do_static_goal_finish = true;
    data_.pending_events.do_resume = resume;
  }

  return accepted;
}

bool EGMTrajectoryInterface::TrajectoryMotion::retrieveExecutionProgress(trajectory::ExecutionProgress* p_progress)
{
  bool result = false;

  boost::lock_guard<boost::mutex> lock(data_.mutex);

  if (data_.execution_progress.has_inputs())
  {
    p_progress->CopyFrom(data_.execution_progress);
    result = data_.has_updated_execution_progress;
    data_.has_updated_execution_progress = false;
  }

  return result;
}




/***********************************************************************************************************************
 * Class definitions: EGMTrajectoryInterface
 */

/************************************************************
 * Primary methods
 */

EGMTrajectoryInterface::EGMTrajectoryInterface(boost::asio::io_service& io_service,
                                               const unsigned short port_number,
                                               const TrajectoryConfiguration& configuration)
:
EGMBaseInterface(io_service, port_number),
configuration_(configuration),
trajectory_motion_(configuration)
{
  if (configuration_.active.base.use_logging)
  {
    std::stringstream ss;
    ss << "port_" << port_number << +"_log.csv";
    p_logger_.reset(new EGMLogger(ss.str()));
  }
}

const std::string& EGMTrajectoryInterface::callback(const UDPServerData& server_data)
{
  // Initialize the callback by:
  // - Parsing and extracting data from the received message.
  // - Updating any pending configuration changes.
  // - Preparing the outputs.
  if (initializeCallback(server_data))
  {
    // Handle demo execution or trajectory execution.
    if (configuration_.active.base.use_demo_outputs)
    {
      outputs_.generateDemoOutputs(inputs_);
    }
    else
    {
      trajectory_motion_.generateOutputs(&outputs_.current, inputs_);
    }

    // Log inputs and outputs.
    if (configuration_.active.base.use_logging && p_logger_)
    {
      logData(inputs_, outputs_, configuration_.active.base.max_logging_duration);
    }

    // Constuct the reply message.
    outputs_.constructReply(configuration_.active.base);

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

bool EGMTrajectoryInterface::initializeCallback(const UDPServerData& server_data)
{
  bool success = false;

  // Parse the received message.
  if (server_data.p_data)
  {
    success = inputs_.parseFromArray(server_data.p_data, server_data.bytes_transferred);
  }

  // Update configurations, if requested to do so.
  if (success && inputs_.isFirstMessage())
  {
    boost::lock_guard<boost::mutex> lock(configuration_.mutex);

    if (configuration_.has_pending_update)
    {
      configuration_.active = configuration_.update;
      configuration_.has_pending_update = false;

      trajectory_motion_.updateConfigurations(configuration_.active);
    }
  }

  // Extract information from the parsed message.
  if (success)
  {
    success = inputs_.extractParsedInformation(configuration_.active.base.axes);

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

TrajectoryConfiguration EGMTrajectoryInterface::getConfiguration()
{
  boost::lock_guard<boost::mutex> lock(configuration_.mutex);

  return configuration_.update;
}

void EGMTrajectoryInterface::setConfiguration(const TrajectoryConfiguration& configuration)
{
  boost::lock_guard<boost::mutex> lock(configuration_.mutex);

  configuration_.update = configuration;
  configuration_.has_pending_update = true;
}

bool EGMTrajectoryInterface::addTrajectory(const trajectory::TrajectoryGoal trajectory,
                                           const bool override_trajectories)
{
  return trajectory_motion_.addTrajectory(trajectory, override_trajectories);
}

bool EGMTrajectoryInterface::stopTrajectory(const bool discard_trajectories)
{
  return trajectory_motion_.stopTrajectory(discard_trajectories);
}

bool EGMTrajectoryInterface::resumeTrajectory()
{
  return trajectory_motion_.resumeTrajectory();
}

bool EGMTrajectoryInterface::updateDurationFactor(double factor)
{
  return trajectory_motion_.updateDurationFactor(factor);
}

bool EGMTrajectoryInterface::startStaticGoal(const bool discard_trajectories)
{
  return trajectory_motion_.startStaticGoal(discard_trajectories);
}

bool EGMTrajectoryInterface::setStaticGoal(const StaticPositionGoal& position_goal, const bool fast_transition)
{
  return trajectory_motion_.setStaticGoal(position_goal, fast_transition);
}

bool EGMTrajectoryInterface::setStaticGoal(const StaticVelocityGoal& velocity_goal, const bool fast_transition)
{
  return trajectory_motion_.setStaticGoal(velocity_goal, fast_transition);
}

bool EGMTrajectoryInterface::finishStaticGoal(const bool resume)
{
  return trajectory_motion_.finishStaticGoal(resume);
}

bool EGMTrajectoryInterface::retrieveExecutionProgress(trajectory::ExecutionProgress* p_execution_progress)
{
  bool result = false;

  if (p_execution_progress)
  {
    result = trajectory_motion_.retrieveExecutionProgress(p_execution_progress);
  }

  return result;
}

} // end namespace egm
} // end namespace abb
