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

#include "abb_libegm/egm_common_auxiliary.h"
#include "abb_libegm/egm_interpolator.h"

namespace abb
{
namespace egm
{
/***********************************************************************************************************************
 * Class definitions: EGMInterpolator::SplineConditions
 */

/************************************************************
 * Primary methods
 */

void EGMInterpolator::SplineConditions::setConditions(const int index,
                                                      const wrapper::trajectory::JointGoal& start,
                                                      const wrapper::trajectory::JointGoal& goal)
{
  alfa    = start.position().values(index);
  d_alfa  = start.velocity().values(index);
  dd_alfa = start.acceleration().values(index);
  beta    = goal.position().values(index);
  d_beta  = goal.velocity().values(index);
  dd_beta = goal.acceleration().values(index);
}

void EGMInterpolator::SplineConditions::setConditions(const EGMInterpolator::Axis axis,
                                                      const wrapper::trajectory::CartesianGoal& start,
                                                      const wrapper::trajectory::CartesianGoal& goal)
{
  switch (axis)
  {
    case EGMInterpolator::X:
    {
      alfa    = start.pose().position().x();
      d_alfa  = start.velocity().x();
      dd_alfa = start.acceleration().x();
      beta    = goal.pose().position().x();
      d_beta  = goal.velocity().x();
      dd_beta = goal.acceleration().x();
    }
    break;

    case EGMInterpolator::Y:
    {
      alfa = start.pose().position().y();
      d_alfa = start.velocity().y();
      dd_alfa = start.acceleration().y();
      beta = goal.pose().position().y();
      d_beta = goal.velocity().y();
      dd_beta = goal.acceleration().y();
    }
    break;

    case EGMInterpolator::Z:
    {
      alfa = start.pose().position().z();
      d_alfa = start.velocity().z();
      dd_alfa = start.acceleration().z();
      beta = goal.pose().position().z();
      d_beta = goal.velocity().z();
      dd_beta = goal.acceleration().z();
    }
    break;
  }
}




/***********************************************************************************************************************
 * Class definitions: EGMInterpolator::SplinePolynomial
 */

/************************************************************
 * Primary methods
 */

void EGMInterpolator::SplinePolynomial::update(const SplineConditions& conditions)
{
  const double T = conditions.duration;
  const double K = saturate(conditions.ramp_down_factor, 0.0, 1.0);

  // Support variables.
  double c1 = 0.0;
  double c2 = 0.0;
  double c3 = 0.0;
  double alfa    = conditions.alfa;
  double d_alfa  = conditions.d_alfa;
  double dd_alfa = conditions.dd_alfa;
  double beta    = conditions.beta;
  double d_beta  = conditions.d_beta;
  double dd_beta = conditions.dd_beta;

  if (conditions.do_ramp_down)
  {
    //---------------------------------------------------------------
    // Calculate the spline polynomial coefficients for:
    // S(t) = A + B*t + C*t^2 + D*t^3
    //
    // Note: Used when ramping down the velocity.
    //       E.g. for stopping use K = 0.0.
    //
    //---------------------
    // Conditions:
    //---------------------
    // 0 < t <= T
    //
    // S(0) = alfa
    // d_S(0) = d_alfa
    //
    // d_S(T) = K*d_alfa, K = [0.0, 1.0];
    // dd_S(T) = 0.0
    //---------------------------------------------------------------
    a_ = alfa;
    b_ = d_alfa;
    c_ = ((K - 1.0)*d_alfa) / T;
    d_ = (-c_)/(3.0*T);
    e_ = 0.0;
    f_ = 0.0;
  }
  else
  {
    switch (conditions.spline_method)
    {
      case TrajectoryConfiguration::Linear:
      {
        //---------------------------------------------------------------
        // Calculate the spline polynomial coefficients for:
        // S(t) = A + B*t
        //
        //---------------------
        // Conditions:
        //---------------------
        // 0 <= t <= T
        //
        // S(0) = alfa
        // S(T) = beta
        //---------------------------------------------------------------
        a_ = alfa;
        b_ = (beta - alfa) / T;
        c_ = 0.0;
        d_ = 0.0;
        e_ = 0.0;
        f_ = 0.0;
      }
      break;

      case TrajectoryConfiguration::Square:
      {
        //---------------------------------------------------------------
        // Calculate the spline polynomial coefficients for:
        // S(t) = A + B*t + C*t^2
        //
        //---------------------
        // Conditions:
        //---------------------
        // 0 <= t <= T
        //
        // S(0) = alfa
        // d_S(0) = d_alfa
        //
        // S(T) = beta
        //---------------------------------------------------------------
        a_ = alfa;
        b_ = d_alfa;
        c_ = (beta - alfa - d_alfa*T) / std::pow(T, 2);
        d_ = 0.0;
        e_ = 0.0;
        f_ = 0.0;
      }
      break;

      case TrajectoryConfiguration::Cubic:
      {
        //---------------------------------------------------------------
        // Calculate the spline polynomial coefficients for:
        // S(t) = A + B*t + C*t^2 + D*t^3
        //
        //---------------------
        // Conditions:
        //---------------------
        // 0 <= t <= T
        //
        // S(0) = alfa
        // d_S(0) = d_alfa
        //
        // S(T) = beta
        // d_S(T) = d_beta
        //---------------------------------------------------------------
        a_ = alfa;
        b_ = d_alfa;

        c1 = beta - alfa - d_alfa*T;
        c2 = d_beta - d_alfa;

        c_ = 3.0*c1 / std::pow(T, 2) - c2 / T;
        d_ = c1 / std::pow(T, 3) - c_ / T;
        e_ = 0.0;
        f_ = 0.0;
      }
      break;

      case TrajectoryConfiguration::Quintic:
      {
        //---------------------------------------------------------------
        // Calculate the spline polynomial coefficients for:
        // S(t) = A + B*t + C*t^2 + D*t^3 + E*t^4 + F*t^5
        //
        //---------------------
        // Conditions:
        //---------------------
        // 0 <= t <= T
        //
        // S(0) = alfa
        // d_S(0) = d_alfa
        // dd_S(0) = dd_alfa
        //
        // S(T) = beta
        // d_S(T) = d_beta
        // dd_S(T) = dd_beta
        //---------------------------------------------------------------
        a_ = alfa;
        b_ = d_alfa;
        c_ = dd_alfa / 2.0;

        c1 = beta - alfa - d_alfa*T - (dd_alfa / 2.0)*std::pow(T, 2);
        c2 = d_beta - d_alfa - dd_alfa*T;
        c3 = dd_beta - dd_alfa;

        d_ = 10.0*c1 / std::pow(T, 3) - 4.0 * c2 / std::pow(T, 2) + c3 / (2.0*T);
        e_ = 5.0*c1 / std::pow(T, 4) - c2 / std::pow(T, 3) - 2.0*d_ / T;
        f_ = c1 / std::pow(T, 5) - d_ / std::pow(T, 2) - e_ / T;
      }
      break;
    }
  }
}

void EGMInterpolator::SplinePolynomial::evaluate(wrapper::trajectory::JointGoal* p_output,
                                                 const int index,
                                                 const double t)
{
  //---------------------------------------------------------------
  // Evaluate:
  //   S(t) = A + B*t + C*t^2 + D*t^3 + E*t^4 + F*t^5
  //   S_prime(t) = B + 2C*t + 3D*t^2 + 4E*t^3 + 5F*t^4
  //   S_bis(t) = 2C + 6D*t + 12E*t^2 + 20F*t^3
  //
  // Condition: 0 <= t <= T
  //---------------------------------------------------------------
  p_output->mutable_position()->set_values(index, calculatePosition(t));
  p_output->mutable_velocity()->set_values(index, calculateVelocity(t));
  p_output->mutable_acceleration()->set_values(index, calculateAcceleration(t));
}

void EGMInterpolator::SplinePolynomial::evaluate(wrapper::trajectory::CartesianGoal* p_output,
                                                 const Axis axis,
                                                 const double t)
{
  //---------------------------------------------------------------
  // Evaluate:
  //   S(t) = A + B*t + C*t^2 + D*t^3 + E*t^4 + F*t^5
  //   S_prime(t) = B + 2C*t + 3D*t^2 + 4E*t^3 + 5F*t^4
  //   S_bis(t) = 2C + 6D*t + 12E*t^2 + 20F*t^3
  //
  // Condition: 0 <= t <= T
  //---------------------------------------------------------------
  switch (axis)
  {
    case X:
    {
      p_output->mutable_pose()->mutable_position()->set_x(calculatePosition(t));
      p_output->mutable_velocity()->set_x(calculateVelocity(t));
      p_output->mutable_acceleration()->set_x(calculateAcceleration(t));
    }
    break;

    case Y:
    {
      p_output->mutable_pose()->mutable_position()->set_y(calculatePosition(t));
      p_output->mutable_velocity()->set_y(calculateVelocity(t));
      p_output->mutable_acceleration()->set_y(calculateAcceleration(t));
    }
    break;

    case Z:
    {
      p_output->mutable_pose()->mutable_position()->set_z(calculatePosition(t));
      p_output->mutable_velocity()->set_z(calculateVelocity(t));
      p_output->mutable_acceleration()->set_z(calculateAcceleration(t));
    }
    break;
  }
}




/***********************************************************************************************************************
 * Class definitions: EGMInterpolator::Slerp
 */

/************************************************************
 * Primary methods
 */

void EGMInterpolator::Slerp::update(const wrapper::Quaternion& start,
                                    const wrapper::Quaternion& goal,
                                    const Conditions& conditions)
{
  duration_ = conditions.duration;

  q0_.CopyFrom(start);
  q1_.CopyFrom(goal);

  normalize(&q0_);
  normalize(&q1_);

  double dot_product = dotProduct(q0_, q1_);

  // Check if Slerp or linear interpolation should be used.
  use_linear_ = std::abs(dot_product) > DOT_PRODUCT_THRESHOLD;

  if (!use_linear_)
  {
    // Reverse one of the quaternions, if the dot product is negative.
    // This is to make the Slerp to take the shorter path.
    if (dot_product < 0.0)
    {
      multiply(&q1_, -1.0);
      dot_product = -dot_product;
    }

    // Saturate the dot product to be within acos input range.
    dot_product = saturate(dot_product, -1.0, 1.0);

    // Calculate the coefficient.
    omega_ = std::acos(dot_product);
  }
}

void EGMInterpolator::Slerp::evaluate(wrapper::trajectory::CartesianGoal* p_output, double t)
{
  // Support variables.
  double a = 1.0;
  double b = 0.0;
  double c = 1.0;
  double d = 0.0;
  double k = 1.0 / std::sin(omega_);

  // Quaternion and angular velocity output to set.
  // Note: The Euler field is internally used to contain angular velocities.
  wrapper::Quaternion* p_q = p_output->mutable_pose()->mutable_quaternion();
  wrapper::Euler* p_av = p_output->mutable_pose()->mutable_euler();

  // Saturate t to be within 0.0 and 1.0.
  t = saturate(t / duration_, 0.0, 1.0);

  if (use_linear_)
  {
    // Calculate quaternion and angular velocity with linear interpolation.
    a = 1.0 - t;
    b = t;
    c = -1.0;
    d = 1.0;
  }
  else
  {
    // Calculate quaternion and angular velocity with Slerp interpolation.
    a = k*std::sin((1.0 - t)*omega_);
    b = k*std::sin(t*omega_);
    c = -omega_*k*std::cos((1.0 - t)*omega_) / duration_;
    d = omega_*k*std::cos(t*omega_) / duration_;
  }

  // Calculate the quaternion output.
  p_q->set_u0(a*q0_.u0() + b*q1_.u0());
  p_q->set_u1(a*q0_.u1() + b*q1_.u1());
  p_q->set_u2(a*q0_.u2() + b*q1_.u2());
  p_q->set_u3(a*q0_.u3() + b*q1_.u3());
  normalize(p_q);

  // Conjugate of the normalized quaternion output.
  wrapper::Quaternion conj_q;
  conj_q.set_u0(p_q->u0());
  conj_q.set_u1(-p_q->u1());
  conj_q.set_u2(-p_q->u2());
  conj_q.set_u3(-p_q->u3());

  // Derivate of either linear or Slerp interpolation.
  wrapper::Quaternion d_q;
  d_q.set_u0(c*q0_.u0() + d*q1_.u0());
  d_q.set_u1(c*q0_.u1() + d*q1_.u1());
  d_q.set_u2(c*q0_.u2() + d*q1_.u2());
  d_q.set_u3(c*q0_.u3() + d*q1_.u3());

  // Calculate the angular velocity output.
  wrapper::Quaternion mult_q(multiply(d_q, conj_q));
  p_av->set_x(2.0*mult_q.u1()*Constants::Conversion::RAD_2_DEG);
  p_av->set_y(2.0*mult_q.u2()*Constants::Conversion::RAD_2_DEG);
  p_av->set_z(2.0*mult_q.u3()*Constants::Conversion::RAD_2_DEG);
}




/***********************************************************************************************************************
 * Class definitions: EGMInterpolator::SoftRamp
 */

/************************************************************
 * Primary methods
 */

void EGMInterpolator::SoftRamp::update(const wrapper::trajectory::PointGoal& start,
                                       const wrapper::trajectory::PointGoal& goal,
                                       const Conditions& conditions)
{
  duration_ = conditions.duration;
  operation_ = conditions.operation;

  switch (operation_)
  {
    case RampDown:
    {
      // Note: The Euler field is internally used to contain angular velocities.
      start_angular_velocity_.CopyFrom(start.robot().cartesian().pose().euler());
    }
    break;

    case RampInPosition:
    case RampInVelocity:
    {
      start_.CopyFrom(start);
      goal_.CopyFrom(goal);
    }
    break;
  }
}

void EGMInterpolator::SoftRamp::evaluate(wrapper::trajectory::JointGoal* p_output,
                                         const bool robot,
                                         const double sample_time,
                                         double t)
{
  double ramp_factor = 0.0;
  double d_ramp_factor = 0.0;

  // Saturate t to be within 0.0 and 1.0.
  t = saturate(t / duration_, 0.0, 1.0);

  switch (operation_)
  {
    case RampInPosition:
    {
      // Ramp factor that goes from 0.0 to 1.0 and its derivate.
      ramp_factor = 0.5*std::cos(M_PI*t + M_PI) + 0.5;
      d_ramp_factor = -0.5*M_PI / duration_*std::sin(M_PI*t + M_PI);

      // Output to set.
      wrapper::Joints* p_p = p_output->mutable_position();
      wrapper::Joints* p_v = p_output->mutable_velocity();
      wrapper::Joints* p_a = p_output->mutable_acceleration();

      const wrapper::Joints& start_rj = (robot ? start_.robot().joints().position()
                                         : start_.external().joints().position());
      const wrapper::Joints& goal_rj = (robot ? goal_.robot().joints().position()
                                        : goal_.external().joints().position());

      // Calculate the position and velocity outputs.
      for (int i = 0;
           i < p_p->values_size() && i < p_v->values_size() && i < p_a->values_size() &&
           i < start_rj.values_size() && i < goal_rj.values_size();
           ++i)
      {
        p_p->set_values(i, start_rj.values(i) + ramp_factor*(goal_rj.values(i) - start_rj.values(i)));
        p_v->set_values(i, d_ramp_factor*(goal_rj.values(i) - start_rj.values(i)));
        p_a->set_values(i, 0.0);
      }
    }

    case RampInVelocity:
    {
      // Ramp factor that goes from 0.0 to 1.0.
      ramp_factor = 0.5*std::cos(M_PI*t + M_PI) + 0.5;

      // Output to set.
      wrapper::Joints* p_p = p_output->mutable_position();
      wrapper::Joints* p_v = p_output->mutable_velocity();
      wrapper::Joints* p_a = p_output->mutable_acceleration();

      const wrapper::Joints& start_rj = (robot ? start_.robot().joints().velocity()
                                         : start_.external().joints().velocity());
      const wrapper::Joints& goal_rj = (robot ? goal_.robot().joints().velocity()
                                        : goal_.external().joints().velocity());

      // Calculate the position and velocity outputs.
      for (int i = 0;
           i < p_p->values_size() && i < p_v->values_size() && i < p_a->values_size() &&
           i < start_rj.values_size() && i < goal_rj.values_size();
           ++i)
      {
        p_a->set_values(i, 0.0);
        p_v->set_values(i, start_rj.values(i) + ramp_factor*(goal_rj.values(i) - start_rj.values(i)));
        p_p->set_values(i, p_p->values(i) + sample_time*p_v->values(i));
      }
    }
    break;
  }
}

void EGMInterpolator::SoftRamp::evaluate(wrapper::trajectory::CartesianGoal* p_output,
                                         const double sample_time,
                                         double t)
{
  double ramp_factor = 0.0;
  double d_ramp_factor = 0.0;
  wrapper::Quaternion previous_q(p_output->mutable_pose()->quaternion());

  // Saturate t to be within 0.0 and 1.0.
  t = saturate(t / duration_, 0.0, 1.0);

  switch (operation_)
  {
    case RampDown:
    {
      // Output to set. Note: The Euler field is internally used to contain angular velocities.
      wrapper::Quaternion* p_q = p_output->mutable_pose()->mutable_quaternion();
      wrapper::Euler* p_av = p_output->mutable_pose()->mutable_euler();

      // Ramp factor that goes from 1.0 to 0.0.
      ramp_factor = 0.5*std::cos(M_PI*t) + 0.5;

      // Calculate the angular velocity output.
      p_av->set_x(ramp_factor*start_angular_velocity_.x());
      p_av->set_y(ramp_factor*start_angular_velocity_.y());
      p_av->set_z(ramp_factor*start_angular_velocity_.z());

      // Calculate the quaternion derivate. Note: p_output contain the previously calculated quaternion.
      wrapper::Quaternion d_q;
      convert(&d_q, p_output->mutable_pose()->quaternion(), *p_av);

      // Calculate the quaternion output.
      p_q->set_u0(p_q->u0() + sample_time*d_q.u0());
      p_q->set_u1(p_q->u1() + sample_time*d_q.u1());
      p_q->set_u2(p_q->u2() + sample_time*d_q.u2());
      p_q->set_u3(p_q->u3() + sample_time*d_q.u3());
      normalize(p_q);
    }
    break;

    case RampInPosition:
    {
      // Output to set. Note: The Euler field is internally used to contain angular velocities.
      wrapper::Cartesian* p_p = p_output->mutable_pose()->mutable_position();
      wrapper::Cartesian* p_v = p_output->mutable_velocity();
      wrapper::Cartesian* p_a = p_output->mutable_acceleration();
      wrapper::Quaternion* p_q = p_output->mutable_pose()->mutable_quaternion();
      wrapper::Euler* p_av = p_output->mutable_pose()->mutable_euler();

      // Ramp factor that goes from 0.0 to 1.0 and its derivate.
      ramp_factor = 0.5*std::cos(M_PI*t + M_PI) + 0.5;
      d_ramp_factor = -0.5*M_PI / duration_*std::sin(M_PI*t + M_PI);

      const wrapper::Cartesian& start_p = start_.robot().cartesian().pose().position();
      const wrapper::Cartesian& goal_p = goal_.robot().cartesian().pose().position();

      // Calculate the position and velocity outputs.
      p_p->set_x(start_p.x() + ramp_factor*(goal_p.x() - start_p.x()));
      p_p->set_y(start_p.y() + ramp_factor*(goal_p.y() - start_p.y()));
      p_p->set_z(start_p.z() + ramp_factor*(goal_p.z() - start_p.z()));
      p_v->set_x(d_ramp_factor*(goal_p.x() - start_p.x()));
      p_v->set_y(d_ramp_factor*(goal_p.y() - start_p.y()));
      p_v->set_z(d_ramp_factor*(goal_p.z() - start_p.z()));
      p_a->set_x(0.0);
      p_a->set_y(0.0);
      p_a->set_z(0.0);

      const wrapper::Quaternion& start_q = start_.robot().cartesian().pose().quaternion();
      const wrapper::Quaternion& goal_q = goal_.robot().cartesian().pose().quaternion();

      // Calculate the quaternion and angular velocity outputs.
      p_q->set_u0(start_q.u0() + ramp_factor*(goal_q.u0() - start_q.u0()));
      p_q->set_u1(start_q.u1() + ramp_factor*(goal_q.u1() - start_q.u1()));
      p_q->set_u2(start_q.u2() + ramp_factor*(goal_q.u2() - start_q.u2()));
      p_q->set_u3(start_q.u3() + ramp_factor*(goal_q.u3() - start_q.u3()));

      normalize(p_q);

      estimateVelocities(p_av, *p_q, previous_q, sample_time);
    }

    case RampInVelocity:
    {
      // Output to set. Note: The Euler field is internally used to contain angular velocities.
      wrapper::Cartesian* p_p = p_output->mutable_pose()->mutable_position();
      wrapper::Cartesian* p_v = p_output->mutable_velocity();
      wrapper::Cartesian* p_a = p_output->mutable_acceleration();
      wrapper::Quaternion* p_q = p_output->mutable_pose()->mutable_quaternion();
      wrapper::Euler* p_av = p_output->mutable_pose()->mutable_euler();

      // Ramp factor that goes from 0.0 to 1.0.
      ramp_factor = 0.5*std::cos(M_PI*t + M_PI) + 0.5;

      const wrapper::Cartesian& start_v = start_.robot().cartesian().velocity();
      const wrapper::Cartesian& goal_v = goal_.robot().cartesian().velocity();

      // Calculate the position and velocity outputs.
      p_a->set_x(0.0);
      p_a->set_y(0.0);
      p_a->set_z(0.0);
      p_v->set_x(start_v.x() + ramp_factor*(goal_v.x() - start_v.x()));
      p_v->set_y(start_v.y() + ramp_factor*(goal_v.y() - start_v.y()));
      p_v->set_z(start_v.z() + ramp_factor*(goal_v.z() - start_v.z()));
      p_p->set_x(p_p->x() + sample_time*p_v->x());
      p_p->set_y(p_p->y() + sample_time*p_v->y());
      p_p->set_z(p_p->z() + sample_time*p_v->z());

      // Note: The Euler field is internally used to contain angular velocities.
      const wrapper::Euler& start_av = start_.robot().cartesian().pose().euler();
      const wrapper::Euler& goal_av = goal_.robot().cartesian().pose().euler();

      // Calculate the quaternion and angular velocity outputs.
      p_av->set_x(start_av.x() + ramp_factor*(goal_av.x() - start_av.x()));
      p_av->set_y(start_av.y() + ramp_factor*(goal_av.y() - start_av.y()));
      p_av->set_z(start_av.z() + ramp_factor*(goal_av.z() - start_av.z()));

      wrapper::Quaternion d_q;
      convert(&d_q, previous_q, *p_av);

      p_q->set_u0(p_q->u0() + sample_time*d_q.u0());
      p_q->set_u1(p_q->u1() + sample_time*d_q.u1());
      p_q->set_u2(p_q->u2() + sample_time*d_q.u2());
      p_q->set_u3(p_q->u3() + sample_time*d_q.u3());
      normalize(p_q);
    }
    break;
  }
}




/***********************************************************************************************************************
 * Class definitions: EGMInterpolator
 */

/************************************************************
 * Primary methods
 */

void EGMInterpolator::update(const wrapper::trajectory::PointGoal& start,
                             const wrapper::trajectory::PointGoal& goal,
                             const Conditions& conditions)
{
  conditions_ = conditions;
  conditions_.duration = std::max(Constants::RobotController::LOWEST_SAMPLE_TIME, conditions_.duration);

  switch (conditions_.operation)
  {
    case Normal:
    case RampDown:
    {
      offset_ = start.robot().joints().position().values_size();
      SplineConditions spline_conditions(conditions_);

      switch (conditions_.mode)
      {
        case EGMJoint:
        {
          // Robot joints.
          for (int i = 0; i < start.robot().joints().position().values_size() && i < spline_polynomials_.size(); ++i)
          {
            spline_conditions.setConditions(i, start.robot().joints(), goal.robot().joints());
            spline_polynomials_[i].update(spline_conditions);
          }

          // External joints.
          for (int i = 0; i < start.external().joints().position().values_size() && i < spline_polynomials_.size(); ++i)
          {
            spline_conditions.setConditions(i, start.external().joints(), goal.external().joints());
            spline_polynomials_[i + offset_].update(spline_conditions);
          }
        }
        break;

        case EGMPose:
        {
          // X, Y and Z.
          spline_conditions.setConditions(X, start.robot().cartesian(), goal.robot().cartesian());
          spline_polynomials_[X].update(spline_conditions);
          spline_conditions.setConditions(Y, start.robot().cartesian(), goal.robot().cartesian());
          spline_polynomials_[Y].update(spline_conditions);
          spline_conditions.setConditions(Z, start.robot().cartesian(), goal.robot().cartesian());
          spline_polynomials_[Z].update(spline_conditions);

          // Orientation.
          if (conditions_.operation == Normal)
          {
            slerp_.update(start.robot().cartesian().pose().quaternion(),
                          goal.robot().cartesian().pose().quaternion(), conditions);
          }
          else
          {
            soft_ramp_.update(start, goal, conditions);
          }

          // External joints.
          for (int i = 0; i < start.external().joints().position().values_size() && i < spline_polynomials_.size(); ++i)
          {
            spline_conditions.setConditions(i, start.external().joints(), goal.external().joints());
            spline_polynomials_[i + offset_].update(spline_conditions);
          }
        }
        break;
      }
    }
    break;

    case RampInPosition:
    case RampInVelocity:
    {
      soft_ramp_.update(start, goal, conditions);
    }
    break;
  }
}

void EGMInterpolator::evaluate(wrapper::trajectory::PointGoal* p_output, const double sample_time, double t)
{
  t = saturate(t, 0.0, conditions_.duration);

  switch (conditions_.operation)
  {
    case Normal:
    case RampDown:
    {
      switch (conditions_.mode)
      {
        case EGMJoint:
        {
          // Robot joints.
          for (int i = 0;
               i < p_output->robot().joints().position().values_size() && i < spline_polynomials_.size(); ++i)
          {
            spline_polynomials_[i].evaluate(p_output->mutable_robot()->mutable_joints(), i, t);
          }

          // External joints.
          for (int i = 0;
               i < p_output->external().joints().position().values_size() && i < spline_polynomials_.size(); ++i)
          {
            spline_polynomials_[i + offset_].evaluate(p_output->mutable_external()->mutable_joints(), i, t);
          }
        }
        break;

        case EGMPose:
        {
          // X, Y and Z.
          spline_polynomials_[X].evaluate(p_output->mutable_robot()->mutable_cartesian(), X, t);
          spline_polynomials_[Y].evaluate(p_output->mutable_robot()->mutable_cartesian(), Y, t);
          spline_polynomials_[Z].evaluate(p_output->mutable_robot()->mutable_cartesian(), Z, t);

          // Orientation.
          if (conditions_.operation == Normal)
          {
            slerp_.evaluate(p_output->mutable_robot()->mutable_cartesian(), t);
          }
          else
          {
            soft_ramp_.evaluate(p_output->mutable_robot()->mutable_cartesian(), sample_time, t);
          }

          // External joints.
          for (int i = 0;
               i < p_output->external().joints().position().values_size() && i < spline_polynomials_.size(); ++i)
          {
            spline_polynomials_[i + offset_].evaluate(p_output->mutable_external()->mutable_joints(), i, t);
          }
        }
        break;
      }
    }
    break;

    case RampInPosition:
    case RampInVelocity:
    {
      switch (conditions_.mode)
      {
        case EGMJoint:
        {
          // Robot joints.
          soft_ramp_.evaluate(p_output->mutable_robot()->mutable_joints(), true, sample_time, t);

          // External joints.
          soft_ramp_.evaluate(p_output->mutable_external()->mutable_joints(), false, sample_time, t);
        }
        break;

        case EGMPose:
        {
          // Cartesian pose.
          soft_ramp_.evaluate(p_output->mutable_robot()->mutable_cartesian(), sample_time, t);

          // External joints.
          soft_ramp_.evaluate(p_output->mutable_external()->mutable_joints(), false, sample_time, t);
        }
        break;
      }
    }
    break;
  }
}

} // end namespace egm
} // end namespace abb
