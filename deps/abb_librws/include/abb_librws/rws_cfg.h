/***********************************************************************************************************************
 *
 * Copyright (c) 2020, ABB Schweiz AG
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

/**
 * The intention of this file is to provide representations
 * of RobotWare system configuration instances.
 *
 * Please see the "Technical reference manual - System parameters"
 * (document ID: 3HAC050948-001, revision: N) for more information
 * about how configurations are structured in RobotWare.
 */

#ifndef RWS_CFG_H
#define RWS_CFG_H

#include <string>

#include "rws_rapid.h"

namespace abb
{
namespace rws
{
/**
 * \brief Namespace for configuration domains.
 */
namespace cfg
{

/**
 * \brief Namespace for types belonging to the motion (MOC) domain.
 */
namespace moc
{
/**
 * \brief Representation of a 'Arm' instance.
 */
struct Arm
{
  /**
   * \brief The instance's name.
   */
  std::string name;

  /**
   * \brief The lower joint bound.
   *
   * The unit depends on the type of joint, i.e. [rad] for rotational and [m] for linear.
   *
   */
  double lower_joint_bound;

  /**
   * \brief The upper joint bound.
   *
   * The unit depends on the type of joint, i.e. [rad] for rotational and [m] for linear.
   */
  double upper_joint_bound;
};

/**
 * \brief Representation of a 'Joint' instance.
 */
struct Joint
{
  /**
   * \brief The instance's name.
   */
  std::string name;

  /**
   * \brief The joint's logical axis.
   */
  int logical_axis;

  /**
   * \brief The joint's kinematic axis number.
   */
  int kinematic_axis_number;

  /**
   * \brief The arm used by the joint.
   */
  std::string use_arm;

  /**
   * \brief The transmission used by the joint.
   */
  std::string use_transmission;
};

/**
 * \brief Representation of a 'Mechanical Unit' instance.
 */
struct MechanicalUnit
{
  /**
   * \brief The instance's name.
   */
  std::string name;

  /**
   * \brief The robot used by the mechanical unit.
   */
  std::string use_robot;

  /**
   * \brief The singles used by the mechanical unit.
   */
  std::vector<std::string> use_singles;
};

/**
 * \brief Representation of a 'Robot' instance.
 */
struct Robot
{
  /**
   * \brief The instance's name.
   */
  std::string name;

  /**
   * \brief The robot's type.
   */
  std::string use_robot_type;

  /**
   * \brief The joints used by the robot.
   */
  std::vector<std::string> use_joints;

  /**
   * \brief The robot's base frame.
   */
  Pose base_frame;

  /**
   * \brief The robot or single that moves the robot's base frame.
   */
  std::string base_frame_moved_by;
};

/**
 * \brief Representation of a 'Single' instance.
 */
struct Single
{
  /**
   * \brief The instance's name.
   */
  std::string name;

  /**
   * \brief The single's type.
   */
  std::string use_single_type;

  /**
   * \brief The joint used by the single.
   */
  std::string use_joint;

  /**
   * \brief The single's base frame.
   */
  Pose base_frame;

  /**
   * \brief The robot or single that moves the single's base frame.
   */
  std::string base_frame_coordinated;
};

/**
 * \brief Representation of a 'Transmission' instance.
 */
struct Transmission
{
  /**
   * \brief The instance's name.
   */
  std::string name;

  /**
   * \brief Specifies if the corresponding joint uses rotational motion (otherwise linear).
   */
  bool rotating_move;
};
} // end namespace moc

/**
 * \brief Namespace for types belonging to the controller (SYS) domain.
 */
namespace sys
{
/**
 * \brief Representation of a 'Mechanical Unit Group' instance.
 */
struct MechanicalUnitGroup
{
  /**
   * \brief The instance's name.
   */
  std::string name;

  /**
   * \brief The robot (with TCP) used by the group.
   */
  std::string robot;

  /**
   * \brief The mechanical units (without TCP) used by the group.
   */
  std::vector<std::string> mechanical_units;
};

/**
 * \brief Representation of a 'Present Option' instance.
 */
struct PresentOption
{
  /**
   * \brief The instance's name.
   */
  std::string name;

  /**
   * \brief The option's description.
   */
  std::string description;
};
} // end namespace sys

} // end namespace cfg
} // end namespace rws
} // end namespace abb

#endif
