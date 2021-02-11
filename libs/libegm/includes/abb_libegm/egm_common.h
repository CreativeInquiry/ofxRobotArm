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

#ifndef EGM_COMMON_H
#define EGM_COMMON_H

#include "abb_libegm_export.h"

namespace abb
{
namespace egm
{
/**
 * \brief Enum for the number of axes of the robot.
 */
enum RobotAxes
{
  None  = 0, ///< \brief No robot axes are expected (i.e. only external axes).
  Six   = 6, ///< \brief A six axes robot.
  Seven = 7  ///< \brief A seven axes robot.
};

/**
 * \brief Enum for the supported EGM modes (i.e. the corresponding RAPID instructions).
 */
enum EGMModes
{
  EGMJoint, ///< \brief The EGM joint mode.
  EGMPose   ///< \brief The EGM pose mode.
};

/**
 * \brief Struct containing various constant values.
 */
struct Constants
{
  /**
   * \brief Constants related to the robot controller system.
   */
  struct ABB_LIBEGM_EXPORT RobotController
  {
    /**
     * \brief Lowest sample time [s] used in EGM communication.
     */
    static const double LOWEST_SAMPLE_TIME;

    /**
     * \brief Default port number assumed for EGM communication.
     */
    static const unsigned short DEFAULT_PORT_NUMBER;

    /**
     * \brief Default number of robot joints.
     */
    static const int DEFAULT_NUMBER_OF_ROBOT_JOINTS;

    /**
     * \brief Default number of external joints.
     */
    static const int DEFAULT_NUMBER_OF_EXTERNAL_JOINTS;

    /**
     * \brief Maximum number of joints.
     */
    static const int MAX_NUMBER_OF_JOINTS;
  };

  /**
   * \brief Constants related to the conversions between units.
   */
  struct ABB_LIBEGM_EXPORT Conversion
  {
    /**
     * \brief Conversion value from radians to degrees.
     */
    static const double RAD_2_DEG;

    /**
     * \brief Conversion value from degrees to radians.
     */
    static const double DEG_2_RAD;

    /**
     * \brief Conversion value from millimeter to meter.
     */
    static const double MM_TO_M;

    /**
     * \brief Conversion value from milliseconds to seconds.
     */
    static const double MS_TO_S;

    /**
     * \brief Conversion value from seconds to microseconds.
     */
    static const double S_TO_US;
  };
};

/**
 * \brief Struct for an EGM user interface's base configuration.
 */
struct BaseConfiguration
{
  /**
   * \brief Default constructor.
   */
  BaseConfiguration()
  :
  axes(Six),
  use_demo_outputs(false),
  use_velocity_outputs(false),
  use_logging(false),
  max_logging_duration(60.0)
  {}

  /**
   * \brief Value specifying if a six or seven axes robot is used.
   *
   * Note: If set to a seven axes robot, then an implicit mapping of joint values is performed.
   */
  RobotAxes axes;

  /**
   * \brief Flag indicating if demo outputs should be used.
   *
   * Note: Overrides any other execution mode. Mainly used to verify that the EGM communication channel
   *       works as intended.
   */
  bool use_demo_outputs;

  /**
   * \brief Flag indicating if the messages, sent to the robot controller, should include velocity outputs.
   *
   * Note: If set to false, then no velocity values are sent (they are optional).
   */
  bool use_velocity_outputs;

  /**
   * \brief Flag indicating if the interface should log data.
   */
  bool use_logging;

  /**
   * \brief Maximum duration [s] to log data.
   */
  double max_logging_duration;
};

/**
 * \brief Struct for the EGM trajectory user interface's configuration.
 */
struct TrajectoryConfiguration
{
  /**
   * \brief Enum for the available spline polynomial interpolation methods.
   *
   * Note: Cartesian orientation uses Slerp interpolation instead.
   *
   * Boundary conditions (between trajectory points):
   * |
   * |--Linear:
   * |  |-- Start and goal positions.
   * |
   * |--Square:
   * |  |-- Start and goal positions.
   * |  |-- Start velocity.
   * |
   * |--Cubic:
   * |  |-- Start and goal positions.
   * |  |-- Start and goal velocities.
   * |
   * |--Quintic:
   *    |-- Start and goal positions.
   *    |-- Start and goal velocities.
   *    |-- Start and goal accelerations.
   */
  enum SplineMethod
  {
    Linear, ///< \brief Use a first degree polynomial.
    Square, ///< \brief Use a second degree polynomial.
    Cubic,  ///< \brief Use a third degree polynomial
    Quintic ///< \brief Use a fifth degree polynomial.
  };

  /**
   * \brief A constructor.
   *
   * \param base_configuration specifying the base configurations.
   */
  TrajectoryConfiguration(const BaseConfiguration& base_configuration = BaseConfiguration())
  :
  base(base_configuration),
  spline_method(Quintic)
  {}

  /**
   * \brief The base configurations.
   */
  BaseConfiguration base;

  /**
   * \brief Value specifying which spline method to use in the interpolation.
   */
  SplineMethod spline_method;
};

} // end namespace egm
} // end namespace abb

#endif // EGM_COMMON_H
