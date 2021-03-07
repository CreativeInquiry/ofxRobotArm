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

#ifndef RWS_STATE_MACHINE_INTERFACE_H
#define RWS_STATE_MACHINE_INTERFACE_H

#include "rws_interface.h"

namespace abb
{
namespace rws
{
/**
 * \brief User friendly interface to ABB robot controller systems, which are based on a corresponding RobotWare AddIn.
 *
 * Note: This class assumes that the StateMachine AddIn was used during system installation. The AddIn loads a RAPID
 *       state machine implementation, into each motion task in the system. Each state machine instance provide several
 *       services, which consists of system configurations and RAPID modules.
 *
 *       The exact configurations and RAPID modules loaded depends on the specifications of the robot controller system:
 *       - If single or multiple robot(s).
 *       - If the RobotWare option Externally Guided Motion (EGM) is present.
 *       - If the SmartGripper (SG) product is present.
 */
class RWSStateMachineInterface : public RWSInterface
{
private:
  /**
   * \brief Declaration of a class representing the services provided by the StateMachine AddIn.
   */
  class Services;

public:
  /**
   * \brief Representation of the StateMachine AddIn's different states.
   */
  enum States
  {
    STATE_IDLE,              ///< \brief Idle state.
    STATE_INITIALIZE,        ///< \brief Initialization state.
    STATE_RUN_RAPID_ROUTINE, ///< \brief Running RAPID routine state.
    STATE_RUN_EGM_ROUTINE,   ///< \brief Running EGM routine state.
    STATE_UNKNOWN            ///< \brief Unknown state.
  };

  /**
   * \brief Representation of the StateMachine AddIn's supported EGM actions.
   *
   * Note: Requires that the EGM option exists in the controller system.
   */
  enum EGMActions
  {
    EGM_ACTION_STOP,      ///< \brief Stop action.
    EGM_ACTION_RUN_JOINT, ///< \brief Joint action.
    EGM_ACTION_RUN_POSE,  ///< \brief Pose action.
    EGM_ACTION_UNKNOWN    ///< \brief Unknown action.
  };

  /**
   * \brief Representation of the StateMachine AddIn's supported SmartGripper commands.
   *
   * Note: Requires that the SmartGripper product exists in the controller system.
   */
  enum SGCommands
  {
    SG_COMMAND_NONE,         ///< \brief No command.
    SG_COMMAND_INITIALIZE,   ///< \brief Initialize command.
    SG_COMMAND_CALIBRATE,    ///< \brief Calibrate command.
    SG_COMMAND_MOVE_TO,      ///< \brief Move command.
    SG_COMMAND_GRIP_IN,      ///< \brief Inwards grip command.
    SG_COMMAND_GRIP_OUT,     ///< \brief Outwards grip command.
    SG_COMMAND_BLOW_ON_1,    ///< \brief Blow on 1 command.
    SG_COMMAND_BLOW_ON_2,    ///< \brief Blow on 2 command.
    SG_COMMAND_BLOW_OFF_1,   ///< \brief Blow off 1 command.
    SG_COMMAND_BLOW_OFF_2,   ///< \brief Blow off 2 command.
    SG_COMMAND_VACUUM_ON_1,  ///< \brief Vacuum on 1 command.
    SG_COMMAND_VACUUM_ON_2,  ///< \brief Vacuum on 2 command.
    SG_COMMAND_VACUUM_OFF_1, ///< \brief Vacuum off 1 command.
    SG_COMMAND_VACUUM_OFF_2  ///< \brief Vacuum off 2 command.
  };

  /**
   * \brief Resource identifiers for resources loaded by the StateMachine AddIn.
   */
  struct ResourceIdentifiers
  {
    /**
     * \brief IO signal resources.
     */
    struct ABB_LIBRWS_EXPORT IOSignals
    {
      /**
       * \brief IO signal for requesting start of EGM joint motions (i.e. EGM joint mode).
       *
       * Note: Requires that the EGM option exists in the controller system.
       */
      static const std::string EGM_START_JOINT;

      /**
       * \brief IO signal for requesting start of EGM pose motions (i.e. EGM pose mode).
       *
       * Note: Requires that the EGM option exists in the controller system.
       */
      static const std::string EGM_START_POSE;

      /**
       * \brief IO signal for requesting start of EGM position streaming (i.e. only feedback).
       *
       * Note: Requires that the EGM option exists in the controller system.
       */
      static const std::string EGM_START_STREAM;

      /**
       * \brief IO signal for requesting stop of EGM motions.
       *
       * Note: Requires that the EGM option exists in the controller system.
       */
      static const std::string EGM_STOP;

      /**
       * \brief IO signal for requesting stop of EGM position streaming.
       *
       * Note: Requires that the EGM option exists in the controller system.
       */
      static const std::string EGM_STOP_STREAM;

      /**
       * \brief Prefix for IO signals, used for checking if a mechanical unit is stationary or not.
       */
      static const std::string OUTPUT_STATIONARY;

      /**
       * \brief IO signal for requesting execution of RAPID routines.
       */
      static const std::string RUN_RAPID_ROUTINE;

      /**
       * \brief IO signal for requesting execution of SmartGripper routines.
       *
       * Note: Requires that the SmartGripper product exists in the controller system.
       */
      static const std::string RUN_SG_ROUTINE;

      /**
       * \brief IO signal for updating the status signal for an external system.
       *
       * Note: Only used if the StateMachine AddIn's watchdog has been activated.
       */
      static const std::string WD_EXTERNAL_STATUS;

      /**
       * \brief IO signal for requesting a system stop.
       *
       * Note: Only used if the StateMachine AddIn's watchdog has been activated.
       *
       * Important: Not equivalent to an emergency stop.
       */
      static const std::string WD_STOP_REQUEST;
    };

    /**
     * \brief RAPID resources.
     */
    struct RAPID
    {
      /**
       * \brief RAPID modules.
       */
      struct ABB_LIBRWS_EXPORT Modules
      {
        /**
         * \brief RAPID module providing functionality for using EGM.
         *
         * Note: Requires that the EGM option exists in the controller system.
         */
        static const std::string T_ROB_EGM;

        /**
         * \brief RAPID module managing the StateMachine AddIn's RAPID execution.
         */
        static const std::string T_ROB_MAIN;

        /**
         * \brief RAPID module providing functionality for executing RAPID routines.
         */
        static const std::string T_ROB_RAPID;

        /**
         * \brief RAPID module providing functionality for using a SmartGripper.
         *
         * Note: Requires that the SmartGripper product exists in the controller system.
         */
        static const std::string T_ROB_SG;

        /**
         * \brief RAPID module providing utility functionality.
         */
        static const std::string T_ROB_UTILITY;

        /**
         * \brief RAPID module providing functionality for a watchdog.
         */
        static const std::string T_ROB_WATCHDOG;
      };

      /**
       * \brief Predefiend RAPID procedures.
       */
      struct ABB_LIBRWS_EXPORT Procedures
      {
        /**
         * \brief Predefined RAPID procedure "runCallByVar".
         */
        static const std::string RUN_CALL_BY_VAR;

        /**
         * \brief Predefined RAPID procedure "runModuleLoad".
         */
        static const std::string RUN_MODULE_LOAD;

        /**
         * \brief Predefined RAPID procedure "runModuleUnload".
         */
        static const std::string RUN_MODULE_UNLOAD;

        /**
         * \brief Predefined RAPID procedure "runMoveAbsJ".
         */
        static const std::string RUN_MOVE_ABS_J;

        /**
         * \brief Predefined RAPID procedure "runMoveJ".
         */
        static const std::string RUN_MOVE_J;

        /**
         * \brief Predefined RAPID procedure "runMoveToCalibrationPosition".
         */
        static const std::string RUN_MOVE_TO_CALIBRATION_POSITION;
      };

      /**
       * \brief RAPID symbols.
       */
      struct ABB_LIBRWS_EXPORT Symbols
      {
        /**
         * \brief RAPID symbol indicating the current EGM action of a RAPID motion task.
         *
         * Note: Requires that the EGM option exists in the controller system.
         */
        static const RWSClient::RAPIDSymbolResource EGM_CURRENT_ACTION;

        /**
         * \brief RAPID symbol containing settings to different EGM RAPID instructions.
         *
         * Note: Requires that the EGM option exists in the controller system.
         */
        static const RWSClient::RAPIDSymbolResource EGM_SETTINGS;

        /**
         * \brief RAPID symbol indicating the current state of a RAPID motion task.
         */
        static const RWSClient::RAPIDSymbolResource MAIN_CURRENT_STATE;

        /**
         * \brief RAPID symbol containing name input to the predefined routine "runCallByVar".
         */
        static const RWSClient::RAPIDSymbolResource RAPID_CALL_BY_VAR_NAME_INPUT;

        /**
         * \brief RAPID symbol containing number input to the predefined routine "runCallByVar".
         */
        static const RWSClient::RAPIDSymbolResource RAPID_CALL_BY_VAR_NUM_INPUT;

        /**
         * \brief RAPID symbol containing module file path input to the predefined routines
         *        "runModuleLoad" and "runModuleUnload".
         */
        static const RWSClient::RAPIDSymbolResource RAPID_MODULE_FILE_PATH_INPUT;

        /**
         * \brief RAPID symbol containing jointtarget input to the predefined routine "runMoveAbsJ".
         */
        static const RWSClient::RAPIDSymbolResource RAPID_MOVE_JOINT_TARGET_INPUT;

        /**
         * \brief RAPID symbol containing robtarget input to the predefined routine "runMoveJ".
         */
        static const RWSClient::RAPIDSymbolResource RAPID_MOVE_ROB_TARGET_INPUT;

        /**
         * \brief RAPID symbol containing speed input to the predefined routines "runMoveAbsJ" and "runMoveJ".
         */
        static const RWSClient::RAPIDSymbolResource RAPID_MOVE_SPEED_INPUT;

        /**
         * \brief RAPID symbol containing routine name input specifying a RAPID routine to run.
         */
        static const RWSClient::RAPIDSymbolResource RAPID_ROUTINE_NAME_INPUT;

        /**
         * \brief RAPID symbol containing command input for specifying the desired gripper command.
         *
         * Note: Requires that the SmartGripper product exists in the controller system.
         */
        static const RWSClient::RAPIDSymbolResource SG_COMMAND_INPUT;

        /**
         * \brief RAPID symbol containing settings to different SmartGripper RAPID instructions.
         *
         * Note: Requires that the SmartGripper product exists in the controller system.
         */
        static const RWSClient::RAPIDSymbolResource SG_SETTINGS;

        /**
         * \brief RAPID symbol containing position input for specifying the position a SmartGripper should move to.
         *
         * Note: Requires that the SmartGripper product exists in the controller system.
         */
        static const RWSClient::RAPIDSymbolResource SG_TARGET_POSTION_INPUT;

        /**
         * \brief RAPID symbol containing base frame extracted during initialization, for a RAPID motion task.
         */
        static const RWSClient::RAPIDSymbolResource UTILITY_BASE_FRAME;

        /**
         * \brief RAPID symbol containing calibration target extracted during initialization, for a RAPID motion task.
         */
        static const RWSClient::RAPIDSymbolResource UTILITY_CALIBRATION_TARGET;

        /**
         * \brief RAPID symbol indicating if a watchdog is active or not.
         */
        static const RWSClient::RAPIDSymbolResource WATCHDOG_ACTIVE;

        /**
         * \brief RAPID symbol indicating if a watchdog should check an external status signal or not.
         */
        static const RWSClient::RAPIDSymbolResource WATCHDOG_CHECK_EXTERNAL_STATUS;
      };
    };
  };

  /**
   * \brief Representation of a custom RAPID record, for settings to EGMSetupUC RAPID instructions.
   */
  struct EGMSetupUCSettings : public RAPIDRecord
  {
    EGMSetupUCSettings()
    :
    RAPIDRecord("EGMSetupUCSettings")
    {
      components_.push_back(&use_filtering);
      components_.push_back(&comm_timeout);
    }

    /**
     * \brief Flag indicating if the EGM controller should apply extra filtering on the EGM corrections.
     *
     * Note: If true: Applies extra filtering on the corrections, but also introduces some extra delays and latency.
     *       Else: Raw corrections will be used.
     */
    RAPIDBool use_filtering;

    /**
     * \brief Communication timeout [s].
     */
    RAPIDNum comm_timeout;
  };

  /**
   * \brief Representation of a custom RAPID record, for settings to EGMAct RAPID instructions.
   */
  struct EGMActivateSettings : public RAPIDRecord
  {
  public:
    /**
     * \brief A default constructor.
     */
    EGMActivateSettings()
    :
    RAPIDRecord("EGMActivateSettings")
    {
      components_.push_back(&tool);
      components_.push_back(&wobj);
      components_.push_back(&correction_frame);
      components_.push_back(&sensor_frame);
      components_.push_back(&cond_min_max);
      components_.push_back(&lp_filter);
      components_.push_back(&sample_rate);
      components_.push_back(&max_speed_deviation);
    };

    /**
     * \brief Copy constructor.
     *
     * \param other containing the values to copy.
     */
    EGMActivateSettings(const EGMActivateSettings& other)
    :
    RAPIDRecord(other.record_type_name_)
    {
      if (this != &other)
      {
        tool = other.tool;
        wobj = other.wobj;
        correction_frame = other.correction_frame;
        sensor_frame = other.sensor_frame;
        cond_min_max = other.cond_min_max;
        lp_filter = other.lp_filter;
        sample_rate = other.sample_rate;
        max_speed_deviation = other.max_speed_deviation;
        components_.clear();
        components_.push_back(&tool);
        components_.push_back(&wobj);
        components_.push_back(&correction_frame);
        components_.push_back(&sensor_frame);
        components_.push_back(&cond_min_max);
        components_.push_back(&lp_filter);
        components_.push_back(&sample_rate);
        components_.push_back(&max_speed_deviation);
      }
    }

    /**
     * \brief The tool to use.
     */
    ToolData tool;

    /**
     * \brief The work object to use.
     */
    WObjData wobj;

    /**
     * \brief Specifies the correction frame.
     *
     * Note: Only used in EGM pose mode.
     */
    Pose correction_frame;

    /**
     * \brief Specifies the sensor frame.
     *
     * Note: Only used in EGM pose mode.
     */
    Pose sensor_frame;

    /**
     * \brief Condition value [deg or mm] for when the EGM correction is considered to be finished.
     *
     * E.g.: for joint mode, then the condition is fulfilled when the joints are within [-cond_min_max, cond_min_max].
     */
    RAPIDNum cond_min_max;

    /**
     * \brief Low pass filer bandwidth of the EGM controller [Hz].
     */
    RAPIDNum lp_filter;

    /**
     * \brief Sample rate for the EGM communication [ms].
     *
     * Note: Only multiples of 4 are allowed (i.e. 4, 8, 16, etc...).
     */
    RAPIDNum sample_rate;

    /**
     * \brief Maximum admitted joint speed change [deg/s]:
     *
     * Note: Take care if setting this higher than the lowest max speed [deg/s],
     *       out of all the axis max speeds (found in the robot's data sheet).
     */
    RAPIDNum max_speed_deviation;
  };

  /**
   * \brief Representation of a custom RAPID record, for settings to EGMRun RAPID instructions.
   */
  struct EGMRunSettings : public RAPIDRecord
  {
    /**
     * \brief A default constructor.
     */
    EGMRunSettings()
    :
    RAPIDRecord("EGMRunSettings")
    {
      components_.push_back(&cond_time);
      components_.push_back(&ramp_in_time);
      components_.push_back(&offset);
      components_.push_back(&pos_corr_gain);
    }

    /**
     * \brief Copy constructor.
     *
     * \param other containing the values to copy.
     */
    EGMRunSettings(const EGMRunSettings& other)
    :
    RAPIDRecord(other.record_type_name_)
    {
      if (this != &other)
      {
        cond_time = other.cond_time;
        ramp_in_time = other.ramp_in_time;
        offset = other.offset;
        pos_corr_gain = other.pos_corr_gain;
        components_.clear();
        components_.push_back(&cond_time);
        components_.push_back(&ramp_in_time);
        components_.push_back(&offset);
        components_.push_back(&pos_corr_gain);
      }
    }

    /**
     * \brief Condition time [s].
     */
    RAPIDNum cond_time;

    /**
     * \brief Ramp in time [s].
     */
    RAPIDNum ramp_in_time;

    /**
     * \brief A static offset applied on top of the references supplied by the external system.
     *
     * Note: Only used in EGM pose mode.
     */
    Pose offset;

    /**
     * \brief Position correction gain of the EGM controller.
     */
    RAPIDNum pos_corr_gain;
  };

  /**
   * \brief Representation of a custom RAPID record, for settings to EGMStop RAPID instructions.
   */
  struct EGMStopSettings : public RAPIDRecord
  {
    /**
     * \brief A default constructor.
     */
    EGMStopSettings()
    :
    RAPIDRecord("EGMStopSettings")
    {
      components_.push_back(&ramp_out_time);
    }

    /**
     * \brief Desired duration for ramping out EGM motions [s].
     */
    RAPIDNum ramp_out_time;
  };

  /**
   * \brief Representation of a custom RAPID record, for settings to EGM RAPID instructions.
   */
  struct EGMSettings : public RAPIDRecord
  {
    /**
     * \brief A default constructor.
     */
    EGMSettings()
    :
    RAPIDRecord("EGMSettings")
    {
      components_.push_back(&allow_egm_motions);
      components_.push_back(&use_presync);
      components_.push_back(&setup_uc);
      components_.push_back(&activate);
      components_.push_back(&run);
      components_.push_back(&stop);
    }

    /**
     * \brief Copy constructor.
     *
     * \param other containing the values to copy.
     */
    EGMSettings(const EGMSettings& other)
    :
    RAPIDRecord(other.record_type_name_)
    {
      if (this != &other)
      {
        allow_egm_motions = other.allow_egm_motions;
        use_presync = other.use_presync;
        setup_uc = other.setup_uc;
        activate = other.activate;
        run = other.run;
        stop = other.stop;
        components_.clear();
        components_.push_back(&allow_egm_motions);
        components_.push_back(&use_presync);
        components_.push_back(&setup_uc);
        components_.push_back(&activate);
        components_.push_back(&run);
        components_.push_back(&stop);
      }
    }

    /**
     * \brief Flag indicating if EGM motions are allowed to start.
     */
    RAPIDBool allow_egm_motions;

    /**
     * \brief Flag indicating if the motion tasks should be synchronized before starting EGM.
     *
     * Note: Only used in multi robot systems.
     */
    RAPIDBool use_presync;

    /**
     * \brief Settings for EGMSetupUC instructions.
     */
    EGMSetupUCSettings setup_uc;

    /**
     * \brief Settings for EGMAct instructions.
     */
    EGMActivateSettings activate;

    /**
     * \brief Settings for EGMRun instructions.
     */
    EGMRunSettings run;

    /**
     * \brief Settings for EGMStop instructions.
     */
    EGMStopSettings stop;
  };

  /**
   * \brief Representation of a custom RAPID record, for settings to SmartGripper RAPID instructions.
   */
  struct SGSettings : public RAPIDRecord
  {
    /**
     * \brief A default constructor.
     */
    SGSettings()
    :
    RAPIDRecord("SGSettings")
    {
      components_.push_back(&max_speed);
      components_.push_back(&hold_force);
      components_.push_back(&physical_limit);
    }

    /**
     * \brief Allowed maximum speed [mm/s] for the gripper.
     */
    RAPIDNum max_speed;

    /**
     * \brief Expected force [N] used for gripping.
     */
    RAPIDNum hold_force;

    /**
     * \brief The physical limit [mm] (if the gripper should operate in a smaller travel range).
     */
    RAPIDNum physical_limit;
  };

  /**
   * \brief A constructor.
   *
   * \param ip_address specifying the robot controller's IP address.
   */
  RWSStateMachineInterface(const std::string& ip_address)
  :
  RWSInterface(ip_address,
               SystemConstants::General::DEFAULT_PORT_NUMBER,
               SystemConstants::General::DEFAULT_USERNAME,
               SystemConstants::General::DEFAULT_PASSWORD),
  services_(this)
  {}

  /**
   * \brief A constructor.
   *
   * \param ip_address specifying the robot controller's IP address.
   * \param username for the username to the RWS authentication process.
   * \param password for the password to the RWS authentication process.
   */
  RWSStateMachineInterface(const std::string& ip_address, const std::string& username, const std::string& password)
  :
  RWSInterface(ip_address,
               SystemConstants::General::DEFAULT_PORT_NUMBER,
               username,
               password),
  services_(this)
  {}

  /**
   * \brief A constructor.
   *
   * \param ip_address specifying the robot controller's IP address.
   * \param port for the port used by the RWS server.
   */
  RWSStateMachineInterface(const std::string& ip_address, const unsigned short port)
  :
  RWSInterface(ip_address,
               port,
               SystemConstants::General::DEFAULT_USERNAME,
               SystemConstants::General::DEFAULT_PASSWORD),
  services_(this)
  {}

  /**
   * \brief A constructor.
   *
   * \param ip_address specifying the robot controller's IP address.
   * \param port for the port used by the RWS server.
   * \param username for the username to the RWS authentication process.
   * \param password for the password to the RWS authentication process.
   */
  RWSStateMachineInterface(const std::string& ip_address,
                           const unsigned short port,
                           const std::string& username,
                           const std::string& password)
  :
  RWSInterface(ip_address,
               port,
               username,
               password),
  services_(this)
  {}

  /**
   * \brief Services provided by the StateMachine AddIn.
   *
   * Note: I.e. a representation of RAPID modules, and related configurations, loaded by the AddIn.
   */
  const Services& services() const { return services_; }

private:
  /**
   * \brief Representation of the services provided by the StateMachine AddIn.
   *
   * Note: I.e. a representation of RAPID modules, and related configurations, loaded by the AddIn.
   */
  class Services
  {
  private:
    /**
     * \brief Declaration of a class representing the EGM services provided by the StateMachine AddIn.
     */
    class EGM;

    /**
     * \brief Declaration of a class representing the main services provided by the StateMachine AddIn.
     */
    class Main;

    /**
     * \brief Declaration of a class representing the RAPID services provided by the StateMachine AddIn.
     */
    class RAPID;

    /**
     * \brief Declaration of a class representing the SmartGripper services provided by the StateMachine AddIn.
     */
    class SG;

    /**
     * \brief Declaration of a class representing the utility services provided by the StateMachine AddIn.
     */
    class Utility;

    /**
     * \brief Declaration of a class representing the watchdog services provided by the StateMachine AddIn.
     */
    class Watchdog;

  public:
    /**
     * \brief A constructor.
     *
     * \param p_rws_interface for the RWS interface instance.
     */
    Services(RWSStateMachineInterface* p_rws_interface)
    :
    egm_(p_rws_interface),
    main_(p_rws_interface),
    rapid_(p_rws_interface),
    sg_(p_rws_interface),
    utility_(p_rws_interface),
    watchdog_(p_rws_interface)
    {}

    /**
     * \brief EGM services provided by the StateMachine AddIn.
     */
    const EGM& egm() const { return egm_; }

    /**
     * \brief Main services provided by the StateMachine AddIn.
     */
    const Main& main() const { return main_; }

    /**
     * \brief RAPID services provided by the StateMachine AddIn.
     */
    const RAPID& rapid() const { return rapid_; }

    /**
     * \brief SmartGripper services provided by the StateMachine AddIn.
     */
    const SG& sg() const { return sg_; }

    /**
     * \brief Utility services provided by the StateMachine AddIn.
     */
    const Utility& utility() const { return utility_; }

    /**
     * \brief Watchdog services provided by the StateMachine AddIn.
     */
    const Watchdog& watchdog() const { return watchdog_; }

  private:
    /**
     * \brief Representation of the EGM services provided by the StateMachine AddIn.
     *
     * Note: Requires that the EGM option exists in the controller system.
     */
    class EGM
    {
    public:
      /**
       * \brief A constructor.
       *
       * \param p_rws_interface for the RWS interface instance.
       */
      EGM(RWSStateMachineInterface* p_rws_interface) : p_rws_interface_(p_rws_interface) {}

      /**
       * \brief Get the current EGM action.
       *
       * \param task specifying the RAPID task.
       *
       * \return EGMActions indicating the current EGM action.
       */
      EGMActions getCurrentAction(const std::string& task) const;

      /**
       * \brief Get the settings for the EGM RAPID instructions.
       *
       * \param task specifying the RAPID task.
       * \param p_settings for storing the retrieved data.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool getSettings(const std::string& task, EGMSettings* p_settings) const;

      /**
       * \brief Set the settings for the EGM RAPID instructions.
       *
       * \param task specifying the RAPID task.
       * \param settings containing the new data.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool setSettings(const std::string& task, const EGMSettings& settings) const;

      /**
       * \brief Signal the StateMachine AddIn to start EGM joint motions.
       *
       * \return bool indicating if the signaling was successful or not.
       */
      bool signalEGMStartJoint() const;

      /**
       * \brief Signal the StateMachine AddIn to start EGM pose motions.
       *
       * \return bool indicating if the signaling was successful or not.
       */
      bool signalEGMStartPose() const;

      /**
       * \brief Signal the StateMachine AddIn to start EGM position streaming.
       *
       * \return bool indicating if the signaling was successful or not.
       */
      bool signalEGMStartStream() const;

      /**
       * \brief Signal the StateMachine AddIn to stop any current EGM motions.
       *
       * \return bool indicating if the signaling was successful or not.
       */
      bool signalEGMStop() const;

      /**
       * \brief Signal the StateMachine AddIn to stop any current position streaming.
       *
       * \return bool indicating if the signaling was successful or not.
       */
      bool signalEGMStopStream() const;

    private:
      /**
       * \brief The RWS interface instance.
       */
      RWSStateMachineInterface* p_rws_interface_;
    };

    /**
     * \brief Representation of the main services provided by the StateMachine AddIn.
     */
    class Main
    {
    public:
      /**
       * \brief A constructor.
       *
       * \param p_rws_interface for the RWS interface instance.
       */
      Main(RWSStateMachineInterface* p_rws_interface) : p_rws_interface_(p_rws_interface) {}

      /**
       * \brief Get the current state.
       *
       * \param task specifying the RAPID task.
       *
       * \return States indicating the current state of the StateMachine.
       */
      States getCurrentState(const std::string& task) const;

      /**
       * \brief Checks if a motion task is in the idle state or not.
       *
       * \param task specifying the RAPID task.
       *
       * \return TriBool indicating if the state is idle or not.
       */
      TriBool isStateIdle(const std::string& task) const;

      /**
       * \brief Checks if a mechanical unit is stationary or not.
       *
       * \param mechanical_unit specifying the mechanical unit to check.
       *
       * \return TriBool indicating if the mechanical unit is stationary or not.
       */
      TriBool isStationary(const std::string& mechanical_unit) const;

    private:
      /**
       * \brief The RWS interface instance.
       */
      RWSStateMachineInterface* p_rws_interface_;
    };

    /**
     * \brief Representation of the RAPID services provided by the StateMachine AddIn.
     */
    class RAPID
    {
    public:
      /**
       * \brief A constructor.
       *
       * \param p_rws_interface for the RWS interface instance.
       */
      RAPID(RWSStateMachineInterface* p_rws_interface) : p_rws_interface_(p_rws_interface) {}

      /**
       * \brief Request the execution of the predefined RAPID procedure "runCallByVar".
       *
       * \param task specifying the RAPID task.
       * \param routine_name specifying routine name.
       * \param routine_number specifying routine number.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool runCallByVar(const std::string& task,
                        const std::string& routine_name,
                        const unsigned int routine_number) const;

      /**
       * \brief Request the execution of the predefined RAPID procedure "runModuleLoad".
       *
       * \param task specifying the RAPID task.
       * \param file_path specifying file path to the module.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool runModuleLoad(const std::string& task, const std::string& file_path) const;

      /**
       * \brief Request the execution of the predefined RAPID procedure "runModuleUnload".
       *
       * \param task specifying the RAPID task.
       * \param file_path specifying file path to the module.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool runModuleUnload(const std::string& task, const std::string& file_path) const;

      /**
       * \brief Request the execution of the predefined RAPID procedure "runMoveAbsJ".
       *
       * \param task specifying the RAPID task.
       * \param joint_target specifying jointtarget goal.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool runMoveAbsJ(const std::string& task, const JointTarget& joint_target) const;

      /**
       * \brief Request the execution of the predefined RAPID procedure "runMoveJ".
       *
       * \param task specifying the RAPID task.
       * \param rob_target specifying the robtarget goal.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool runMoveJ(const std::string& task, const RobTarget& rob_target) const;

      /**
       * \brief Request the execution of the predefined RAPID procedure "runMoveToCalibrationPosition".
       *
       * \param task specifying the RAPID task.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool runMoveToCalibrationPosition(const std::string& task) const;

      /**
       * \brief Set the move speed for the predefined RAPID procedures "runMoveAbsJ" and "runMoveJ".
       *
       * \param task specifying the RAPID task.
       * \param speed_data containing the new data.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool setMoveSpeed(const std::string& task, const SpeedData& speed_data) const;

      /**
       * \brief Set the routine name specifying which routine to run.
       *
       * \param task specifying the RAPID task.
       * \param routine_name containing the new data.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool setRoutineName(const std::string& task, const std::string& routine_name) const;

      /**
       * \brief Signal the StateMachine AddIn to run RAPID routine(s).
       *
       * \return bool indicating if the signaling was successful or not.
       */
      bool signalRunRAPIDRoutine() const;

    private:
      /**
       * \brief The RWS interface instance.
       */
      RWSStateMachineInterface* p_rws_interface_;
    };

    /**
     * \brief Representation of the SmartGripper services provided by the StateMachine AddIn.
     *
     * Note: Requires that the SmartGripper product exists in the controller system.
     */
    class SG
    {
    public:
      /**
       * \brief A constructor.
       *
       * \param p_rws_interface for the RWS interface instance.
       */
      SG(RWSStateMachineInterface* p_rws_interface) : p_rws_interface_(p_rws_interface) {}

      /**
       * \brief Request turning off both SmartGrippers' first blow.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool dualBlow1Off() const;

      /**
       * \brief Request turning on both SmartGrippers' first blow.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool dualBlow1On() const;

      /**
       * \brief Request turning off both SmartGrippers' second blow.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool dualBlow2Off() const;

      /**
       * \brief Request turning on both SmartGrippers' second blow.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool dualBlow2On() const;

      /**
       * \brief Request calibration of both SmartGrippers.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool dualCalibrate() const;

      /**
       * \brief Get the settings for both SmartGrippers' RAPID instructions.
       *
       * \param p_left_settings for storing the retrieved data for the left gripper.
       * \param p_right_settings for storing the retrieved data for the right gripper.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool dualGetSettings(SGSettings* p_left_settings, SGSettings* p_right_settings) const;

      /**
       * \brief Request inwards grip of both SmartGrippers.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool dualGripIn() const;

      /**
       * \brief Request outwards grip of both SmartGrippers.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool dualGripOut() const;

      /**
       * \brief Request initialization of both SmartGrippers.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool dualInitialize() const;

      /**
       * \brief Request move of both SmartGrippers.
       *
       * \param left_position specifying targeted position [mm] for the left gripper.
       * \param right_position specifying targeted position [mm] for the right gripper.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool dualMoveTo(const float left_position, const float right_position) const;

      /**
       * \brief Set the settings for both SmartGrippers' instructions.
       *
       * \param left_settings containing the new data for the left gripper.
       * \param right_settings containing the new data for the right gripper.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool dualSetSettings(const SGSettings& left_settings, const SGSettings& right_settings) const;

      /**
       * \brief Request turning off both SmartGrippers' first vacuum.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool dualVacuum1Off() const;

      /**
       * \brief Request turning on both SmartGrippers' first vacuum.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool dualVacuum1On() const;

      /**
       * \brief Request turning off both SmartGrippers' second vacuum.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool dualVacuum2Off() const;

      /**
       * \brief Request turning on both SmartGrippers' second vacuum.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool dualVacuum2On() const;

      /**
       * \brief Request turning off the left SmartGripper's first blow.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool leftBlow1Off() const;

      /**
       * \brief Request turning on the left SmartGripper's first blow.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool leftBlow1On() const;

      /**
       * \brief Request turning off the left SmartGripper's second blow.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool leftBlow2Off() const;

      /**
       * \brief Request turning on the left SmartGripper's second blow.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool leftBlow2On() const;

      /**
       * \brief Request calibration of the left SmartGripper.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool leftCalibrate() const;

      /**
       * \brief Get the settings for the left SmartGripper's RAPID instructions.
       *
       * \param p_settings for storing the retrieved data for the left gripper.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool leftGetSettings(SGSettings* p_settings) const;

      /**
       * \brief Request inwards grip of the left SmartGripper.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool leftGripIn() const;

      /**
       * \brief Request outwards grip of the left SmartGripper.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool leftGripOut() const;

      /**
       * \brief Request initialization of the left SmartGripper.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool leftInitialize() const;

      /**
       * \brief Request move of the left SmartGripper.
       *
       * \param position specifying targeted position [mm] for the left gripper.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool leftMoveTo(const float position) const;

      /**
       * \brief Set the settings for the left SmartGripper's instructions.
       *
       * \param settings containing the new data for the left gripper.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool leftSetSettings(const SGSettings& settings) const;

      /**
       * \brief Request turning off the left SmartGripper's first vacuum.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool leftVacuum1Off() const;

      /**
       * \brief Request turning on the left SmartGripper's first vacuum.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool leftVacuum1On() const;

      /**
       * \brief Request turning off the left SmartGripper's second vacuum.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool leftVacuum2Off() const;

      /**
       * \brief Request turning on the left SmartGripper's second vacuum.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool leftVacuum2On() const;

      /**
       * \brief Request turning off the right SmartGripper's first blow.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool rightBlow1Off() const;

      /**
       * \brief Request turning on the right SmartGripper's first blow.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool rightBlow1On() const;

      /**
       * \brief Request turning off the right SmartGripper's second blow.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool rightBlow2Off() const;

      /**
       * \brief Request turning on the right SmartGripper's second blow.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool rightBlow2On() const;

      /**
       * \brief Request calibration of the right SmartGripper.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool rightCalibrate() const;

      /**
       * \brief Get the settings for the right SmartGripper's RAPID instructions.
       *
       * \param p_settings for storing the retrieved data for the right gripper.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool rightGetSettings(SGSettings* p_settings) const;

      /**
       * \brief Request inwards grip of the right SmartGripper.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool rightGripIn() const;

      /**
       * \brief Request outwards grip of the right SmartGripper.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool rightGripOut() const;

      /**
       * \brief Request initialization of the right SmartGripper.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool rightInitialize() const;

      /**
       * \brief Request move of the right SmartGripper.
       *
       * \param position specifying targeted position [mm] for the right gripper.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool rightMoveTo(const float position) const;

      /**
       * \brief Set the settings for the right SmartGripper's instructions.
       *
       * \param settings containing the new data for the right gripper.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool rightSetSettings(const SGSettings& settings) const;

      /**
       * \brief Request turning off the right SmartGripper's first vacuum.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool rightVacuum1Off() const;

      /**
       * \brief Request turning on the right SmartGripper's first vacuum.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool rightVacuum1On() const;

      /**
       * \brief Request turning off the right SmartGripper's second vacuum.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool rightVacuum2Off() const;

      /**
       * \brief Request turning on the right SmartGripper's second vacuum.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool rightVacuum2On() const;

      /**
       * \brief Signal the StateMachine to run SmartGripper routine(s).
       *
       * \return bool indicating if the signaling was successful or not.
       */
      bool signalRunSGRoutine() const;

    private:
      /**
       * \brief Get the settings for a SmartGripper's RAPID instructions.
       *
       * \param task specifying the RAPID task.
       * \param p_settings for storing the retrieved data.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool getSettings(const std::string& task, SGSettings* p_settings) const;

      /**
       * \brief Set command input for specifying a SmartGripper's desired command.
       *
       * \param task specifying the RAPID task.
       * \param command specifying the desired command.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool setCommandInput(const std::string& task, const SGCommands& command) const;

      /**
       * \brief Set the settings for a SmartGripper's RAPID instructions.
       *
       * \param task specifying the RAPID task.
       * \param settings specifying the settings.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool setSettings(const std::string& task, const SGSettings& settings) const;

      /**
       * \brief Set target position input for specifying where to move a SmartGripper.
       *
       * \param task specifying the RAPID task.
       * \param position specifying targeted position [mm] for a SmartGripper.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool setTargetPositionInput(const std::string& task, const float position) const;

      /**
       * \brief The RWS interface instance.
       */
      RWSStateMachineInterface* p_rws_interface_;
    };

    /**
     * \brief Representation of the utility services provided by the StateMachine AddIn.
     */
    class Utility
    {
    public:
      /**
       * \brief A constructor.
       *
       * \param p_rws_interface for the RWS interface instance.
       */
      Utility(RWSStateMachineInterface* p_rws_interface) : p_rws_interface_(p_rws_interface) {}

      /**
       * \brief Get a motion task's base frame, extracted during initialization of the task.
       *
       * \param task specifying the RAPID task.
       * \param p_base_frame for storing the retrieved data.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool getBaseFrame(const std::string& task, Pose* p_base_frame) const;

      /**
       * \brief Get a motion task's calibration target, extracted during initialization of the task.
       *
       * \param task specifying the RAPID task.
       * \param p_calibration_joint_target for storing the retrieved data.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool getCalibrationTarget(const std::string& task, JointTarget* p_calibration_joint_target) const;

    private:
      /**
       * \brief The RWS interface instance.
       */
      RWSStateMachineInterface* p_rws_interface_;
    };

    /**
     * \brief Representation of the watchdog services provided by the StateMachine AddIn.
     */
    class Watchdog
    {
    public:
      /**
       * \brief A constructor.
       *
       * \param p_rws_interface for the RWS interface instance.
       */
      Watchdog(RWSStateMachineInterface* p_rws_interface) : p_rws_interface_(p_rws_interface) {}

      /**
       * \brief Checks if the watchdog is active or not.
       *
       * \param task specifying the RAPID task.
       *
       * \return TriBool indicating if the watchdog is active or not.
       */
      TriBool isActive(const std::string& task) const;

      /**
       * \brief Checks if the watchdog is set to watch an external status signal or not.
       *
       * \param task specifying the RAPID task.
       *
       * \return TriBool indicating if the watchdog is set to watch an external status signal or not.
       */
      TriBool isCheckingExternalStatus(const std::string& task) const;

      /**
       * \brief Set the external status signal, which the watchdog can watch.
       *
       * \return bool indicating if the communication was successful or not.
       */
      bool setExternalStatusSignal() const;

      /**
       * \brief Signal the watchdog to stop the StateMachine.
       *
       * \return bool indicating if the signaling was successful or not.
       */
      bool signalStopRequest() const;

    private:
      /**
       * \brief The RWS interface instance.
       */
      RWSStateMachineInterface* p_rws_interface_;
    };

    /**
     * \brief EGM services provided by the StateMachine AddIn.
     */
    EGM egm_;

    /**
     * \brief Main services provided by the StateMachine AddIn.
     */
    Main main_;

    /**
     * \brief RAPID services provided by the StateMachine AddIn.
     */
    RAPID rapid_;

    /**
     * \brief SmartGripper services provided by the StateMachine AddIn.
     */
    SG sg_;

    /**
     * \brief Utility services provided by the StateMachine AddIn.
     */
    Utility utility_;

    /**
     * \brief Watchdog services provided by the StateMachine AddIn.
     */
    Watchdog watchdog_;
  };

  /**
   * \brief Toggles an IO signal.
   *
   * \param iosignal specifying the IO signal to toggle.
   *
   * \return bool indicating if the toggling was successful or not.
   */
  bool toggleIOSignal(const std::string& iosignal);

  /**
   * \brief Services provided by the StateMachine AddIn.
   */
  Services services_;
};

} // end namespace rws
} // end namespace abb

#endif
