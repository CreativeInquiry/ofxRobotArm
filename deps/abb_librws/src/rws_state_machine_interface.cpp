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

#include "abb_librws/rws_state_machine_interface.h"

namespace abb
{
namespace rws
{
/***********************************************************************************************************************
 * Struct definitions: RWSStateMachineInterface::ResourceIdentifiers
 */

typedef RWSClient::RAPIDSymbolResource                                   RAPIDSymbolResource;
typedef RWSStateMachineInterface::States                                 States;
typedef RWSStateMachineInterface::EGMActions                             EGMActions;
typedef RWSStateMachineInterface::ResourceIdentifiers::RAPID::Symbols    Symbols;
typedef RWSStateMachineInterface::ResourceIdentifiers::IOSignals         IOSignals;
typedef RWSStateMachineInterface::ResourceIdentifiers::RAPID::Modules    Modules;
typedef RWSStateMachineInterface::ResourceIdentifiers::RAPID::Procedures Procedures;
typedef RWSStateMachineInterface::ResourceIdentifiers::RAPID::Symbols    Symbols;

const std::string IOSignals::EGM_START_JOINT    = "EGM_START_JOINT";
const std::string IOSignals::EGM_START_POSE     = "EGM_START_POSE";
const std::string IOSignals::EGM_START_STREAM   = "EGM_START_STREAM";
const std::string IOSignals::EGM_STOP           = "EGM_STOP";
const std::string IOSignals::EGM_STOP_STREAM    = "EGM_STOP_STREAM";
const std::string IOSignals::OUTPUT_STATIONARY  = "OUTPUT_STATIONARY";
const std::string IOSignals::RUN_RAPID_ROUTINE  = "RUN_RAPID_ROUTINE";
const std::string IOSignals::RUN_SG_ROUTINE     = "RUN_SG_ROUTINE";
const std::string IOSignals::WD_EXTERNAL_STATUS = "WD_EXTERNAL_STATUS";
const std::string IOSignals::WD_STOP_REQUEST    = "WD_STOP_REQUEST";

const std::string Modules::T_ROB_EGM      = "TRobEGM";
const std::string Modules::T_ROB_MAIN     = "TRobMain";
const std::string Modules::T_ROB_RAPID    = "TRobRAPID";
const std::string Modules::T_ROB_SG       = "TRobSG";
const std::string Modules::T_ROB_UTILITY  = "TRobUtility";
const std::string Modules::T_ROB_WATCHDOG = "TRobWatchdog";

const std::string Procedures::RUN_CALL_BY_VAR                  = "runCallByVar";
const std::string Procedures::RUN_MODULE_LOAD                  = "runModuleLoad";
const std::string Procedures::RUN_MODULE_UNLOAD                = "runModuleUnload";
const std::string Procedures::RUN_MOVE_ABS_J                   = "runMoveAbsJ";
const std::string Procedures::RUN_MOVE_J                       = "runMoveJ";
const std::string Procedures::RUN_MOVE_TO_CALIBRATION_POSITION = "runMoveToCalibrationPosition";

const RAPIDSymbolResource Symbols::EGM_CURRENT_ACTION(Modules::T_ROB_EGM, "current_action");
const RAPIDSymbolResource Symbols::EGM_SETTINGS(Modules::T_ROB_EGM, "settings");
const RAPIDSymbolResource Symbols::MAIN_CURRENT_STATE(Modules::T_ROB_MAIN, "current_state");
const RAPIDSymbolResource Symbols::RAPID_CALL_BY_VAR_NAME_INPUT(Modules::T_ROB_RAPID, "callbyvar_name_input");
const RAPIDSymbolResource Symbols::RAPID_CALL_BY_VAR_NUM_INPUT(Modules::T_ROB_RAPID, "callbyvar_num_input");
const RAPIDSymbolResource Symbols::RAPID_MODULE_FILE_PATH_INPUT(Modules::T_ROB_RAPID, "module_file_path_input");
const RAPIDSymbolResource Symbols::RAPID_MOVE_JOINT_TARGET_INPUT(Modules::T_ROB_RAPID, "move_jointtarget_input");
const RAPIDSymbolResource Symbols::RAPID_MOVE_ROB_TARGET_INPUT(Modules::T_ROB_RAPID, "move_robtarget_input");
const RAPIDSymbolResource Symbols::RAPID_MOVE_SPEED_INPUT(Modules::T_ROB_RAPID, "move_speed_input");
const RAPIDSymbolResource Symbols::RAPID_ROUTINE_NAME_INPUT(Modules::T_ROB_RAPID, "routine_name_input");
const RAPIDSymbolResource Symbols::SG_COMMAND_INPUT(Modules::T_ROB_SG, "command_input");
const RAPIDSymbolResource Symbols::SG_SETTINGS(Modules::T_ROB_SG, "settings");
const RAPIDSymbolResource Symbols::SG_TARGET_POSTION_INPUT(Modules::T_ROB_SG, "target_position_input");
const RAPIDSymbolResource Symbols::UTILITY_BASE_FRAME(Modules::T_ROB_UTILITY, "base_frame");
const RAPIDSymbolResource Symbols::UTILITY_CALIBRATION_TARGET(Modules::T_ROB_UTILITY, "calibration_target");
const RAPIDSymbolResource Symbols::WATCHDOG_ACTIVE(Modules::T_ROB_WATCHDOG, "active");
const RAPIDSymbolResource Symbols::WATCHDOG_CHECK_EXTERNAL_STATUS(Modules::T_ROB_WATCHDOG, "check_external_status");




/***********************************************************************************************************************
 * Class definitions: RWSStateMachineInterface::Services::EGM
 */

/************************************************************
 * Primary methods
 */

EGMActions RWSStateMachineInterface::Services::EGM::getCurrentAction(const std::string& task) const
{
  EGMActions result;
  RAPIDNum temp_current_action;

  if (p_rws_interface_->getRAPIDSymbolData(task, Symbols::EGM_CURRENT_ACTION, &temp_current_action))
  {
    switch ((int) temp_current_action.value)
    {
      case EGM_ACTION_STOP:
        result = EGM_ACTION_STOP;
      break;

      case EGM_ACTION_RUN_JOINT:
        result = EGM_ACTION_RUN_JOINT;
      break;

      case EGM_ACTION_RUN_POSE:
        result = EGM_ACTION_RUN_POSE;
      break;

      default:
        result = EGM_ACTION_UNKNOWN;
      break;
    }
  }
  else
  {
    result = EGM_ACTION_UNKNOWN;
  }

  return result;
}

bool RWSStateMachineInterface::Services::EGM::getSettings(const std::string& task, EGMSettings* p_settings) const
{
  return p_rws_interface_->getRAPIDSymbolData(task, Symbols::EGM_SETTINGS, p_settings);
}

bool RWSStateMachineInterface::Services::EGM::setSettings(const std::string& task, const EGMSettings& settings) const
{
  return p_rws_interface_->setRAPIDSymbolData(task, Symbols::EGM_SETTINGS, settings);
}

bool RWSStateMachineInterface::Services::EGM::signalEGMStartJoint() const
{
  return p_rws_interface_->toggleIOSignal(IOSignals::EGM_START_JOINT);
}

bool RWSStateMachineInterface::Services::EGM::signalEGMStartPose() const
{
  return p_rws_interface_->toggleIOSignal(IOSignals::EGM_START_POSE);
}

bool RWSStateMachineInterface::Services::EGM::signalEGMStartStream() const
{
  return p_rws_interface_->toggleIOSignal(IOSignals::EGM_START_STREAM);
}

bool RWSStateMachineInterface::Services::EGM::signalEGMStop() const
{
  return p_rws_interface_->toggleIOSignal(IOSignals::EGM_STOP);
}

bool RWSStateMachineInterface::Services::EGM::signalEGMStopStream() const
{
  return p_rws_interface_->toggleIOSignal(IOSignals::EGM_STOP_STREAM);
}




/***********************************************************************************************************************
 * Class definitions: RWSStateMachineInterface::Services::Main
 */

/************************************************************
 * Primary methods
 */

States RWSStateMachineInterface::Services::Main::getCurrentState(const std::string& task) const
{
  States result;
  RAPIDNum temp_current_state;

  if (p_rws_interface_->getRAPIDSymbolData(task, Symbols::MAIN_CURRENT_STATE, &temp_current_state))
  {
    switch ((int) temp_current_state.value)
    {
      case STATE_IDLE:
        result = STATE_IDLE;
      break;

      case STATE_INITIALIZE:
        result = STATE_INITIALIZE;
      break;

      case STATE_RUN_RAPID_ROUTINE:
        result = STATE_RUN_RAPID_ROUTINE;
      break;

      case STATE_RUN_EGM_ROUTINE:
        result = STATE_RUN_EGM_ROUTINE;
      break;

      default:
        result = STATE_UNKNOWN;
      break;
    }
  }
  else
  {
    result = STATE_UNKNOWN;
  }

  return result;
}


TriBool RWSStateMachineInterface::Services::Main::isStateIdle(const std::string& task) const

{
  TriBool result;
  States temp_current_state = getCurrentState(task);

  if (temp_current_state != STATE_UNKNOWN)
  {
    result = (temp_current_state == STATE_IDLE);
  }

  return result;
}

TriBool RWSStateMachineInterface::Services::Main::isStationary(const std::string& mechanical_unit) const
{
  TriBool result;

  std::string temp_stationary = p_rws_interface_->getIOSignal(IOSignals::OUTPUT_STATIONARY + "_" + mechanical_unit);

  if (!temp_stationary.empty())
  {
    result = (temp_stationary == SystemConstants::IOSignals::HIGH);
  }

  return result;
}




/***********************************************************************************************************************
 * Class definitions: RWSStateMachineInterface::Services::RAPID
 */

/************************************************************
 * Primary methods
 */

bool RWSStateMachineInterface::Services::RAPID::runCallByVar(const std::string& task,
                                                             const std::string& routine_name,
                                                             const unsigned int routine_number) const
{
  RAPIDString temp_routine_name(routine_name);
  RAPIDNum temp_routine_number(routine_number);
  return p_rws_interface_->setRAPIDSymbolData(task, Symbols::RAPID_CALL_BY_VAR_NAME_INPUT, temp_routine_name) &&
         p_rws_interface_->setRAPIDSymbolData(task, Symbols::RAPID_CALL_BY_VAR_NUM_INPUT, temp_routine_number) &&
         setRoutineName(task, Procedures::RUN_CALL_BY_VAR) && signalRunRAPIDRoutine();
}

bool RWSStateMachineInterface::Services::RAPID::runModuleLoad(const std::string& task,
                                                              const std::string& file_path) const
{
  RAPIDString temp_file_path(file_path);
  return p_rws_interface_->setRAPIDSymbolData(task, Symbols::RAPID_MODULE_FILE_PATH_INPUT, temp_file_path) &&
         setRoutineName(task, Procedures::RUN_MODULE_LOAD) && signalRunRAPIDRoutine();
}

bool RWSStateMachineInterface::Services::RAPID::runModuleUnload(const std::string& task,
                                                                const std::string& file_path) const
{
  RAPIDString temp_file_path(file_path);
  return p_rws_interface_->setRAPIDSymbolData(task, Symbols::RAPID_MODULE_FILE_PATH_INPUT, temp_file_path) &&
         setRoutineName(task, Procedures::RUN_MODULE_UNLOAD) && signalRunRAPIDRoutine();
}

bool RWSStateMachineInterface::Services::RAPID::runMoveAbsJ(const std::string& task,
                                                            const JointTarget& joint_target) const
{
  return p_rws_interface_->setRAPIDSymbolData(task, Symbols::RAPID_MOVE_JOINT_TARGET_INPUT, joint_target) &&
         setRoutineName(task, Procedures::RUN_MOVE_ABS_J) && signalRunRAPIDRoutine();
}

bool RWSStateMachineInterface::Services::RAPID::runMoveJ(const std::string& task, const RobTarget& rob_target) const
{
  return p_rws_interface_->setRAPIDSymbolData(task, Symbols::RAPID_MOVE_ROB_TARGET_INPUT, rob_target) &&
         setRoutineName(task, Procedures::RUN_MOVE_J) && signalRunRAPIDRoutine();
}

bool RWSStateMachineInterface::Services::RAPID::runMoveToCalibrationPosition(const std::string& task) const
{
  return setRoutineName(task, Procedures::RUN_MOVE_TO_CALIBRATION_POSITION) && signalRunRAPIDRoutine();
}

bool RWSStateMachineInterface::Services::RAPID::setMoveSpeed(const std::string& task, const SpeedData& speed_data) const
{
  return p_rws_interface_->setRAPIDSymbolData(task, Symbols::RAPID_MOVE_SPEED_INPUT, speed_data);
}

bool RWSStateMachineInterface::Services::RAPID::setRoutineName(const std::string& task,
                                                               const std::string& routine_name) const
{
  RAPIDString temp_routine_name(routine_name);
  return p_rws_interface_->setRAPIDSymbolData(task, Symbols::RAPID_ROUTINE_NAME_INPUT, temp_routine_name);
}

bool RWSStateMachineInterface::Services::RAPID::signalRunRAPIDRoutine() const
{
  return p_rws_interface_->toggleIOSignal(IOSignals::RUN_RAPID_ROUTINE);
}




/***********************************************************************************************************************
 * Class definitions: RWSStateMachineInterface::Services::SG
 */

/************************************************************
 * Primary methods
 */

bool RWSStateMachineInterface::Services::SG::dualBlow1Off() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_BLOW_OFF_1) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_BLOW_OFF_1) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::dualBlow1On() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_BLOW_ON_1) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_BLOW_ON_1) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::dualBlow2Off() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_BLOW_OFF_2) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_BLOW_OFF_2) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::dualBlow2On() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_BLOW_ON_2) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_BLOW_ON_2) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::dualCalibrate() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_CALIBRATE) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_CALIBRATE) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::dualGetSettings(SGSettings* p_left_settings,
                                                             SGSettings* p_right_settings) const
{
  return getSettings(SystemConstants::RAPID::TASK_ROB_L, p_left_settings) &&
         getSettings(SystemConstants::RAPID::TASK_ROB_R, p_right_settings);
}

bool RWSStateMachineInterface::Services::SG::dualGripIn() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_GRIP_IN) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_GRIP_IN) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::dualGripOut() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_GRIP_OUT) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_GRIP_OUT) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::dualInitialize() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_INITIALIZE) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_INITIALIZE) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::dualMoveTo(const float left_position, const float right_position) const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_MOVE_TO) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_MOVE_TO) &&
         setTargetPositionInput(SystemConstants::RAPID::TASK_ROB_L, left_position) &&
         setTargetPositionInput(SystemConstants::RAPID::TASK_ROB_R, right_position) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::dualSetSettings(const SGSettings& left_settings,
                                                             const SGSettings& right_settings) const
{
  return setSettings(SystemConstants::RAPID::TASK_ROB_L, left_settings) &&
         setSettings(SystemConstants::RAPID::TASK_ROB_R, right_settings);
}

bool RWSStateMachineInterface::Services::SG::dualVacuum1Off() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_VACUUM_OFF_1) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_VACUUM_OFF_1) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::dualVacuum1On() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_VACUUM_ON_1) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_VACUUM_ON_1) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::dualVacuum2Off() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_VACUUM_OFF_2) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_VACUUM_OFF_2) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::dualVacuum2On() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_VACUUM_ON_2) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_VACUUM_ON_2) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::leftBlow1Off() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_BLOW_OFF_1) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_NONE) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::leftBlow1On() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_BLOW_ON_1) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_NONE) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::leftBlow2Off() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_BLOW_OFF_2) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_NONE) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::leftBlow2On() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_BLOW_ON_2) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_NONE) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::leftCalibrate() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_CALIBRATE) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_NONE) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::leftGetSettings(SGSettings* p_settings) const
{
  return getSettings(SystemConstants::RAPID::TASK_ROB_L, p_settings);
}

bool RWSStateMachineInterface::Services::SG::leftGripIn() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_GRIP_IN) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_NONE) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::leftGripOut() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_GRIP_OUT) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_NONE) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::leftInitialize() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_INITIALIZE) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_NONE) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::leftMoveTo(const float position) const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_MOVE_TO) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_NONE) &&
         setTargetPositionInput(SystemConstants::RAPID::TASK_ROB_L, position) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::leftSetSettings(const SGSettings& settings) const
{
  return setSettings(SystemConstants::RAPID::TASK_ROB_L, settings);
}

bool RWSStateMachineInterface::Services::SG::leftVacuum1Off() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_VACUUM_OFF_1) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_NONE) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::leftVacuum1On() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_VACUUM_ON_1) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_NONE) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::leftVacuum2Off() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_VACUUM_OFF_2) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_NONE) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::leftVacuum2On() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_VACUUM_ON_2) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_NONE) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::rightBlow1Off() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_NONE) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_BLOW_OFF_1) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::rightBlow1On() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_NONE) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_BLOW_ON_1) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::rightBlow2Off() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_NONE) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_BLOW_OFF_2) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::rightBlow2On() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_NONE) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_BLOW_ON_2) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::rightCalibrate() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_NONE) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_CALIBRATE) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::rightGetSettings(SGSettings* p_settings) const
{
  return getSettings(SystemConstants::RAPID::TASK_ROB_R, p_settings);
}

bool RWSStateMachineInterface::Services::SG::rightGripIn() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_NONE) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_GRIP_IN) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::rightGripOut() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_NONE) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_GRIP_OUT) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::rightInitialize() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_NONE) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_INITIALIZE) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::rightMoveTo(const float position) const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_NONE) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_MOVE_TO) &&
         setTargetPositionInput(SystemConstants::RAPID::TASK_ROB_R, position) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::rightSetSettings(const SGSettings& settings) const
{
  return setSettings(SystemConstants::RAPID::TASK_ROB_R, settings);
}

bool RWSStateMachineInterface::Services::SG::rightVacuum1Off() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_NONE) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_VACUUM_OFF_1) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::rightVacuum1On() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_NONE) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_VACUUM_ON_1) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::rightVacuum2Off() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_NONE) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_VACUUM_OFF_2) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::rightVacuum2On() const
{
  return setCommandInput(SystemConstants::RAPID::TASK_ROB_L, SG_COMMAND_NONE) &&
         setCommandInput(SystemConstants::RAPID::TASK_ROB_R, SG_COMMAND_VACUUM_ON_2) &&
         signalRunSGRoutine();
}

bool RWSStateMachineInterface::Services::SG::signalRunSGRoutine() const
{
  return p_rws_interface_->toggleIOSignal(IOSignals::RUN_SG_ROUTINE);
}

/************************************************************
 * Auxiliary methods
 */

bool RWSStateMachineInterface::Services::SG::getSettings(const std::string& task, SGSettings* p_settings) const
{
  return p_rws_interface_->getRAPIDSymbolData(task, Symbols::SG_SETTINGS, p_settings);
}

bool RWSStateMachineInterface::Services::SG::setCommandInput(const std::string& task, const SGCommands& command) const
{
  RAPIDNum temp_command(command);
  return p_rws_interface_->setRAPIDSymbolData(task, Symbols::SG_COMMAND_INPUT, temp_command);
}

bool RWSStateMachineInterface::Services::SG::setSettings(const std::string& task, const SGSettings& settings) const
{
  return p_rws_interface_->setRAPIDSymbolData(task, Symbols::SG_SETTINGS, settings);
}

bool RWSStateMachineInterface::Services::SG::setTargetPositionInput(const std::string& task, const float position) const
{
  RAPIDNum temp_position(position);
  return p_rws_interface_->setRAPIDSymbolData(task, Symbols::SG_TARGET_POSTION_INPUT, temp_position);
}




/***********************************************************************************************************************
 * Class definitions: RWSStateMachineInterface::Services::Utility
 */

/************************************************************
 * Primary methods
 */

bool RWSStateMachineInterface::Services::Utility::getBaseFrame(const std::string& task, Pose* p_base_frame) const
{
  return p_rws_interface_->getRAPIDSymbolData(task, Symbols::UTILITY_BASE_FRAME, p_base_frame);
}

bool RWSStateMachineInterface::Services::Utility::getCalibrationTarget(const std::string& task,
                                                                       JointTarget* p_calibration_joint_target) const
{
  return p_rws_interface_->getRAPIDSymbolData(task, Symbols::UTILITY_CALIBRATION_TARGET, p_calibration_joint_target);
}




/***********************************************************************************************************************
 * Class definitions: RWSStateMachineInterface::Services::Watchdog
 */

/************************************************************
 * Primary methods
 */

TriBool RWSStateMachineInterface::Services::Watchdog::isActive(const std::string& task) const
{
  TriBool result;
  RAPIDBool temp_active;

  if (p_rws_interface_->getRAPIDSymbolData(task, Symbols::WATCHDOG_ACTIVE, &temp_active))
  {
    result = temp_active.value;
  }

  return result;
}

TriBool RWSStateMachineInterface::Services::Watchdog::isCheckingExternalStatus(const std::string& task) const
{
  TriBool result;
  RAPIDBool temp_check_external_status;

  if (p_rws_interface_->getRAPIDSymbolData(task, Symbols::WATCHDOG_CHECK_EXTERNAL_STATUS, &temp_check_external_status))
  {
    result = temp_check_external_status.value;
  }

  return result;
}

bool RWSStateMachineInterface::Services::Watchdog::setExternalStatusSignal() const
{
  return p_rws_interface_->setIOSignal(IOSignals::WD_EXTERNAL_STATUS, SystemConstants::IOSignals::HIGH);
}

bool RWSStateMachineInterface::Services::Watchdog::signalStopRequest() const
{
  return p_rws_interface_->toggleIOSignal(IOSignals::WD_STOP_REQUEST);
}




/***********************************************************************************************************************
 * Class definitions: RWSStateMachineInterface
 */

/************************************************************
 * Auxiliary methods
 */

bool RWSStateMachineInterface::toggleIOSignal(const std::string& iosignal)
{
  bool result = false;
  int max_number_of_attempts = 5;

  if (isAutoMode().isTrue())
  {
    for (int i = 0; i < max_number_of_attempts && !result; ++i)
    {
      result = setIOSignal(iosignal, SystemConstants::IOSignals::LOW);
      if (result)
      {
        result = (getIOSignal(iosignal) == SystemConstants::IOSignals::LOW);
      }
    }

    if (result)
    {
      result = false;

      for (int i = 0; i < max_number_of_attempts && !result; ++i)
      {
        result = setIOSignal(iosignal, SystemConstants::IOSignals::HIGH);
        if (result)
        {
          result = (getIOSignal(iosignal) == SystemConstants::IOSignals::HIGH);
        }
      }
    }
  }

  return result;
}

} // end namespace rws
} // end namespace abb
