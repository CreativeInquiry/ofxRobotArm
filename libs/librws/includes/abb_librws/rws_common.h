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

#ifndef RWS_COMMON_H
#define RWS_COMMON_H

#include <ostream>
#include <string>
#include <vector>

#include "Poco/DOM/AutoPtr.h"
#include "Poco/DOM/Document.h"

#include "abb_librws_export.h"

namespace abb
{
namespace rws
{
/**
 * \brief A struct for representing XML attributes.
 */
struct XMLAttribute
{
  /**
   * \brief A default constructor.
   */
  XMLAttribute() {}

  /**
   * \brief A constructor.
   *
   * \param name for the attribute's name.
   * \param value for the attribute's value.
   */
  XMLAttribute(const std::string& name, const std::string& value) : name(name), value(value) {}

  /**
   * \brief The name of the attribute.
   */
  std::string name;

  /**
   * \brief The value of the attribute.
   */
  std::string value;
};

/**
 * \brief Returns all children of the given node that have the specifed attribute (name and value).
 *
 * \param p_root for the root node to search.
 * \param attribute specifying the XML attribute (name and value) that the children should have.
 *
 * \return std::vector<Poco::XML::Node*> with the children (empty if none are found, or if the root node is nullptr).
 */
std::vector<Poco::XML::Node*> xmlFindNodes(Poco::XML::Node* p_root, const XMLAttribute& attribute);

/**
 * \brief A function for finding all nodes in an XML document that has the specifed attribute (name and value).
 *
 * \param p_xml_document for the XML document to search.
 * \param attribute specifying the XML attribute (name and value) that the XML node should have.
 *
 * \return std::vector<Poco::XML::Node*> containing the found nodes.
 */
std::vector<Poco::XML::Node*> xmlFindNodes(Poco::AutoPtr<Poco::XML::Document> p_xml_document,
                                           const XMLAttribute& attribute);

/**
 * \brief A function for finding the text content of an XML node in an XML document. It stops after the first hit.
 *
 * \param p_xml_document for the XML document to search.
 * \param attribute specifying the XML attribute (name and value) that the XML text node should have.
 *
 * \return std::string containing the text content. Empty if none were found.
 */
std::string xmlFindTextContent(Poco::AutoPtr<Poco::XML::Document> p_xml_document, const XMLAttribute& attribute);

/**
 * \brief A function for finding the text content of an XML node. If not found, then it checks the node's children.
 *
 * \param p_node for the XML node to search.
 * \param attribute specifying the XML attribute (name and value) that the XML text node (child) should have.
 *
 * \return std::string containing the text content. Empty if none were found.
 */
std::string xmlFindTextContent(const Poco::XML::Node* p_node, const XMLAttribute& attribute);

/**
 * \brief A function for getting an XML node's attribute value.
 *
 * \param p_node for the XML node to process.
 * \param name specifying the XML attribute's name to get the value of.
 *
 * \return std::string with the attribute's value. Empty if the attribute was not found.
 */
std::string xmlNodeGetAttributeValue(const Poco::XML::Node* p_node, const std::string& name);

/**
 * \brief A function for checking if an XML node has the specified attribute.
 *
 * \param p_node for the XML node to check.
 * \param attribute specifying the XML attribute (name and value) to check for.
 *
 * \return bool indicating if the attribute was found or not.
 */
bool xmlNodeHasAttribute(const Poco::XML::Node* p_node, const XMLAttribute& attribute);

/**
 * \brief A function for checking if an XML node has the specified attribute.
 *
 * \param p_node for the XML node to check.
 * \param name specifying the attribute's name.
 * \param value specifying the attribute's value.
 *
 * \return bool indicating if the attribute was found or not.
 */
bool xmlNodeHasAttribute(const Poco::XML::Node* p_node, const std::string& name, const std::string& value);

/**
 * \brief Struct containing various constant values defined by default robot controller systems.
 */
struct SystemConstants
{
  /**
   * \brief Controller states related constants.
   */
  struct ABB_LIBRWS_EXPORT ContollerStates
  {
    /**
     * \brief Robot controller motor on.
     */
    static const std::string CONTROLLER_MOTOR_ON;

    /**
     * \brief Robot controller motor off.
     */
    static const std::string CONTROLLER_MOTOR_OFF;

    /**
     * \brief Robot controller auto mode.
     */
    static const std::string PANEL_OPERATION_MODE_AUTO;

    /**
     * \brief RAPID running.
     */
    static const std::string RAPID_EXECUTION_RUNNING;
  };

  /**
   * \brief General constants.
   */
  struct ABB_LIBRWS_EXPORT General
  {
    /**
     * \brief Default name of an application using RWS.
     */
    static const std::string EXTERNAL_APPLICATION;

    /**
     * \brief Default location of an application using RWS.
     */
    static const std::string EXTERNAL_LOCATION;

    /**
     * \brief Default port number for RWS communication.
     */
    static const unsigned short DEFAULT_PORT_NUMBER;

    /**
     * \brief Default password (for unconfigured robot controller systems).
     */
    static const std::string DEFAULT_PASSWORD;

    /**
     * \brief Default username (for unconfigured robot controller systems).
     */
    static const std::string DEFAULT_USERNAME;

    /**
     * \brief Local user.
     */
    static const std::string LOCAL;

    /**
     * \brief Mechanical unit name for ROB_1.
     */
    static const std::string MECHANICAL_UNIT_ROB_1;

    /**
     * \brief Mechanical unit name for ROB_2.
     */
    static const std::string MECHANICAL_UNIT_ROB_2;

    /**
     * \brief Mechanical unit name for ROB_3.
     */
    static const std::string MECHANICAL_UNIT_ROB_3;

    /**
     * \brief Mechanical unit name for ROB_4.
     */
    static const std::string MECHANICAL_UNIT_ROB_4;

    /**
     * \brief Mechanical unit name for ROB_L.
     */
    static const std::string MECHANICAL_UNIT_ROB_L;

    /**
     * \brief Mechanical unit name for ROB_R.
     */
    static const std::string MECHANICAL_UNIT_ROB_R;

    /**
     * \brief Remote user.
     */
    static const std::string REMOTE;

    /**
     * \brief Base coordinate system.
     */
    static const std::string COORDINATE_BASE;

    /**
     * \brief World coordinate system.
     */
    static const std::string COORDINATE_WORLD;

    /**
     * \brief Tool coordinate system.
     */
    static const std::string COORDINATE_TOOL;

    /**
     * \brief Work object (wobj) coordinate system.
     */
    static const std::string COORDINATE_WOBJ;
  };

  /**
   * \brief IO signal related constants.
   */
  struct ABB_LIBRWS_EXPORT IOSignals
  {
    /**
     * \brief Name of defined IO signal for smart gripper left position.
     *
     * Note: Requires the Smart Gripper product.
     */
    static const std::string HAND_ACTUAL_POSITION_L;

    /**
     * \brief Name of defined IO signal for smart gripper right position.
     *
     * Note: Requires the Smart Gripper product.
     */
    static const std::string HAND_ACTUAL_POSITION_R;

    /**
     * \brief Name of defined IO signal for smart gripper left speed.
     *
     * Note: Requires the Smart Gripper product.
     */
    static const std::string HAND_ACTUAL_SPEED_L;

    /**
     * \brief Name of defined IO signal for smart gripper right speed.
     *
     * Note: Requires the Smart Gripper product.
     */
    static const std::string HAND_ACTUAL_SPEED_R;

    /**
     * \brief Name of defined IO signal for smart gripper left calibration status.
     *
     * Note: Requires the Smart Gripper product.
     */
    static const std::string HAND_STATUS_CALIBRATED_L;

    /**
     * \brief Name of defined IO signal for smart gripper right calibration status.
     *
     * Note: Requires the Smart Gripper product.
     */
    static const std::string HAND_STATUS_CALIBRATED_R;

    /**
     * \brief High digital IO signal.
     */
    static const std::string HIGH;

    /**
     * \brief Low digital IO signal.
     */
    static const std::string LOW;
  };

  /**
   * \brief RAPID related constants.
   */
  struct ABB_LIBRWS_EXPORT RAPID
  {
    /**
     * \brief RAPID boolean false.
     */
    static const std::string RAPID_FALSE;

    /**
     * \brief RAPID boolean true.
     */
    static const std::string RAPID_TRUE;

    /**
     * \brief Default name for the first robot RAPID motion task.
     */
    static const std::string TASK_ROB_1;

    /**
     * \brief Default name for the second robot RAPID motion task.
     */
    static const std::string TASK_ROB_2;

    /**
     * \brief Default name for the third robot RAPID motion task.
     */
    static const std::string TASK_ROB_3;

    /**
     * \brief Default name for the fourth robot RAPID motion task.
     */
    static const std::string TASK_ROB_4;

    /**
     * \brief Default name for the IRB14000 (a.k.a YuMi) left arm robot RAPID motion task.
     */
    static const std::string TASK_ROB_L;

    /**
     * \brief Default name for the IRB14000 (a.k.a YuMi) right arm robot RAPID motion task.
     */
    static const std::string TASK_ROB_R;

    /**
     * \brief RAPID data type bool.
     */
    static const std::string TYPE_BOOL;

    /**
     * \brief RAPID data type dnum.
     */
    static const std::string TYPE_DNUM;

    /**
     * \brief RAPID data type num.
     */
    static const std::string TYPE_NUM;

    /**
     * \brief RAPID data type string.
     */
    static const std::string TYPE_STRING;
  };

  /**
   * \brief Robot Web Services (RWS) related constants.
   */
  struct RWS
  {
    /**
     * \brief XML attributes specifying names with corresponding values.
     */
    struct ABB_LIBRWS_EXPORT XMLAttributes
    {
      /**
       * \brief Class & active type.
       */
      static const XMLAttribute CLASS_ACTIVE;

      /**
       * \brief Class & cfg-dt-instance-li.
       */
      static const XMLAttribute CLASS_CFG_DT_INSTANCE_LI;

      /**
       * \brief Class & cfg-ia-t-li.
       */
      static const XMLAttribute CLASS_CFG_IA_T_LI;

      /**
       * \brief Class & controller type.
       */
      static const XMLAttribute CLASS_CTRL_TYPE;

      /**
       * \brief Class & controller execution state.
       */
      static const XMLAttribute CLASS_CTRLEXECSTATE;

      /**
       * \brief Class & controller state.
       */
      static const XMLAttribute CLASS_CTRLSTATE;

      /**
       * \brief Class & data type.
       */
      static const XMLAttribute CLASS_DATTYP;

      /**
       * \brief Class & excstate type.
       */
      static const XMLAttribute CLASS_EXCSTATE;

      /**
       * \brief Class & ios-signal.
       */
      static const XMLAttribute CLASS_IOS_SIGNAL;

      /**
       * \brief Class & lvalue.
       */
      static const XMLAttribute CLASS_LVALUE;

      /**
       * \brief Class & motiontask.
       */
      static const XMLAttribute CLASS_MOTIONTASK;

      /**
       * \brief Class & name.
       */
      static const XMLAttribute CLASS_NAME;

      /**
       * \brief Class & operation mode.
       */
      static const XMLAttribute CLASS_OPMODE;

      /**
       * \brief Class & rap-module-info-li.
       */
      static const XMLAttribute CLASS_RAP_MODULE_INFO_LI;

      /**
       * \brief Class & rap-task-li.
       */
      static const XMLAttribute CLASS_RAP_TASK_LI;

      /**
       * \brief Class & RobotWare version name.
       */
      static const XMLAttribute CLASS_RW_VERSION_NAME;

      /**
       * \brief Class & state.
       */
      static const XMLAttribute CLASS_STATE;

      /**
       * \brief Class & sys-option-li.
       */
      static const XMLAttribute CLASS_SYS_OPTION_LI;

      /**
       * \brief Class & sys-system-li.
       */
      static const XMLAttribute CLASS_SYS_SYSTEM_LI;

      /**
       * \brief Class & type.
       */
      static const XMLAttribute CLASS_TYPE;

      /**
       * \brief Class & value.
       */
      static const XMLAttribute CLASS_VALUE;

      /**
       * \brief Class & option.
       */
      static const XMLAttribute CLASS_OPTION;
    };

    /**
     * \brief Identifiers in different RWS messages. E.g. XML attribute names/values.
     */
    struct ABB_LIBRWS_EXPORT Identifiers
    {
      /**
       * \brief Active type.
       */
      static const std::string ACTIVE;

      /**
       * \brief Configuration motion domain type: arm.
       */
      static const std::string ARM;

      /**
       * \brief XML attribute name: class.
       */
      static const std::string CLASS;

      /**
       * \brief Configuration instance list item.
       */
      static const std::string CFG_DT_INSTANCE_LI;

      /**
       * \brief Configuration list item.
       */
      static const std::string CFG_IA_T_LI;

      /**
       * \brief Controller type.
       */
      static const std::string CTRL_TYPE;

      /**
       * \brief Controller execution state.
       */
      static const std::string CTRLEXECSTATE;

      /**
       * \brief Controller state.
       */
      static const std::string CTRLSTATE;

      /**
       * \brief Data type.
       */
      static const std::string DATTYP;

      /**
       * \brief Execution state type.
       */
      static const std::string EXCSTATE;

      /**
       * \brief Home directory.
       */
      static const std::string HOME_DIRECTORY;

      /**
       * \brief IO signal.
       */
      static const std::string IOS_SIGNAL;

      /**
       * \brief Mechanical unit.
       */
      static const std::string MECHANICAL_UNIT;

      /**
       * \brief Mechanical unit group.
       */
      static const std::string MECHANICAL_UNIT_GROUP;

      /**
       * \brief Motion topic in the system configurations (abbreviated as moc).
       */
      static const std::string MOC;

      /**
       * \brief Motion task.
       */
      static const std::string MOTIONTASK;

      /**
       * \brief Name.
       */
      static const std::string NAME;

      /**
       * \brief Lvalue.
       */
      static const std::string LVALUE;

      /**
       * \brief Opmode.
       */
      static const std::string OPMODE;

      /**
       * \brief Options present on the controller.
       */
      static const std::string PRESENT_OPTIONS;

      /**
       * \brief RAPID module info list item.
       */
      static const std::string RAP_MODULE_INFO_LI;

      /**
       * \brief RAPID task list item.
       */
      static const std::string RAP_TASK_LI;

      /**
       * \brief Robot.
       */
      static const std::string ROBOT;

      /**
       * \brief RobotWare version name.
       */
      static const std::string RW_VERSION_NAME;

      /**
       * \brief Single.
       */
      static const std::string SINGLE;

      /**
       * \brief State.
       */
      static const std::string STATE;

      /**
       * \brief Controller topic in the system configurations (abbreviated as sys).
       */
      static const std::string SYS;

      /**
       * \brief Sys option list item.
       */
      static const std::string SYS_OPTION_LI;

      /**
       * \brief Sys system list item.
       */
      static const std::string SYS_SYSTEM_LI;

      /**
       * \brief Title.
       */
      static const std::string TITLE;

      /**
       * \brief Type.
       */
      static const std::string TYPE;

      /**
       * \brief Value.
       */
      static const std::string VALUE;

      /**
       * \brief Option.
       */
      static const std::string OPTION;
    };

    /**
     * \brief RWS queries.
     */
    struct ABB_LIBRWS_EXPORT Queries
    {
      /**
       * \brief Release action query.
       */
      static const std::string ACTION_RELEASE;

      /**
       * \brief Request action query.
       */
      static const std::string ACTION_REQUEST;

      /**
       * \brief Reset program pointer action query.
       */
      static const std::string ACTION_RESETPP;

      /**
       * \brief Set action query.
       */
      static const std::string ACTION_SET;

      /**
       * \brief Set controller state action query.
       */
      static const std::string ACTION_SETCTRLSTATE;

      /**
       * \brief Set locale.
       */
      static const std::string ACTION_SET_LOCALE;

      /**
       * \brief Start action query.
       */
      static const std::string ACTION_START;

      /**
       * \brief Stop action query.
       */
      static const std::string ACTION_STOP;

      /**
       * \brief Task query.
       */
      static const std::string TASK;
    };

    /**
     * \brief RWS resources and queries.
     */
    struct ABB_LIBRWS_EXPORT Resources
    {
      /**
       * \brief Instances.
       */
      static const std::string INSTANCES;

      /**
       * \brief Jointtarget.
       */
      static const std::string JOINTTARGET;

      /**
       * \brief Logout.
       */
      static const std::string LOGOUT;

      /**
       * \brief Robtarget.
       */
      static const std::string ROBTARGET;

      /**
       * \brief Configurations.
       */
      static const std::string RW_CFG;

      /**
       * \brief Signals.
       */
      static const std::string RW_IOSYSTEM_SIGNALS;

      /**
       * \brief Mastership.
       */
      static const std::string RW_MASTERSHIP;

      /**
       * \brief Mechanical units.
       */
      static const std::string RW_MOTIONSYSTEM_MECHUNITS;

      /**
       * \brief Panel controller state.
       */
      static const std::string RW_PANEL_CTRLSTATE;

      /**
       * \brief Panel operation mode.
       */
      static const std::string RW_PANEL_OPMODE;

      /**
       * \brief RAPID execution.
       */
      static const std::string RW_RAPID_EXECUTION;

      /**
       * \brief RAPID modules.
       */
      static const std::string RW_RAPID_MODULES;

      /**
       * \brief RAPID symbol data.
       */
      static const std::string RW_RAPID_SYMBOL_DATA_RAPID;

      /**
       * \brief RAPID symbol properties.
       */
      static const std::string RW_RAPID_SYMBOL_PROPERTIES_RAPID;

      /**
       * \brief RAPID tasks.
       */
      static const std::string RW_RAPID_TASKS;

      /**
       * \brief RobotWare system.
       */
      static const std::string RW_SYSTEM;
    };

    /**
     * \brief RWS services.
     */
    struct ABB_LIBRWS_EXPORT Services
    {
      /**
       * \brief Controller service.
       */
      static const std::string CTRL;

      /**
       * \brief Fileservice.
       */
      static const std::string FILESERVICE;

      /**
       * \brief RobotWare service.
       */
      static const std::string RW;

      /**
       * \brief Subscription service.
       */
      static const std::string SUBSCRIPTION;

      /**
       * \brief User service.
       */
      static const std::string USERS;
    };
  };
};

/**
 * \brief A class for a three-valued boolean.
 */
class TriBool
{
public:
  /**
   * \brief An enum for the different accepted values.
   */
  enum Values
  {
    UNKNOWN_VALUE, ///< Unknown value. E.g. in case of communication failure.
    TRUE_VALUE,    ///< True value.
    FALSE_VALUE    ///< False value.
  };

  /**
   * \brief A default constructor.
   */
  TriBool() : value(UNKNOWN_VALUE) {}

  /**
   * \brief A constructor.
   */
  TriBool(const Values initial_value) : value(initial_value) {}

  /**
   * \brief A constructor.
   *
   * \param value for the initial true or false value.
   */
  TriBool(const bool initial_value) : value(initial_value ? TRUE_VALUE : FALSE_VALUE) {}

  /**
   * \brief Operator for assignment.
   *
   * \param rhs for right hand side value.
   *
   * \return TriBool& self.
   */
  TriBool& operator=(const Values& rhs)
  {
    value = rhs;

    return *this;
  }

  /**
   * \brief Operator for assignment.
   *
   * \param rhs for right hand side value.
   *
   * \return TriBool& self.
   */
  TriBool& operator=(const bool& rhs)
  {
    value = (rhs ? TRUE_VALUE : FALSE_VALUE);

    return *this;
  }

  /**
   * \brief Operator for equal to comparison.
   *
   * \param rhs for right hand side value.
   *
   * \return bool indicating if the comparision was equal or not.
   */
  bool operator==(const TriBool& rhs) const
  {
    return value == rhs.value;
  }

  /**
   * \brief Operator for equal to comparison.
   *
   * \param rhs for right hand side value.
   *
   * \return bool indicating if the comparision was equal or not.
   */
  bool operator==(const Values& rhs) const
  {
    return value == rhs;
  }

  /**
   * \brief Operator for not equal to comparison.
   *
   * \param rhs for right hand side value.
   *
   * \return bool indicating if the comparision was not equal or not.
   */
  bool operator!=(const TriBool& rhs) const
  {
    return value != rhs.value;
  }

  /**
   * \brief Operator for not equal to comparison.
   *
   * \param rhs for right hand side value.
   *
   * \return bool indicating if the comparision was not equal or not.
   */
  bool operator!=(const Values& rhs) const
  {
    return value != rhs;
  }

  /**
   * \brief Operator for stream insertion.
   *
   * \param stream for the output stream.
   * \param rhs for the right hand side value, to insert into the output stream.
   *
   * \return std::ostream& for the output stream.
   */
  friend std::ostream& operator<<(std::ostream& stream, const TriBool& rhs)
  {
    return stream << rhs.toString();
  }

  /**
   * \brief A method for checking if the tri bool is unknown.
   *
   * \return bool indicating if the tri bool is unknown or not.
   */
  bool isUnknown() const
  {
    return value == UNKNOWN_VALUE;
  }

  /**
   * \brief A method for checking if the tri bool is true.
   *
   * \return bool indicating if the tri bool is true or not.
   */
  bool isTrue() const
  {
    return value == TRUE_VALUE;
  }

  /**
   * \brief A method for checking if the tri bool is false.
   *
   * \return bool indicating if the tri bool is false or not.
   */
  bool isFalse() const
  {
    return value == FALSE_VALUE;
  }

  /**
   * \brief A method for converting the tri bool to a text string.
   *
   * \return std::string containing the string representation of the value.
   */
  std::string toString() const
  {
    return (isUnknown() ? "unknown" : (isTrue() ? "true" : "false"));
  }

private:
  /**
   * \brief The tri bool value.
   */
  Values value;
};

} // end namespace rws
} // end namespace abb

#endif
