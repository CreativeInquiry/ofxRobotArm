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

#ifndef RWS_CLIENT_H
#define RWS_CLIENT_H

#include <deque>
#include <sstream>
#include <vector>

#include "Poco/DOM/DOMParser.h"

#include "rws_common.h"
#include "rws_rapid.h"
#include "rws_poco_client.h"

namespace abb
{
namespace rws
{
/**
 * \brief A class for a Robot Web Services (RWS) client based on a POCO client.
 *
 * Note: Only a subset of the features available in RWS are implemented here.
 *
 * See http://developercenter.robotstudio.com/webservice/api_reference for details about RWS.
 *
 * TODO:
 * - Flesh out the subscription functionality. E.g. implement a "subscription manager".
 *
 */
class RWSClient : public POCOClient
{
public:
  /**
   * \brief A struct for containing an evaluated communication result.
   */
  struct RWSResult
  {
    /**
     * \brief For indicating if the communication was successfull or not.
     */
    bool success;

    /**
     * \brief For containing any parsed result in XML format. If no data is parsed, then it will be null.
     */
    Poco::AutoPtr<Poco::XML::Document> p_xml_document;

    /**
     * \brief Container for an error message (if one occurred).
     */
    std::string error_message;

    /**
     * \brief A default constructor.
     */
    RWSResult() : success(false) {}
  };

  /**
   * \brief A class for representing a RAPID symbol resource.
   */
  struct RAPIDSymbolResource
  {
    /**
     * \brief A constructor.
     *
     * \param module specifying the name of the RAPID module containing the symbol.
     * \param name specifying the name of the RAPID symbol.
     */
    RAPIDSymbolResource(const std::string& module, const std::string& name)
    :
    module(module),
    name(name)
    {}

    /**
     * \brief The RAPID module name.
     */
    std::string module;

    /**
     * \brief The RAPID symbol name.
     */
    std::string name;
  };

  /**
   * \brief A class for representing a RAPID resource.
   */
  struct RAPIDResource
  {
    /**
     * \brief A constructor.
     *
     * \param task specifying the name of the RAPID task containing the symbol.
     * \param module specifying the name of the RAPID module containing the symbol.
     * \param name specifying the name of the RAPID symbol.
     */
    RAPIDResource(const std::string& task, const std::string& module, const std::string& name)
    :
    task(task),
    module(module),
    name(name)
    {}

    /**
     * \brief A constructor.
     *
     * \param task specifying the name of the RAPID task containing the symbol.
     * \param symbol specifying the names of the RAPID module and the the symbol.
     */
    RAPIDResource(const std::string& task, const RAPIDSymbolResource& symbol)
    :
    task(task),
    module(symbol.module),
    name(symbol.name)
    {}

    /**
     * \brief The RAPID task name.
     */
    std::string task;

    /**
     * \brief The RAPID module name.
     */
    std::string module;

    /**
     * \brief The RAPID symbol name.
     */
    std::string name;
  };

  /**
   * \brief A class for representing a file resource.
   */
  struct FileResource
  {
    /**
     * \brief A constructor.
     *
     * \param filename specifying the name of the file.
     * \param directory specifying the directory of the file on the robot controller (set to $home by default).
     */
    FileResource(const std::string& filename,
                 const std::string& directory = SystemConstants::RWS::Identifiers::HOME_DIRECTORY)
    :
    filename(filename),
    directory(directory)
    {}

    /**
     * \brief The file's name.
     */
    std::string filename;

    /**
     * \brief The file's directory on the robot controller.
     */
    std::string directory;
  };

  /**
   * \brief A class for representing subscription resources.
   */
  class SubscriptionResources
  {
  public:
    /**
     * \brief An enum for specifying subscription priority.
     */
    enum Priority
    {
      LOW,    ///< Low priority.
      MEDIUM, ///< Medium priority.
      HIGH    ///< High priority. Only RobotWare 6.05 (or newer) and for IO signals and persistant RAPID variables.
    };

    /**
     * \brief A struct for containing information about a subscription resource.
     */
    struct SubscriptionResource
    {
      /**
       * \brief URI of the resource.
       */
      std::string resource_uri;

      /**
       * \brief Priority of the subscription.
       */
      Priority priority;

      /**
       * \brief A constructor.
       *
       * \param resource_uri for the URI of the resource.
       * \param priority for the priority of the subscription.
       */
      SubscriptionResource(const std::string& resource_uri, const Priority priority)
      :
      resource_uri(resource_uri),
      priority(priority)
      {}
    };

    /**
     * \brief A method to add information about a subscription resource.
     *
     * \param resource_uri for the URI of the resource.
     * \param priority for the priority of the subscription.
     */
    void add(const std::string& resource_uri, const Priority priority);

    /**
     * \brief A method to add information about a IO signal subscription resource.
     *
     * \param iosignal for the IO signal's name.
     * \param priority for the priority of the subscription.
     */
    void addIOSignal(const std::string& iosignal, const Priority priority);

    /**
     * \brief A method to add information about a RAPID persistant symbol subscription resource.
     *
     * \param resource specifying the RAPID task, module and symbol names for the RAPID resource.
     * \param priority for the priority of the subscription.
     */
    void addRAPIDPersistantVariable(const RAPIDResource& resource, const Priority priority);

    /**
     * \brief A method for retrieving the contained subscription resources information.
     *
     * \return std::vector<SubscriptionResource> containing information of the subscription resources.
     */
    const std::vector<SubscriptionResource>& getResources() const { return resources_; }

  private:
    /**
     * \brief A vector of subscription resources.
     */
    std::vector<SubscriptionResource> resources_;
  };

  /**
   * \brief An enumeration of controller coordinate frames.
   */
  enum Coordinate
  {
    BASE,  ///< \brief Base frame coordinate.
    WORLD, ///< \brief World frame coordinate.
    TOOL,  ///< \brief Tool frame coordinate.
    WOBJ,  ///< \brief Work object (wobj) frame coordinate.
    ACTIVE ///< \brief Currently active coordinate.
  };

  /**
   * \brief A constructor.
   *
   * \param ip_address specifying the robot controller's IP address.
   */
  RWSClient(const std::string& ip_address)
  :
  POCOClient(ip_address,
             SystemConstants::General::DEFAULT_PORT_NUMBER,
             SystemConstants::General::DEFAULT_USERNAME,
             SystemConstants::General::DEFAULT_PASSWORD)
  {}

  /**
   * \brief A constructor.
   *
   * \param ip_address specifying the robot controller's IP address.
   * \param username for the username to the RWS authentication process.
   * \param password for the password to the RWS authentication process.
   */
  RWSClient(const std::string& ip_address, const std::string& username, const std::string& password)
  :
  POCOClient(ip_address,
             SystemConstants::General::DEFAULT_PORT_NUMBER,
             username,
             password)
  {}

  /**
   * \brief A constructor.
   *
   * \param ip_address specifying the robot controller's IP address.
   * \param port for the port used by the RWS server.
   */
  RWSClient(const std::string& ip_address, const unsigned short port)
  :
  POCOClient(ip_address,
             port,
             SystemConstants::General::DEFAULT_USERNAME,
             SystemConstants::General::DEFAULT_PASSWORD)
  {}

  /**
   * \brief A constructor.
   *
   * \param ip_address specifying the robot controller's IP address.
   * \param port for the port used by the RWS server.
   * \param username for the username to the RWS authentication process.
   * \param password for the password to the RWS authentication process.
   */
  RWSClient(const std::string& ip_address,
            const unsigned short port,
            const std::string& username,
            const std::string& password)
  :
  POCOClient(ip_address,
             port,
             username,
             password)
  {}

  /**
   * \brief A destructor.
   */
  ~RWSClient()
  {
    logout();
  }

  /**
   * \brief Retrieves a list of controller resources (e.g. controller identity and clock information).
   *
   * \return RWSResult containing the result.
   */
  RWSResult getContollerService();

  /**
   * \brief A method for retrieving the configuration instances of a type, belonging to a specific configuration topic.
   *
   * \param topic specifying the configuration topic.
   * \param type specifying the type in the configuration topic.
   *
   * \return RWSResult containing the result.
   */
  RWSResult getConfigurationInstances(const std::string& topic, const std::string& type);

  /**
   * \brief A method for retrieving all available IO signals on the controller.
   *
   * \return RWSResult containing the result.
   */
  RWSResult getIOSignals();

  /**
   * \brief A method for retrieving the value of an IO signal.
   *
   * \param iosignal for the IO signal's name.
   *
   * \return RWSResult containing the result.
   */
  RWSResult getIOSignal(const std::string& iosignal);

  /**
   * \brief A method for retrieving static information about a mechanical unit.
   *
   * \param mechunit for the mechanical unit's name.
   *
   * \return RWSResult containing the result.
   */
  RWSResult getMechanicalUnitStaticInfo(const std::string& mechunit);

  /**
   * \brief A method for retrieving dynamic information about a mechanical unit.
   *
   * \param mechunit for the mechanical unit's name.
   *
   * \return RWSResult containing the result.
   */
  RWSResult getMechanicalUnitDynamicInfo(const std::string& mechunit);

  /**
   * \brief A method for retrieving the current jointtarget values of a mechanical unit.
   *
   * \param mechunit for the mechanical unit's name.
   *
   * \return RWSResult containing the result.
   */
  RWSResult getMechanicalUnitJointTarget(const std::string& mechunit);

  /**
   * \brief A method for retrieving the current robtarget values of a mechanical unit.
   *
   * \param mechunit for the mechanical unit's name.
   * \param coordinate for the coordinate mode (base, world, tool, or wobj) in which the robtarget will be reported.
   * \param tool for the tool frame relative to which the robtarget will be reported.
   * \param wobj for the work object (wobj) relative to which the robtarget will be reported.
   *
   * \return RWSResult containing the result.
   */
  RWSResult getMechanicalUnitRobTarget(const std::string& mechunit,
                                       const Coordinate& coordinate = ACTIVE,
                                       const std::string& tool = "",
                                       const std::string& wobj = "");

  /**
   * \brief A method for retrieving the data of a RAPID symbol.
   *
   * \param resource specifying the RAPID task, module and symbol names for the RAPID resource.
   *
   * \return RWSResult containing the result.
   */
  RWSResult getRAPIDSymbolData(const RAPIDResource& resource);

  /**
   * \brief A method for retrieving the data of a RAPID symbol (parsed into a struct representing the RAPID data).
   *
   * \param resource specifying the RAPID task, module and symbol names for the RAPID resource.
   * \param p_data for containing the retrieved data.
   *
   * \return RWSResult containing the result.
   */
  RWSResult getRAPIDSymbolData(const RAPIDResource& resource, RAPIDSymbolDataAbstract* p_data);

  /**
   * \brief A method for retrieving the properties of a RAPID symbol.
   *
   * \param resource specifying the RAPID task, module and symbol names for the RAPID resource.
   *
   * \return RWSResult containing the result.
   */
  RWSResult getRAPIDSymbolProperties(const RAPIDResource& resource);

  /**
   * \brief A method for retrieving the execution state of RAPID.
   *
   * \return RWSResult containing the result.
   */
  RWSResult getRAPIDExecution();

  /**
   * \brief A method for retrieving information about the RAPID modules of a RAPID task.
   *
   * \param task specifying the RAPID task.
   *
   * \return RWSResult containing the result.
   */
  RWSResult getRAPIDModulesInfo(const std::string& task);

  /**
   * \brief A method for retrieving the RAPID tasks that are defined in the robot controller system.
   *
   * \return RWSResult containing the result.
   */
  RWSResult getRAPIDTasks();

  /**
   * \brief A method for retrieving info about the current robot controller system.
   *
   * \return RWSResult containing the result.
   */
  RWSResult getRobotWareSystem();

  /**
   * \brief A method for retrieving the robot controller's speed ratio for RAPID motions (e.g. MoveJ and MoveL).
   *
   * \return RWSResult containing the result.
   */
  RWSResult getSpeedRatio();

  /**
   * \brief A method for retrieving the controller state.
   *
   * \return RWSResult containing the result.
   */
  RWSResult getPanelControllerState();

  /**
   * \brief A method for retrieving the operation mode of the controller.
   *
   * \return RWSResult containing the result.
   */
  RWSResult getPanelOperationMode();

  /**
   * \brief A method for setting the value of an IO signal.
   *
   * \param iosignal for the IO signal's name.
   * \param value for the IO signal's new value.
   *
   * \return RWSResult containing the result.
   */
  RWSResult setIOSignal(const std::string& iosignal, const std::string& value);

  /**
   * \brief A method for setting the data of a RAPID symbol.
   *
   * \param resource specifying the RAPID task, module and symbol names for the RAPID resource.
   * \param data for the RAPID symbol's new data.
   *
   * \return RWSResult containing the result.
   */
  RWSResult setRAPIDSymbolData(const RAPIDResource& resource, const std::string& data);

  /**
   * \brief A method for setting the data of a RAPID symbol (based on the provided struct representing the RAPID data).
   *
   * \param resource specifying the RAPID task, module and symbol names for the RAPID resource.
   * \param data for the RAPID symbol's new data.
   *
   * \return RWSResult containing the result.
   */
  RWSResult setRAPIDSymbolData(const RAPIDResource& resource, const RAPIDSymbolDataAbstract& data);

  /**
   * \brief A method for starting RAPID execution in the robot controller.
   *
   * \return RWSResult containing the result.
   */
  RWSResult startRAPIDExecution();

  /**
   * \brief A method for stopping RAPID execution in the robot controller.
   *
   * \return RWSResult containing the result.
   */
  RWSResult stopRAPIDExecution();

  /**
   * \brief A method for reseting the RAPID program pointer in the robot controller.
   *
   * \return RWSResult containing the result.
   */
  RWSResult resetRAPIDProgramPointer();

  /**
   * \brief A method for turning on the robot controller's motors.
   *
   * \return RWSResult containing the result.
   */
  RWSResult setMotorsOn();

  /**
   * \brief A method for turning off the robot controller's motors.
   *
   * \return RWSResult containing the result.
   */
  RWSResult setMotorsOff();

  /**
   * \brief A method for setting the robot controller's speed ratio for RAPID motions (e.g. MoveJ and MoveL).
   *
   * Note: The ratio must be an integer in the range [0, 100] (ie: inclusive).
   *
   * \param ratio specifying the new ratio.
   *
   * \return RWSResult containing the result.
   *
   * \throw std::out_of_range if argument is out of range.
   * \throw std::runtime_error if failed to create a string from the argument.
   */
  RWSResult setSpeedRatio(unsigned int ratio);

  /**
   * \brief A method for retrieving a file from the robot controller.
   *
   * Note: Depending on the file, then the content can be in text or binary format.
   *
   * \param resource specifying the file's directory and name.
   * \param p_file_content for containing the retrieved file content.
   *
   * \return RWSResult containing the result.
   */
  RWSResult getFile(const FileResource& resource, std::string* p_file_content);

  /**
   * \brief A method for uploading a file to the robot controller.
   *
   * \param resource specifying the file's directory and name.
   * \param file_content for the file's content.
   *
   * \return RWSResult containing the result.
   */
  RWSResult uploadFile(const FileResource& resource, const std::string& file_content);

  /**
   * \brief A method for deleting a file from the robot controller.
   *
   * \param resource specifying the file's directory and name.
   *
   * \return RWSResult containing the result.
   */
  RWSResult deleteFile(const FileResource& resource);

  /**
   * \brief A method for starting for a subscription.
   *
   * \param resources specifying the resources to subscribe to.
   *
   * \return RWSResult containing the result.
   */
  RWSResult startSubscription(const SubscriptionResources& resources);

  /**
   * \brief A method for waiting for a subscription event.
   *
   * \return RWSResult containing the result.
   */
  RWSResult waitForSubscriptionEvent();

  /**
   * \brief A method for ending a active subscription.
   *
   * \return RWSResult containing the result.
   */
  RWSResult endSubscription();

  /**
   * \brief Force close the active subscription connection.
   *
   * This will cause waitForSubscriptionEvent() to return or throw.
   * It does not delete the subscription from the controller.
   *
   * The preferred way to close the subscription is to request the robot controller to end it via
   * endSubscription(). This function can be used to force the connection to close immediately in
   * case the robot controller is not responding.
   *
   * This function blocks until an active waitForSubscriptionEvent() has finished.
   *
   */
  void forceCloseSubscription();

  /**
   * \brief A method for logging out the currently active RWS session.
   *
   * \return RWSResult containing the result.
   */
  RWSResult logout();

  /**
   * \brief A method for registering a user as local.
   *
   * \param username specifying the user name.
   * \param application specifying the external application.
   * \param location specifying the location.
   *
   * \return RWSResult containing the result.
   */
  RWSResult registerLocalUser(const std::string& username = SystemConstants::General::DEFAULT_USERNAME,
                              const std::string& application = SystemConstants::General::EXTERNAL_APPLICATION,
                              const std::string& location = SystemConstants::General::EXTERNAL_LOCATION);

  /**
   * \brief A method for registering a user as remote.
   *
   * \param username specifying the user name.
   * \param application specifying the external application.
   * \param location specifying the location.
   *
   * \return RWSResult containing the result.
   */
  RWSResult registerRemoteUser(const std::string& username = SystemConstants::General::DEFAULT_USERNAME,
                               const std::string& application = SystemConstants::General::EXTERNAL_APPLICATION,
                               const std::string& location = SystemConstants::General::EXTERNAL_LOCATION);

  /**
   * \brief Method for parsing a communication result into a XML document.
   *
   * \param result containing the result of the parsing.
   * \param poco_result containing the POCO result.
   */
  void parseMessage(RWSResult* result, const POCOResult& poco_result);

  /**
   * \brief Method for retrieving the internal log as a text string.
   *
   * \param verbose indicating if the log text should be verbose or not.
   *
   * \return std::string containing the log text. An empty text string is returned if the log is empty.
   */
  std::string getLogText(const bool verbose = false);

  /**
   * \brief Method for retrieving only the most recently logged event as a text string.
   *
   * \param verbose indicating if the log text should be verbose or not.
   *
   * \return std::string containing the log text. An empty text string is returned if the log is empty.
   */
  std::string getLogTextLatestEvent(const bool verbose = false);

private:
  /**
   * \brief A struct for representing conditions, for the evaluation of an attempted RWS communication.
   */
  struct EvaluationConditions
  {
    /**
     * \brief A default constructor.
     */
    EvaluationConditions() : parse_message_into_xml(false) {};

    /**
     * \brief A method for reseting the conditions.
     */
    void reset()
    {
      parse_message_into_xml = false;
      accepted_outcomes.clear();
    }

    /**
     * \brief Indication for if the received message should be parsed into a xml document.
     */
    bool parse_message_into_xml;

    /**
     * \brief Vector containing the accepted HTTP outcomes.
     */
    std::vector<Poco::Net::HTTPResponse::HTTPStatus> accepted_outcomes;
  };

  /**
   * \brief Method for checking a communication result against the accepted outcomes.
   *
   * \param result containing the result of the check.
   * \param poco_result containing the POCO result.
   * \param conditions containing the conditions for the evaluation.
   */
  void checkAcceptedOutcomes(RWSResult* result, const POCOResult& poco_result, const EvaluationConditions& conditions);

  /**
   * \brief Method for evaluating the result from a POCO communication.
   *
   * \param poco_result for the POCO result to evaluate.
   * \param conditions specifying the conditions for the evaluation.
   *
   * \return RWSResult containing the evaluated result.
   */
  RWSResult evaluatePOCOResult(const POCOResult& poco_result, const EvaluationConditions& conditions);

  /**
   * \brief Method for generating a configuration URI path.
   *
   * \param topic for the configuration topic.
   * \param type for the configuration type (belonging to the topic).
   *
   * \return std::string containing the path.
   */
  std::string generateConfigurationPath(const std::string& topic, const std::string& type);

  /**
   * \brief Method for generating an IO signal URI path.
   *
   * \param iosignal for the signal's name.
   *
   * \return std::string containing the path.
   */
  std::string generateIOSignalPath(const std::string& iosignal);

  /**
   * \brief Method for generating a mechanical unit resource URI path.
   *
   * \param mechunit for the mechanical unit's name.
   *
   * \return std::string containing the path.
   */
  std::string generateMechanicalUnitPath(const std::string& mechunit);

  /**
   * \brief Method for generating a RAPID data resource URI path.
   *
   * \param resource specifying the RAPID task, module and symbol names for the RAPID resource.
   *
   * \return std::string containing the path.
   */
  std::string generateRAPIDDataPath(const RAPIDResource& resource);

  /**
   * \brief Method for generating a RAPID properties resource URI path.
   *
   * \param resource specifying the RAPID task, module and symbol names for the RAPID resource.
   *
   * \return std::string containing the path.
   */
  std::string generateRAPIDPropertiesPath(const RAPIDResource& resource);

  /**
   * \brief Method for generating a file resource URI path.
   *
   * \param resource specifying the file's directory and name.
   *
   * \return std::string containing the path.
   */
  std::string generateFilePath(const FileResource& resource);

  /**
   * \brief Static constant for the log's size.
   */
  static const size_t LOG_SIZE = 20;

  /**
   * \brief Static constant for the default RWS subscription timeout [microseconds].
   */
  static const Poco::Int64 DEFAULT_SUBSCRIPTION_TIMEOUT = 40e6;

  /**
   * \brief Container for logging communication results.
   */
  std::deque<POCOResult> log_;

  /**
   * \brief A subscription group id.
   */
  std::string subscription_group_id_;
};

} // end namespace rws
} // end namespace abb

#endif
