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

#include <sstream>
#include <stdexcept>

#include "Poco/SAX/InputSource.h"

#include "abb_librws/rws_client.h"

namespace
{
static const char EXCEPTION_CREATE_STRING[]{"Failed to create string"};
}

namespace abb
{
namespace rws
{
using namespace Poco::Net;

typedef SystemConstants::RWS::Identifiers   Identifiers;
typedef SystemConstants::RWS::Queries       Queries;
typedef SystemConstants::RWS::Resources     Resources;
typedef SystemConstants::RWS::Services      Services;
typedef SystemConstants::RWS::XMLAttributes XMLAttributes;

/***********************************************************************************************************************
 * Class definitions: RWSClient::SubscriptionResources
 */

/************************************************************
 * Primary methods
 */

void RWSClient::SubscriptionResources::addIOSignal(const std::string& iosignal, const Priority priority)
{
  std::string resource_uri = Resources::RW_IOSYSTEM_SIGNALS;
  resource_uri += "/";
  resource_uri += iosignal;
  resource_uri += ";";
  resource_uri += Identifiers::STATE;

  add(resource_uri, priority);
}

void RWSClient::SubscriptionResources::addRAPIDPersistantVariable(const RAPIDResource& resource,
                                                                  const Priority priority)
{
  std::string resource_uri = Resources::RW_RAPID_SYMBOL_DATA_RAPID;
  resource_uri += "/";
  resource_uri += resource.task;
  resource_uri += "/";
  resource_uri += resource.module;
  resource_uri += "/";
  resource_uri += resource.name;
  resource_uri += ";";
  resource_uri += Identifiers::VALUE;

  add(resource_uri, priority);
}

void RWSClient::SubscriptionResources::add(const std::string& resource_uri, const Priority priority)
{
  resources_.push_back(SubscriptionResource(resource_uri, priority));
}




/***********************************************************************************************************************
 * Class definitions: RWSClient
 */

/************************************************************
 * Primary methods
 */

RWSClient::RWSResult RWSClient::getContollerService()
{
  std::string uri = Services::CTRL;

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(httpGet(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::getConfigurationInstances(const std::string& topic, const std::string& type)
{
  std::string uri = generateConfigurationPath(topic, type) + Resources::INSTANCES;

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(httpGet(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::getIOSignals()
{
  std::string const & uri = SystemConstants::RWS::Resources::RW_IOSYSTEM_SIGNALS;

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(httpGet(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::getIOSignal(const std::string& iosignal)
{
  std::string uri = generateIOSignalPath(iosignal);

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(httpGet(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::getMechanicalUnitStaticInfo(const std::string& mechunit)
{
  std::string uri = generateMechanicalUnitPath(mechunit) + "?resource=static";

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(httpGet(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::getMechanicalUnitDynamicInfo(const std::string& mechunit)
{
  std::string uri = generateMechanicalUnitPath(mechunit) + "?resource=dynamic";

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(httpGet(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::getMechanicalUnitJointTarget(const std::string& mechunit)
{
  std::string uri = generateMechanicalUnitPath(mechunit) + Resources::JOINTTARGET;

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(httpGet(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::getMechanicalUnitRobTarget(const std::string& mechunit,
                                                           const Coordinate& coordinate,
                                                           const std::string& tool,
                                                           const std::string& wobj)
{
  std::string uri = generateMechanicalUnitPath(mechunit) + Resources::ROBTARGET;

  std::string args = "";
  if (!tool.empty())
  {
    args += "&tool=" + tool;
  }
  if (!wobj.empty())
  {
    args += "&wobj=" + wobj;
  }

  const std::string coordinate_arg = "?coordinate=";
  switch (coordinate)
  {
    case BASE:
      uri += coordinate_arg + SystemConstants::General::COORDINATE_BASE + args;
    break;
    case WORLD:
      uri += coordinate_arg + SystemConstants::General::COORDINATE_WORLD + args;
    break;
    case TOOL:
      uri += coordinate_arg + SystemConstants::General::COORDINATE_TOOL + args;
    break;
    case WOBJ:
      uri += coordinate_arg + SystemConstants::General::COORDINATE_WOBJ + args;
    break;
    default:
      // If the "ACTIVE" enumeration is passed in (or any other non-identified value),
      // do not add any arguments to this command
    break;
  }

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(httpGet(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::getRAPIDExecution()
{
  std::string uri = Resources::RW_RAPID_EXECUTION;

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(httpGet(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::getRAPIDModulesInfo(const std::string& task)
{
  std::string uri = Resources::RW_RAPID_MODULES + "?" + Queries::TASK + task;

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(httpGet(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::getRAPIDTasks()
{
  std::string uri = Resources::RW_RAPID_TASKS;

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(httpGet(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::getRobotWareSystem()
{
  std::string uri = Resources::RW_SYSTEM;

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(httpGet(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::getSpeedRatio()
{
  std::string uri = "/rw/panel/speedratio";

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(httpGet(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::getPanelControllerState()
{
  std::string uri = Resources::RW_PANEL_CTRLSTATE;

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(httpGet(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::getPanelOperationMode()
{
  std::string uri = Resources::RW_PANEL_OPMODE;

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(httpGet(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::getRAPIDSymbolData(const RAPIDResource& resource)
{
  std::string uri = generateRAPIDDataPath(resource);

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(httpGet(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::getRAPIDSymbolData(const RAPIDResource& resource, RAPIDSymbolDataAbstract* p_data)
{
  RWSResult result;
  std::string data_type;

  if (p_data)
  {
    RWSResult temp_result = getRAPIDSymbolProperties(resource);

    if (temp_result.success)
    {
      data_type = xmlFindTextContent(temp_result.p_xml_document, XMLAttributes::CLASS_DATTYP);

      if (p_data->getType().compare(data_type) == 0)
      {
        result = getRAPIDSymbolData(resource);

        if (result.success)
        {
          std::string value = xmlFindTextContent(result.p_xml_document, XMLAttributes::CLASS_VALUE);

          if (!value.empty())
          {
            p_data->parseString(value);
          }
          else
          {
            result.success = false;
            result.error_message = "getRAPIDSymbolData(...): RAPID value string was empty";
          }
        }
      }
    }
  }

  return result;
}

RWSClient::RWSResult RWSClient::getRAPIDSymbolProperties(const RAPIDResource& resource)
{
  std::string uri = generateRAPIDPropertiesPath(resource);

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(httpGet(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::setIOSignal(const std::string& iosignal, const std::string& value)
{
  std::string uri = generateIOSignalPath(iosignal) + "?" + Queries::ACTION_SET;
  std::string content = Identifiers::LVALUE + "=" + value;

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = false;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_NO_CONTENT);

  return evaluatePOCOResult(httpPost(uri, content), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::setRAPIDSymbolData(const RAPIDResource& resource, const std::string& data)
{
  std::string uri = generateRAPIDDataPath(resource) + "?" + Queries::ACTION_SET;
  std::string content = Identifiers::VALUE + "=" + data;

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = false;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_NO_CONTENT);

  return evaluatePOCOResult(httpPost(uri, content), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::setRAPIDSymbolData(const RAPIDResource& resource, const RAPIDSymbolDataAbstract& data)
{
  return setRAPIDSymbolData(resource, data.constructString());
}

RWSClient::RWSResult RWSClient::startRAPIDExecution()
{
  std::string uri = Resources::RW_RAPID_EXECUTION + "?" + Queries::ACTION_START;
  std::string content = "regain=continue&execmode=continue&cycle=forever&condition=none&stopatbp=disabled&alltaskbytsp=false";

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = false;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_NO_CONTENT);

  return evaluatePOCOResult(httpPost(uri, content), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::stopRAPIDExecution()
{
  std::string uri = Resources::RW_RAPID_EXECUTION + "?" + Queries::ACTION_STOP;
  std::string content = "stopmode=stop";

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = false;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_NO_CONTENT);

  return evaluatePOCOResult(httpPost(uri, content), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::resetRAPIDProgramPointer()
{
  std::string uri = Resources::RW_RAPID_EXECUTION + "?" + Queries::ACTION_RESETPP;

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = false;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_NO_CONTENT);

  return evaluatePOCOResult(httpPost(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::setMotorsOn()
{
  std::string uri = Resources::RW_PANEL_CTRLSTATE + "?" + Queries::ACTION_SETCTRLSTATE;
  std::string content = "ctrl-state=motoron";

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = false;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_NO_CONTENT);

  return evaluatePOCOResult(httpPost(uri, content), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::setMotorsOff()
{
  std::string uri = Resources::RW_PANEL_CTRLSTATE + "?" + Queries::ACTION_SETCTRLSTATE;
  std::string content = "ctrl-state=motoroff";

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = false;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_NO_CONTENT);

  return evaluatePOCOResult(httpPost(uri, content), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::setSpeedRatio(unsigned int ratio)
{
  if(ratio > 100) throw std::out_of_range("Speed ratio argument out of range (should be 0 <= ratio <= 100)");

  std::stringstream ss;
  ss << ratio;
  if(ss.fail()) throw std::runtime_error(EXCEPTION_CREATE_STRING);

  std::string uri = "/rw/panel/speedratio?action=setspeedratio";
  std::string content = "speed-ratio=" + ss.str();

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = false;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_NO_CONTENT);

  return evaluatePOCOResult(httpPost(uri, content), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::getFile(const FileResource& resource, std::string* p_file_content)
{
  RWSResult rws_result;
  POCOClient::POCOResult poco_result;

  if (p_file_content)
  {
    std::string uri = generateFilePath(resource);
    poco_result = httpGet(uri);

    EvaluationConditions evaluation_conditions;
    evaluation_conditions.parse_message_into_xml = false;
    evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

    rws_result = evaluatePOCOResult(poco_result, evaluation_conditions);

    if (rws_result.success)
    {
      *p_file_content = poco_result.poco_info.http.response.content;
    }
  }

  return rws_result;
}

RWSClient::RWSResult RWSClient::uploadFile(const FileResource& resource, const std::string& file_content)
{
  std::string uri = generateFilePath(resource);
  std::string content = file_content;

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = false;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_CREATED);

  return evaluatePOCOResult(httpPut(uri, content), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::deleteFile(const FileResource& resource)
{
  std::string uri = generateFilePath(resource);

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = false;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_NO_CONTENT);

  return evaluatePOCOResult(httpDelete(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::startSubscription(const SubscriptionResources& resources)
{
  RWSResult result;

  if (!webSocketExist())
  {
    std::vector<SubscriptionResources::SubscriptionResource> temp = resources.getResources();

    // Generate content for a subscription HTTP post request.
    std::stringstream subscription_content;
    for (std::size_t i = 0; i < temp.size(); ++i)
    {
      subscription_content << "resources=" << i
                           << "&"
                           << i << "=" << temp.at(i).resource_uri
                           << "&"
                           << i << "-p=" << temp.at(i).priority
                           << (i < temp.size() - 1 ? "&" : "");
    }

    // Make a subscription request.
    EvaluationConditions evaluation_conditions;
    evaluation_conditions.parse_message_into_xml = false;
    evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_CREATED);
    POCOClient::POCOResult poco_result = httpPost(Services::SUBSCRIPTION, subscription_content.str());
    result = evaluatePOCOResult(poco_result, evaluation_conditions);

    if (result.success)
    {
      std::string poll = "/poll/";
      subscription_group_id_ = findSubstringContent(poco_result.poco_info.http.response.header_info, poll, "\n");
      poll += subscription_group_id_;

      // Create a WebSocket for receiving subscription events.
      EvaluationConditions evaluation_conditions;
      evaluation_conditions.parse_message_into_xml = false;
      evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_SWITCHING_PROTOCOLS);
      result = evaluatePOCOResult(webSocketConnect(poll, "robapi2_subscription", DEFAULT_SUBSCRIPTION_TIMEOUT),
                                  evaluation_conditions);

      if (!result.success)
      {
        subscription_group_id_.clear();
      }
    }
  }

  return result;
}

RWSClient::RWSResult RWSClient::waitForSubscriptionEvent()
{
  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(webSocketReceiveFrame(), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::endSubscription()
{
  RWSResult result;

  if (webSocketExist())
  {
    if (!subscription_group_id_.empty())
    {
      std::string uri = Services::SUBSCRIPTION + "/" + subscription_group_id_;

      EvaluationConditions evaluation_conditions;
      evaluation_conditions.parse_message_into_xml = false;
      evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

      result = evaluatePOCOResult(httpDelete(uri), evaluation_conditions);
    }
  }

  return result;
}

void RWSClient::forceCloseSubscription()
{
  webSocketShutdown();
}

RWSClient::RWSResult RWSClient::logout()
{
  std::string uri = Resources::LOGOUT;

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = true;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);

  return evaluatePOCOResult(httpGet(uri), evaluation_conditions);
}

RWSClient::RWSResult RWSClient::registerLocalUser(const std::string& username,
                                                  const std::string& application,
                                                  const std::string& location)
{
  std::string uri = Services::USERS;
  std::string content = "username=" + username +
             "&application=" + application +
             "&location=" + location +
             "&ulocale=" + SystemConstants::General::LOCAL;

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = false;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_CREATED);

  RWSResult result = evaluatePOCOResult(httpPost(uri, content), evaluation_conditions);

  return result;
}

RWSClient::RWSResult RWSClient::registerRemoteUser(const std::string& username,
                                                   const std::string& application,
                                                   const std::string& location)
{
  std::string uri = Services::USERS;
  std::string content = "username=" + username +
                        "&application=" + application +
                        "&location=" + location +
                        "&ulocale=" + SystemConstants::General::REMOTE;

  EvaluationConditions evaluation_conditions;
  evaluation_conditions.parse_message_into_xml = false;
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_OK);
  evaluation_conditions.accepted_outcomes.push_back(HTTPResponse::HTTP_CREATED);

  RWSResult result = evaluatePOCOResult(httpPost(uri, content), evaluation_conditions);

  return result;
}

/************************************************************
 * Auxiliary methods
 */

RWSClient::RWSResult RWSClient::evaluatePOCOResult(const POCOResult& poco_result,
                                                   const EvaluationConditions& conditions)
{
  RWSResult result;

  checkAcceptedOutcomes(&result, poco_result, conditions);

  if (result.success && conditions.parse_message_into_xml)
  {
    parseMessage(&result, poco_result);
  }

  if (log_.size() >= LOG_SIZE)
  {
    log_.pop_back();
  }
  log_.push_front(poco_result);

  return result;
}

void RWSClient::checkAcceptedOutcomes(RWSResult* result,
                                      const POCOResult& poco_result,
                                      const EvaluationConditions& conditions)
{
  if (result)
  {
    if (poco_result.status == POCOResult::OK && poco_result.exception_message.empty())
    {
      for (size_t i = 0; i < conditions.accepted_outcomes.size() && !result->success; ++i)
      {
        result->success = (poco_result.poco_info.http.response.status == conditions.accepted_outcomes.at(i));
      }

      if (!result->success)
      {
        result->error_message = "checkAcceptedOutcomes(...): RWS response status not accepted";
      }
    }
  }
}

void RWSClient::parseMessage(RWSResult* result, const POCOResult& poco_result)
{
  if (result)
  {
    std::stringstream ss;

    if (!poco_result.poco_info.http.response.content.empty())
    {
      ss << poco_result.poco_info.http.response.content;
    }
    else if (!poco_result.poco_info.websocket.frame_content.empty())
    {
      ss << poco_result.poco_info.websocket.frame_content;
    }
    else
    {
      // XML parsing: Missing message
      result->success = false;
      result->error_message = "parseMessage(...): RWS response was empty";
    }

    if (result->success)
    {
      try
      {
        Poco::XML::InputSource input_source(ss);
        result->p_xml_document = Poco::XML::DOMParser().parse(&input_source);
      }
      catch (...)
      {
        // XML parsing: Failed
        result->success = false;
        result->error_message = "parseMessage(...): XML parser failed to parse RWS response";
      }
    }
  }
}

std::string RWSClient::getLogText(const bool verbose)
{
  if (log_.size() == 0)
  {
    return "";
  }

  std::stringstream ss;

  for (size_t i = 0; i < log_.size(); ++i)
  {
    std::stringstream temp;
    temp << i + 1 << ". ";
    ss << temp.str() << log_[i].toString(verbose, temp.str().size()) << std::endl;
  }

  return ss.str();
}

std::string RWSClient::getLogTextLatestEvent(const bool verbose)
{
  return (log_.size() == 0 ? "" : log_[0].toString(verbose, 0));
}

std::string RWSClient::generateConfigurationPath(const std::string& topic, const std::string& type)
{
  return Resources::RW_CFG + "/" + topic + "/" + type;
}

std::string RWSClient::generateIOSignalPath(const std::string& iosignal)
{
  return Resources::RW_IOSYSTEM_SIGNALS + "/" + iosignal;
}

std::string RWSClient::generateMechanicalUnitPath(const std::string& mechunit)
{
  return Resources::RW_MOTIONSYSTEM_MECHUNITS + "/" + mechunit;
}

std::string RWSClient::generateRAPIDDataPath(const RAPIDResource& resource)
{
  return Resources::RW_RAPID_SYMBOL_DATA_RAPID + "/" + resource.task + "/" + resource.module + "/" + resource.name;
}

std::string RWSClient::generateRAPIDPropertiesPath(const RAPIDResource& resource)
{
  return Resources::RW_RAPID_SYMBOL_PROPERTIES_RAPID + "/" + resource.task + "/" + resource.module + "/"+ resource.name;
}

std::string RWSClient::generateFilePath(const FileResource& resource)
{
  return Services::FILESERVICE + "/" + resource.directory + "/" + resource.filename;
}

} // end namespace rws
} // end namespace abb
