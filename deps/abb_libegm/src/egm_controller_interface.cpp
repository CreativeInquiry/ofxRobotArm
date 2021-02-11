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

#include "abb_libegm/egm_common_auxiliary.h"
#include "abb_libegm/egm_controller_interface.h"

namespace abb
{
namespace egm
{
/***********************************************************************************************************************
 * Class definitions: EGMControllerInterface::ControllerMotion
 */

// See https://stackoverflow.com/questions/16957458/static-const-in-c-class-undefined-reference/16957554
const unsigned int EGMControllerInterface::ControllerMotion::WRITE_TIMEOUT_MS;

/************************************************************
 * Primary methods
 */

void EGMControllerInterface::ControllerMotion::initialize(const bool first_message)
{
  if (first_message)
  {
    boost::lock_guard<boost::mutex> lock(read_mutex_);
    boost::lock_guard<boost::mutex> lock2(write_mutex_);

    read_data_ready_ = false;
    write_data_ready_ = false;
  }
}

void EGMControllerInterface::ControllerMotion::writeInputs(const wrapper::Input& inputs)
{
 boost::lock_guard<boost::mutex> lock(read_mutex_);

 inputs_.CopyFrom(inputs);

 read_data_ready_ = true;
 read_condition_variable_.notify_all();
}

void EGMControllerInterface::ControllerMotion::readOutputs(wrapper::Output* p_outputs)
{
  bool timed_out = false;

  boost::unique_lock<boost::mutex> lock(write_mutex_);

  while (!write_data_ready_ && !timed_out)
  {
    timed_out = !write_condition_variable_.timed_wait(lock, boost::posix_time::milliseconds(WRITE_TIMEOUT_MS));
  }

  if (!timed_out && p_outputs)
  {
    copyPresent(p_outputs, outputs_);
    write_data_ready_ = false;
  }
}

bool EGMControllerInterface::ControllerMotion::waitForMessage(const unsigned int timeout_ms)
{
  boost::unique_lock<boost::mutex> lock(read_mutex_);

  bool timed_out = false;

  while (!read_data_ready_ && !timed_out)
  {
    if (timeout_ms <= 0)
    {
      read_condition_variable_.wait(lock);
    }
    else
    {
      timed_out = !read_condition_variable_.timed_wait(lock, boost::posix_time::milliseconds(timeout_ms));
    }
  }

  return !timed_out;
}

void EGMControllerInterface::ControllerMotion::readInputs(wrapper::Input* p_inputs)
{
  boost::lock_guard<boost::mutex> lock(read_mutex_);

  p_inputs->CopyFrom(inputs_);
  read_data_ready_ = false;
}

void EGMControllerInterface::ControllerMotion::writeOutputs(const wrapper::Output& outputs)
{
  boost::lock_guard<boost::mutex> lock(write_mutex_);

  outputs_.CopyFrom(outputs);

  write_data_ready_ = true;
  write_condition_variable_.notify_all();
}




/***********************************************************************************************************************
 * Class definitions: EGMControllerInterface
 */

/************************************************************
 * Primary methods
 */

EGMControllerInterface::EGMControllerInterface(boost::asio::io_service& io_service,
                                               const unsigned short port_number,
                                               const BaseConfiguration& configuration)
:
EGMBaseInterface(io_service, port_number, configuration)
{
  if (configuration_.active.use_logging)
  {
    std::stringstream ss;
    ss << "port_" << port_number << +"_log.csv";
    p_logger_.reset(new EGMLogger(ss.str()));
  }
}

const std::string& EGMControllerInterface::callback(const UDPServerData& server_data)
{
  // Initialize the callback by:
  // - Parsing and extracting data from the received message.
  // - Updating any pending configuration changes.
  // - Preparing the outputs.
  if (initializeCallback(server_data))
  {
    // Additional initialization for direct motion references.
    controller_motion_.initialize(inputs_.isFirstMessage());

    // Handle demo execution or external controller execution.
    if (configuration_.active.use_demo_outputs)
    {
      outputs_.generateDemoOutputs(inputs_);
    }
    else
    {
      // Make the current inputs available (to the external control loop), and notify that it is available.
      controller_motion_.writeInputs(inputs_.current());

      if (inputs_.isFirstMessage() || inputs_.statesOk())
      {
        // Wait for new outputs (from the external control loop), or until a timeout occurs.
        controller_motion_.readOutputs(&outputs_.current);
      }
    }

    // Log inputs and outputs, if set to do so.
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
 * User interaction methods
 */

bool EGMControllerInterface::waitForMessage(const unsigned int timeout_ms)
{
  return controller_motion_.waitForMessage(timeout_ms);
}

void EGMControllerInterface::read(wrapper::Input* p_inputs)
{
  controller_motion_.readInputs(p_inputs);
}

void EGMControllerInterface::write(const wrapper::Output& outputs)
{
  controller_motion_.writeOutputs(outputs);
}

} // end namespace egm
} // end namespace abb
