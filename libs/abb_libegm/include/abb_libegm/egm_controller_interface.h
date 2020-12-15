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

#ifndef EGM_CONTROLLER_INTERFACE_H
#define EGM_CONTROLLER_INTERFACE_H

#include "egm_base_interface.h"

namespace abb
{
namespace egm
{
/**
 * \brief Class for an EGM contoller user interface.
 *
 * The class provides behavior for motion control, and this includes:
 * - Processing asynchronous callbacks from an UDP server.
 * - Notifying an external control loop about new messages.
 * - Incorporating external control loop inputs, and sending them to the robot controller.
 *
 * Pseudocode for the usage of the class methods (inside an external control loop);
 * 1. if(waitForMessage(...))
 * 1.1. read(...)
 * 1.2. write(...)
 * 1.3. Repeat from 1.
 */
class EGMControllerInterface : public EGMBaseInterface
{
public:
  /**
   * \brief A constructor.
   *
   * \param io_service for operating boost asio's asynchronous functions.
   * \param port_number for the server's UDP socket.
   * \param configuration for the interface's configuration.
   */
  EGMControllerInterface(boost::asio::io_service& io_service,
                         const unsigned short port_number,
                         const BaseConfiguration& configuration = BaseConfiguration());

  /**
   * \brief Wait for the next EGM message.
   *
   * \param timeout_ms for specifying a timeout in [ms]. If omitted, then the method waits forever.
   *
   * \return bool indicating if the wait was successful or not. I.e. returns false if a timeout has occurred.
   */
  bool waitForMessage(const unsigned int timeout_ms = 0);

  /**
   * \brief Read EGM inputs received from the robot controller.
   *
   * \param p_inputs for containing the inputs.
   */
  void read(wrapper::Input* p_inputs);

  /**
   * \brief Write EGM outputs to send to the robot controller.
   *
   * \param outputs containing the outputs.
   */
  void write(const wrapper::Output& outputs);

private:
  /**
   * \brief Class for managing controller motion data, between the inner loop and an external control loop.
   */
  class ControllerMotion
  {
  public:
    /**
     * \brief Default constructor.
     */
    ControllerMotion() : read_data_ready_(false), write_data_ready_(false) {}

    /**
     * \brief Initialize the motion data for a new communication session.
     *
     * \param first_message indicating if it is the first message in a communication session.
     */
    void initialize(const bool first_message);

    /**
     * \brief Wait for the next message.
     *
     * \param timeout_ms for specifying a timeout in [ms]. If omitted, then the method waits forever.
     *
     * \return bool indicating if the wait was successful or not. I.e. returns false if a timeout has occurred.
     */
    bool waitForMessage(const unsigned int timeout_ms);

    /**
     * \brief Write the current inputs (from the inner loop, to the intermediate storage).
     *
     * \param inputs for containing the inputs.
     */
    void writeInputs(const wrapper::Input& inputs);

    /**
     * \brief Read the current inputs (from the intermediate storage, to the external loop).
     *
     * \param p_inputs for containing the inputs.
     */
    void readInputs(wrapper::Input* p_inputs);

    /**
     * \brief Write the current outputs (from the external loop, to the intermediate storage).
     *
     * \param outputs for containing the outputs.
     */
    void writeOutputs(const wrapper::Output& outputs);

    /**
     * \brief Read the current outputs (from the intermediate storage, to the inner loop).
     *
     * \param p_outputs for containing the outputs.
     */
    void readOutputs(wrapper::Output* p_outputs);

  private:
    /**
     * \brief Static constant timeout [ms] for waiting on external control loop inputs.
     */
    static const unsigned int WRITE_TIMEOUT_MS = 24;

    /**
     * \brief Mutex for protecting read data.
     */
    boost::mutex read_mutex_;

    /**
     * \brief Mutex for protecting write data.
     */
    boost::mutex write_mutex_;

    /**
     * \brief Condition variable for waiting on read data.
     */
    boost::condition_variable read_condition_variable_;

    /**
     * \brief Condition variable for waiting on write data.
     */
    boost::condition_variable write_condition_variable_;

    /**
     * \brief Flag indicating if read data is ready.
     */
    bool read_data_ready_;

    /**
     * \brief Flag indicating if write data is ready.
     */
    bool write_data_ready_;

    /**
     * \brief Container for the inputs received from the robot controller.
     */
    wrapper::Input inputs_;

    /**
     * \brief Container for the outputs to send to the robot controller.
     */
    wrapper::Output outputs_;
  };

  /**
   * \brief Handle callback requests from an UDP server.
   *
   * \param server_data containing the UDP server's callback data.
   *
   * \return string& containing the reply.
   */
  const std::string& callback(const UDPServerData& server_data);

  /**
   * \brief The interface's controller motion data (between internal loop and external controller loop).
   */
  ControllerMotion controller_motion_;
};

} // end namespace egm
} // end namespace abb

#endif // EGM_CONTROLLER_INTERFACE_H
