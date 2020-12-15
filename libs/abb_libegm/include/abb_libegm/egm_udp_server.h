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

#ifndef EGM_UDP_SERVER_H
#define EGM_UDP_SERVER_H

#include <boost/asio.hpp>

namespace abb
{
namespace egm
{
/**
 * \brief Struct for containing data from the UDPServer class.
 */
struct UDPServerData
{
  /**
   * \brief Default constructor.
   */
  UDPServerData()
  :
  port_number(0),
  bytes_transferred(0)
  {}

  /**
   * \brief Port number for the server's UDP socket.
   */
  size_t port_number;

  /**
   * \brief The received data.
   */
  char* p_data;

  /**
   * \brief Bytes transferred to the server.
   */
  int bytes_transferred;
};

/**
 * \brief Abstract class for a user interface to the UDPServer class.
 */
class AbstractUDPServerInterface
{
/**
 * \brief A friend to the interface.
 */
friend class UDPServer;

private:
  /**
   * \brief Pure virtual method for handling callback requests from a UDPServer instance.
   *
   * \Param server_data containing the UDP server's callback data.
   *
   * \return string& containing the reply.
   */
  virtual const std::string& callback(const UDPServerData& data) = 0;
};

/**
 * \brief Class for an asynchronous UDP server.
 *
 * The server receives UDP messages from a client, passes the messages to a callback and returns a reply to the client.
 */
class UDPServer
{
public:
  /**
   * \brief A constructor.
   *
   * \param io_service for operating boost asio's asynchronous functions.
   * \param port_number for the server's UDP socket.
   * \param p_interface that processes the received messages.
   */
  UDPServer(boost::asio::io_service& io_service,
            unsigned short port_number,
            AbstractUDPServerInterface* p_interface);

  /**
   * \brief A destructor.
   */
  ~UDPServer();

  /**
   * \brief Checks if the server was successfully initialized or not.
   *
   * \return bool true if and only if the server was initialized correctly.
   */
  bool isInitialized() const;

private:
  /**
   * \brief Start an asynchronous receive.
   */
  void startAsynchronousReceive();

  /**
   * \brief Callback for handling an asynchronous receive.
   *
   * \param error for containing an error code.
   * \param bytes_transferred is the number of bytes received.
   */
  void receiveCallback(const boost::system::error_code& error, const std::size_t bytes_transferred);

  /**
   * \brief Callback for handling an asynchronous send.
   *
   * \param error for containing an error code.
   * \param bytes_transferred is the number of bytes transmitted.
   */
  void sendCallback(const boost::system::error_code& error, const std::size_t bytes_transferred);

  /**
   * \brief Static constant for the socket's buffer size.
   */
  static const size_t BUFFER_SIZE = 1024;

  /**
   * \brief The server's UDP socket.
   */
  boost::shared_ptr<boost::asio::ip::udp::socket> p_socket_;

  /**
   * \brief The address of the calling computer (e.g. an ABB robot controller or a virtual controller in RobotStudio).
   */
  boost::asio::ip::udp::endpoint remote_endpoint_;

  /**
   * \brief A buffer for storing the server's serialized inbound messages (i.e. the robot's outbound messages).
   */
  char receive_buffer_[BUFFER_SIZE];

  /**
   * \brief Pointer to an object that is derived from AbstractUDPSeverInterface, which processes the received messages.
   */
  AbstractUDPServerInterface* p_interface_;

  /**
   * \brief Container for server data.
   */
  UDPServerData server_data_;

  /**
   * \brief Flag indicating if the server was initialized successfully or not.
   */
  bool initialized_;
};

} // end namespace egm
} // end namespace abb

#endif // EGM_UDP_SERVER_H
