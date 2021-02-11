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

#include <boost/bind.hpp>

#include "abb_libegm/egm_udp_server.h"

namespace abb
{
namespace egm
{
/***********************************************************************************************************************
 * Class definitions: UDPServer
 */

UDPServer::UDPServer(boost::asio::io_service& io_service,
                     unsigned short port_number,
                     AbstractUDPServerInterface* p_interface)
:
initialized_(false),
p_interface_(p_interface)
{
  bool success = true;

  try
  {
    server_data_.port_number = port_number;
    p_socket_.reset(new boost::asio::ip::udp::socket(io_service,
                                                     boost::asio::ip::udp::endpoint(boost::asio::ip::udp::v4(),
                                                                                    port_number)));
  }
  catch (std::exception e)
  {
    success = false;
  }

  if (success)
  {
    initialized_ = true;
    startAsynchronousReceive();
  }
}

UDPServer::~UDPServer()
{
  if (p_socket_)
  {
    p_socket_->close();
    p_socket_.reset();
  }
}

bool UDPServer::isInitialized() const
{
  return initialized_;
}

void UDPServer::startAsynchronousReceive()
{
  if (p_socket_)
  {
    p_socket_->async_receive_from(boost::asio::buffer(receive_buffer_),
                                  remote_endpoint_,
                                  boost::bind(&UDPServer::receiveCallback,
                                              this,
                                              boost::asio::placeholders::error,
                                              boost::asio::placeholders::bytes_transferred));
  }
}

void UDPServer::receiveCallback(const boost::system::error_code& error, const std::size_t bytes_transferred)
{
  server_data_.p_data = receive_buffer_;
  server_data_.bytes_transferred = (int) bytes_transferred;

  if (error == boost::system::errc::success && p_interface_)
  {
    // Process the received data via the callback method (creates the reply message).
    const std::string& reply = p_interface_->callback(server_data_);

    if (!reply.empty() && p_socket_)
    {
      // Send the response message to the robot controller.
      p_socket_->async_send_to(boost::asio::buffer(reply),
                               remote_endpoint_,
                               boost::bind(&UDPServer::sendCallback,
                                           this,
                                           boost::asio::placeholders::error,
                                           boost::asio::placeholders::bytes_transferred));
    }
  }

  // Add another asynchronous operation to the boost io_service object.
  startAsynchronousReceive();
}

void UDPServer::sendCallback(const boost::system::error_code& error, const std::size_t bytes_transferred) {}

} // end namespace egm
} // end namespace abb
