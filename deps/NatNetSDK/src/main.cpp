//
// receiver.cpp
// ~~~~~~~~~~~~
//
// Copyright (c) 2003-2019 Christopher M. Kohlhoff (chris at kohlhoff dot com)
//
// Distributed under the Boost Software License, Version 1.0. (See accompanying
// file LICENSE_1_0.txt or copy at http://www.boost.org/LICENSE_1_0.txt)
//

#include <array>
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <inttypes.h>
#include <stdio.h>

constexpr const char* MULTICAST_ADDRESS = "239.255.42.99";
constexpr int PORT_COMMAND = 1510;
constexpr int PORT_DATA = 1511;
constexpr int MAX_PACKETSIZE = 100000;  // max size of packet (actual packet size is dynamic)

void Unpack(char* pData);
void buildConnectPacket(std::vector<char>& buffer);
void UnpackCommand(char* pData);

using boost::asio::ip::udp;

class receiver
{
public:
  receiver(boost::asio::io_service& io_service,
      const boost::asio::ip::address& listen_address,
      const boost::asio::ip::address& multicast_address)
    : socket_(io_service)
    , sender_endpoint_()
    , data_(20000)
  {
    // Create the socket so that multiple may be bound to the same address.
    boost::asio::ip::udp::endpoint listen_endpoint(
        listen_address, PORT_DATA);
    socket_.open(listen_endpoint.protocol());
    socket_.set_option(boost::asio::ip::udp::socket::reuse_address(true));
    socket_.bind(listen_endpoint);

    // Join the multicast group.
    socket_.set_option(
        boost::asio::ip::multicast::join_group(multicast_address));

    do_receive();
  }

private:
  void do_receive()
  {
    socket_.async_receive_from(
        boost::asio::buffer(data_.data(), data_.size()), sender_endpoint_,
        [this](boost::system::error_code ec, std::size_t /*length*/)
        {
          if (!ec)
          {
            // std::cout.write(data_.data(), length);
            // std::cout << std::endl;
            Unpack(data_.data());

            do_receive();
          } else {
            std::cerr << "async_receive_from error: " << ec.message() << std::endl;
          }
        });
  }

  boost::asio::ip::udp::socket socket_;
  boost::asio::ip::udp::endpoint sender_endpoint_;
  std::vector<char> data_;
};

int main(int argc, char* argv[])
{
  try
  {
    // Connect to command port to query version

    if (argc != 2)
    {
      std::cerr << "Usage: natnettest <host>\n";
      return 1;
    }

    boost::asio::io_service io_service_cmd;

    udp::socket socket_cmd(io_service_cmd, udp::endpoint(udp::v4(), 0));

    udp::resolver resolver_cmd(io_service_cmd);
    udp::endpoint endpoint_cmd = *resolver_cmd.resolve({udp::v4(), argv[1], std::to_string(PORT_COMMAND)});

    std::vector<char> connectCmd;
    buildConnectPacket(connectCmd);
    socket_cmd.send_to(boost::asio::buffer(connectCmd.data(), connectCmd.size()), endpoint_cmd);

    std::vector<char> reply(MAX_PACKETSIZE);
    udp::endpoint sender_endpoint;
    /*size_t reply_length =*/ socket_cmd.receive_from(
        boost::asio::buffer(reply, MAX_PACKETSIZE), sender_endpoint);

    UnpackCommand(reply.data());

    // Listen on multicast address
    boost::asio::io_service io_service;
    receiver r(io_service,
        boost::asio::ip::address::from_string("0.0.0.0"),
        boost::asio::ip::address::from_string(MULTICAST_ADDRESS));
    io_service.run();
  }
  catch (std::exception& e)
  {
    std::cerr << "Exception: " << e.what() << "\n";
  }

  return 0;
}