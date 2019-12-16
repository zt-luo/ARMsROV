#pragma once

#include <ctime>
#include <iostream>
#include <string>
#include <thread>

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>

#include "MavlinkHandler.h"

using namespace std;

using namespace boost::asio;
using namespace boost::asio::ip;

extern MavlinkHandler mavlinkHandler;

class UdpServer
{
private:
  std::thread* _thread;

  udp::socket _socket;
  udp::endpoint _remote_endpoint;

  boost::array<char, 128> _recv_buffer;

  void start_receive();
  void handle_receive(const boost::system::error_code& error, size_t bytes_transferred);
  void handle_send(boost::shared_ptr<std::string> message, const boost::system::error_code& error,
                   std::size_t bytes_transferred);

public:
  UdpServer(io_service& io_service);
  ~UdpServer();
};

class UdpClient
{
private:
  io_service _io_service;

  udp::socket _socket;
  udp::endpoint _remote_endpoint;

public:
  UdpClient();
  ~UdpClient();

  void sendData(char* data, size_t len);
};
