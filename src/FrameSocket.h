#pragma once

#include "FrameMessage.h"

#include <iostream>
#include <boost/array.hpp>
#include <boost/asio.hpp>

namespace holovision {

class FrameSocket {
public:
  FrameSocket();
  void connect();
  FrameMessage poll_depth();
private:
  boost::asio::io_service _io_service;
  std::unique_ptr<boost::asio::ip::tcp::socket> _socket;
};

} // namespace holovision
