#pragma once

#include "MeshMessage.h"

#include <iostream>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <boost/array.hpp>
#include <boost/asio.hpp>

namespace holovision {

// outbound TCP socket
class MeshSocket {
public:
  MeshSocket(std::string);
  void connect();
  void send_mesh(pcl::PolygonMesh::Ptr);
private:
  std::string _ip;
  boost::asio::io_service _io_service;
  std::unique_ptr<boost::asio::ip::tcp::socket> _socket;
};

} // namespace holovision
