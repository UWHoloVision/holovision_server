#include "MeshSocket.h"

namespace holovision {

MeshSocket::MeshSocket(std::string ip): 
  _ip(ip), _socket(std::make_unique<boost::asio::ip::tcp::socket>(_io_service)) {}

void MeshSocket::connect() {
  _socket->connect(
    boost::asio::ip::tcp::endpoint(
      boost::asio::ip::address::from_string(_ip),
      9091
    )
  );
}

void MeshSocket::send_mesh(pcl::PolygonMesh::Ptr mesh) {
  boost::system::error_code err;
  boost::asio::streambuf buf;
  std::ostream os(&buf);

  auto mesh_msg = create_mesh_message(mesh);
  write_mesh_message(std::move(mesh_msg), os);
  auto payload_size = 8 + 
    (mesh_msg.n_points * 3 * 4) + 
    (mesh_msg.n_triangles * 3 * 4);
  std::cout << "writing " << payload_size << " bytes" << std::endl;
  boost::asio::write(
    *_socket,
    buf,
    boost::asio::transfer_exactly(payload_size),
    err
  );

  if (err) {
    std::cerr << "Failed sending mesh " << err.message() << std::endl;
  }
}

} // namespace holovision
