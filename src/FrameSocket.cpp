#include "FrameSocket.h"

namespace holovision {

FrameSocket::FrameSocket(): 
  _socket(std::make_unique<boost::asio::ip::tcp::socket>(_io_service)) {}

void FrameSocket::connect() {
  _socket->connect(
    boost::asio::ip::tcp::endpoint(
      boost::asio::ip::address::from_string("192.168.0.102"),
      9090
    )
  );
}

FrameMessage FrameSocket::poll_depth() {
  boost::system::error_code header_err;
  boost::asio::streambuf header_buf;
  boost::asio::read(
    *_socket, header_buf, 
    boost::asio::transfer_exactly(8), header_err);
  if (header_err) {
    std::cerr << "Failed reading header " << header_err.message() << std::endl;
  }
  std::istream header_in(&header_buf);
  auto header_tup = read_msg_header(header_in);

  boost::system::error_code body_err;
  boost::asio::streambuf body_buf;
  boost::asio::read(
    *_socket, body_buf, 
    boost::asio::transfer_exactly(std::get<1>(header_tup)), body_err);
  if (body_err) {
    std::cerr << "Failed reading body " << body_err.message() << std::endl;
  }
  std::istream body_in(&body_buf);
  return read_msg_body(body_in);
}

} // namespace holovision
