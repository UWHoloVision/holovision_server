#include "FrameMessage.h"

namespace holovision {

std::ostream& operator<<(std::ostream &os, const FrameMessage &fm) {
  os << "frame_id: " << std::to_string(fm.frame_id) << std::endl;
  os << "width: " << std::to_string(fm.width) << std::endl;
  os << "height: " << std::to_string(fm.height) << std::endl;
  os << "bytes/point: " << std::to_string(fm.bytes_per_point) << std::endl;
  os << "points/pixel: " << std::to_string(fm.points_per_pixel) << std::endl;
  os << "frame_to_origin: " << std::endl << fm.frame_to_origin << std::endl;
  os << "intrinsics: " << std::endl << fm.intrinsics << std::endl;
  os << "extrinsics: " << std::endl << fm.extrinsics << std::endl;
  os << "frame type: ";
  if (fm.d) {
    os << "depth" << std::endl;
  }
  else if (fm.r && fm.g && fm.b) {
    os << "color" << std::endl;
  }
  else {
    os << "undefined" << std::endl;
  }
  return os;
}

void fill_matrix4f(std::istream& in, Eigen::Matrix4f& m) {
  for (auto col = 0; col < 4; col++)
  for (auto row = 0; row < 4; row++) {
    float cell;
    assert(in.read(
      reinterpret_cast<char*>(&cell),
      sizeof cell
    ));
    m(col, row) = cell;
  }
}

std::tuple<int32_t, int32_t> read_msg_header(std::istream& in) {
  int32_t header_len;
  assert(in.read(
    reinterpret_cast<char*>(&header_len),
    sizeof header_len
  ));
  int32_t body_len;
  assert(in.read(
    reinterpret_cast<char*>(&body_len),
    sizeof body_len
  ));
  return std::make_tuple(header_len, body_len);
}

FrameMessage read_msg_body(std::istream& in) {
  FrameMessage fm;
  assert(in.read(
    reinterpret_cast<char*>(&(fm.frame_id)),
    sizeof fm.frame_id
  ));
  assert(in.read(
    reinterpret_cast<char*>(&(fm.width)),
    sizeof fm.width
  ));
  assert(in.read(
    reinterpret_cast<char*>(&(fm.height)),
    sizeof fm.height
  ));
  assert(in.read(
    reinterpret_cast<char*>(&(fm.bytes_per_point)),
    sizeof fm.bytes_per_point
  ));
  assert(in.read(
    reinterpret_cast<char*>(&(fm.points_per_pixel)),
    sizeof fm.points_per_pixel
  ));
  fill_matrix4f(in, fm.frame_to_origin);
  fill_matrix4f(in, fm.intrinsics);
  fill_matrix4f(in, fm.extrinsics);
  // depth
  if (fm.bytes_per_point == 2 && fm.points_per_pixel == 1) {
    fm.d = std::make_unique<Eigen::MatrixXi>(fm.height, fm.width);
    for (auto col = 0; col < fm.height; col++)
    for (auto row = 0; row < fm.width; row++) {
      int16_t d;
      assert(in.read(
        reinterpret_cast<char*>(&d),
        sizeof d
      ));
      (*fm.d)(col, row) = d;
    }
  }
  // color
  if (fm.bytes_per_point == 1 && fm.points_per_pixel == 3) {
    fm.r = std::make_unique<Eigen::MatrixXi>(fm.height, fm.width);
    fm.g = std::make_unique<Eigen::MatrixXi>(fm.height, fm.width);
    fm.b = std::make_unique<Eigen::MatrixXi>(fm.height, fm.width);
    for (auto col = 0; col < fm.height; col++)
    for (auto row = 0; row < fm.width; row++) {
      int8_t r, g, b;
      assert(in.read(reinterpret_cast<char*>(&r), sizeof r));
      assert(in.read(reinterpret_cast<char*>(&r), sizeof g));
      assert(in.read(reinterpret_cast<char*>(&r), sizeof b));
      (*fm.r)(col, row) = r;
      (*fm.g)(col, row) = g;
      (*fm.b)(col, row) = b;
    }
  }
  return fm;
}

FrameMessage read_msg_from_file(const std::string path) {
  std::ifstream msg_stream(path, std::ios::binary);
  if (!msg_stream) {
    throw std::runtime_error("Couldn't open file");
  }
  return read_msg_body(msg_stream);
}

} // namespace holovision
