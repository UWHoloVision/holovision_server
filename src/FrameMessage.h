#pragma once
#include <iostream>
#include <Eigen/Dense>
#include <cassert>
#include <fstream>

namespace holovision {

struct FrameMessage {
  int64_t frame_id;
  int32_t width;
  int32_t height;
  int32_t bytes_per_point;
  int32_t points_per_pixel;
  // transform matrices
  Eigen::Matrix4f frame_to_origin;
  Eigen::Matrix4f intrinsics;
  Eigen::Matrix4f extrinsics;
  // depth frame
  std::unique_ptr<Eigen::MatrixXi> d;
  // color frame
  std::unique_ptr<Eigen::MatrixXi> r;
  std::unique_ptr<Eigen::MatrixXi> g;
  std::unique_ptr<Eigen::MatrixXi> b;
  
  friend std::ostream& operator<<(std::ostream&, const FrameMessage&);
};

std::ostream& operator<<(std::ostream&, const FrameMessage&);

std::tuple<int32_t, int32_t> read_msg_header(std::istream&);
FrameMessage read_msg_body(std::istream&);
FrameMessage read_msg_from_file(const std::string);


} // namespace holovision
