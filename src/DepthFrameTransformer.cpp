#include "DepthFrameTransformer.h"

namespace holovision {

const std::string DepthFrameTransformer::DEFAULT_UV_UNPROJECTION_BIN_PATH = 
  "../src/short_throw_depth_camera_space_projection.bin";

DepthFrameTransformer::DepthFrameTransformer(
  FrameMessage&& frame, 
  const std::string unproj_path) :
  _frame(std::move(frame)),
  _unproj_path(std::move(unproj_path)) {}


DepthFrameTransformer::DepthFrameTransformer(FrameMessage&& frame) : 
  _frame(std::move(frame)), 
  _unproj_path(DepthFrameTransformer::DEFAULT_UV_UNPROJECTION_BIN_PATH) {}


Eigen::MatrixXf DepthFrameTransformer::get_pts_matrix() {
  Eigen::MatrixXf d_pts = *_pts;
  return d_pts;
}

void DepthFrameTransformer::get_points(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr) {
  compute_cameraview_to_world();
  compute_uv_unprojection();
  compute_cameraview_pointcloud();
  compute_world_pointcloud();

  for(auto col = 0; col < _pts->cols(); col++) {
    auto x = (*_pts)(0, col);
    auto y = (*_pts)(1, col);
    auto z = (*_pts)(2, col);

    pcl::PointXYZ p; // Will this point be lost after this function ends?
    p.x = x;
    p.y = y;
    p.z = z;
    cloud_ptr->points.push_back(std::move(p));
  }
}

void DepthFrameTransformer::compute_cameraview_to_world() {
  _cameraview_to_world = (
    _frame.extrinsics.transpose() * 
    _frame.frame_to_origin.transpose().inverse()
  ).inverse();
}

void DepthFrameTransformer::compute_uv_unprojection() {
  std::ifstream bin_stream(_unproj_path, std::ios::binary);
  if (!bin_stream) {
    throw std::runtime_error("Couldn't open bin file");
  }
  // will transpose later
  _u = std::make_unique<Eigen::MatrixXf>(_frame.width, _frame.height);
  _v = std::make_unique<Eigen::MatrixXf>(_frame.width, _frame.height);

  for (auto col = 0; col < _frame.width; col++)
  for (auto row = 0; row < _frame.height; row++) {
    float x;
    float y;
    assert(bin_stream.read(reinterpret_cast<char*>(&x), sizeof x));
    assert(bin_stream.read(reinterpret_cast<char*>(&y), sizeof y));
    (*_u)(col, row) = x;
    (*_v)(col, row) = y;
  }
  _u->transposeInPlace();
  _v->transposeInPlace();
}

void DepthFrameTransformer::compute_cameraview_pointcloud() {
  _Z = std::make_unique<Eigen::MatrixXf>(
    Eigen::MatrixXf::Zero(_frame.height, _frame.width));

  for (auto col = 0; col < _frame.height; col++)
  for (auto row = 0; row < _frame.width; row++) {
    if ((*_frame.d)(col, row) > 64000) // depth filter
      continue;
    (*_Z)(col, row) = -1.0 * (*_frame.d)(col, row) / 
      (sqrt( pow((*_u)(col, row), 2) + pow((*_v)(col, row), 2) + 1 ) * 1000);
  }
  // 3D + 1 column of ones to multiply w/ 4x4 transform matrices
  _pts = std::make_unique<Eigen::MatrixXf>(4, _frame.height * _frame.width);

  auto i = 0;
  for (auto col = 0; col < _frame.height; col++)
  for (auto row = 0; row < _frame.width; row++) {
    (*_pts)(0, i) = (*_Z)(col, row) * (*_u)(col, row);
    (*_pts)(1, i) = (*_Z)(col, row) * (*_v)(col, row);
    (*_pts)(2, i) = (*_Z)(col, row);
    (*_pts)(3, i) = 1.0f;
    i++;
  }
}

void DepthFrameTransformer::compute_world_pointcloud() {
  (*_pts) = (_cameraview_to_world * (*_pts)).eval();
}


} // namespace holovision
