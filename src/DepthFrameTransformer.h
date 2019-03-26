#pragma once

#include "FrameMessage.h"

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>


namespace holovision {

class DepthFrameTransformer {
public:
  static const std::string DEFAULT_UV_UNPROJECTION_BIN_PATH;

  DepthFrameTransformer(FrameMessage&&, const std::string);
  DepthFrameTransformer(FrameMessage&&);

  void get_points(pcl::PointCloud<pcl::PointXYZ>::Ptr);

private:
  const FrameMessage _frame;
  const std::string _unproj_path;
  // transform matrices
  Eigen::Matrix4f _cameraview_to_world;
  // u, v unprojection mappings
  std::unique_ptr<Eigen::MatrixXf> _u;
  std::unique_ptr<Eigen::MatrixXf> _v;
  std::unique_ptr<Eigen::MatrixXf> _Z; 
  std::unique_ptr<Eigen::MatrixXf> _pts;

  void compute_cameraview_to_world();
  void compute_uv_unprojection();
  void compute_cameraview_pointcloud();
  void compute_world_pointcloud();
};

} // namespace holovision
