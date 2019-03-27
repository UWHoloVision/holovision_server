#pragma once

#include "FrameMessage.h"

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>


namespace holovision {

class RGBFrameTransformer {
public:
    // Constructor
    RGBFrameTransformer(FrameMessage&&);
    // Associate depth points to RGB colors provided in the frame
    void get_RGBD_pts(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, Eigen::MatrixXf&&);
private:
    // The associated RGB frame
    const FrameMessage _frame;
    // transform matrices
    Eigen::Matrix4f _world_to_camera_view;
    Eigen::Matrix4f _camera_projection;
    std::unique_ptr<Eigen::MatrixXf> _colors;
    void compute_world_to_cameraview();
};

} // namespace holovision
