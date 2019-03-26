#include <iostream>

#include "DepthFrameTransformer.h"
#include "FrameMessage.h"
#include "PointCloudUtils.h"
#include "Visualizer.h"

#include "Dbg.h"


Eigen::Matrix4f world_to_camera_view(Eigen::Matrix4f& frame_to_origin, Eigen::Matrix4f& extrinsics) {
  return extrinsics.transpose() * frame_to_origin.inverse(); // TODO: check
}
void extract_colors(holovision::FrameMessage& c_frame, Eigen::MatrixXf& world_pts, Eigen::Matrix4f& world_to_camera_view) {
  Eigen::MatrixXf camera_view_pv = (world_to_camera_view * world_pts).transpose(); // 4xN
  // reset last col to ones
  auto camera_proj_pv = (c_frame.intrinsics * camera_view_pv).transpose(); // Nx4
}
void color(holovision::FrameMessage& c_frame, Eigen::MatrixXf& world_pts) {
  // world_coord_to_camera_view()
}


int main (int argc, char** argv) {  
  holovision::render_30_depth_frames_as_mesh();
  return (0);
}
