#include "Dbg.h"

namespace holovision {

// renders depth frames of a room as a mesh
void render_30_depth_frames_as_mesh() {
  std::vector<std::string> depthframes {
    "262756114425",
    "262770095130",
    "262787112869",
    "262804676458",
    "262821526614",
    "262841530182",
    "262861511362",
    "262882370805",
    "262901224047",
    "262926316404",
    "262945139941",
    "262974089575",
    "262994070791",
    "263012209872",
    "263030148604",
    "263047451348",
    "263069004290",
    "263086626496",
    "263115216794",
    "263131867766",
    "263147084842",
    "263166075851",
    "263182736775",
    "263202717914",
    "263219368928",
    "263236019877",
    "263252415245",
    "263290184514",
    "263311650065",
    "263327047749"
  };
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for (auto depthframe: depthframes) {
    auto d_msg = holovision::read_msg_from_file("../out/" + depthframe + ".bin");
    holovision::DepthFrameTransformer dft(std::move(d_msg));
    dft.get_points(cloud);
  }
  auto mesh = holovision::pointcloud_to_mesh(cloud);
  holovision::Visualizer visualizer(mesh);
  visualizer.render();
}

} // namespace holovision
