#include "Dbg.h"

namespace holovision {

// renders depth frames of a room as a mesh
void colorpoints_pipeline() {
  std::vector<std::string> depthframes {
    "262756114425", "262770095130", "262787112869", "262804676458",
    "262821526614", "262841530182", "262861511362", "262882370805",
    "262901224047", "262926316404", "262945139941", "262974089575",
    "262994070791", "263012209872", "263030148604", "263047451348",
    "263069004290", "263086626496", "263115216794", "263131867766",
    "263147084842", "263166075851", "263182736775", "263202717914",
    "263219368928", "263236019877", "263252415245", "263290184514",
    "263311650065", "263327047749"
  };
  std::vector<std::string> colorframes {
    "262754255709", "262768576911", "262785229469", "262802215094",
    "262820865946", "262841515137", "262861165146", "262881148246",
    "262900132159", "262924111809", "262942429637", "262973736463",
    "262991055103", "263011371243", "263028023791", "263047673827",
    "263066990805", "263084975559", "263112951875", "263128605247",
    "263146923090", "263165906983", "263183225665", "263201876554",
    "263216863843", "263233183336", "263248836728", "263289469013",
    "263308785966", "263327103799"
  };
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr agg_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  for (auto i = 0; i < depthframes.size(); i++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::cout << "frame " << i << std::endl;
    // read depth frame
    auto depthframe = depthframes.at(i);
    auto d_msg = holovision::read_msg_from_file("../out/" + depthframe + ".bin");
    holovision::DepthFrameTransformer dft(std::move(d_msg));
    dft.get_points(pointcloud);
    // XYZ -> XYZRGB conversion
    pcl::copyPointCloud(*pointcloud, *colorcloud);
    // read color frame
    auto colorframe = colorframes.at(i);
    auto r_msg = holovision::read_msg_from_file("../out/" + colorframe + ".bin");
    holovision::RGBFrameTransformer rgbft(std::move(r_msg));
    // compute rgbd points
    rgbft.get_RGBD_pts(colorcloud, std::move(dft.get_pts_matrix()));
    std::cout << "completed frame " << i << std::endl;
    // add to agg_cloud
    *agg_cloud += *colorcloud;
  }
  holovision::Visualizer visualizer(agg_cloud);
  visualizer.render();
}

// read frames, then build mesh and send to hololens
void meshsocket_pipeline(int frames) {
  FrameSocket fs;
  fs.connect();
  pcl::PointCloud<pcl::PointXYZ>::Ptr d_pts(new pcl::PointCloud<pcl::PointXYZ>);
  for(auto i = 0; i < frames; i++) {
    std::cout << "frame " << i << std::endl;
    auto d_msg = fs.poll_depth();
    auto rgb_msg = fs.poll_depth(); // ignored
    DepthFrameTransformer dft(std::move(d_msg));
    dft.get_points(d_pts);
  }
  auto mesh = pointcloud_to_mesh(d_pts);
  MeshSocket ms;
  ms.connect();
  ms.send_mesh(mesh);
  // visualize
  holovision::Visualizer visualizer(mesh);
  visualizer.render();
}

void render_30_depth_frames_from_socket() {
  FrameSocket fs;
  fs.connect();
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
  for(auto i = 0; i < 30; i++) {
    std::cout << "frame " << i << std::endl;
    auto d_msg = fs.poll_depth();
    auto rgb_msg = fs.poll_depth();
    holovision::DepthFrameTransformer dft(std::move(d_msg));
    dft.get_points(cloud);
  }
  // auto mesh = holovision::pointcloud_to_mesh(cloud);
  // holovision::Visualizer visualizer(cloud);
  // visualizer.render();
}

} // namespace holovision
