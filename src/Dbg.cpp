#include "Dbg.h"

namespace holovision {

std::vector<std::string> glob_snapshots(const std::string& pattern) {
  // glob struct resides on the stack
  glob_t glob_result;
  memset(&glob_result, 0, sizeof(glob_result));
  // do the glob operation
  int return_value = glob(pattern.c_str(), GLOB_TILDE, NULL, &glob_result);
  if(return_value != 0) {
    globfree(&glob_result);
    std::stringstream ss;
    ss << "glob() failed with return_value " << return_value << endl;
    throw std::runtime_error(ss.str());
  }
  // collect all the filenames into a std::list<std::string>
  std::vector<std::string> filenames;
  for(size_t i = 0; i < glob_result.gl_pathc; ++i) {
    auto filename = std::string(glob_result.gl_pathv[i]);
    // replace extension
    filename.replace(filename.end()-3, filename.end(), "bin");
    filenames.push_back(filename);
  }
  // cleanup
  globfree(&glob_result);
  return filenames;
}

std::vector<std::string> get_depthframe_files() {
  return glob_snapshots("../out/*.pgm");
}
std::vector<std::string> get_colorframe_files() {
  return glob_snapshots("../out/*.ppm");
}

// renders depth frames of a room as a mesh
void colorpoints_pipeline() {
  std::vector<std::string> depthframes = get_depthframe_files();
  std::vector<std::string> colorframes = get_colorframe_files();

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr agg_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  holovision::ColorSegmentation color_segmentor;
  for (auto i = 0; i < depthframes.size(); i++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorcloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::cout << "frame " << i << std::endl;
    // read depth frame
    auto depthframe = depthframes.at(i);
    auto d_msg = holovision::read_msg_from_file(depthframe);
    holovision::DepthFrameTransformer dft(std::move(d_msg));
    
    // Apply depth transform
    dft.get_points(pointcloud);
    // read color frame
    auto colorframe = colorframes.at(i);
    auto r_msg = holovision::read_msg_from_file(colorframe);
    holovision::RGBFrameTransformer rgbft(std::move(r_msg));
    
    // compute rgbd points
    rgbft.get_RGBD_pts(colorcloud, pointcloud, std::move(dft.get_pts_matrix()));
    // add to agg_cloud
    *agg_cloud += *filtercloud;
  }
  holovision::Visualizer visualizer(agg_cloud);
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
  auto mesh = holovision::pointcloud_to_mesh(cloud);
  holovision::Visualizer visualizer(cloud);
  visualizer.render();
}

} // namespace holovision
