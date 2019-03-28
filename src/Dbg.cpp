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
    // apply filtering
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filter_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    color_segmentor.segmentColors(colorcloud, filter_cloud);
    std::cout << "Segmentation done" << std::endl;
    std::cout << colorcloud->points.size() << std::endl;
    std::cout << "Done" << std::endl;
    std::cout << filter_cloud->points.size() << std::endl;
    // Aggregate clouds
    *agg_cloud += *filter_cloud;
  }

  // Add registration
  holovision::Registration registration;
  pcl::PointCloud<pcl::PointXYZ>::Ptr agg_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::copyPointCloud(*agg_cloud, *agg_cloud_xyz);
  registration.register_points(agg_cloud_xyz);
  std::cout << "done registering clouds" <<std::endl;

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr merged_clouds(new pcl::PointCloud<pcl::PointXYZRGB>);
  // Apply on source: ONlY FOR VISUALIZATION PURPOSES: turn it off for demo
  // registration.apply_transform_on_source(merged_clouds);

  // Apply on tumors
  pcl::PointCloud<pcl::PointXYZ>::Ptr tumor_1_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr tumor_2_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  registration.apply_transform(registration.tumor_1, tumor_1_cloud);
  std::cout << registration.tumor_1->points.size()<<std::endl;
  registration.apply_transform(registration.tumor_2, tumor_2_cloud);

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tumor_1_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(pcl::PointXYZ xyz: tumor_1_cloud->points){
      pcl::PointXYZRGB pt(255, 255, 255);
        pt.x = xyz.x;
        pt.y = xyz.y;
        pt.z = xyz.z;
      tumor_1_cloud_rgb->points.push_back(std::move(pt));
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr tumor_2_cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  for(pcl::PointXYZ xyz: tumor_2_cloud->points){
      pcl::PointXYZRGB pt(255, 255, 255);
        pt.x = xyz.x;
        pt.y = xyz.y;
        pt.z = xyz.z;
        tumor_2_cloud_rgb->points.push_back(std::move(pt));
  }

  *merged_clouds += *agg_cloud;
  *merged_clouds += *tumor_1_cloud_rgb;
  *merged_clouds += *tumor_2_cloud_rgb;
  // REMOVE ONE OFF
  merged_clouds->height = 1;
  merged_clouds->width = agg_cloud->points.size();

  // pcl::io::savePCDFileASCII ("breast_source_cloud.pcd", *agg_cloud);
  // pcl::io::savePLYFile("breast_source_cloud.ply", *agg_cloud);
  // std::cout<<"Done saving point cloud"<<std::endl;

  // Apply visualization
  holovision::Visualizer visualizer(merged_clouds);
  visualizer.render();
}

// read frames, then build mesh and send to hololens
void meshsocket_pipeline(int n_frames) {
  FrameSocket fs;
  fs.connect();
  pcl::PointCloud<pcl::PointXYZ>::Ptr agg_pts(new pcl::PointCloud<pcl::PointXYZ>);
  for(auto i = 0; i < n_frames; i++) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr d_pts(new pcl::PointCloud<pcl::PointXYZ>);
    // read n frames
    std::cout << "frame " << i << std::endl;
    auto d_msg = fs.poll_depth();
    auto rgb_msg = fs.poll_depth(); // ignored
    DepthFrameTransformer dft(std::move(d_msg));
    // add to pt cloud
    dft.get_points(d_pts);
    downsample_voxel_approx(d_pts); // to help w/ unity limit
    if (d_pts->size() + agg_pts->size() > 65534) {
      break; // unity limit for mesh
    }
    *agg_pts += *d_pts;
  }
  auto mesh = pointcloud_to_mesh(agg_pts);
  MeshSocket ms;
  ms.connect();
  // write mesh
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
  auto mesh = holovision::pointcloud_to_mesh(cloud);
  holovision::Visualizer visualizer(cloud);
  visualizer.render();
}

} // namespace holovision
