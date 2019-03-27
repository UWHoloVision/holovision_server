#include "Dbg.h"

namespace holovision {

// renders depth frames of a room as a mesh
void colorpoints_pipeline() {
  std::vector<std::string> depthframes {
  "7353594134873", "7353609438210", "7353626165327", "7353645098646",
  "7353661784886", "7353678429122", "7353695486386", "7353712914172",
  "7353729474088", "7353748941345", "7353769005952", "7353788343047",
  "7353804261904", "7353819644851", "7353836920067", "7353853635836",
  "7353872780169", "7353890789226", "7353905104548", "7353923892730",
  "7353940250858", "7353961500827", "7353976907693", "7353991613922",
  "7354008051477", "7354025476026", "7354044877011", "7354063673988",
  "7354082332509", "7354111135316"
  };
  std::vector<std::string> colorframes {
  "7353593051478", "7353608371830", "7353625357424", "7353644341342",
  "7353660993939", "7353676980357", "7353693632947", "7353710951601",
  "7353727937215", "7353747920266", "7353767570313", "7353787220327",
  "7353803206759", "7353818527149", "7353835845791", "7353852498340",
  "7353872148377", "7353889133967", "7353905120451", "7353923105213",
  "7353939091659", "7353960073886", "7353975727314", "7353989382385",
  "7354006701042", "7354024352774", "7354043669724", "7354062653666",
  "7354082636726", "7354110279985"
  };
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr agg_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
  holovision::ColorSegmentation color_segmentor;
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
