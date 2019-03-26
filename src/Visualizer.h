#pragma once

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

using namespace pcl::visualization;


namespace holovision {

class Visualizer { 
public:
  Visualizer(pcl::PointCloud<pcl::PointXYZ>::ConstPtr);
  Visualizer(pcl::PolygonMesh::ConstPtr mesh);

  void render();

private:
  PCLVisualizer::Ptr _viewer;

  void config();
};

} // namespace holovision
