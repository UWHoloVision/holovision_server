#include "Visualizer.h"


namespace holovision {

Visualizer::Visualizer(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointcloud): 
    _viewer(new PCLVisualizer("Point Cloud Viewer")) {
  config();
  _viewer->addPointCloud<pcl::PointXYZ>(pointcloud, "point cloud");
  _viewer->setPointCloudRenderingProperties(
    pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "point cloud");
}

Visualizer::Visualizer(pcl::PolygonMesh::ConstPtr mesh):
    _viewer(new PCLVisualizer("Mesh Viewer")) {
  config();
  _viewer->addPolygonMesh(*mesh, "polygon");
}

void Visualizer::render() {
  while(!_viewer->wasStopped()) {
    _viewer->spinOnce(100);
    boost::this_thread::sleep(boost::posix_time::microseconds(100000));
  }
}

void Visualizer::config() {
  _viewer->setBackgroundColor (0, 0, 0);
  _viewer->addCoordinateSystem (0.6);
  _viewer->initCameraParameters ();
}

} // namespace holovision