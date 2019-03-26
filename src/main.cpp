#include <iostream>

#include "DepthFrameTransformer.h"
#include "FrameMessage.h"

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/surface/mls.h>

#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <vtkFillHolesFilter.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

// depth transforms
// TODO: put in class
Eigen::Matrix4f camera_view_to_world_coord(Eigen::Matrix4f& frame_to_origin, Eigen::Matrix4f& extrinsics) {
  return (extrinsics.transpose() * frame_to_origin.transpose().inverse()).inverse();
}

std::tuple<Eigen::MatrixXf, Eigen::MatrixXf>
  read_projection_bin_from_file(const std::string path, int width, int height) {

  std::ifstream bin_stream(path, std::ios::binary);
  if (!bin_stream) {
    throw std::runtime_error("Couldn't open bin file");
  }
  // transposed
  Eigen::MatrixXf u_proj = Eigen::MatrixXf::Zero(width, height);
  Eigen::MatrixXf v_proj = Eigen::MatrixXf::Zero(width, height);

  for (auto col = 0; col < width; col++)
  for (auto row = 0; row < height; row++) {
    float x;
    float y;
    assert(bin_stream.read(reinterpret_cast<char*>(&x), sizeof x));
    assert(bin_stream.read(reinterpret_cast<char*>(&y), sizeof y));
    u_proj(col, row) = x;
    v_proj(col, row) = y;
  }
  u_proj.transposeInPlace();
  v_proj.transposeInPlace();
  // auto c = 0;
  // for (auto row = 0; row < height; row++)
  // for (auto col = 0; col < width; col++) {
  //   auto x = u_proj(row, col);
  //   if (!isnan(x) && !isinf(x)) {
  //     if (col == 28) {
  //       std::cout << row << " " << col << " " << x << std::endl;
  //     }
  //     c++;
  //   }
  // }
  /*
  (row, col, val); sorted by col
  (212, 28, 1.3967143), 
  (213, 28, 1.3962201), 
  (214, 28, 1.3957561), 
  (215, 28, 1.3953221), 
  (216, 28, 1.3949186), 
  (217, 28, 1.394545), 
  (218, 28, 1.3942004), 
  (219, 28, 1.3938868), 
  (220, 28, 1.3936028)...
  */
  // std::cout << "u_values not inf/nan: " << c << std::endl;
  return std::forward_as_tuple(std::move(u_proj), std::move(v_proj));
}

Eigen::MatrixXf pointcloud_camera_view(
  const holovision::FrameMessage& fm, 
  Eigen::MatrixXf& u_proj, 
  Eigen::MatrixXf& v_proj) {

  Eigen::MatrixXf Z = Eigen::MatrixXf::Zero(fm.height, fm.width);

  for (auto col = 0; col < fm.height; col++)
  for (auto row = 0; row < fm.width; row++) {
    if ((*fm.d)(col, row) > 64000) 
      continue;
    Z(col, row) = -1.0 * (*fm.d)(col, row) / 
      (sqrt( pow(u_proj(col, row), 2) + pow(v_proj(col, row), 2) + 1 ) * 1000);
  }
  assert(u_proj.rows() == fm.height);
  assert(u_proj.cols() == fm.width);
  Eigen::MatrixXf pts(4, fm.height * fm.width);

  auto i = 0;
  for (auto col = 0; col < fm.height; col++)
  for (auto row = 0; row < fm.width; row++) {
    pts(0, i) = Z(col, row) * u_proj(col, row);
    pts(1, i) = Z(col, row) * v_proj(col, row);
    pts(2, i) = Z(col, row);
    pts(3, i) = 1; // for mapping to world?
    // if (i >= 13212 && i <= 13214) {
    //   std::cout << i << " " << pts(0, i) << " " << pts(1, i) << " " << pts(2, i) << " " << pts(3, i) << std::endl;
    // }
    i++;
  }
  return pts;
/*[
  (13212, array([-0.01394946,  0.68715318, -0.49842444])), 
  (13213, array([-0.01037235,  0.68636667, -0.49789277])), 
  (13214, array([-0.00681136,  0.68638304, -0.49793165]))
  ]*/
}

Eigen::MatrixXf camera_view_to_world(Eigen::MatrixXf& camera_view_points, Eigen::Matrix4f& camera_view_to_world) {
  Eigen::MatrixXf transformed = camera_view_to_world * camera_view_points;
  // drop ones col?
  return transformed; // TODO: check
}

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


pcl::visualization::PCLVisualizer::Ptr simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (0.8);
  viewer->initCameraParameters ();
  return (viewer);
}

pcl::visualization::PCLVisualizer::Ptr simpleVisTri(pcl::PolygonMesh& tris)
{
  using namespace pcl::visualization;
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  // viewer->addPointCloud<pcl::PointXYZ> (cloud, "sample cloud");
  // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");

  viewer->addPolygonMesh(tris, "polygon");
  viewer->addCoordinateSystem (0.6);
  viewer->initCameraParameters ();
  return (viewer);
}

void voxelize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr) {
  // Create the filtering object
  // pcl::VoxelGrid<pcl::PointXYZ> sor;
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud (basic_cloud_ptr);
  // sor.setLeafSize (0.01f, 0.01f, 0.01f);
  sor.setLeafSize (0.005f, 0.005f, 0.005f);
  sor.filter (*cloud);
}

void mls_approx_normals(pcl::PointCloud<pcl::PointNormal>::Ptr normals, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);

  // Init object (second point type is for the normals, even if unused)
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
 
  mls.setComputeNormals (true);

  // Set parameters
  mls.setInputCloud (cloud);
  mls.setPolynomialOrder (2);
  mls.setSearchMethod (tree);
  mls.setSearchRadius (0.03);
  // Reconstruct
  mls.process(*normals);
}

void approx_normals(pcl::PointCloud<pcl::PointNormal>::Ptr normal_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  // est normals
  pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud (cloud);
  n.setInputCloud (cloud);
  n.setSearchMethod (tree);
  // n.setKSearch(20);
  n.setKSearch(40);
  n.compute (*normals);
  //* normals should not contain the point normals + surface curvatures

  // Concatenate the XYZ and normal fields*
  pcl::concatenateFields (*cloud, *normals, *normal_cloud);
  //* cloud_with_normals = cloud + normals
}

void visualize(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {
  // est normals
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals (new pcl::PointCloud<pcl::PointNormal>);
  // approx_normals(cloud_with_normals, cloud);
  mls_approx_normals(cloud_with_normals, cloud);

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2 (new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud (cloud_with_normals);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;

  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius (0.025);

  // Set typical values for the parameters
  gp3.setMu (2.5);
  gp3.setMaximumNearestNeighbors (100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);

// from forum
  // gp3.setSearchRadius (0.5);
  // // Set typical values for the parameters
  // gp3.setMu (2.5);
  // gp3.setMaximumNearestNeighbors (100);
  // gp3.setMaximumSurfaceAngle(M_PI);
  // gp3.setMinimumAngle(M_PI/4);
  // gp3.setMaximumAngle(M_PI/2.0);

  // Get result
  gp3.setInputCloud (cloud_with_normals);
  gp3.setSearchMethod (tree2);
  gp3.reconstruct (triangles);

  // trying hole filling
  vtkSmartPointer<vtkPolyData> input;
  pcl::VTKUtils::mesh2vtk(triangles,input);
  vtkSmartPointer<vtkFillHolesFilter> fillHolesFilter =
      vtkSmartPointer<vtkFillHolesFilter>::New();
  fillHolesFilter->SetInputData(input);
  fillHolesFilter->SetHoleSize(1000.0);
  fillHolesFilter->Update ();
  vtkSmartPointer<vtkPolyData> polyData = fillHolesFilter->GetOutput();
  pcl::VTKUtils::vtk2mesh(polyData,triangles); 

  // viewer time
  auto viewer = simpleVisTri(triangles);
  while (!viewer->wasStopped ())
  {
    viewer->spinOnce(100);
    boost::this_thread::sleep (boost::posix_time::microseconds (100000));
  }
}

void to_point_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr, Eigen::MatrixXf& pts) {
  for(auto col = 0; col < pts.cols(); col++) {
    pcl::PointXYZ basic_point;
    if (isnan(pts(0, col)) || isinf(pts(0, col)))
      continue;
    if (isnan(pts(1, col)) || isinf(pts(1, col)))
      continue;
    if (isnan(pts(2, col)) || isinf(pts(2, col)))
      continue;
    basic_point.x = pts(0, col);
    basic_point.y = pts(1, col);
    basic_point.z = pts(2, col);
    // std::cout << basic_point << std::endl;
    basic_cloud_ptr->points.push_back(basic_point);
  }
}

int main (int argc, char** argv) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  
  std::vector<std::string> paths {
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  for (auto path: paths) {
    auto d_msg = holovision::read_msg_from_file("../out/" + path + ".bin");
    holovision::DepthFrameTransformer dft(std::move(d_msg));
    dft.get_points(basic_cloud_ptr);
  }
  voxelize(cloud, basic_cloud_ptr);
  visualize(cloud);
  return (0);
}
