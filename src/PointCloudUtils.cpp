#include "PointCloudUtils.h"

namespace holovision {

void downsample_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) {
  pcl::VoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(pointcloud);
  sor.setLeafSize(0.005f, 0.005f, 0.005f);
  sor.filter(*pointcloud);
}

void downsample_voxel_approx(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) {
  pcl::ApproximateVoxelGrid<pcl::PointXYZ> sor;
  sor.setInputCloud(pointcloud);
  sor.setLeafSize(0.005f, 0.005f, 0.005f);
  sor.filter(*pointcloud);
}

void approx_normals_mls(
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud,
    pcl::PointCloud<pcl::PointNormal>::Ptr normalcloud) {
  // Create a KD-Tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
  mls.setComputeNormals(true);
  mls.setInputCloud(pointcloud);
  mls.setPolynomialOrder(2);
  mls.setSearchMethod(tree);
  mls.setSearchRadius(0.03);
  // Reconstruct
  mls.process(*normalcloud);
}

void approx_normals_kd(
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud, 
    pcl::PointCloud<pcl::PointNormal>::Ptr normalcloud) {
  // estimated normals
  pcl::PointCloud<pcl::Normal>::Ptr est_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> estimator;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pointcloud);
  estimator.setInputCloud(pointcloud);
  estimator.setSearchMethod(tree);
  estimator.setKSearch(40);
  estimator.compute(*est_normals);
  // Concat XYZ and normals
  pcl::concatenateFields(*pointcloud, *est_normals, *normalcloud);
}

void triangulate_greedy(
    pcl::PointCloud<pcl::PointNormal>::Ptr normalcloud, 
    pcl::PolygonMesh::Ptr mesh) {
  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(new pcl::search::KdTree<pcl::PointNormal>);
  tree2->setInputCloud(normalcloud);
  // triangulation params
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  gp3.setSearchRadius(0.025);
  gp3.setMu(2.5);
  gp3.setMaximumNearestNeighbors(100);
  gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
  gp3.setMinimumAngle(M_PI/18); // 10 degrees
  gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
  gp3.setNormalConsistency(false);
  // result
  gp3.setInputCloud(normalcloud);
  gp3.setSearchMethod(tree2);
  gp3.reconstruct(*mesh);
}

void fill_holes(pcl::PolygonMesh::Ptr mesh) {
  vtkSmartPointer<vtkPolyData> input;
  pcl::VTKUtils::mesh2vtk(*mesh, input);
  auto fillHolesFilter = vtkSmartPointer<vtkFillHolesFilter>::New();
  fillHolesFilter->SetInputData(input);
  fillHolesFilter->SetHoleSize(1000.0);
  fillHolesFilter->Update();
  vtkSmartPointer<vtkPolyData> polyData = fillHolesFilter->GetOutput();
  pcl::VTKUtils::vtk2mesh(polyData, *mesh);
}

pcl::PolygonMesh::Ptr pointcloud_to_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud) {
  pcl::PolygonMesh::Ptr mesh(new pcl::PolygonMesh);
  pcl::PointCloud<pcl::PointNormal>::Ptr normalcloud(new pcl::PointCloud<pcl::PointNormal>);
  holovision::downsample_voxel_approx(pointcloud);
  holovision::approx_normals_mls(pointcloud, normalcloud);
  holovision::triangulate_greedy(normalcloud, mesh);
  holovision::fill_holes(mesh);
  return mesh;
}

} // namespace holovision
