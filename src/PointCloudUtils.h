#pragma once

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

// utils for working with point clouds
namespace holovision {

  void downsample_voxel(pcl::PointCloud<pcl::PointXYZ>::Ptr);
  
  void downsample_voxel_approx(pcl::PointCloud<pcl::PointXYZ>::Ptr);

  void approx_normals_mls(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr);

  void approx_normals_kd(pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointNormal>::Ptr);

  void triangulate_greedy(pcl::PointCloud<pcl::PointNormal>::Ptr, pcl::PolygonMesh::Ptr);

  void fill_holes(pcl::PolygonMesh::Ptr);

  pcl::PolygonMesh::Ptr pointcloud_to_mesh(pcl::PointCloud<pcl::PointXYZ>::Ptr);
}
