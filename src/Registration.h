#pragma once

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <iostream>
#include <pcl/registration/icp.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/filter.h>

#include <pcl/features/normal_3d.h>

#include <pcl/registration/icp_nl.h>
#include <pcl/registration/transforms.h>
#include <Eigen/Dense>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>

#include <boost/make_shared.hpp>

#include <iterator>
#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <string>

namespace holovision {

class Registration {
public:
    // Constructor
    Registration();
    // The real-time point cloud is the target point cloud
    void register_points(pcl::PointCloud<pcl::PointXYZ>::Ptr tgt);
    // applying the transform to the a block of points
    void apply_transform(pcl::PointCloud<pcl::PointXYZ>::Ptr input, pcl::PointCloud<pcl::PointXYZ>::Ptr output);
    // Source cloud path
    void apply_transform_on_source(pcl::PointCloud<pcl::PointXYZ>::Ptr output);

    static const std::string SOURCE_CLOUD_PATH;
    static const std::string TUMOR_1_CLOUD_PATH;
    static const std::string TUMOR_2_CLOUD_PATH;
    // Source tumor path
    // static const std::string SOURCE_TUMOR_PATH;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tumor_1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr tumor_2;
private:
    // Source point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr src; // make this unique pointer later perhaps.
    // Source tumor cloud
    Eigen::Matrix4f _transformation;
};

} // namespace holovision
