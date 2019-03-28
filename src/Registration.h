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




namespace holovision {

class Registration {
public:
    // Constructor
    Registration();
    // The real-time point cloud is the target point cloud
    void register_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt);
    // applying the transform to the a block of points
    void apply_transform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);
    // Source cloud path
    void apply_transform_on_source(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output);

    static const std::string SOURCE_CLOUD_PATH;
    // Source tumor path
    // static const std::string SOURCE_TUMOR_PATH;
private:
    // Source point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr src; // make this unique pointer later perhaps.
    // Source tumor cloud
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr tumor;
    Eigen::Matrix4f _transformation;
};

} // namespace holovision
