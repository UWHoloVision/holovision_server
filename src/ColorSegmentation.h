#pragma once

#include "FrameMessage.h"

#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Dense>


namespace holovision {

class ColorSegmentation {
public:
    // Constructor
    ColorSegmentation();
    // Associate depth points to RGB colors provided in the frame
    void segmentColors(pcl::PointCloud<pcl::PointXYZRGB>::Ptr, pcl::PointCloud<pcl::PointXYZRGB>::Ptr );
    static const std::string MEAN_BALL_PATH;
    static const std::string MEAN_BG_PATH;
    static const std::string CV_BALL_PATH;
    static const std::string CV_BG_PATH;
private:
    // transform matrices
    Eigen::Vector3f mean_ball; 
    Eigen::Matrix3f cv_ball; 
    Eigen::Vector3f mean_bg; 
    Eigen::Matrix3f cv_bg; 
    // Prediction per point: can make this 5xfaster by making it
    // a vector operation, but short of time right now.
    // bool predict(pcl::PointXYZRGB& pt);
};

} // namespace holovision
