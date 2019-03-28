#include "Registration.h"

namespace holovision {

const std::string Registration::SOURCE_CLOUD_PATH = 
  "../src/source_point_cloud_breast.pcd";

Registration::Registration(){
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::io::loadPCDFile (SOURCE_CLOUD_PATH , *cloud_in);
    src = cloud_in;
    std::cout << "Finished setting the source point cloud" << std::endl;
    // Set in tumor cloud here.
}

void Registration::register_points(pcl::PointCloud<pcl::PointXYZRGB>::Ptr tgt){
    // Apply voxelization
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_src (new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_tgt (new pcl::PointCloud<pcl::PointXYZRGB>);

    pcl::VoxelGrid<pcl::PointXYZRGB> grid;
    grid.setLeafSize (0.05, 0.05, 0.05);
    grid.setInputCloud (src);
    grid.filter (*m_src);

    grid.setInputCloud (tgt);
    grid.filter (*m_tgt);

    //remove NAN points from the cloud
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*m_src, *m_src, indices);
    pcl::removeNaNFromPointCloud(*m_tgt, *m_tgt, indices);

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB> icp;
    icp.setInputSource(m_src);
    icp.setInputTarget(m_tgt);
    
    pcl::PointCloud<pcl::PointXYZRGB> Final;
    icp.setMaximumIterations(50);
    std::cout <<"Beginning registration" << std::endl;
    icp.align(Final);

    std::cout << "has converged:" << icp.hasConverged() << " score: " <<
    icp.getFitnessScore() << std::endl;

    _transformation = icp.getFinalTransformation(); // source -> target
}

void Registration::apply_transform(pcl::PointCloud<pcl::PointXYZRGB>::Ptr input, pcl::PointCloud<pcl::PointXYZRGB>::Ptr output){
    pcl::transformPointCloud(*input, *output, _transformation);
}

void Registration::apply_transform_on_source(pcl::PointCloud<pcl::PointXYZRGB>::Ptr output){
    pcl::transformPointCloud(*src, *output, _transformation);
}

} //namespace holovision

