#include "RGBFrameTransformer.h"

namespace holovision {

RGBFrameTransformer::RGBFrameTransformer(
    FrameMessage&& frame
): _frame(std::move(frame)), _camera_projection(frame.intrinsics){}

void RGBFrameTransformer::compute_world_to_cameraview(){
    _world_to_camera_view = _frame.extrinsics.transpose() *
        _frame.frame_to_origin.transpose().inverse();
}

void RGBFrameTransformer::get_RGBD_pts(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud, Eigen::MatrixXf&& world_pts){
    compute_world_to_cameraview();

    Eigen::MatrixXf camera_view_pts = (
        _world_to_camera_view * world_pts
    ).transpose();
    // add col of ones
    camera_view_pts.col(camera_view_pts.cols()-1) = 
        Eigen::VectorXf::Ones(camera_view_pts.rows());

    Eigen::MatrixXf camera_projection_pts = (
        _camera_projection * camera_view_pts.transpose()
    ).transpose();

    Eigen::MatrixXf pixel_coordinates_2d_pts = (
        camera_projection_pts.array().colwise() / 
        camera_projection_pts.col(2).array()
    );
    pixel_coordinates_2d_pts = pixel_coordinates_2d_pts.block(0,0,pixel_coordinates_2d_pts.rows(), 2);

    pixel_coordinates_2d_pts *= 0.5;
    Eigen::VectorXf centre(2);
    centre << 0.5, 0.5;
    pixel_coordinates_2d_pts = pixel_coordinates_2d_pts.rowwise() + centre.transpose();

    Eigen::VectorXf pixel_x = pixel_coordinates_2d_pts.col(0);
    Eigen::VectorXf pixel_y = pixel_coordinates_2d_pts.col(1);

    // Multiplication by image width
    pixel_x = pixel_x * 1280;
    pixel_y = (Eigen::VectorXf::Ones(pixel_y.rows())-pixel_y)*720;

    assert(pixel_x.rows() == pixel_y.rows());
    _frame.r->rowwise().reverseInPlace();
    _frame.g->rowwise().reverseInPlace();
    _frame.b->rowwise().reverseInPlace();
    _frame.r->colwise().reverseInPlace();
    _frame.g->colwise().reverseInPlace();
    _frame.b->colwise().reverseInPlace();

    // new xyzrgb points, with uncolored points filtered out
    std::vector<pcl::PointXYZRGB> newpts;
    newpts.reserve(cloud->points.size());
    for(auto i = 0; i < pixel_x.rows(); i++) {
        int pi_x = (int)pixel_x(i);
        int pi_y = (int)pixel_y(i);

        if(pi_x < 1280 and pi_y < 720 and pi_x >= 0 and pi_y >= 0) {
            uint8_t r = (*_frame.r)(pi_y, pi_x); //Confirm this height, width access
            uint8_t g = (*_frame.g)(pi_y, pi_x);
            uint8_t b = (*_frame.b)(pi_y, pi_x);
            pcl::PointXYZRGB pt(r, g, b);
            pt.x = cloud->points[i].x;
            pt.y = cloud->points[i].y;
            pt.z = cloud->points[i].z;
            newpts.push_back(std::move(pt));
        }
    }
    cloud->points.clear();
    cloud->points.insert(cloud->points.end(), newpts.begin(), newpts.end());

}

}
